/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "pose_graph.h"
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <sstream>
#include <rclcpp/time.hpp>


namespace {

nav_msgs::msg::Path BuildCameraTrajectoryMessage(const std::list<KeyFrame*> &keyframes)
{
    nav_msgs::msg::Path trajectory;
    trajectory.header.frame_id = "global";
    if (keyframes.empty())
        return trajectory;

    trajectory.header.stamp = keyframes.back()->header_stamp;
    trajectory.poses.reserve(keyframes.size());
    for (auto it = keyframes.begin(); it != keyframes.end(); ++it)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getCameraPose(P, R);
        Quaterniond Q{R};

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.frame_id = "global";
        pose_stamped.header.stamp = (*it)->header_stamp;
        pose_stamped.pose.position.x = P.x();
        pose_stamped.pose.position.y = P.y();
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        trajectory.poses.push_back(pose_stamped);
    }

    return trajectory;
}

std::string HeaderStampToTumTimestamp(const builtin_interfaces::msg::Time &stamp)
{
    std::ostringstream stream;
    stream << stamp.sec << "." << std::setw(9) << std::setfill('0') << stamp.nanosec;
    return stream.str();
}

double HeaderStampSeconds(const builtin_interfaces::msg::Time &stamp)
{
    return rclcpp::Time(stamp).seconds();
}

}  // namespace

PoseGraph::PoseGraph()
{
    earliest_loop_index = -1;
    t_drift = Eigen::Vector3d(0, 0, 0);
    yaw_drift = 0;
    r_drift = Eigen::Matrix3d::Identity();
    w_t_vio = Eigen::Vector3d(0, 0, 0);
    w_r_vio = Eigen::Matrix3d::Identity();
    global_index = 0;
    sequence_cnt = 0;
    sequence_loop.push_back(0);
    base_sequence = 1;
    use_imu = 0;
    optimization_running = false;
    last_optimization_header_time = std::numeric_limits<double>::quiet_NaN();
}

PoseGraph::~PoseGraph()
{
    t_optimization.detach();
}

void PoseGraph::registerPub(rclcpp::Node::SharedPtr node){

    pub_trajectory = node->create_publisher<nav_msgs::msg::Path>("/ov_slam/trajectory", 1000);
}

void PoseGraph::setIMUFlag(bool _use_imu)
{
    use_imu = _use_imu;
    if(use_imu)
    {
        printf("[POSEGRAPH]: VIO input, perfrom 4 DoF (x, y, z, yaw) pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize4DoF, this);
    }
    else
    {
        printf("[POSEGRAPH]: VO input, perfrom 6 DoF pose graph optimization\n");
        t_optimization = std::thread(&PoseGraph::optimize6DoF, this);
    }

}

bool PoseGraph::hasPendingOptimization()
{
    std::lock_guard<std::mutex> lock(m_optimize_buf);
    return !optimize_buf.empty();
}

bool PoseGraph::isOptimizationRunning()
{
    return optimization_running.load();
}

void PoseGraph::waitForOptimizationIdle()
{
    while (true)
    {
        if (!hasPendingOptimization() && !isOptimizationRunning())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            if (!hasPendingOptimization() && !isOptimizationRunning())
                return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

bool PoseGraph::requestGlobalOptimization()
{
    std::lock_guard<std::mutex> keyframe_lock(m_keyframelist);
    if (keyframelist.empty())
        return false;

    bool has_loop_closure = false;
    for (const KeyFrame* keyframe : keyframelist)
    {
        if (keyframe->has_loop)
        {
            has_loop_closure = true;
            break;
        }
    }
    if (!has_loop_closure)
        return false;

    std::lock_guard<std::mutex> optimize_lock(m_optimize_buf);
    earliest_loop_index = 0;
    optimize_buf.push(keyframelist.back()->index);
    optimize_global_buf.push(true);
    return true;
}

void PoseGraph::loadVocabulary(std::string voc_path)
{
    voc = new BriefVocabulary(voc_path);
    db.setVocabulary(*voc, false, 0);
}

void PoseGraph::addKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    //shift to base frame
    Vector3d vio_P_cur;
    Matrix3d vio_R_cur;
    if (sequence_cnt != cur_kf->sequence)
    {
        sequence_cnt++;
        sequence_loop.push_back(0);
        w_t_vio = Eigen::Vector3d(0, 0, 0);
        w_r_vio = Eigen::Matrix3d::Identity();
        m_drift.lock();
        t_drift = Eigen::Vector3d(0, 0, 0);
        r_drift = Eigen::Matrix3d::Identity();
        m_drift.unlock();
    }
    
    cur_kf->getVioPose(vio_P_cur, vio_R_cur);
    vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
    vio_R_cur = w_r_vio *  vio_R_cur;
    cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
    cur_kf->index = global_index;
    global_index++;
	int loop_index = -1;
    int loop_inlier_count = 0;
    bool enqueue_optimization = false;
    if (flag_detect_loop)
    {
        TicToc tmp_t;
        loop_index = detectLoop(cur_kf, cur_kf->index, &loop_inlier_count);
    }
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
	if (loop_index != -1)
	{
        printf("[POSEGRAPH]:  %d detect loop with %d (%d inliers)\n",
               cur_kf->index, loop_index, loop_inlier_count);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (old_kf == nullptr)
        {
            printf("[POSEGRAPH]: reject loop candidate %d for keyframe %d: keyframe not found\n",
                   loop_index, cur_kf->index);
        }
        else if ((cur_kf->has_loop && loop_index == old_kf->index) || cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;

            Vector3d w_P_old, w_P_cur, vio_P_cur;
            Matrix3d w_R_old, w_R_cur, vio_R_cur;
            old_kf->getVioPose(w_P_old, w_R_old);
            cur_kf->getVioPose(vio_P_cur, vio_R_cur);

            Vector3d relative_t;
            Quaterniond relative_q;
            relative_t = cur_kf->getLoopRelativeT();
            relative_q = (cur_kf->getLoopRelativeQ()).toRotationMatrix();
            w_P_cur = w_R_old * relative_t + w_P_old;
            w_R_cur = w_R_old * relative_q;
            double shift_yaw;
            Matrix3d shift_r;
            Vector3d shift_t; 
            if(use_imu)
            {
                shift_yaw = Utility::R2ypr(w_R_cur).x() - Utility::R2ypr(vio_R_cur).x();
                shift_r = Utility::ypr2R(Vector3d(shift_yaw, 0, 0));
            }
            else
                shift_r = w_R_cur * vio_R_cur.transpose();
            shift_t = w_P_cur - w_R_cur * vio_R_cur.transpose() * vio_P_cur; 
            // shift vio pose of whole sequence to the world frame
            if (old_kf->sequence != cur_kf->sequence && sequence_loop[cur_kf->sequence] == 0)
            {  
                w_r_vio = shift_r;
                w_t_vio = shift_t;
                vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                vio_R_cur = w_r_vio *  vio_R_cur;
                cur_kf->updateVioPose(vio_P_cur, vio_R_cur);
                list<KeyFrame*>::iterator it = keyframelist.begin();
                for (; it != keyframelist.end(); it++)   
                {
                    if((*it)->sequence == cur_kf->sequence)
                    {
                        Vector3d vio_P_cur;
                        Matrix3d vio_R_cur;
                        (*it)->getVioPose(vio_P_cur, vio_R_cur);
                        vio_P_cur = w_r_vio * vio_P_cur + w_t_vio;
                        vio_R_cur = w_r_vio *  vio_R_cur;
                        (*it)->updateVioPose(vio_P_cur, vio_R_cur);
                    }
                }
                sequence_loop[cur_kf->sequence] = 1;
            }
            enqueue_optimization = true;
        }
	}
	m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getVioPose(P, R);
    P = r_drift * P + t_drift;
    R = r_drift * R;
    cur_kf->updatePose(P, R);
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = cur_kf->header_stamp;
    pose_stamped.header.frame_id = "global";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    path[sequence_cnt].poses.push_back(pose_stamped);
    path[sequence_cnt].header = pose_stamped.header;

    if (cur_kf->has_loop)
    {
        KeyFrame* connected_KF = getKeyFrame(cur_kf->loop_index);
        if (connected_KF == nullptr)
        {
            printf("[POSEGRAPH]: skip loop for keyframe %d: loop keyframe %d not found\n",
                   cur_kf->index, cur_kf->loop_index);
            cur_kf->has_loop = false;
            cur_kf->loop_index = -1;
        }
    }

	keyframelist.push_back(cur_kf);
    if (enqueue_optimization)
    {
        std::lock_guard<std::mutex> optimize_lock(m_optimize_buf);
        optimize_buf.push(cur_kf->index);
        optimize_global_buf.push(false);
    }
    write_trajectory_to_bag(BuildCameraTrajectoryMessage(keyframelist));
    publish();
	m_keyframelist.unlock();
}

void PoseGraph::loadKeyFrame(KeyFrame* cur_kf, bool flag_detect_loop)
{
    cur_kf->index = global_index;
    global_index++;
    int loop_index = -1;
    int loop_inlier_count = 0;
    bool enqueue_optimization = false;
    if (flag_detect_loop)
       loop_index = detectLoop(cur_kf, cur_kf->index, &loop_inlier_count);
    else
    {
        addKeyFrameIntoVoc(cur_kf);
    }
    if (loop_index != -1)
    {
        printf("[POSEGRAPH]:  %d detect loop with %d (%d inliers)\n",
               cur_kf->index, loop_index, loop_inlier_count);
        KeyFrame* old_kf = getKeyFrame(loop_index);
        if (old_kf == nullptr)
        {
            printf("[POSEGRAPH]: reject loop candidate %d for keyframe %d: keyframe not found\n",
                   loop_index, cur_kf->index);
        }
        else if (cur_kf->findConnection(old_kf))
        {
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
                earliest_loop_index = loop_index;
            enqueue_optimization = true;
        }
    }
    m_keyframelist.lock();
    Vector3d P;
    Matrix3d R;
    cur_kf->getPose(P, R);
    Quaterniond Q{R};
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = cur_kf->header_stamp;
    pose_stamped.header.frame_id = "global";
    pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
    pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
    pose_stamped.pose.position.z = P.z();
    pose_stamped.pose.orientation.x = Q.x();
    pose_stamped.pose.orientation.y = Q.y();
    pose_stamped.pose.orientation.z = Q.z();
    pose_stamped.pose.orientation.w = Q.w();
    base_path.poses.push_back(pose_stamped);
    base_path.header = pose_stamped.header;

    keyframelist.push_back(cur_kf);
    if (enqueue_optimization)
    {
        std::lock_guard<std::mutex> optimize_lock(m_optimize_buf);
        optimize_buf.push(cur_kf->index);
        optimize_global_buf.push(false);
    }
    //publish();
    m_keyframelist.unlock();
}

KeyFrame* PoseGraph::getKeyFrame(int index)
{
//    unique_lock<mutex> lock(m_keyframelist);
    list<KeyFrame*>::iterator it = keyframelist.begin();
    for (; it != keyframelist.end(); it++)   
    {
        if((*it)->index == index)
            break;
    }
    if (it != keyframelist.end())
        return *it;
    else
        return NULL;
}

int PoseGraph::detectLoop(KeyFrame* keyframe, int frame_index, int *loop_inlier_count)
{
    if (loop_inlier_count != nullptr)
        *loop_inlier_count = 0;

    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[frame_index] = compressed_image;
    }
    TicToc tmp_t;
    //first query; then add this frame into database!
    QueryResults ret;
    TicToc t_query;
    int max_frame_id_allowed = std::max(0, frame_index - RECALL_IGNORE_RECENT_COUNT);
    db.query(keyframe->brief_descriptors, ret, 0, max_frame_id_allowed);
    //printf("[POSEGRAPH]: query time: %f", t_query.toc());
    //cout << "Searching for Image " << frame_index << ". " << ret << endl;

    TicToc t_add;
    db.add(keyframe->brief_descriptors);
    //printf("[POSEGRAPH]: add feature time: %f", t_add.toc());
    cv::Mat loop_result;
    if (DEBUG_IMAGE)
    {
        loop_result = compressed_image.clone();
        if (ret.size() > 0)
            putText(loop_result, "neighbour score:" + to_string(ret[0].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
    }
    // visual loop result 
    if (DEBUG_IMAGE)
    {
        const unsigned int debug_result_count = std::min<unsigned int>(ret.size(), 3);
        for (unsigned int i = 0; i < debug_result_count; i++)
        {
            int tmp_index = ret[i].Id;
            auto it = image_pool.find(tmp_index);
            if (it == image_pool.end())
                continue;
            cv::Mat tmp_image = (it->second).clone();
            putText(tmp_image, "index:  " + to_string(tmp_index) + "loop score:" + to_string(ret[i].Score), cv::Point2f(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));
            cv::hconcat(loop_result, tmp_image, loop_result);
        }
    }

    //for (unsigned int i = 0; i < ret.size(); i++)
    //    cout << i << " - " <<  ret[i].Score << endl;

/*
    if (DEBUG_IMAGE)
    {
        cv::imshow("loop_result", loop_result);
        cv::waitKey(20);
    }
*/
    //if (find_loop && frame_index > 50)
    if (frame_index < 50)
        return -1;

    struct LoopCandidate
    {
        int index;
        double timestamp;
    };
    std::vector<LoopCandidate> candidates;
    candidates.reserve(ret.size());
    for (unsigned int i = 0; i < ret.size(); i++)
    {
        const int candidate_index = static_cast<int>(ret[i].Id);
        if (candidate_index >= max_frame_id_allowed || ret[i].Score <= MIN_SCORE)
            continue;
        KeyFrame* old_kf = getKeyFrame(candidate_index);
        if (old_kf == nullptr)
            continue;
        candidates.push_back({candidate_index, HeaderStampSeconds(old_kf->header_stamp)});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const LoopCandidate &a, const LoopCandidate &b)
              {
                  if (a.timestamp == b.timestamp)
                      return a.index < b.index;
                  return a.timestamp < b.timestamp;
              });

    struct LoopMatch
    {
        int index = -1;
        int loop_feat_num = 0;
        Eigen::Matrix<double, 8, 1> loop_info = Eigen::Matrix<double, 8, 1>::Zero();
    };

    const double strong_loop_feat_num = MIN_LOOP_NUM * 1.5;
    const int max_weak_loop_candidates = 3;
    LoopMatch best_weak_match;
    int weak_loop_candidates = 0;

    for (const LoopCandidate &candidate : candidates)
    {
        KeyFrame* old_kf = getKeyFrame(candidate.index);
        int loop_feat_num = 0;
        if(old_kf == nullptr || !keyframe->findConnection(old_kf, &loop_feat_num))
            continue;

        if (loop_feat_num > strong_loop_feat_num)
        {
            if (loop_inlier_count != nullptr)
                *loop_inlier_count = loop_feat_num;
            return candidate.index;
        }

        if (loop_feat_num > MIN_LOOP_NUM)
        {
            weak_loop_candidates++;
            if (loop_feat_num > best_weak_match.loop_feat_num)
            {
                best_weak_match.index = candidate.index;
                best_weak_match.loop_feat_num = loop_feat_num;
                best_weak_match.loop_info = keyframe->loop_info;
            }
            if (weak_loop_candidates >= max_weak_loop_candidates)
                break;
        }
    }

    if (best_weak_match.index != -1)
    {
        keyframe->has_loop = true;
        keyframe->loop_index = best_weak_match.index;
        keyframe->loop_info = best_weak_match.loop_info;
        if (loop_inlier_count != nullptr)
            *loop_inlier_count = best_weak_match.loop_feat_num;
        return best_weak_match.index;
    }

    // failure
    return -1;
}

void PoseGraph::addKeyFrameIntoVoc(KeyFrame* keyframe)
{
    // put image into image_pool; for visualization
    cv::Mat compressed_image;
    if (DEBUG_IMAGE)
    {
        int feature_num = keyframe->keypoints.size();
        cv::resize(keyframe->image, compressed_image, cv::Size(376, 240));
        putText(compressed_image, "feature_num:" + to_string(feature_num), cv::Point2f(10, 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255));
        image_pool[keyframe->index] = compressed_image;
    }

    db.add(keyframe->brief_descriptors);
}

bool PoseGraph::prepareOptimizationRequest(int &cur_index, int &first_looped_index,
                                           bool &global_optimization,
                                           double &current_optimization_header_time)
{
    cur_index = -1;
    first_looped_index = -1;
    global_optimization = false;
    current_optimization_header_time = 0.0;

    {
        std::lock_guard<std::mutex> optimize_lock(m_optimize_buf);
        while(!optimize_buf.empty())
        {
            cur_index = optimize_buf.front();
            first_looped_index = earliest_loop_index;
            global_optimization = global_optimization ||
                                  (!optimize_global_buf.empty() && optimize_global_buf.front());
            optimize_buf.pop();
            if (!optimize_global_buf.empty())
                optimize_global_buf.pop();
        }
        if (cur_index != -1)
            optimization_running = true;
    }

    if (cur_index == -1)
        return false;

    bool should_optimize = true;
    bool defer_optimization = false;
    {
        std::lock_guard<std::mutex> lock(m_keyframelist);
        if (keyframelist.empty())
        {
            should_optimize = false;
        }
        else
        {
            KeyFrame* latest_kf = keyframelist.back();
            current_optimization_header_time = HeaderStampSeconds(latest_kf->header_stamp);
            if (!global_optimization && MIN_OPTIMIZATION_TIME_DIFF > 0.0 &&
                !std::isnan(last_optimization_header_time) &&
                current_optimization_header_time <= last_optimization_header_time + MIN_OPTIMIZATION_TIME_DIFF)
            {
                should_optimize = false;
                defer_optimization = true;
            }
            else
            {
                cur_index = latest_kf->index;
            }
        }
    }

    if (!should_optimize)
    {
        if (defer_optimization)
        {
            std::lock_guard<std::mutex> optimize_lock(m_optimize_buf);
            optimize_buf.push(cur_index);
            optimize_global_buf.push(global_optimization);
        }
        optimization_running = false;
        return false;
    }

    return true;
}

void PoseGraph::optimize4DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        bool global_optimization = false;
        double current_optimization_header_time = 0.0;
        if (prepareOptimizationRequest(cur_index, first_looped_index, global_optimization,
                                       current_optimization_header_time))
        {
            const Vector3d optimization_T_i_c = tic;
            const Matrix3d optimization_R_i_c = qic;
            printf("[POSEGRAPH]: optimize pose graph \n");
            TicToc tmp_t1;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);
            if (cur_kf == nullptr)
            {
                m_keyframelist.unlock();
                optimization_running = false;
                continue;
            }

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            Quaterniond q_array[max_length];
            double euler_array[max_length][3];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            options.max_solver_time_in_seconds = global_optimization ? 30 : 5;
            options.max_num_iterations = global_optimization ? 100 : 20;
            options.num_threads = 1;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* angle_local_parameterization = AngleLocalParameterization::Create();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i] = tmp_q;

                Vector3d euler_angle = Utility::R2ypr(tmp_q.toRotationMatrix());
                euler_array[i][0] = euler_angle.x();
                euler_array[i][1] = euler_angle.y();
                euler_array[i][2] = euler_angle.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(euler_array[i], 1, angle_local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(euler_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                  if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                  {
                    Vector3d euler_conncected = Utility::R2ypr(q_array[i-j].toRotationMatrix());
                    Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                    relative_t = q_array[i-j].inverse() * relative_t;
                    double relative_yaw = euler_array[i][0] - euler_array[i-j][0];
                    ceres::CostFunction* cost_function = FourDOFError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                   relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, NULL, euler_array[i-j], 
                                            t_array[i-j], 
                                            euler_array[i], 
                                            t_array[i]);
                  }
                }

                //add loop edge
                if((*it)->has_loop)
                {
                    KeyFrame* connected_kf = getKeyFrame((*it)->loop_index);
                    if ((*it)->loop_index < first_looped_index || connected_kf == nullptr)
                        continue;
                    int connected_index = connected_kf->local_index;
                    Vector3d euler_conncected = Utility::R2ypr(q_array[connected_index].toRotationMatrix());
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    double relative_yaw = (*it)->getLoopRelativeYaw();
                    ceres::CostFunction* cost_function = FourDOFWeightError::Create( relative_t.x(), relative_t.y(), relative_t.z(),
                                                                               relative_yaw, euler_conncected.y(), euler_conncected.z());
                    problem.AddResidualBlock(cost_function, loss_function, euler_array[connected_index], 
                                                                  t_array[connected_index], 
                                                                  euler_array[i], 
                                                                  t_array[i]);
                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();
            double t_create = tmp_t1.toc();


            TicToc tmp_t2;
            ceres::Solve(options, &problem, &summary);
            double t_opt = tmp_t2.toc();
            //std::cout << summary.BriefReport() << "\n";

            //printf("[POSEGRAPH]: pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("[POSEGRAPH]: optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            TicToc tmp_t3;
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q;
                tmp_q = Utility::ypr2R(Vector3d(euler_array[i][0], euler_array[i][1], euler_array[i][2]));
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            yaw_drift = Utility::R2ypr(cur_r).x() - Utility::R2ypr(vio_r).x();
            r_drift = Utility::ypr2R(Vector3d(yaw_drift, 0, 0));
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;
            //cout << "yaw drift " << yaw_drift << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            refreshKeyframeCalibrationUnlocked(optimization_T_i_c, optimization_R_i_c,
                                               nullptr, CalibrationRefresh::ExtrinsicsOnly);
            m_keyframelist.unlock();
            updatePath();
            double t_update = tmp_t3.toc();

            // Nice debug print
            printf("[POSEGRAPH]: creation %.3f ms | optimization %.3f ms | update %.3f ms | %.3f dyaw, %.3f dpos\n", t_create, t_opt, t_update, yaw_drift, t_drift.norm());
            last_optimization_header_time = current_optimization_header_time;
            optimization_running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return;
}

void PoseGraph::optimize6DoF()
{
    while(true)
    {
        int cur_index = -1;
        int first_looped_index = -1;
        bool global_optimization = false;
        double current_optimization_header_time = 0.0;
        if (prepareOptimizationRequest(cur_index, first_looped_index, global_optimization,
                                       current_optimization_header_time))
        {
            const Vector3d optimization_T_i_c = tic;
            const Matrix3d optimization_R_i_c = qic;
            printf("[POSEGRAPH]: optimize pose graph \n");
            TicToc tmp_t;
            m_keyframelist.lock();
            KeyFrame* cur_kf = getKeyFrame(cur_index);
            if (cur_kf == nullptr)
            {
                m_keyframelist.unlock();
                optimization_running = false;
                continue;
            }

            int max_length = cur_index + 1;

            // w^t_i   w^q_i
            double t_array[max_length][3];
            double q_array[max_length][4];
            double sequence_array[max_length];

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //ptions.minimizer_progress_to_stdout = true;
            options.max_solver_time_in_seconds = global_optimization ? 30 : 5;
            options.max_num_iterations = global_optimization ? 100 : 20;
            options.num_threads = 1;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(0.1);
            //loss_function = new ceres::CauchyLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            list<KeyFrame*>::iterator it;

            int i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                (*it)->local_index = i;
                Quaterniond tmp_q;
                Matrix3d tmp_r;
                Vector3d tmp_t;
                (*it)->getVioPose(tmp_t, tmp_r);
                tmp_q = tmp_r;
                t_array[i][0] = tmp_t(0);
                t_array[i][1] = tmp_t(1);
                t_array[i][2] = tmp_t(2);
                q_array[i][0] = tmp_q.w();
                q_array[i][1] = tmp_q.x();
                q_array[i][2] = tmp_q.y();
                q_array[i][3] = tmp_q.z();

                sequence_array[i] = (*it)->sequence;

                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);

                if ((*it)->index == first_looped_index || (*it)->sequence == 0)
                {   
                    problem.SetParameterBlockConstant(q_array[i]);
                    problem.SetParameterBlockConstant(t_array[i]);
                }

                //add edge
                for (int j = 1; j < 5; j++)
                {
                    if (i - j >= 0 && sequence_array[i] == sequence_array[i-j])
                    {
                        Vector3d relative_t(t_array[i][0] - t_array[i-j][0], t_array[i][1] - t_array[i-j][1], t_array[i][2] - t_array[i-j][2]);
                        Quaterniond q_i_j = Quaterniond(q_array[i-j][0], q_array[i-j][1], q_array[i-j][2], q_array[i-j][3]);
                        Quaterniond q_i = Quaterniond(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                        relative_t = q_i_j.inverse() * relative_t;
                        Quaterniond relative_q = q_i_j.inverse() * q_i;
                        ceres::CostFunction* vo_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                        problem.AddResidualBlock(vo_function, NULL, q_array[i-j], t_array[i-j], q_array[i], t_array[i]);
                    }
                }

                //add loop edge
                
                if((*it)->has_loop)
                {
                    KeyFrame* connected_kf = getKeyFrame((*it)->loop_index);
                    if ((*it)->loop_index < first_looped_index || connected_kf == nullptr)
                        continue;
                    int connected_index = connected_kf->local_index;
                    Vector3d relative_t;
                    relative_t = (*it)->getLoopRelativeT();
                    Quaterniond relative_q;
                    relative_q = (*it)->getLoopRelativeQ();
                    ceres::CostFunction* loop_function = RelativeRTError::Create(relative_t.x(), relative_t.y(), relative_t.z(),
                                                                                relative_q.w(), relative_q.x(), relative_q.y(), relative_q.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(loop_function, loss_function, q_array[connected_index], t_array[connected_index], q_array[i], t_array[i]);                    
                }
                
                if ((*it)->index == cur_index)
                    break;
                i++;
            }
            m_keyframelist.unlock();

            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";
            
            //printf("[POSEGRAPH]: pose optimization time: %f \n", tmp_t.toc());
            /*
            for (int j = 0 ; j < i; j++)
            {
                printf("[POSEGRAPH]: optimize i: %d p: %f, %f, %f\n", j, t_array[j][0], t_array[j][1], t_array[j][2] );
            }
            */
            m_keyframelist.lock();
            i = 0;
            for (it = keyframelist.begin(); it != keyframelist.end(); it++)
            {
                if ((*it)->index < first_looped_index)
                    continue;
                Quaterniond tmp_q(q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]);
                Vector3d tmp_t = Vector3d(t_array[i][0], t_array[i][1], t_array[i][2]);
                Matrix3d tmp_r = tmp_q.toRotationMatrix();
                (*it)-> updatePose(tmp_t, tmp_r);

                if ((*it)->index == cur_index)
                    break;
                i++;
            }

            Vector3d cur_t, vio_t;
            Matrix3d cur_r, vio_r;
            cur_kf->getPose(cur_t, cur_r);
            cur_kf->getVioPose(vio_t, vio_r);
            m_drift.lock();
            r_drift = cur_r * vio_r.transpose();
            t_drift = cur_t - r_drift * vio_t;
            m_drift.unlock();
            //cout << "t_drift " << t_drift.transpose() << endl;
            //cout << "r_drift " << Utility::R2ypr(r_drift).transpose() << endl;

            it++;
            for (; it != keyframelist.end(); it++)
            {
                Vector3d P;
                Matrix3d R;
                (*it)->getVioPose(P, R);
                P = r_drift * P + t_drift;
                R = r_drift * R;
                (*it)->updatePose(P, R);
            }
            refreshKeyframeCalibrationUnlocked(optimization_T_i_c, optimization_R_i_c,
                                               nullptr, CalibrationRefresh::ExtrinsicsOnly);
            m_keyframelist.unlock();
            updatePath();

            // Nice debug print
            printf("[POSEGRAPH]: pose optimization in %.3f seconds | %.3f dori, %.3f dpos\n", tmp_t.toc(), Utility::R2ypr(r_drift).norm(), t_drift.norm());
            last_optimization_header_time = current_optimization_header_time;
            optimization_running = false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    return;
}

void PoseGraph::updatePath()
{
    m_keyframelist.lock();
    list<KeyFrame*>::iterator it;
    for (int i = 1; i <= sequence_cnt; i++)
    {
        path[i].poses.clear();
    }
    base_path.poses.clear();

    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        Vector3d P;
        Matrix3d R;
        (*it)->getPose(P, R);
        Quaterniond Q;
        Q = R;
//        printf("[POSEGRAPH]: path p: %f, %f, %f\n",  P.x(),  P.z(),  P.y() );

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header.stamp = (*it)->header_stamp;
        pose_stamped.header.frame_id = "global";
        pose_stamped.pose.position.x = P.x() + VISUALIZATION_SHIFT_X;
        pose_stamped.pose.position.y = P.y() + VISUALIZATION_SHIFT_Y;
        pose_stamped.pose.position.z = P.z();
        pose_stamped.pose.orientation.x = Q.x();
        pose_stamped.pose.orientation.y = Q.y();
        pose_stamped.pose.orientation.z = Q.z();
        pose_stamped.pose.orientation.w = Q.w();
        if((*it)->sequence == 0)
        {
            base_path.poses.push_back(pose_stamped);
            base_path.header = pose_stamped.header;
        }
        else
        {
            path[(*it)->sequence].poses.push_back(pose_stamped);
            path[(*it)->sequence].header = pose_stamped.header;
        }

    }
    // write_trajectory_to_bag(BuildCameraTrajectoryMessage(keyframelist));
    publish();
    m_keyframelist.unlock();
}

void PoseGraph::savePoseGraph()
{
    m_keyframelist.lock();
    TicToc tmp_t;
    FILE *pFile;
    printf("[POSEGRAPH]: pose graph path: %s\n",POSE_GRAPH_SAVE_PATH.c_str());
    printf("[POSEGRAPH]: pose graph saving... \n");
    string file_path = POSE_GRAPH_SAVE_PATH + "pose_graph.txt";
    pFile = fopen (file_path.c_str(),"w");
    //fprintf(pFile, "index time_stamp Tx Ty Tz Qw Qx Qy Qz loop_index loop_info\n");
    list<KeyFrame*>::iterator it;
    for (it = keyframelist.begin(); it != keyframelist.end(); it++)
    {
        std::string image_path, descriptor_path, brief_path, keypoints_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_image.png";
            imwrite(image_path.c_str(), (*it)->image);
        }
        Quaterniond VIO_tmp_Q{(*it)->vio_R_w_i};
        Quaterniond PG_tmp_Q{(*it)->R_w_i};
        Quaterniond EXT_tmp_Q{(*it)->R_i_c};
        Vector3d VIO_tmp_T = (*it)->vio_T_w_i;
        Vector3d PG_tmp_T = (*it)->T_w_i;
        Vector3d EXT_tmp_T = (*it)->T_i_c;

        fprintf (pFile, " %d %f %d %u %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f %f %d %f %f %f %f %f %f %f",(*it)->index, (*it)->time_stamp,
	                                    (*it)->header_stamp.sec, (*it)->header_stamp.nanosec,
	                                    VIO_tmp_T.x(), VIO_tmp_T.y(), VIO_tmp_T.z(),
	                                    PG_tmp_T.x(), PG_tmp_T.y(), PG_tmp_T.z(),
                                    VIO_tmp_Q.w(), VIO_tmp_Q.x(), VIO_tmp_Q.y(), VIO_tmp_Q.z(), 
                                    PG_tmp_Q.w(), PG_tmp_Q.x(), PG_tmp_Q.y(), PG_tmp_Q.z(), 
                                    (*it)->loop_index, 
                                    (*it)->loop_info(0), (*it)->loop_info(1), (*it)->loop_info(2), (*it)->loop_info(3),
                                    (*it)->loop_info(4), (*it)->loop_info(5), (*it)->loop_info(6), (*it)->loop_info(7),
	                                    (int)(*it)->keypoints.size(),
	                                    EXT_tmp_T.x(), EXT_tmp_T.y(), EXT_tmp_T.z(),
	                                    EXT_tmp_Q.w(), EXT_tmp_Q.x(), EXT_tmp_Q.y(), EXT_tmp_Q.z());

        int camera_model = -1;
        int camera_width = 0;
        int camera_height = 0;
        std::vector<double> camera_parameters;
        if ((*it)->getCameraParameters(camera_model, camera_width, camera_height, camera_parameters))
        {
            fprintf(pFile, " %d %d %d %d",
                    camera_model, camera_width, camera_height,
                    (int)camera_parameters.size());
            for (double parameter : camera_parameters)
                fprintf(pFile, " %.17g", parameter);
        }
        fprintf(pFile, "\n");

        // write keypoints, brief_descriptors   vector<cv::KeyPoint> keypoints vector<BRIEF::bitset> brief_descriptors;
        assert((*it)->keypoints.size() == (*it)->brief_descriptors.size());
        brief_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_briefdes.dat";
        std::ofstream brief_file(brief_path, std::ios::binary);
        keypoints_path = POSE_GRAPH_SAVE_PATH + to_string((*it)->index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "w");
        for (int i = 0; i < (int)(*it)->keypoints.size(); i++)
        {
            brief_file << (*it)->brief_descriptors[i] << endl;
            fprintf(keypoints_file, "%f %f %f %f\n", (*it)->keypoints[i].pt.x, (*it)->keypoints[i].pt.y, 
                                                     (*it)->keypoints_norm[i].pt.x, (*it)->keypoints_norm[i].pt.y);
        }
        brief_file.close();
        fclose(keypoints_file);
    }
    fclose(pFile);

    printf("[POSEGRAPH]: save pose graph time: %f s\n", tmp_t.toc() / 1000);
    m_keyframelist.unlock();
}

void PoseGraph::writeFinalTrajectoryToBag()
{
    std::lock_guard<std::mutex> lock(m_keyframelist);
    write_final_trajectory_to_bag(BuildCameraTrajectoryMessage(keyframelist));
}

void PoseGraph::saveFinalTrajectoryTum(const std::string &file_path)
{
    std::lock_guard<std::mutex> lock(m_keyframelist);
    std::ofstream tum_file(file_path);
    tum_file << "# timestamp tx ty tz qx qy qz qw\n";
    for (KeyFrame *keyframe : keyframelist)
    {
        Vector3d P;
        Matrix3d R;
        keyframe->getCameraPose(P, R);
        Quaterniond Q(R);
        tum_file << std::fixed << std::setprecision(9)
                 << HeaderStampToTumTimestamp(keyframe->header_stamp) << " "
                 << P.x() << " " << P.y() << " " << P.z() << " "
                 << Q.x() << " " << Q.y() << " " << Q.z() << " " << Q.w() << "\n";
    }
}

void PoseGraph::updateAllKeyframeExtrinsics(const Eigen::Vector3d &_T_i_c, const Eigen::Matrix3d &_R_i_c)
{
    std::lock_guard<std::mutex> lock(m_keyframelist);
    refreshKeyframeCalibrationUnlocked(_T_i_c, _R_i_c, nullptr, CalibrationRefresh::ExtrinsicsOnly);
}

void PoseGraph::updateAllKeyframeCalibration(const Eigen::Vector3d &_T_i_c, const Eigen::Matrix3d &_R_i_c,
                                             const camodocal::CameraPtr &_camera)
{
    std::lock_guard<std::mutex> lock(m_keyframelist);
    refreshKeyframeCalibrationUnlocked(_T_i_c, _R_i_c, _camera, CalibrationRefresh::FullCalibration);
}

void PoseGraph::refreshKeyframeCalibrationUnlocked(const Eigen::Vector3d &_T_i_c,
                                                   const Eigen::Matrix3d &_R_i_c,
                                                   const camodocal::CameraPtr &_camera,
                                                   CalibrationRefresh refresh)
{
    for (KeyFrame *keyframe : keyframelist)
    {
        if (refresh == CalibrationRefresh::FullCalibration)
            keyframe->updateCalibration(_T_i_c, _R_i_c, _camera);
        else
            keyframe->updateExtrinsics(_T_i_c, _R_i_c);
    }
}

void PoseGraph::saveDebugTrajectoriesAndLoopEdges(
    const std::string &output_dir,
    const std::map<int, std::string> &session_output_names)
{
    std::filesystem::create_directories(output_dir);
    const std::filesystem::path output_path(output_dir);

    std::map<int, std::ofstream> tum_files;
    std::map<int, std::ofstream> csv_files;
    std::map<int, int> csv_counters;
    std::ofstream loop_edges((output_path / "loop_edges.txt").string());
    loop_edges << "# from_index to_index from_sequence to_sequence "
               << "from_time to_time "
               << "from_tx from_ty from_tz from_qx from_qy from_qz from_qw "
               << "to_tx to_ty to_tz to_qx to_qy to_qz to_qw "
               << "relative_tx relative_ty relative_tz "
               << "relative_qx relative_qy relative_qz relative_qw relative_yaw\n";

    std::lock_guard<std::mutex> lock(m_keyframelist);
    for (KeyFrame *keyframe : keyframelist)
    {
        auto tum_file_it = tum_files.find(keyframe->sequence);
        if (tum_file_it == tum_files.end())
        {
            std::string output_name = "session_" + std::to_string(keyframe->sequence);
            auto name_it = session_output_names.find(keyframe->sequence);
            if (name_it != session_output_names.end() && !name_it->second.empty())
                output_name = name_it->second;

            auto tum_inserted = tum_files.emplace(
                keyframe->sequence, std::ofstream((output_path / (output_name + ".txt")).string()));
            tum_file_it = tum_inserted.first;
            tum_file_it->second << "# timestamp tx ty tz qx qy qz qw\n";

            auto csv_inserted = csv_files.emplace(
                keyframe->sequence, std::ofstream((output_path / (output_name + ".csv")).string()));
            csv_inserted.first->second << "# counter,sec,nsec,x,y,z,qx,qy,qz,qw\n";
            csv_counters[keyframe->sequence] = 0;
        }

        Vector3d P;
        Matrix3d R;
        keyframe->getCameraPose(P, R);
        Quaterniond Q(R);
        tum_file_it->second << std::fixed << std::setprecision(9)
                            << HeaderStampToTumTimestamp(keyframe->header_stamp) << " "
                            << P.x() << " " << P.y() << " " << P.z() << " "
                            << Q.x() << " " << Q.y() << " " << Q.z() << " " << Q.w() << "\n";

        auto csv_file_it = csv_files.find(keyframe->sequence);
        csv_file_it->second << csv_counters[keyframe->sequence]++ << ","
                            << keyframe->header_stamp.sec << ","
                            << keyframe->header_stamp.nanosec << ","
                            << std::fixed << std::setprecision(12)
                            << P.x() << "," << P.y() << "," << P.z() << ","
                            << Q.x() << "," << Q.y() << "," << Q.z() << "," << Q.w() << "\n";

        if (!keyframe->has_loop)
            continue;

        KeyFrame *connected = getKeyFrame(keyframe->loop_index);
        if (!connected)
            continue;

        Vector3d connected_P;
        Matrix3d connected_R;
        connected->getCameraPose(connected_P, connected_R);
        Quaterniond connected_Q(connected_R);

        Vector3d relative_t = keyframe->getLoopRelativeT();
        Quaterniond relative_q = keyframe->getLoopRelativeQ();
        loop_edges << std::fixed << std::setprecision(9)
                   << keyframe->index << " " << connected->index << " "
                   << keyframe->sequence << " " << connected->sequence << " "
                   << keyframe->time_stamp << " " << connected->time_stamp << " "
                   << P.x() << " " << P.y() << " " << P.z() << " "
                   << Q.x() << " " << Q.y() << " " << Q.z() << " " << Q.w() << " "
                   << connected_P.x() << " " << connected_P.y() << " " << connected_P.z() << " "
                   << connected_Q.x() << " " << connected_Q.y() << " "
                   << connected_Q.z() << " " << connected_Q.w() << " "
                   << relative_t.x() << " " << relative_t.y() << " " << relative_t.z() << " "
                   << relative_q.x() << " " << relative_q.y() << " "
                   << relative_q.z() << " " << relative_q.w() << " "
                   << keyframe->getLoopRelativeYaw() << "\n";
    }
}

void PoseGraph::loadPoseGraph()
{
    TicToc tmp_t;
    FILE * pFile;
    string file_path = POSE_GRAPH_LOAD_PATH + "pose_graph.txt";
    printf("[POSEGRAPH]: lode pose graph from: %s \n", file_path.c_str());
    printf("[POSEGRAPH]: pose graph loading...\n");
    pFile = fopen (file_path.c_str(),"r");
    if (pFile == NULL)
    {
        printf("[POSEGRAPH]: lode previous pose graph error: wrong previous pose graph path or no previous pose graph \n the system will start with new pose graph \n");
        return;
    }
    int index;
    int header_stamp_sec;
    unsigned int header_stamp_nanosec;
    double time_stamp;
    double VIO_Tx, VIO_Ty, VIO_Tz;
    double PG_Tx, PG_Ty, PG_Tz;
    double EXT_Tx, EXT_Ty, EXT_Tz;
    double VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz;
    double PG_Qw, PG_Qx, PG_Qy, PG_Qz;
    double EXT_Qw, EXT_Qx, EXT_Qy, EXT_Qz;
    double loop_info_0, loop_info_1, loop_info_2, loop_info_3;
    double loop_info_4, loop_info_5, loop_info_6, loop_info_7;
    int loop_index;
    int keypoints_num;
    Eigen::Matrix<double, 8, 1 > loop_info;
	    int cnt = 0;
	    char line[4096];
	    while (fgets(line, sizeof(line), pFile) != NULL)
	    {
	        std::istringstream line_stream(line);
	        if (!(line_stream >> index >> time_stamp
	                          >> header_stamp_sec >> header_stamp_nanosec
	                          >> VIO_Tx >> VIO_Ty >> VIO_Tz
	                          >> PG_Tx >> PG_Ty >> PG_Tz
	                          >> VIO_Qw >> VIO_Qx >> VIO_Qy >> VIO_Qz
	                          >> PG_Qw >> PG_Qx >> PG_Qy >> PG_Qz
	                          >> loop_index
	                          >> loop_info_0 >> loop_info_1 >> loop_info_2 >> loop_info_3
	                          >> loop_info_4 >> loop_info_5 >> loop_info_6 >> loop_info_7
	                          >> keypoints_num))
	            continue;

	        if (!(line_stream >> EXT_Tx >> EXT_Ty >> EXT_Tz
	                          >> EXT_Qw >> EXT_Qx >> EXT_Qy >> EXT_Qz))
	        {
	            EXT_Tx = tic.x();
	            EXT_Ty = tic.y();
	            EXT_Tz = tic.z();
	            Quaterniond latest_ext_q(qic);
	            EXT_Qw = latest_ext_q.w();
	            EXT_Qx = latest_ext_q.x();
	            EXT_Qy = latest_ext_q.y();
	            EXT_Qz = latest_ext_q.z();
	        }

	        camodocal::CameraPtr frame_camera = m_camera;
	        int camera_model = -1;
	        int camera_width = 0;
	        int camera_height = 0;
	        int camera_param_count = 0;
	        if (line_stream >> camera_model >> camera_width >> camera_height >> camera_param_count)
	        {
	            std::vector<double> camera_parameters;
	            camera_parameters.reserve(std::max(0, camera_param_count));
	            bool camera_parameters_valid = camera_param_count > 0;
	            for (int i = 0; i < camera_param_count; i++)
	            {
	                double parameter = 0.0;
	                if (!(line_stream >> parameter))
	                {
	                    camera_parameters_valid = false;
	                    break;
	                }
	                camera_parameters.push_back(parameter);
	            }
	            if (camera_parameters_valid)
	            {
	                frame_camera = camodocal::CameraFactory::instance()->generateCamera(
	                    static_cast<camodocal::Camera::ModelType>(camera_model),
	                    "cam0",
	                    cv::Size(camera_width, camera_height));
	                frame_camera->readParameters(camera_parameters);
	            }
	        }
	        /*
        printf("[POSEGRAPH]: I read: %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %d\n", index, time_stamp, 
                                    VIO_Tx, VIO_Ty, VIO_Tz, 
                                    PG_Tx, PG_Ty, PG_Tz, 
                                    VIO_Qw, VIO_Qx, VIO_Qy, VIO_Qz, 
                                    PG_Qw, PG_Qx, PG_Qy, PG_Qz, 
                                    loop_index,
                                    loop_info_0, loop_info_1, loop_info_2, loop_info_3, 
                                    loop_info_4, loop_info_5, loop_info_6, loop_info_7,
                                    keypoints_num);
        */
        cv::Mat image;
        std::string image_path, descriptor_path;
        if (DEBUG_IMAGE)
        {
            image_path = POSE_GRAPH_LOAD_PATH + to_string(index) + "_image.png";
            image = cv::imread(image_path.c_str(), 0);
        }

        Vector3d VIO_T(VIO_Tx, VIO_Ty, VIO_Tz);
        Vector3d PG_T(PG_Tx, PG_Ty, PG_Tz);
        Vector3d EXT_T(EXT_Tx, EXT_Ty, EXT_Tz);
        Quaterniond VIO_Q;
        VIO_Q.w() = VIO_Qw;
        VIO_Q.x() = VIO_Qx;
        VIO_Q.y() = VIO_Qy;
        VIO_Q.z() = VIO_Qz;
        Quaterniond PG_Q;
        PG_Q.w() = PG_Qw;
        PG_Q.x() = PG_Qx;
        PG_Q.y() = PG_Qy;
        PG_Q.z() = PG_Qz;
        Matrix3d VIO_R, PG_R;
        VIO_R = VIO_Q.toRotationMatrix();
        PG_R = PG_Q.toRotationMatrix();
        Quaterniond EXT_Q(EXT_Qw, EXT_Qx, EXT_Qy, EXT_Qz);
        Matrix3d EXT_R = EXT_Q.toRotationMatrix();
        builtin_interfaces::msg::Time header_stamp;
        header_stamp.sec = static_cast<int32_t>(header_stamp_sec);
        header_stamp.nanosec = static_cast<uint32_t>(header_stamp_nanosec);
        Eigen::Matrix<double, 8, 1 > loop_info;
        loop_info << loop_info_0, loop_info_1, loop_info_2, loop_info_3, loop_info_4, loop_info_5, loop_info_6, loop_info_7;

        if (loop_index != -1)
            if (earliest_loop_index > loop_index || earliest_loop_index == -1)
            {
                earliest_loop_index = loop_index;
            }

        // load keypoints, brief_descriptors   
        string brief_path = POSE_GRAPH_LOAD_PATH + to_string(index) + "_briefdes.dat";
        std::ifstream brief_file(brief_path, std::ios::binary);
        string keypoints_path = POSE_GRAPH_LOAD_PATH + to_string(index) + "_keypoints.txt";
        FILE *keypoints_file;
        keypoints_file = fopen(keypoints_path.c_str(), "r");
        vector<cv::KeyPoint> keypoints;
        vector<cv::KeyPoint> keypoints_norm;
        vector<BRIEF::bitset> brief_descriptors;
        for (int i = 0; i < keypoints_num; i++)
        {
            BRIEF::bitset tmp_des;
            brief_file >> tmp_des;
            brief_descriptors.push_back(tmp_des);
            cv::KeyPoint tmp_keypoint;
            cv::KeyPoint tmp_keypoint_norm;
            double p_x, p_y, p_x_norm, p_y_norm;
            if(!fscanf(keypoints_file,"%lf %lf %lf %lf", &p_x, &p_y, &p_x_norm, &p_y_norm))
                printf("[POSEGRAPH]:  fail to load pose graph \n");
            tmp_keypoint.pt.x = p_x;
            tmp_keypoint.pt.y = p_y;
            tmp_keypoint_norm.pt.x = p_x_norm;
            tmp_keypoint_norm.pt.y = p_y_norm;
            keypoints.push_back(tmp_keypoint);
            keypoints_norm.push_back(tmp_keypoint_norm);
        }
        brief_file.close();
        fclose(keypoints_file);

        KeyFrame* keyframe = new KeyFrame(header_stamp, index, VIO_T, VIO_R, PG_T, PG_R, image, loop_index, loop_info, keypoints, keypoints_norm, brief_descriptors, EXT_T, EXT_R, frame_camera);
        loadKeyFrame(keyframe, 0);
        if (cnt % 20 == 0)
        {
            publish();
        }
        cnt++;
    }
    fclose (pFile);
    printf("[POSEGRAPH]: load pose graph time: %f s\n", tmp_t.toc()/1000);
    base_sequence = 0;
}

void PoseGraph::publish()
{
    if (pub_trajectory)
        pub_trajectory->publish(BuildCameraTrajectoryMessage(keyframelist));
}
