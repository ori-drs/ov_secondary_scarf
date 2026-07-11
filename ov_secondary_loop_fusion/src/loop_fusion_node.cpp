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

#include <vector>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <std_msgs/msg/bool.hpp>
#include <filesystem>
#include <algorithm>
#include <iostream>
//#include <ros/package.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <cctype>
#include <atomic>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "utility/compressed_image.h"
#include "pose_graph.h"
#include "loop_fusion_io.h"
#include "parameters.h"
#include "loop_fusion_node.h"


App::App(rclcpp::Node::SharedPtr, const CommandLineConfig &) {}


using namespace std;

queue<sensor_msgs::msg::CompressedImage::ConstSharedPtr> image_buf;
std::queue<sensor_msgs::msg::PointCloud::ConstSharedPtr> point_buf;
//queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<nav_msgs::msg::Odometry::ConstSharedPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;
double MIN_SCORE = 0.015;
double PNP_INFLATION = 1.0;
int RECALL_IGNORE_RECENT_COUNT = 50;
double MAX_THETA_DIFF = 30.0;
double MAX_POS_DIFF = 20.0;
int MIN_LOOP_NUM = 25;
int BRIEF_MATCH_HAMMING_THRESH = 80;
double MIN_OPTIMIZATION_TIME_DIFF;
const double TIMESTAMP_SYNC_TOLERANCE = 0.000001; // seconds
double SKIP_FIRST_SECONDS = 0.0;
bool SERIAL_PROCESSING = false;
const size_t DEFAULT_SUBSCRIPTION_QUEUE_SIZE = 2000;
const size_t SERIAL_SUBSCRIPTION_QUEUE_SIZE = 5000;
const size_t SERIAL_INPUT_BUFFER_LIMIT = SERIAL_SUBSCRIPTION_QUEUE_SIZE;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;

camodocal::CameraPtr m_camera;
double max_focallength = 460.0;
Eigen::Vector3d tic = Eigen::Vector3d::Zero();
Eigen::Matrix3d qic = Eigen::Matrix3d::Identity();
std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string POSE_GRAPH_LOAD_PATH;
std::string OUTPUT_PATH;
std::string TRAJECTORY_BAG_TOPIC = "/ov_slam/trajectory";
std::string ODOMETRY_BAG_TOPIC = "/ov_slam/odometry";
std::string IMAGE_BAG_TOPIC = "/ov_slam/image/compressed";
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;
double image_stream_start_time = -1;
double keyframe_stream_start_time = -1;
bool pose_graph_auto_saved = false;
bool has_image_arrival = false;
std::atomic_bool keyframe_processing_active = false;
std::chrono::steady_clock::time_point last_image_arrival;
rclcpp::TimerBase::SharedPtr image_inactive_timer;

struct KeyframeProcessingGuard
{
    ~KeyframeProcessingGuard()
    {
        keyframe_processing_active = false;
    }
};

void throw_serial_buffer_full(const char *buffer_name)
{
    const std::string message =
        std::string("[POSEGRAPH]: serial input buffer '") + buffer_name +
        "' is full. Use a lower bag play rate or disable serial.";
    RCLCPP_ERROR(rclcpp::get_logger("loop_fusion"), "%s", message.c_str());
    throw std::runtime_error(message);
}

std::string final_trajectory_tum_path()
{
    return (std::filesystem::path(OUTPUT_PATH) / "ov_slam" / "trajectory_final.txt").string();
}

std::string trim_copy(const std::string &value)
{
    const auto first = std::find_if_not(value.begin(), value.end(),
                                        [](unsigned char c) { return std::isspace(c); });
    const auto last = std::find_if_not(value.rbegin(), value.rend(),
                                       [](unsigned char c) { return std::isspace(c); }).base();
    if (first >= last)
        return "";
    return std::string(first, last);
}

std::string strip_inline_comment(const std::string &value)
{
    const size_t comment_start = value.find('#');
    if (comment_start == std::string::npos)
        return value;
    return value.substr(0, comment_start);
}

bool is_null_path_value(const std::string &value)
{
    std::string normalized = trim_copy(value);
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    return normalized.empty() || normalized == "none" || normalized == "null";
}

std::runtime_error invalid_output_path_error()
{
    return std::runtime_error("\033[31minvalid output_path\033[0m");
}

void refresh_calibration_for_output()
{
    posegraph.updateAllKeyframeCalibration(tic, qic, m_camera);
}

void write_final_trajectory_outputs()
{
    posegraph.writeFinalTrajectoryToBag();
    posegraph.saveFinalTrajectoryTum(final_trajectory_tum_path());
}

void save_pose_graph_outputs(bool include_bag_trajectory)
{
    refresh_calibration_for_output();
    posegraph.savePoseGraph();
    if (include_bag_trajectory)
        posegraph.writeFinalTrajectoryToBag();
    posegraph.saveFinalTrajectoryTum(final_trajectory_tum_path());
}

camodocal::CameraPtr camera_from_info(const sensor_msgs::msg::CameraInfo &msg)
{
    camodocal::Camera::ModelType model_type;
    if (msg.distortion_model == "plumb_bob")
        model_type = camodocal::Camera::ModelType::PINHOLE;
    else if (msg.distortion_model == "equidistant")
        model_type = camodocal::Camera::ModelType::KANNALA_BRANDT;
    else
        throw std::runtime_error("Invalid distortion model, unable to parse (plumb_bob, equidistant)");

    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCamera(
            model_type, "cam0", cv::Size(msg.width, msg.height));

    std::vector<double> parameters{
        msg.d.at(0),
        msg.d.at(1),
        msg.d.at(2),
        msg.d.at(3),
        msg.k.at(0),
        msg.k.at(4),
        msg.k.at(2),
        msg.k.at(5)};
    camera->readParameters(parameters);
    return camera;
}

void new_sequence()
{
    printf("[POSEGRAPH]: new sequence\n");
    sequence++;
    printf("[POSEGRAPH]: sequence cnt %d \n", sequence);
    if (sequence > 5)
    {
        //ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        //ROS_BREAK();
        RCLCPP_WARN(rclcpp::get_logger("loop_fusion"),
                    "Only support 5 sequences; extra sequences will be ignored.");
    }
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
    image_stream_start_time = -1;
    keyframe_stream_start_time = -1;
    pose_graph_auto_saved = false;
}

void App::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg)
{
    //ROS_INFO("image_callback!");
    {
        std::lock_guard<std::mutex> lock(m_buf);
        if (SERIAL_PROCESSING && image_buf.size() >= SERIAL_INPUT_BUFFER_LIMIT)
            throw_serial_buffer_full("image");
        image_buf.push(image_msg);
    }
    //printf("[POSEGRAPH]:  image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    const double image_time = rclcpp::Time(image_msg->header.stamp).seconds();
    if (last_image_time < 0.0)
        last_image_time = image_time;
    else if (image_time - last_image_time > 1.0 || image_time < last_image_time)
    {
        //ROS_WARN("image discontinue! detect a new sequence!");
        new_sequence();
    }
    last_image_time = image_time;

    if (image_stream_start_time < 0.0)
        image_stream_start_time = image_time;

    // Use wall time for inactivity checks to avoid ROS/sim time mismatches.
    last_image_arrival = std::chrono::steady_clock::now();
    has_image_arrival = true;
    if (pose_graph_auto_saved)
        pose_graph_auto_saved = false;
}

void App::point_callback(const sensor_msgs::msg::PointCloud::SharedPtr point_msg)
{
    //ROS_INFO("point_callback!");
    std::lock_guard<std::mutex> lock(m_buf);
    if (SERIAL_PROCESSING && point_buf.size() >= SERIAL_INPUT_BUFFER_LIMIT)
        throw_serial_buffer_full("point");
    point_buf.push(point_msg); // In ROS1 it was just "push"
}

void App::pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    //ROS_INFO("pose_callback!");
    std::lock_guard<std::mutex> lock(m_buf);
    if (SERIAL_PROCESSING && pose_buf.size() >= SERIAL_INPUT_BUFFER_LIMIT)
        throw_serial_buffer_full("pose");
    pose_buf.push(pose_msg);
    /*
    printf("[POSEGRAPH]: pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,
                                                       pose_msg->pose.pose.position.y,
                                                       pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,
                                                       pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,
                                                       pose_msg->pose.pose.orientation.z);
    */
}

void App::extrinsic_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void App::intrinsics_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    m_process.lock();
    assert(msg->k.size()==9);
    assert(msg->d.size()>=4);
    m_camera = camera_from_info(*msg);
    max_focallength = std::max(msg->k.at(0), msg->k.at(4));
    m_process.unlock();
}

void process()
{
    while (true)
    {
        sensor_msgs::msg::CompressedImage::ConstSharedPtr image_msg = NULL;
        sensor_msgs::msg::PointCloud::ConstSharedPtr point_msg = NULL;
        nav_msgs::msg::Odometry::ConstSharedPtr pose_msg = NULL;

        auto stamp_seconds = [](const rclcpp::Time &t) { return t.seconds(); };

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            const double image_front_time = stamp_seconds(rclcpp::Time(image_buf.front()->header.stamp));
            const double pose_front_time  = stamp_seconds(rclcpp::Time(pose_buf.front()->header.stamp));
            const double point_front_time = stamp_seconds(rclcpp::Time(point_buf.front()->header.stamp));

            if (SERIAL_PROCESSING)
            {
                if (image_front_time + TIMESTAMP_SYNC_TOLERANCE < pose_front_time)
                {
                    image_buf.pop();
                    printf("[POSEGRAPH]: serial throw stale image | image: %.9f  pose: %.9f\n", image_front_time, pose_front_time);
                }
                else if (point_front_time + TIMESTAMP_SYNC_TOLERANCE < pose_front_time)
                {
                    point_buf.pop();
                    printf("[POSEGRAPH]: serial throw stale point | point: %.9f  pose: %.9f\n", point_front_time, pose_front_time);
                }
                else if (pose_front_time + TIMESTAMP_SYNC_TOLERANCE < image_front_time ||
                         pose_front_time + TIMESTAMP_SYNC_TOLERANCE < point_front_time)
                {
                    pose_buf.pop();
                    printf("[POSEGRAPH]: serial throw unmatched pose | pose: %.9f  image: %.9f  point: %.9f\n",
                           pose_front_time, image_front_time, point_front_time);
                }
                else
                {
                    pose_msg = pose_buf.front();
                    pose_buf.pop();
                    image_msg = image_buf.front();
                    image_buf.pop();
                    point_msg = point_buf.front();
                    point_buf.pop();
                }
            }
            else if (image_front_time - pose_front_time > TIMESTAMP_SYNC_TOLERANCE)
            {
                pose_buf.pop();
                printf("[POSEGRAPH]: throw pose at beginning | image: %.9f  pose: %.9f\n", image_front_time, pose_front_time);
            }
            else if (image_front_time - point_front_time > TIMESTAMP_SYNC_TOLERANCE)
            {
                point_buf.pop();
                printf("[POSEGRAPH]: throw point at beginning | image: %.9f  point: %.9f\n", image_front_time, point_front_time);
            }
            else if (stamp_seconds(rclcpp::Time(image_buf.back()->header.stamp)) + TIMESTAMP_SYNC_TOLERANCE >= pose_front_time
                     && stamp_seconds(rclcpp::Time(point_buf.back()->header.stamp)) + TIMESTAMP_SYNC_TOLERANCE >= pose_front_time)
            {
                // printf("[POSEGRAPH]: sync ok | image(front/back): %.9f/%.9f  point(front/back): %.9f/%.9f  pose(front): %.9f  tol: %.3fms\n",
                //        image_front_time,
                //        stamp_seconds(rclcpp::Time(image_buf.back()->header.stamp)),
                //        point_front_time,
                //        stamp_seconds(rclcpp::Time(point_buf.back()->header.stamp)),
                //        pose_front_time,
                //        TIMESTAMP_SYNC_TOLERANCE * 1000.0);
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (stamp_seconds(rclcpp::Time(image_buf.front()->header.stamp)) + TIMESTAMP_SYNC_TOLERANCE < pose_front_time)
                    image_buf.pop();
                image_msg = image_buf.front();
                image_buf.pop();

                while (stamp_seconds(rclcpp::Time(point_buf.front()->header.stamp)) + TIMESTAMP_SYNC_TOLERANCE < pose_front_time)
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        if (pose_msg != NULL)
            keyframe_processing_active = true;
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            KeyframeProcessingGuard processing_guard;
            //printf("[POSEGRAPH]:  pose time %f \n", pose_msg->header.stamp.toSec());
            //printf("[POSEGRAPH]:  point time %f \n", point_msg->header.stamp.toSec());
            //printf("[POSEGRAPH]:  image time %f \n", image_msg->header.stamp.toSec());
            const double image_time = stamp_seconds(rclcpp::Time(image_msg->header.stamp));
            if (keyframe_stream_start_time < 0.0)
                keyframe_stream_start_time = image_time;
            if (image_time - keyframe_stream_start_time < SKIP_FIRST_SECONDS)
            {
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            cv::Mat image = DecodeCompressedMonoImage(*image_msg);
            //cv::equalizeHist(image, image);

            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);

                    //printf("[POSEGRAPH]: u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                Vector3d keyframe_T_i_c;
                Matrix3d keyframe_R_i_c;
                camodocal::CameraPtr keyframe_camera;
                m_process.lock();
                keyframe_T_i_c = tic;
                keyframe_R_i_c = qic;
                keyframe_camera = m_camera;
                m_process.unlock();

                nav_msgs::msg::Odometry image_time_pose =
                    ov_secondary::BuildCameraOdometryMessage(*pose_msg, keyframe_T_i_c, keyframe_R_i_c);
                image_time_pose.header.stamp = image_msg->header.stamp;
                write_odometry_to_bag(image_time_pose);

                write_compressed_image_to_bag(*image_msg);

                KeyFrame* keyframe = new KeyFrame(image_msg->header.stamp, frame_index, T, R, image,
                                   point_3d, point_2d_uv, point_2d_normal, point_id, sequence,
                                   keyframe_T_i_c, keyframe_R_i_c, keyframe_camera);
                m_process.lock();
                start_flag = 1;
                posegraph.addKeyFrame(keyframe, 1);
                m_process.unlock();
                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            save_pose_graph_outputs(true);
            m_process.unlock();
            printf("[POSEGRAPH]: save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("[POSEGRAPH]: program shutting down...\n");
            rclcpp::shutdown();
        }
        if (c == 'n')
            new_sequence();

        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

// Note: ROS2 throws an error if the same parameter is declared twice
template <class T>
void declareParameter(rclcpp::Node::SharedPtr nh, const std::string &param_field, const T &type) {
  try {
    nh->declare_parameter(param_field, type);
  } catch (rclcpp::exceptions::ParameterAlreadyDeclaredException &e) {
    std::cerr << "Error declaring parameter: " << e.what() << "\n";
  }
}

void getParamOrExit(rclcpp::Node::SharedPtr nh, const std::string &param_field, std::string& variable){
  //std::cout << param_field << " is of string\n" << std::flush;
  declareParameter(nh, param_field, rclcpp::PARAMETER_STRING);
  if (!nh->get_parameter(param_field, variable)) {
    throw std::invalid_argument("Exiting. Couldn't find param: " + param_field);
  }
  std::cout << param_field << ": " << variable <<" (string)\n";
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv); // ros::init_options::AnonymousName);
    rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("loop_fusion");
  
    CommandLineConfig app_params;
    std::shared_ptr<App> app = std::make_shared<App>(nh, app_params);
    posegraph.registerPub(nh);

    auto save_topic = nh->create_subscription<std_msgs::msg::Bool>(
        "save_pose_graph",
        10,
        [](const std_msgs::msg::Bool::SharedPtr msg) {
            if (!msg->data) {
                return;
            }
            m_process.lock();
            save_pose_graph_outputs(false);
            m_process.unlock();
            printf("[POSEGRAPH]: save pose graph finish\n");
        });

    image_inactive_timer = nh->create_wall_timer(
        std::chrono::milliseconds(200),
        []() {
            if (!has_image_arrival || pose_graph_auto_saved)
                return;

            const auto now = std::chrono::steady_clock::now();
            const auto idle = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_image_arrival);
            if (idle.count() < 5000)
                return;

            bool input_idle = false;
            m_buf.lock();
            const bool input_queues_empty = image_buf.empty() && point_buf.empty() && pose_buf.empty();
            input_idle = input_queues_empty && !keyframe_processing_active.load();
            const bool serial_incomplete_tail = SERIAL_PROCESSING && !input_queues_empty &&
                (image_buf.empty() || point_buf.empty() || pose_buf.empty());
            if (serial_incomplete_tail)
            {
                if (!m_process.try_lock())
                {
                    m_buf.unlock();
                    return;
                }
                const size_t image_queue_size = image_buf.size();
                const size_t point_queue_size = point_buf.size();
                const size_t pose_queue_size = pose_buf.size();
                while (!image_buf.empty())
                    image_buf.pop();
                while (!point_buf.empty())
                    point_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                input_idle = !keyframe_processing_active.load();
                m_process.unlock();
                printf("[POSEGRAPH]: cleared unmatched serial input tail before autosave "
                       "(image: %zu, point: %zu, pose: %zu)\n",
                       image_queue_size, point_queue_size, pose_queue_size);
            }
            m_buf.unlock();
            if (!input_idle || posegraph.isOptimizationRunning())
                return;

            if (!m_process.try_lock())
                return;
            const bool final_optimization_requested = posegraph.requestGlobalOptimization();
            m_process.unlock();

            if (final_optimization_requested)
            {
                printf("[POSEGRAPH]: running final global optimization before autosave\n");
                posegraph.waitForOptimizationIdle();
            }

            m_buf.lock();
            const bool input_still_idle =
                image_buf.empty() && point_buf.empty() && pose_buf.empty() &&
                !keyframe_processing_active.load();
            m_buf.unlock();
            if (!input_still_idle || posegraph.hasPendingOptimization() || posegraph.isOptimizationRunning())
                return;

            if (!m_process.try_lock())
                return;
            save_pose_graph_outputs(true);
            m_process.unlock();
            close_bag_writer();
            pose_graph_auto_saved = true;
            printf("[POSEGRAPH]: auto-saved pose graph and closed bag writer after 5s idle with no pending optimization\n");
        });
    
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

   
    string config_file, vocabulary_file, brief_pattern_file;
    getParamOrExit(nh, "config_file", config_file);
    getParamOrExit(nh, "vocabulary_file", vocabulary_file);
    getParamOrExit(nh, "brief_pattern_file", brief_pattern_file);
    
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    BRIEF_PATTERN_FILE = brief_pattern_file;

    posegraph.loadVocabulary(vocabulary_file);

    //ROW = fsSettings["image_height"];
    //COL = fsSettings["image_width"];
    //int pn = config_file.find_last_of('/');
    //std::string configPath = config_file.substr(0, pn);
    //std::string cam0Calib;
    //fsSettings["cam0_calib"] >> cam0Calib;
    //std::string cam0Path = configPath + "/" + cam0Calib;
    //printf("[POSEGRAPH]: cam calib path: %s\n", cam0Path.c_str());
    //m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    const cv::FileNode output_path_node = fsSettings["output_path"];
    if (output_path_node.empty() || output_path_node.isNone())
        throw invalid_output_path_error();
    if (!output_path_node.isString())
        throw invalid_output_path_error();
    output_path_node >> OUTPUT_PATH;
    OUTPUT_PATH = trim_copy(strip_inline_comment(OUTPUT_PATH));
    if (is_null_path_value(OUTPUT_PATH))
        throw invalid_output_path_error();

    fsSettings["pose_graph_load_path"] >> POSE_GRAPH_LOAD_PATH;
    if (!POSE_GRAPH_LOAD_PATH.empty() && POSE_GRAPH_LOAD_PATH.back() != '/')
        POSE_GRAPH_LOAD_PATH += "/";
    fsSettings["save_image"] >> DEBUG_IMAGE;
    if (!fsSettings["trajectory_bag_topic"].empty())
        fsSettings["trajectory_bag_topic"] >> TRAJECTORY_BAG_TOPIC;
    if (!fsSettings["odometry_bag_topic"].empty())
        fsSettings["odometry_bag_topic"] >> ODOMETRY_BAG_TOPIC;
    if (!fsSettings["image_bag_topic"].empty())
        fsSettings["image_bag_topic"] >> IMAGE_BAG_TOPIC;
    fsSettings["skip_dist"] >> SKIP_DIS;
    fsSettings["skip_cnt"] >> SKIP_CNT;
    if (!fsSettings["skip_first_seconds"].empty())
        fsSettings["skip_first_seconds"] >> SKIP_FIRST_SECONDS;
    if (!fsSettings["serial"].empty())
    {
        if (fsSettings["serial"].isString())
        {
            std::string serial_value;
            fsSettings["serial"] >> serial_value;
            std::transform(serial_value.begin(), serial_value.end(), serial_value.begin(),
                           [](unsigned char c) { return std::tolower(c); });
            SERIAL_PROCESSING = serial_value == "true" || serial_value == "1" ||
                                serial_value == "yes" || serial_value == "on";
        }
        else
        {
            int serial_value = 0;
            fsSettings["serial"] >> serial_value;
            SERIAL_PROCESSING = serial_value != 0;
        }
    }
    fsSettings["min_score"] >> MIN_SCORE;
    fsSettings["pnp_inflation"] >> PNP_INFLATION;
    fsSettings["recall_ignore_recent_ct"] >> RECALL_IGNORE_RECENT_COUNT;
    fsSettings["max_theta_diff"] >> MAX_THETA_DIFF;
    fsSettings["max_pos_diff"] >> MAX_POS_DIFF;
    fsSettings["min_loop_feat_num"] >> MIN_LOOP_NUM;
    if (!fsSettings["brief_match_hamming_thresh"].empty())
        fsSettings["brief_match_hamming_thresh"] >> BRIEF_MATCH_HAMMING_THRESH;
    fsSettings["min_optimization_time_diff"] >> MIN_OPTIMIZATION_TIME_DIFF;

    std::filesystem::path output_path(OUTPUT_PATH);
    if (std::filesystem::exists(output_path) && !std::filesystem::is_directory(output_path))
    {
        throw invalid_output_path_error();
    }
    if (!std::filesystem::exists(output_path))
    {
        std::filesystem::create_directories(output_path);
        printf("[POSEGRAPH]: created output path: %s\n", output_path.string().c_str());
    }
    const std::filesystem::path ov_slam_output_path = output_path / "ov_slam";
    std::filesystem::remove_all(ov_slam_output_path);
    std::filesystem::create_directories(ov_slam_output_path);

    POSE_GRAPH_SAVE_PATH = (ov_slam_output_path / "pose_graph").string();
    std::filesystem::create_directories(POSE_GRAPH_SAVE_PATH);
    if (POSE_GRAPH_SAVE_PATH.back() != '/')
        POSE_GRAPH_SAVE_PATH += "/";
    initialize_bag_writer((ov_slam_output_path / "ov_slam_bag").string());
    printf("[POSEGRAPH]: writing trajectory bag: %s\n", (ov_slam_output_path / "ov_slam_bag").string().c_str());

    int LOAD_PREVIOUS_POSE_GRAPH;
    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];

    int USE_IMU = fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("[POSEGRAPH]: load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("[POSEGRAPH]: load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("[POSEGRAPH]: no previous pose graph\n");
        load_flag = 1;
    }

    /*
    // Get camera information
    printf("[POSEGRAPH]: waiting for camera info topic...\n");
    auto msg1 = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/vins_estimator/intrinsics", ros::Duration(ros::DURATION_MAX));
    intrinsics_callback(msg1);
    printf("[POSEGRAPH]: received camera info message!\n");
    std::cout << m_camera.get()->parametersToString() << std::endl;

    // Get camera to imu information
    printf("[POSEGRAPH]: waiting for camera to imu extrinsics topic...\n");
    auto msg2 = ros::topic::waitForMessage<nav_msgs::Odometry>("/vins_estimator/extrinsic", ros::Duration(ros::DURATION_MAX));
    extrinsic_callback(msg2);
    printf("[POSEGRAPH]: received camera to imu extrinsics message!\n");
    std::cout << qic.transpose() << std::endl;
    std::cout << tic.transpose() << std::endl;
    */

    // Get camera information
    printf("[POSEGRAPH]: waiting for camera info topic...\n");
    sensor_msgs::msg::CameraInfo msg1;
    rclcpp::wait_for_message(msg1, nh, "/ov_msckf/loop_intrinsics");
    app->intrinsics_callback(std::make_shared<sensor_msgs::msg::CameraInfo>(msg1));
    printf("[POSEGRAPH]: received camera info message!\n");
    std::cout << m_camera.get()->parametersToString() << std::endl;

    // Get camera to imu information
    printf("[POSEGRAPH]: waiting for camera to imu extrinsics topic...\n");
    nav_msgs::msg::Odometry msg2;
    rclcpp::wait_for_message(msg2, nh, "/ov_msckf/loop_extrinsic");
    app->extrinsic_callback(std::make_shared<nav_msgs::msg::Odometry>(msg2));
    printf("[POSEGRAPH]: received camera to imu extrinsics message!\n");
    std::cout << qic.transpose() << std::endl;
    std::cout << tic.transpose() << std::endl;

    // Setup the rest of the publishers
    const size_t subscription_queue_size = SERIAL_PROCESSING ?
        SERIAL_SUBSCRIPTION_QUEUE_SIZE : DEFAULT_SUBSCRIPTION_QUEUE_SIZE;
    printf("[POSEGRAPH]: serial processing: %s, input subscription queue: %zu\n",
           SERIAL_PROCESSING ? "true" : "false", subscription_queue_size);
    auto sub_image = nh->create_subscription<sensor_msgs::msg::CompressedImage>(kLoopImageTopic, subscription_queue_size, std::bind(&App::image_callback, app.get(), std::placeholders::_1));
    auto sub_pose = nh->create_subscription<nav_msgs::msg::Odometry>("/ov_msckf/loop_pose", subscription_queue_size, std::bind(&App::pose_callback, app.get(), std::placeholders::_1));
    auto sub_extrinsic = nh->create_subscription<nav_msgs::msg::Odometry>("/ov_msckf/loop_extrinsic", subscription_queue_size, std::bind(&App::extrinsic_callback, app.get(), std::placeholders::_1));
    auto sub_intrinsics = nh->create_subscription<sensor_msgs::msg::CameraInfo>("/ov_msckf/loop_intrinsics", subscription_queue_size, std::bind(&App::intrinsics_callback, app.get(), std::placeholders::_1));
    auto sub_point = nh->create_subscription<sensor_msgs::msg::PointCloud>("/ov_msckf/loop_feats", subscription_queue_size, std::bind(&App::point_callback, app.get(), std::placeholders::_1));

    std::thread measurement_process;
    std::thread keyboard_command_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    
    RCLCPP_INFO_STREAM(nh->get_logger(), "loop_fusion_node ready");
    rclcpp::spin(nh);
    m_process.lock();
    refresh_calibration_for_output();
    write_final_trajectory_outputs();
    m_process.unlock();
    close_bag_writer();

    return 0;
}
