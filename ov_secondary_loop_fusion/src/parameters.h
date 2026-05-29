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

#pragma once

#include "camodocal/camera_models/CameraFactory.h"
#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>

extern camodocal::CameraPtr m_camera;
extern double max_focallength;
extern double MIN_SCORE;
extern double PNP_INFLATION;
extern int RECALL_IGNORE_RECENT_COUNT;
extern double MAX_THETA_DIFF;
extern double MAX_POS_DIFF;
extern int MIN_LOOP_NUM;
extern int BRIEF_MATCH_HAMMING_THRESH;
extern double MIN_OPTIMIZATION_TIME_DIFF;
extern Eigen::Vector3d tic;
extern Eigen::Matrix3d qic;
extern int VISUALIZATION_SHIFT_X;
extern int VISUALIZATION_SHIFT_Y;
extern std::string BRIEF_PATTERN_FILE;
extern std::string POSE_GRAPH_SAVE_PATH;
extern std::string POSE_GRAPH_LOAD_PATH;
extern int ROW;
extern int COL;
extern std::string OUTPUT_PATH;
extern std::string TRAJECTORY_BAG_TOPIC;
extern std::string ODOMETRY_BAG_TOPIC;
extern std::string IMAGE_BAG_TOPIC;
extern int DEBUG_IMAGE;

void initialize_bag_writer(const std::string &bag_path);
void close_bag_writer();
void write_trajectory_to_bag(const nav_msgs::msg::Path &trajectory);
void write_final_trajectory_to_bag(const nav_msgs::msg::Path &trajectory);
void write_odometry_to_bag(const nav_msgs::msg::Odometry &odometry);
void write_compressed_image_to_bag(const sensor_msgs::msg::CompressedImage &image);
