#pragma once

#include <filesystem>
#include <mutex>
#include <memory>
#include <string>

#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <rosbag2_cpp/writer.hpp>

namespace ov_secondary
{

struct BagTopics
{
    std::string trajectory = "/ov_slam/trajectory";
    std::string final_trajectory = "/ov_slam/trajectory_final";
    std::string odometry = "/ov_slam/odometry";
    std::string image = "/ov_slam/image/compressed";
};

class OutputBagWriter
{
public:
    void open(const std::string &bag_path);
    void close();
    void writeTrajectory(const nav_msgs::msg::Path &trajectory, const std::string &topic);
    void writeFinalTrajectory(const nav_msgs::msg::Path &trajectory, const std::string &topic);
    void writeOdometry(const nav_msgs::msg::Odometry &odometry, const std::string &topic);
    void writeCompressedImage(const sensor_msgs::msg::CompressedImage &image, const std::string &topic);

private:
    std::mutex mutex_;
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    bool final_trajectory_written_ = false;
};

nav_msgs::msg::Odometry BuildCameraOdometryMessage(const nav_msgs::msg::Odometry &imu_odometry,
                                                   const Eigen::Vector3d &imu_to_camera_translation,
                                                   const Eigen::Matrix3d &imu_to_camera_rotation);

}  // namespace ov_secondary
