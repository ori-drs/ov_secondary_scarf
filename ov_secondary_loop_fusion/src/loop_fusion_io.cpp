#include "loop_fusion_io.h"

#include <stdexcept>

#include <rclcpp/time.hpp>

#include "parameters.h"

namespace ov_secondary
{

void OutputBagWriter::open(const std::string &bag_path)
{
    if (bag_path.empty())
        throw std::runtime_error("bag_path must not be empty");

    const std::filesystem::path path(bag_path);
    if (std::filesystem::exists(path))
        throw std::runtime_error("bag_path already exists: " + bag_path);

    const auto parent = path.parent_path();
    if (!parent.empty())
        std::filesystem::create_directories(parent);

    std::lock_guard<std::mutex> lock(mutex_);
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    writer_->open(bag_path);
    final_trajectory_written_ = false;
}

void OutputBagWriter::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_)
        return;

    writer_->close();
    writer_.reset();
}

void OutputBagWriter::writeTrajectory(const nav_msgs::msg::Path &trajectory, const std::string &topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_)
        return;

    nav_msgs::msg::Path message = trajectory;
    writer_->write(message, topic, rclcpp::Time(message.header.stamp));
}

void OutputBagWriter::writeFinalTrajectory(const nav_msgs::msg::Path &trajectory, const std::string &topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_ || final_trajectory_written_)
        return;

    nav_msgs::msg::Path message = trajectory;
    writer_->write(message, topic, rclcpp::Time(message.header.stamp));
    final_trajectory_written_ = true;
}

void OutputBagWriter::writeOdometry(const nav_msgs::msg::Odometry &odometry, const std::string &topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_)
        return;

    nav_msgs::msg::Odometry message = odometry;
    writer_->write(message, topic, rclcpp::Time(message.header.stamp));
}

void OutputBagWriter::writeCompressedImage(const sensor_msgs::msg::CompressedImage &image, const std::string &topic)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!writer_)
        return;

    sensor_msgs::msg::CompressedImage message = image;
    writer_->write(message, topic, rclcpp::Time(message.header.stamp));
}

nav_msgs::msg::Odometry BuildCameraOdometryMessage(const nav_msgs::msg::Odometry &imu_odometry,
                                                   const Eigen::Vector3d &imu_to_camera_translation,
                                                   const Eigen::Matrix3d &imu_to_camera_rotation)
{
    nav_msgs::msg::Odometry camera_odometry = imu_odometry;
    const Eigen::Vector3d world_to_imu_translation(
        imu_odometry.pose.pose.position.x,
        imu_odometry.pose.pose.position.y,
        imu_odometry.pose.pose.position.z);
    const Eigen::Quaterniond world_to_imu_rotation(
        imu_odometry.pose.pose.orientation.w,
        imu_odometry.pose.pose.orientation.x,
        imu_odometry.pose.pose.orientation.y,
        imu_odometry.pose.pose.orientation.z);

    const Eigen::Vector3d world_to_camera_translation =
        world_to_imu_translation + world_to_imu_rotation * imu_to_camera_translation;
    const Eigen::Quaterniond world_to_camera_rotation(
        world_to_imu_rotation.toRotationMatrix() * imu_to_camera_rotation);

    camera_odometry.pose.pose.position.x = world_to_camera_translation.x();
    camera_odometry.pose.pose.position.y = world_to_camera_translation.y();
    camera_odometry.pose.pose.position.z = world_to_camera_translation.z();
    camera_odometry.pose.pose.orientation.x = world_to_camera_rotation.x();
    camera_odometry.pose.pose.orientation.y = world_to_camera_rotation.y();
    camera_odometry.pose.pose.orientation.z = world_to_camera_rotation.z();
    camera_odometry.pose.pose.orientation.w = world_to_camera_rotation.w();
    return camera_odometry;
}

namespace
{

OutputBagWriter output_bag_writer;
constexpr const char *kFinalTrajectoryTopic = "/ov_slam/trajectory_final";

}  // namespace

}  // namespace ov_secondary

void initialize_bag_writer(const std::string &bag_path)
{
    ov_secondary::output_bag_writer.open(bag_path);
}

void close_bag_writer()
{
    ov_secondary::output_bag_writer.close();
}

void write_trajectory_to_bag(const nav_msgs::msg::Path &trajectory)
{
    ov_secondary::output_bag_writer.writeTrajectory(trajectory, TRAJECTORY_BAG_TOPIC);
}

void write_final_trajectory_to_bag(const nav_msgs::msg::Path &trajectory)
{
    ov_secondary::output_bag_writer.writeFinalTrajectory(trajectory, ov_secondary::kFinalTrajectoryTopic);
}

void write_odometry_to_bag(const nav_msgs::msg::Odometry &odometry)
{
    ov_secondary::output_bag_writer.writeOdometry(odometry, ODOMETRY_BAG_TOPIC);
}

void write_compressed_image_to_bag(const sensor_msgs::msg::CompressedImage &image)
{
    ov_secondary::output_bag_writer.writeCompressedImage(image, IMAGE_BAG_TOPIC);
}
