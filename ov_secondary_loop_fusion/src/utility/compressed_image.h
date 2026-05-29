#pragma once

#include <stdexcept>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

inline constexpr const char *kLoopImageTopic = "/ov_msckf/cam0/image_raw/compressed";

inline cv::Mat DecodeCompressedMonoImage(const sensor_msgs::msg::CompressedImage &image_msg)
{
    if (image_msg.data.empty())
        throw std::runtime_error("Compressed image message has empty data at stamp " +
                                 std::to_string(rclcpp::Time(image_msg.header.stamp).seconds()));

    const cv::Mat compressed(1,
                             static_cast<int>(image_msg.data.size()),
                             CV_8UC1,
                             const_cast<unsigned char *>(image_msg.data.data()));
    cv::Mat image = cv::imdecode(compressed, cv::IMREAD_GRAYSCALE);
    if (image.empty())
        throw std::runtime_error("Failed to decode compressed image at stamp " +
                                 std::to_string(rclcpp::Time(image_msg.header.stamp).seconds()));

    return image;
}
