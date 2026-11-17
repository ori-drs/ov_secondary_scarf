#pragma once

#include <stdexcept>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

inline constexpr const char *kLoopImageTopic = "/ov_msckf/cam0/image_raw";
inline constexpr int kBagImageJpegQuality = 95;

inline cv::Mat ToMonoImage(const sensor_msgs::msg::Image &image_msg)
{
    return cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8)->image;
}

inline sensor_msgs::msg::CompressedImage EncodeJpegImage(const sensor_msgs::msg::Image &image_msg)
{
    const bool is_color = sensor_msgs::image_encodings::numChannels(image_msg.encoding) > 1;
    const std::string encoding = is_color ? sensor_msgs::image_encodings::BGR8
                                          : sensor_msgs::image_encodings::MONO8;
    const cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image_msg, nullptr, encoding);

    sensor_msgs::msg::CompressedImage compressed;
    compressed.header = image_msg.header;
    compressed.format = encoding + "; jpeg compressed " + encoding;
    const std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, kBagImageJpegQuality};
    if (!cv::imencode(".jpg", cv_image->image, compressed.data, params))
        throw std::runtime_error("Failed to JPEG-encode image at stamp " +
                                 std::to_string(rclcpp::Time(image_msg.header.stamp).seconds()));
    return compressed;
}
