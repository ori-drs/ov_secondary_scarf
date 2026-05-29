#include <rclcpp/node.hpp>
#include <filesystem> //C++17

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

using namespace std;

struct CommandLineConfig
{
    string temp;

};

class App {
public:
    App(rclcpp::Node::SharedPtr node, const CommandLineConfig &app_params);

    ~App(){
    }
    
    
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr image_msg);
    void pose_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void extrinsic_callback(const nav_msgs::msg::Odometry::SharedPtr pose_msg);
    void intrinsics_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void point_callback(const sensor_msgs::msg::PointCloud::SharedPtr point_msg);


private:
    
};
