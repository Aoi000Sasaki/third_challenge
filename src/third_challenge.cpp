#include <cmath>
#include "third_challenge/third_challenge.hpp"

thirdChallenge::thirdChallenge() : Node("third_challenge")
{
    frontal_threshold = this->declare_parameter<double>("frontal_threshold", 0.15);
    max_omega = this->declare_parameter<double>("max_omega", 0.2);
    cmd_vel_.mode = mode;

    sub_box_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/ros2_yolo/boxes",
        rclcpp::QoS(1).reliable(),
        std::bind(&thirdChallenge::box_callback, this, std::placeholders::_1));
    // sub_box_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ros2_yolo/box", rclcpp::QoS(1).reliable(), std::bind(&thirdChallenge::box_callback, this, std::placeholders::_1));
    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/color/camera_info",
        rclcpp::QoS(1).reliable(),
        std::bind(&thirdChallenge::camera_info_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("roomba/control", rclcpp::QoS(1).reliable());
}

void thirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{
    int box_u_center = (msg.data[4] + msg.data[2]) / 2;
    double x_normalized = (box_u_center - camera_model_.cx()) / camera_model_.fx();
    double azimuth = std::atan2(x_normalized, 1.0);
    double azimuth_degrees = azimuth * 180.0 / M_PI;
    RCLCPP_INFO(this->get_logger(), "radius: %f degrees: %f", azimuth, azimuth_degrees);

    if (std::abs(azimuth) > frontal_threshold)
    {
        cmd_vel_.cntl.linear.x = 0.0;
        cmd_vel_.cntl.angular.z = -max_omega * azimuth / std::abs(azimuth);

        RCLCPP_INFO(this->get_logger(), "omega: %f\n", cmd_vel_.cntl.angular.z);
        cmd_vel_pub_->publish(cmd_vel_);
    }
}

void thirdChallenge::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg)
{
    if (!is_model_set)
    {
        camera_model_.fromCameraInfo(msg);
        is_model_set = true;
    }
}