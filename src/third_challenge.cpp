#include <cmath>
#include "third_challenge/third_challenge.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

using namespace std::chrono_literals;

thirdChallenge::thirdChallenge() : Node("third_challenge")
{
    frontal_threshold = this->declare_parameter<double>("frontal_threshold", 0.2);
    max_omega = this->declare_parameter<double>("max_omega", 0.6);
    cmd_vel_.mode = mode;

    sub_box_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ros2_yolo/box", rclcpp::QoS(1).reliable(), std::bind(&thirdChallenge::box_callback, this, std::placeholders::_1));
    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/color/camera_info",
        rclcpp::QoS(1).reliable(),
        std::bind(&thirdChallenge::camera_info_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("roomba/control", rclcpp::QoS(1).reliable());
    timer_ = this->create_wall_timer(100ms, std::bind(&thirdChallenge::timer_callback, this));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void thirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{
    double azimuth;
    if (msg.data.size() == 2)
    {
        RCLCPP_INFO(this->get_logger(), "NO TARGET");
        azimuth = 0;
        detected = false;
    }
    else
    {
        int box_u_center = (msg.data[4] + msg.data[2]) / 2;
        double x_normalized = (box_u_center - camera_model_.cx()) / camera_model_.fx();
        azimuth = std::atan2(x_normalized, 1.0);
        RCLCPP_INFO(this->get_logger(), "TARGET SET: %f rad", azimuth);
        detected = true;
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, azimuth);
    geometry_msgs::msg::Quaternion base_quat;
    tf2::convert(q, base_quat);
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }
    tf2::doTransform(base_quat, odom_quat_, transform);
}

void thirdChallenge::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg)
{
    if (!is_model_set)
    {
        camera_model_.fromCameraInfo(msg);
        is_model_set = true;
    }
}

void thirdChallenge::timer_callback()
{
    if (!detected)
    {
        cmd_vel_.cntl.linear.x = 0.0;
        cmd_vel_.cntl.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_);
        RCLCPP_INFO(this->get_logger(), "[NO TARGET] CMD OMEGA: %f rad/s", cmd_vel_.cntl.angular.z);
        return;
    }
    // odom 座標系で表現された目標角度に向かって旋回する指令を生成
    geometry_msgs::msg::Quaternion base_quat;
    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }
    tf2::doTransform(odom_quat_, base_quat, transform);

    double azimuth = tf2::getYaw(base_quat);

    RCLCPP_INFO(this->get_logger(), "DIFF FROM TARGET: %f", azimuth);

    if (std::abs(azimuth) > frontal_threshold)
    {
        cmd_vel_.cntl.linear.x = 0.0;
        double cmd_omega = -max_omega * azimuth;
        if (std::abs(cmd_omega) > 0.7) {
            cmd_omega = std::copysign(0.7, cmd_omega);
        }
        cmd_vel_.cntl.angular.z = cmd_omega;

        RCLCPP_INFO(this->get_logger(), "[TURNING] CMD OMEGA: %f rad/s", cmd_vel_.cntl.angular.z);
        cmd_vel_pub_->publish(cmd_vel_);
    }
    else
    {
        cmd_vel_.cntl.linear.x = 0;
        cmd_vel_.cntl.angular.z = 0;
        cmd_vel_pub_->publish(cmd_vel_);
        RCLCPP_INFO(this->get_logger(), "[CENTERED] CMD OMEGA: %f rad/s", cmd_vel_.cntl.angular.z);
    }
}