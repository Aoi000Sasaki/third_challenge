// TODO: launchでremap

#include "third_challenge/third_challenge.hpp"

using namespace std::chrono_literals;

thirdChallenge::thirdChallenge() : Node("third_challenge")
{
    // ロボットの正面において，この値（rad）までは正面と見なす
    // もう少し狭くても
    frontal_threshold = this->declare_parameter<double>("frontal_threshold", 0.2);
    // 旋回速度の基準値（rad/s）
    // もう少し大きくても
    base_omega = this->declare_parameter<double>("base_omega", 0.6);
    cmd_vel_.mode = mode;

    // ros2_yoloの暫定的な出力形式に合わせている
    // vision_msgs::msgに変更予定 See: https://github.com/ros-perception/vision_msgs
    sub_box_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/ros2_yolo/box",
                                                                           rclcpp::QoS(1).reliable(),
                                                                           std::bind(&thirdChallenge::box_callback, this, std::placeholders::_1));
    // realsenceが出力するカメラパラメータを取得
    sub_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>("/color/camera_info",
                                                                               rclcpp::QoS(1).reliable(),
                                                                               std::bind(&thirdChallenge::camera_info_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("roomba/control", rclcpp::QoS(1).reliable());
    timer_ = this->create_wall_timer(100ms, std::bind(&thirdChallenge::timer_callback, this));
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

// 検出結果のバウンディングボックスから，odom座標系で表される目標角度を計算する
void thirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{
    double azimuth = 0.0;
    if (msg.data.size() == 2) // 長さが2の場合，検出は無し
    {
        RCLCPP_INFO(this->get_logger(), "NO TARGET");
        detected = false;
    }
    else
    {
        int box_u_center = (msg.data[4] + msg.data[2]) / 2; // バウンディングボックスの中心のu座標（左上を原点としたピクセル単位の画像座標系）
        double x_normalized = (box_u_center - camera_model_.cx()) / camera_model_.fx(); // 画像の中心を原点とする座標系に変換（単位はfxによる）
        azimuth = std::atan2(x_normalized, 1.0); // ヒトのいる方位角を計算 ただし，画像の歪みなどは無視している
        RCLCPP_INFO(this->get_logger(), "TARGET SET: %f rad", azimuth);
        detected = true;
    }
    tf2::Quaternion q;
    q.setRPY(0, 0, azimuth); //方位角のクォータニオン
    geometry_msgs::msg::Quaternion base_quat;
    tf2::convert(q, base_quat); //クォータニオンをgeometry_msgs::msg::Quaternionに変換
    geometry_msgs::msg::TransformStamped transform;
    // base_link座標系からodom座標系への変換を取得
    try {
        transform = tf_buffer_->lookupTransform("base_link", "odom", tf2::TimePointZero);
    } catch (tf2::TransformException& ex) {
        RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        return;
    }
    // base_link座標系から見た方位角をodom座標系に変換
    // ルンバの旋回の影響を受けない座標系で方位角を表現し、保持するため
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

// 一定の周期で呼び出され，制御指令を生成する関数
// ルンバを目標方位角の方に旋回させる
void thirdChallenge::timer_callback()
{
    if (!detected) // 検出されていない場合は旋回しない
    {
        cmd_vel_.cntl.linear.x = 0.0;
        cmd_vel_.cntl.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel_);
        RCLCPP_INFO(this->get_logger(), "[NO TARGET] CMD OMEGA: %f rad/s", cmd_vel_.cntl.angular.z);
        return;
    }
    // odom座標系から見た方位角をbase_link座標系に変換
    // どちらにどのくらい旋回すると目標方位角（ヒトのいる方位）になるかを最終的に計算している
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

    // 目標方位角が閾値以内か判定
    //  閾値以内 → 旋回停止
    //  閾値外 → 旋回
    if (std::abs(azimuth) > frontal_threshold)
    {
        cmd_vel_.cntl.linear.x = 0.0;
        double cmd_omega = -base_omega * azimuth; //差が大きいほど，旋回速度が大きくなるイメージ
        if (std::abs(cmd_omega) > 0.7) { // 旋回速度の上限を設ける
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