#include "first_challenge/first_challenge.hpp"

FirstChallenge::FirstChallenge() : Node("first_challenge")
{
    hz_ = this->declare_parameter<int>("hz", 10);
    goal_dist_ = this->declare_parameter<double>("goal_dist", 1.0);
    velocity_ = this->declare_parameter<double>("velocity", 0.1);

    // subscriber
    // <subscriber名> = this->create_subscription<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable(), std::bind(&<class名>::<コールバック関数名>, this, std::placeholders::_<何番目の引数か>));
    // std::bindを使ってsubするコールバック関数，std::placeholdersを使ってその関数内での引数を指定する
    // std::placeholdersで指定する引数は大体1番目のもの（コールバック関数の引数が1つであるため）
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1).reliable(), std::bind(&FirstChallenge::odometry_callback, this, std::placeholders::_1));

    // publisher
    // <publisher名> = this->create_publisher<<msg型>>("<topic名>", rclcpp::QoS(<確保するtopicサイズ>).reliable());
    cmd_vel_pub_ = this->create_publisher<roomba_500driver_meiji::msg::RoombaCtrl>("/roomba/control", rclcpp::QoS(1).reliable());
}

// odomのコールバック関数
void FirstChallenge::odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = *msg;
}

// センサ情報（今回はodom）を取得できているかの確認用
// センサ情報取得前にアクセスしようとするとセグメンテーションフォルトが起こり，core dump（プロセス終了）する
bool FirstChallenge::can_move()
{
    // odom_はoptional型で定義しているため，has_value()が使える
    // optional型のhas_value()では，値を取得できた場合はtrue，取得できなかった場合はfalseを返す
    return odom_.has_value();
}

// 終了判定
// roombaが一定以上の距離を進んだら終了
// ゴールするまでtrueを返す
bool FirstChallenge::is_goal()
{
    const double dist = calc_distance();

    if(dist < goal_dist_)
        return true;
    else
        return false;
}

// 進んだ距離を計算
double FirstChallenge::calc_distance()
{
    // optional型で定義したmsgの値を取得したい場合は.value()を変数名の直後に追記
    // optional型のvalue()は有効値への参照を返す
    return hypot(odom_.value().pose.pose.position.x, odom_.value().pose.pose.position.y);
}

// roombaの制御入力を決定
void FirstChallenge::run(float velocity, float omega)
{
    // roombaの制御モード
    // 基本的に11（DRIVE_DIRECT）固定で良い
    cmd_vel_.mode = 11;

    // 並進速度と旋回速度を指定
    cmd_vel_.cntl.linear.x = velocity;  // 並進速度
    cmd_vel_.cntl.angular.z = omega;    // 旋回速度

    // cmd_velをpublish
    cmd_vel_pub_->publish(cmd_vel_);
}

// 並進速度と旋回速度を計算
void FirstChallenge::set_cmd_vel()
{
    if(is_goal())
    {
        // 直進
        run(velocity_, 0.0);
    }
    else
    {
        // 停止
        run(0.0, 0.0);
    }
}