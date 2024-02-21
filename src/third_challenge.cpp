#include "third_challenge/third_challenge.hpp"

using namespace std::chrono_literals;

/*
    ・ros2_yoloから検出結果を受け取る
        ・topic: /ros2_yolo/box (Float32MultiArray)
            idx0: stamp.sec
            idx1: stamp.nanosec
            idx2~5: xmin, ymin, xmax, ymax (pixel 単位)
            idx6: reliability
            idx7: class
    ・検出結果を世界座標に変換する際は/color/camera_infoの情報を利用
        ・See: https://mem-archive.com/2018/10/13/post-682/
    ・検出結果をもとに人の方にルンバを旋回させる
        ・人の方位をルンバの旋回に依存しない座標系に変換し、保持
        ・タイマーを使って目標方位に旋回させる
*/
thirdChallenge::thirdChallenge() : Node("third_challenge")
{

}

// 検出結果のバウンディングボックスから，odom座標系で表される目標角度を計算する
void thirdChallenge::box_callback(const std_msgs::msg::Float32MultiArray& msg)
{

}

// カメラパラメータを取得し，カメラモデルを生成する
void thirdChallenge::camera_info_callback(const sensor_msgs::msg::CameraInfo& msg)
{

}

// 一定の周期で呼び出され，制御指令を生成する関数
// ルンバを目標方位角の方に旋回させる
void thirdChallenge::timer_callback()
{

}