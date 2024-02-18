#include "third_challenge/third_challenge.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<thirdChallenge>());
  rclcpp::shutdown();
  return 0;
}