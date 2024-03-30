#include "rclcpp/rclcpp.hpp"
#include "lab1_pkg/talker.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TalkerPub>());
  rclcpp::shutdown();
  return 0;
}