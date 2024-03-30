#include "rclcpp/rclcpp.hpp"
#include "lab1_pkg/relay.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RelayPub>());
  rclcpp::shutdown();
  return 0;
}