#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include "rclcpp/logger.hpp"
#include "rcutils/logging_macros.h"

#include <iostream>
#include <cmath>

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking

public:
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&Safety::scan_callback, this, _1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));

    }

private:

    double speed = 0.0;
    float min_TCC = 2.25f;
    double ri_dot;
    bool collision_detected = false;

    ackermann_msgs::msg::AckermannDriveStamped drive_command;

    /// TODO: create ROS subscribers and publishers

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        this->speed = msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // /// TODO: calculate TTC
        // if( this->collision_detected ){
        //     this->drive_command.drive.speed = 0.0;
        //     this->drive_command.drive.steering_angle = 0.0;
        //     drive_pub_->publish(drive_command);
        //     return;
        // }

        for(size_t i = 0; i < scan_msg->ranges.size(); i++){

            ri_dot = this->speed * std::cos(scan_msg->angle_min + (i*scan_msg->angle_increment));

            float TCCi = scan_msg->ranges[i] / std::max(-ri_dot, 0.0);

            if( std::isinf( TCCi ) || std::isnan(scan_msg->ranges[i]) ){  
                continue;
            }

            if( scan_msg->ranges[i] < scan_msg->range_min ||  scan_msg->ranges[i] > scan_msg->range_max){
                continue;
            }

            if( TCCi < min_TCC ){
                collision_detected = true;
                this->drive_command.drive.speed = 0.0;
                this->drive_command.drive.steering_angle = 0.0;

                drive_pub_->publish(drive_command);
                RCLCPP_INFO(this->get_logger(), "Emergency Break Engaged: " + std::to_string(TCCi));
                break;
            }
        }

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}