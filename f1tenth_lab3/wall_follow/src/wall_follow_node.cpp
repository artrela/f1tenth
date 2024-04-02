#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

#include <math.h>
#include <queue>
#include "limits.h"

using std::placeholders::_1;

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        //: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, _1));

        this->declare_parameter("kp", 1.1);
        this->declare_parameter("kd", 0.8);
        this->declare_parameter("ki", 0.001);
        this->declare_parameter("L", 1.0);
        this->declare_parameter("theta", 50.0);
        this->declare_parameter("des", 1.0);
        this->declare_parameter("clip", 0.001);

        Kp = this->get_parameter("kp").as_double();
        Kd = this->get_parameter("kd").as_double();
        Ki = this->get_parameter("ki").as_double();
        L = this->get_parameter("L").as_double();
        theta = this->get_parameter("theta").as_double();
        D_des = this->get_parameter("des").as_double();
        clip_val = this->get_parameter("clip").as_double();

        RCLCPP_INFO(this->get_logger(), "Kp: %s, Kd: %s, Ki: %s, L: %s, Theta: %s",
                std::to_string(Kp).c_str(),
                std::to_string(Kd).c_str(),
                std::to_string(Ki).c_str(),
                std::to_string(L).c_str(),
                std::to_string(theta).c_str());

        // t_prev = this->get_clock()->now();

    }

private:
    // PID CONTROL PARAMS
    double Kp;
    double Kd;
    double Ki;
    double theta;
    double L; // [m]
    double D_des; // [m]
    double clip_val;

    // double servo_offset = 0.0;
    double e_prev = 0.0;
    double e = 0.0;
    double integral_e = 0.0;

    std::pair<std::queue<double>, double> window; 

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    double toRad(double d){ return d * (M_PI / 180); };

    double get_range(const float* range_data, double angle, double min, double inc)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // angle, as measured from the max (50 deg from max)
        int idx = static_cast<int>(std::floor( ((angle - min) / inc )));

        while( std::isnan(range_data[idx]) || std::isinf(range_data[idx] ) ){ 
            return std::numeric_limits<double>::quiet_NaN(); 
        }

        return range_data[idx];
    }
    
    double smooth_error(double e){
        
        if(window.first.size() == 30){
            window.second -= window.first.front();
            window.first.pop();
        }

        window.first.emplace(e);
        window.second += e;

        return window.second / (double)window.first.size();

    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */

        double min = scan_msg->angle_min;        
        double inc = scan_msg->angle_increment;

        // get distances a, b
        double a = get_range(scan_msg->ranges.data(), toRad(90.f - theta), min, inc); 
        double b = get_range(scan_msg->ranges.data(), toRad(90.f), min, inc);

        if( isnan(a) || isnan(b) ){ return; }


        // project ahead
        double alpha = std::atan2( (a*std::cos(toRad(theta)) - b), (a*std::sin(toRad(theta))));      
        double Dt = b*std::cos(alpha);  
        double Dt_next = Dt + L*std::sin(alpha);

        // calc error & clip to smooth 
        e = -(D_des - Dt_next);

        // update error sources
        // rclcpp::Time t = this->get_clock()->now();
        double dedt = (e - e_prev); 
        // t_prev = t;

        e_prev = e;
        integral_e += e;

        // calc control
        double u = Kp*e + Kd*dedt + Ki*integral_e;

        // create message & pub
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        drive_msg.drive.steering_angle = u;
        if( std::abs(u) >= toRad(0.0) && std::abs(u) <= toRad(10.0)){ 
            drive_msg.drive.speed = 1.5; }
        else if( std::abs(u) > toRad(10.0) && std::abs(u) <= toRad(20.0)){ 
            drive_msg.drive.speed = 1.0; }
        else{ 
            drive_msg.drive.speed = 0.5; }


        drive_pub_->publish(drive_msg);

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}