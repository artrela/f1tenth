#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/// CHECK: include needed ROS msg type headers and libraries

using namespace std;
using std::placeholders::_1;

string wp_file = "src/f1tenth_lab5/waypoints/waypoints.csv";

struct WayPoint{

    float x;
    float y;

    // Function to calculate the distance to another Waypoint
    float distanceTo(WayPoint& other) const {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }
};

struct Vec2{

    float x;
    float y;

};

class PurePursuit : public rclcpp::Node
{
    // Implement PurePursuit
    // This is just a template, you are free to implement your own node!

private:

    // hyperparams
    float L = 1.0; 
    float P = 0.4; // p gain
    float clip_val = M_PI / 2;
    float speed = 1.0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wps_pub_;

    int idx = 0;
    vector<WayPoint> wps;

    visualization_msgs::msg::MarkerArray wps_viz = visualization_msgs::msg::MarkerArray();
    visualization_msgs::msg::Marker target_viz = visualization_msgs::msg::Marker();

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // TODO: create ROS subscribers and publishers

        // odom sub
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&PurePursuit::pose_callback, this, _1));

        // drive pub
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        wps_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);
        target_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/target", 10);

        // parse file & visualize
        parse_wps(wp_file, wps);

        target_viz.header.frame_id = "map";
        target_viz.type = visualization_msgs::msg::Marker::CYLINDER;
        target_viz.scale.x = 0.2; 
        target_viz.scale.y = 0.2; 
        target_viz.scale.z = 0.2;
        target_viz.color.a = 1.0;
        target_viz.color.r = 1.0;
        target_viz.color.g = 0.0;
        target_viz.color.b = 0.0;

    }

    void parse_wps(string fname, vector<WayPoint>& wps){

        ifstream file(fname);
        // Read data
        string line;
        int id = 0;

        int counter = 0;

        while (getline(file, line)) {

            ++counter;
            if (counter % 50 != 0) continue;

            stringstream ss(line);
            string cell;

            WayPoint wp;

            // Read x coordinate
            getline(ss, cell, ',');
            wp.x = stod(cell); 

            // Read y coordinate
            getline(ss, cell, ',');
            wp.y = stod(cell); 

            // Add the waypoint to the vector
            wps.push_back(wp);

            // make waypoint marker
            visualization_msgs::msg::Marker wp_viz;
            wp_viz.header.frame_id = "map";
            wp_viz.type = visualization_msgs::msg::Marker::CYLINDER;
            wp_viz.id = id++;
            wp_viz.pose.position.x = wp.x;
            wp_viz.pose.position.y = wp.y;
            wp_viz.scale.x = 0.1; 
            wp_viz.scale.y = 0.1; 
            wp_viz.scale.z = 0.1;
            wp_viz.color.a = 1.0;
            wp_viz.color.r = 0.0;
            wp_viz.color.g = 1.0;
            wp_viz.color.b = 0.0;

            // save marker values
            wps_viz.markers.push_back(wp_viz);

        }

        file.close();

    }



    // void pose_callback(const geometry_msgs::msg::PoseStamped::ConstPtr &pose_msg)
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg)
    {
        
        wps_pub_->publish(wps_viz);

        // TODO: find the current waypoint to track using methods mentioned in lecture

        // assume you start within the lookahead dist && u are always moving forward
        WayPoint current = {(float)pose_msg->pose.pose.position.x, (float)pose_msg->pose.pose.position.y};
        
        while( current.distanceTo(wps[idx]) < L ){
            idx++;
        }
        idx--;

        // TODO: transform goal point to vehicle frame of reference
        WayPoint target = {wps[idx].x, wps[idx].y};

        target_viz.pose.position.x = target.x;
        target_viz.pose.position.y = target.y;
        target_pub_->publish(target_viz);

        // create goal vector

        tf2::Quaternion quat(
            pose_msg->pose.pose.orientation.x,
            pose_msg->pose.pose.orientation.y,
            pose_msg->pose.pose.orientation.z,
            pose_msg->pose.pose.orientation.w
        );
        // Convert geometry_msgs::Quaternion to tf2::Quaternion

        // Convert quaternion to roll, pitch, yaw
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);

        // yaw is the theta we're interested in
        double theta = yaw;

        Vec2 goal = {target.x - current.x, target.y - current.y};
        float y = -goal.x*sin(theta) + goal.y*cos(theta);

        // TODO: calculate curvature/steering angle
        float gamma = 2.f * y / pow(L, 2.f);

        // TODO: publish drive message, don't forget to limit the steering angle.
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.speed = speed;
        drive_msg.drive.steering_angle = max(-clip_val, min(gamma*P, clip_val));
        drive_pub_->publish(drive_msg);
        
    }

    ~PurePursuit() {}
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuit>());
    rclcpp::shutdown();
    return 0;
}