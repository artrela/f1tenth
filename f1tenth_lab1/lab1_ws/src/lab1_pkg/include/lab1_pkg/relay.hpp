#include "rclcpp/rclcpp.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;

class RelayPub: public rclcpp::Node
{

    public:

        RelayPub(): Node("relay")
        {

            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);

            subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
                "drive", 10, std::bind(&RelayPub::relay_drive, this, _1));

        }

    private:


        void relay_drive( const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr in_msg){

            auto out_msg = ackermann_msgs::msg::AckermannDriveStamped();

            out_msg.drive.speed = in_msg->drive.speed * 3.0;
            out_msg.drive.steering_angle = in_msg->drive.steering_angle * 3.0;

            publisher_->publish(out_msg);

        }

        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
        rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;

};