#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class TalkerPub: public rclcpp::Node
{

    public:

        TalkerPub(): Node("talker")
        {

            this->declare_parameter<double>("v", 0.0);
            this->declare_parameter<double>("d", 0.0);

            publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

            timer_ = this->create_wall_timer(
                500ms, std::bind(&TalkerPub::publish_drive, this));

        }

    private:

        double v_, d_;

        void publish_drive(){

            // parse params
            v_ = this->get_parameter("v").as_double();
            d_ = this->get_parameter("d").as_double();

            auto a_msg = ackermann_msgs::msg::AckermannDriveStamped();
            
            a_msg.drive.speed = this->v_;
            a_msg.drive.steering_angle = this->d_;

            publisher_->publish(a_msg);

        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

};