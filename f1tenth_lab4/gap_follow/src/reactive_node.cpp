#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
/// CHECK: include needed ROS msg type headers and libraries

using std::placeholders::_1;


class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 10);        
        
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(        
            lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, _1));
        
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    double toRad(double d){ return d * (M_PI / 180); };
    double toDeg(double r){ return r * (180 / M_PI); };
    int deg2idx(double d, double min, double inc){ 
        double idx = (toRad(d)-min) / inc;
        std::cout << idx << std::endl;
        return static_cast<int>(idx); 
    };

    const int window = 6;
    const double highest_val = 3.6;//3.5;
    const double max_velo = 2.1;//2.0;
    const double car_width = 1.4;//1.5;
    const double disparity_thresh = 0.35;//0.4;
    const double unsafe_dist = 0.0;//0.1;
    const int bubble = 8;
    double v_prev = 1.0;

    int indices[2] = {0, 0};
    int best = 0;

    void preprocess_lidar(float* ranges, int& closest, const int size, const int* behind_idx, const float& inc, float& farthest)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)

        for(int i = 0; i < size - 1; i++ ){
            
            // cornering
            if( (i < behind_idx[0] || i > behind_idx[1]) && ranges[i] < unsafe_dist ){
                closest = -1;
                return;
            }

            double sum = 0.0;
            double items = 0.0f;
            if( std::isinf(ranges[i]) || std::isnan(ranges[i]) ){
                ranges[i] = highest_val;
            }
            else{
                for(int j = i - window; j < i + window; j++){

                    if( j > size-1 || j < 0 ){ continue; }

                    if( std::isinf(ranges[j]) || std::isnan(ranges[j]) || ranges[j] > highest_val )
                    {   sum += highest_val; }
                    else{
                        sum += ranges[j];
                    }

                    items++;
                }
            }

            ranges[i] = sum / items;
            farthest = std::max(ranges[i], farthest);
            if( ranges[closest] > ranges[i] ){ closest = i; }

        }

        float* copy = new float[size];
        std::copy(ranges, ranges + size, copy);

        // disparity extender
        for(int i = 1; i < size - 1; i++){

            float disparity = ranges[i] - ranges[i-1];

            if( std::abs(disparity)  > disparity_thresh){

                int extend_idx = car_width / std::min(ranges[i-1], ranges[i]) / inc;
                int dir = disparity > 0 ? 1 : -1;
                
                for(int j = i; j < i + extend_idx; j++){

                    if( j*dir >= size-1 || j*dir < 0) continue; 

                    copy[j * dir] = std::min(ranges[i-1], ranges[i]);
                    farthest = std::max(copy[j * dir], farthest);
                    if( ranges[closest] > ranges[j * dir] ){ closest = j * dir; }

                }

                if( dir == 1 ){
                    i = std::min(size, i + extend_idx);
                }

            }

        }

        std::copy(copy, copy + size, ranges);

        delete[] copy;
        copy = nullptr;
        
        return;
    }

    void find_max_gap(float* ranges, int* indice, const int size, const float farthest)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int p1 = indice[0];
        int p2 = indice[1];
        int max_gap = 0;

        while( p2 < size - 1){

            p2++;

            if( ranges[p2] >= farthest*0.3 ){
                
                p1 = p2; // put p1 at p2

                while( ranges[p2] >= farthest*0.3 && p2 != size-1){
                    p2++;
                }

                if( p2 - p1 > max_gap ){
                    max_gap = p2 - p1;
                    indice[0] = p1;
                    indice[1] = p2 - 1;
                    p1 = p2;
                }
                
            }

        }

        return;
    }

    void find_best_point(float* ranges, int* indice, int& best)
    {   
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there


        for( int i = indice[0]; i <= indice[1]; i++ ){
            if( ranges[i] > ranges[best] ){ best = i; }
        }

        return;
    }

    float map_dist(const float& d ){

        double m, c; // Slope and y-intercept for the linear equations
        const static double mid_velo = max_velo / 2;
        const static double low_velo = max_velo / 3;
        const static double min_velo = max_velo / 4;

        if (d == highest_val) {
            return max_velo;
        }
        else if (d > highest_val / 2 && d < highest_val) {
            // Define the linear equation for this range
            // Assuming the velocity at highest_val/2 is some known value, say mid_velo
            m = (max_velo - mid_velo) / (highest_val - highest_val / 2);
            c = max_velo - m * highest_val;
            return m * d + c;
        }
        else if (d > highest_val / 5 && d <= highest_val / 2) {
            // Define the linear equation for this range
            // Assuming the velocity at highest_val/5 is some known value, say low_velo
            m = (mid_velo - low_velo) / (highest_val / 2 - highest_val / 5);
            c = mid_velo - m * (highest_val / 2);
            return m * d + c;
        }
        else {
            // Define the linear equation for d <= highest_val/5
            // Assuming the velocity at 0 is some known value, say min_velo
            m = (low_velo - min_velo) / (highest_val / 5 - 0);
            c = min_velo; // since c = y - mx and x is 0
            return m * d + c;
        }
        
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        const static float min = scan_msg->angle_min;
        const static float inc = scan_msg->angle_increment;
        const static int size = scan_msg->ranges.size();
        const static int behind_idx[2] = { deg2idx(-90.0, min, inc), deg2idx(90.0, min, inc) };

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();

        float* ranges = new float[size];
        std::copy(scan_msg->ranges.data(), scan_msg->ranges.data() + size, ranges);

        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        int closest_pt = 0;
        float farthest = 0;
        preprocess_lidar(ranges, closest_pt, size, behind_idx, inc, farthest);

        if(closest_pt == -1 ){ // obstactle behind you

            drive_msg.drive.speed = v_prev; 
            drive_msg.drive.steering_angle = 0;
            drive_pub_->publish(drive_msg);

            delete [] ranges;
            ranges = nullptr;

            return;
        }

        // Eliminate all points inside 'bubble' (set them to zero)
        for(int i = closest_pt - bubble; i < closest_pt + bubble; i++){
            if( i < 0  || i > size - 1) continue;
            ranges[i] = 0.f;
        }

        // Find max length gap
        indices[0] = 0;
        indices[1] = 1;
        find_max_gap(ranges, indices, size, farthest);

        // Find the best point in the gap 
        best = static_cast<int>( ((indices[1] + indices[0]) / 2) );

        float steering_angle = min + (inc * best);

        drive_msg.drive.speed = map_dist(ranges[best]);
        v_prev = map_dist(ranges[best]);
        drive_msg.drive.steering_angle = steering_angle;

        drive_pub_->publish(drive_msg);

        delete [] ranges;
        ranges = nullptr;

        return;
        
    }




};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}