// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
// RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), x_dist(0.0, x_dist), y_dist(-map_wid, map_wid ) {
RRT::RRT()
: rclcpp::Node("rrt_node"), 
  gen(std::random_device{}()), // Correctly initialized with a seed
  x_dist(0.0, map_y * map_resolution),  // Replace max_x_value with the actual maximum bound for x_dist
  y_dist( -map_wid_m, map_wid_m )
   {
    // ROS publishers
    // TODO: create publishers for the the drive topic, and other topics you might need

    // ROS subscribers
    // TODO: create subscribers as you need
    string pose_topic = "ego_racecar/odom";
    string scan_topic = "/scan";

    pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic, 1, std::bind(&RRT::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occupancy_grid", 10);
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/chosen_path", 10);
    goal_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/goal_point", 10);

    // TODO: create a occupancy grid
    occ_grid = nav_msgs::msg::OccupancyGrid();
    occ_grid.header.frame_id = "ego_racecar/base_link";

    occ_grid.info.height = static_cast<int>(map_x); 
    occ_grid.info.width = static_cast<int>(map_y);
    occ_grid.info.resolution = map_resolution;

    geometry_msgs::msg::Pose origin = geometry_msgs::msg::Pose();
    // origin.position.x = map_resolution;
    origin.position.y = ( - map_x / 2 ) * map_resolution;
    origin.position.z = 0.1;
    this->occ_grid.info.origin = origin;

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    occ_grid.data.assign(map_x * map_y, 0);

    for( size_t i = 0; i < scan_msg->ranges.size(); i++ ){

        float theta = i * scan_msg->angle_increment + scan_msg->angle_min;

        double x = std::floor( scan_msg->ranges[i] * std::cos(theta) / map_resolution ) ;
        double y = std::floor( scan_msg->ranges[i] * std::sin(theta) / map_resolution ) ;

        int idx_x = (int) x;
        int idx_y = (int) y + (int)( map_wid_px );

        occ_grid.data[0] = (char)50;

        for(int j = -dilation; j <= dilation; j++ ) {
            for(int k = -dilation; k <= dilation; k++) {
                
                int x_idx = idx_x + j;
                int y_idx = idx_y + k;

                // Check boundaries
                if(x_idx < 0 || x_idx >= map_y || y_idx < 0 || y_idx >= map_x) continue;

                int idx = y_idx + x_idx * map_y;
                // Ensure idx is within the grid
                // if(idx >= 0 && idx < total_boxes) {
                occ_grid.data[idx] = 100;
            }
        }
    }
    
    // TODO: update your occupancy grid
    occ_grid.header.stamp = this->now();
    grid_pub_->publish(occ_grid);

}

void RRT::pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:
    //

    // tree as std::vector
    std::vector<RRT_Node> tree;

    RRT_Node root_node;
    // new node starts at the center x pos in px centers
    root_node.x = 0;
    root_node.y = 0;
    root_node.is_root = true;
    tree.push_back(root_node);

    /*
    Goal is in meters:
    heading of car is x
    side-to-side is y
    */
    publish_goal( map_x * map_resolution , 0);

    // TODO: fill in the RRT main loop
    for( int i = 0; i < num_nodes; i++){
        
        // sample a pixel center within the occupancy grid
        std::vector<double> sample_ = sample();
        int nearest_idx = nearest(tree, sample_);

        RRT_Node new_node = steer(tree[nearest_idx], sample_);
        new_node.parent = nearest_idx;
        // std::cout << ">> new_node.x: " << new_node.x << "new node y: " << new_node.y << std::endl;

        // publish_goal(sample_[0], sample_[1]);

        if( !check_collision(tree[nearest_idx], new_node) ){
            tree.push_back(new_node);
        // std::cout << tree.size() << std::endl;
        }
        else{ continue; }

        if( is_goal(new_node, map_x * map_resolution, 0) ){
            std::cout << "FOUND GOAL!!!!!!!" << std::endl;
            std::vector<RRT_Node> path = find_path(tree, new_node);
            // std::cout << "constructed path :)" << std::endl;
            publish_path(path);
            break;
        }
    }
    // path found as Path message

}

void RRT::publish_goal(const double& goal_x, const double& goal_y )
{
    
    visualization_msgs::msg::Marker goal_marker = visualization_msgs::msg::Marker();
    goal_marker.header.frame_id = "ego_racecar/base_link";
    goal_marker.header.stamp = this->now();
    goal_marker.action = 0;
    goal_marker.ns = "markers";
    goal_marker.type = 2;

    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    goal_marker.pose.position.z = 0.2;

    goal_marker.scale.x = 0.3;
    goal_marker.scale.y = 0.3;
    goal_marker.scale.z = 0.3;

    goal_marker.color.r = 1.0; 
    goal_marker.color.g = 0.0; 
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;

    goal_pub_->publish(goal_marker);

}

void RRT::publish_path(const std::vector<RRT_Node>& path ){

    // std::cout << "im here to publish ur path <3 " << path.size() << std::endl;

    nav_msgs::msg::Path path_msg = nav_msgs::msg::Path();
    path_msg.header.frame_id = "ego_racecar/base_link";

    for(auto it = path.rbegin(); it != path.rend(); it++ ){

        geometry_msgs::msg::PoseStamped pose_msg = geometry_msgs::msg::PoseStamped();
        pose_msg.pose.position.x = it->x;
        pose_msg.pose.position.y = it->y;
        pose_msg.pose.position.z = 0.1;

        path_msg.poses.push_back(pose_msg);

    }

    path_pub_->publish(path_msg);

}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    // std::vector<double> sampled_point;
    // TODO: fill in this method
    // look up the documentation on how to use std::mt19937 devices with a distribution
    // the generator and the distribution is created for you (check the header file)

    std::vector<double> sampled_point;

    // sample pixel centers
    sampled_point.emplace_back( x_dist(gen) );
    sampled_point.emplace_back( y_dist(gen) );
    
    return sampled_point;
}


int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

     // find the closest node

    std::pair<int, double> closest = {-1, std::numeric_limits<double>::max()};
    RRT_Node node;

    node.x = sampled_point[0];
    node.y = sampled_point[1];
    
    for(int n = 0; n < tree.size(); n++ ){

        double dist = norm(node, tree[n]);
        if( dist < closest.second ){
            closest.first = n;
            closest.second = dist;
        }
    }

    return closest.first;

}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering

    // RRT_Node new_node;

    // new_node.x = sampled_point[0];
    // new_node.y = sampled_point[1];

    // double dist = norm(nearest_node, new_node);

    // if( dist < max_expansion_dist ){
    //     return new_node;
    // }

    // Vec2 new_pose(new_node.x - nearest_node.x, new_node.y - nearest_node.y);
    
    // new_pose.x /= dist * max_expansion_dist;
    // new_pose.y /= dist * max_expansion_dist;

    // new_node.x = new_pose.x + nearest_node.x;
    // new_node.y = new_pose.y + nearest_node.y;

    // return new_node;

    RRT_Node new_node;

    // Direction vector from nearest_node to sampled_point
    double dx = sampled_point[0] - nearest_node.x;
    double dy = sampled_point[1] - nearest_node.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // If the sampled point is within max_expansion_dist, use it directly
    if (dist < max_expansion_dist) {
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
    } else {
        // Normalize the direction vector and scale by max_expansion_dist
        dx /= dist;
        dy /= dist;
        new_node.x = nearest_node.x + dx * max_expansion_dist;
        new_node.y = nearest_node.y + dy * max_expansion_dist;
    }

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    // bool collision = false; 

    // check if node is oob
    // if(new_node.x < 0 || new_node.x > map_x*map_resolution) return true;
    // if(new_node.y < -map_wid_m || new_node.y > map_wid_m) return true;

    // find a and b in the pixel world
    std::vector<double> a = { (new_node.y + map_wid_m ) / map_resolution, new_node.x / map_resolution };
    std::vector<double> b = { (nearest_node.y + map_wid_m ) / map_resolution, nearest_node.x / map_resolution };

    // set up bresenhams 
    double dy = std::abs(a[1] - b[1]);
    double dx = std::abs(a[0] - b[0]);

    // find the major axis 
    int i = dx > dy ? 0 : 1;
    int j = dx > dy ? 1 : 0;

    // 
    if( a[i] > b[i] ){
        std::swap(a, b);
    }

    int t1 = (int) std::floor(a[i]);
    int t2 = (int) std::floor(b[i]);

    for( int u = t1; u < t2; u++){

        double w = ( u - a[i] ) / ( b[i] - a[i] );
        double v = w * ( b[j] - a[j] ) + a[j];

        double x_idx = std::floor(u); // px center -> px
        double y_idx = std::floor(v); // px center -> px

        // check if occupied
        int idx = x_idx + y_idx * map_y;
        

        if( occ_grid.data[idx] == 0 ){
            cout <<  (occ_grid.data[idx] == 0) << " /// " << (occ_grid.data[0] == 50) << endl;
            visualization_msgs::msg::Marker goal_marker = visualization_msgs::msg::Marker();
            goal_marker.header.frame_id = "ego_racecar/base_link";
            goal_marker.header.stamp = this->now();
            goal_marker.action = 0;
            goal_marker.ns = "markers";
            goal_marker.type = 2;
            goal_marker.id = 2;

            goal_marker.pose.position.x = new_node.x;
            goal_marker.pose.position.y = new_node.y;
            goal_marker.pose.position.z = 0.2;

            goal_marker.scale.x = 0.3;
            goal_marker.scale.y = 0.3;
            goal_marker.scale.z = 0.3;

            goal_marker.color.r = 0.0; 
            goal_marker.color.g = 0.0; 
            goal_marker.color.b = 1.0;
            goal_marker.color.a = 1.0;

            goal_pub_->publish(goal_marker);

            return false;
        } 

    } 

    return true;

}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal


    double dist = pow( pow(latest_added_node.x - goal_x, 2) + pow(latest_added_node.y + goal_y, 2), 0.5 );

    return dist < goal_threshold;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path
    
    std::vector<RRT_Node> found_path;
    RRT_Node node = latest_added_node;

    while( !node.is_root ){
        found_path.push_back(node);
        node = tree[node.parent];
    }
    found_path.push_back(tree[0]);

    return found_path;
}

// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    double cost = 0;
    // TODO: fill in this method

    return cost;
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;
    // TODO:: fill in this method

    return neighborhood;
}