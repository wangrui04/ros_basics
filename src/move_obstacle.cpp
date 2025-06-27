#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>
#include <limits>

ros::Publisher cmd_vel_pub;

void maintainDist(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Initialising...");
    int range_offset = 25;
    float threshold = 3.0; // Threshold distance to detect obstacles
    float desired_distance = 0.5; // Desired distance from obstacles
    float stop_threshold = 0.05; // Threshold to stop the robot if within this distance of an obstacle

    float linear_speed = 0.0; // Linear speed of the robot
    float max_linear_speed = 0.2; // Maximum linear speed
    
    float min_front_distance = std::numeric_limits<float>::max();
    float min_back_distance = std::numeric_limits<float>::max();
    
    int num_ranges = msg->ranges.size();
    int back_center = num_ranges / 2; // Center index for back scanning

    float Kp = 0.5; // Proportional gain for distance control

    ROS_INFO("Scanning for obstacles in front...");
    // Scanning the right side
    for (int i = num_ranges-range_offset; i < num_ranges; i++){
        float range = msg->ranges[i];
        //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_front_distance = std::min(min_front_distance, range); // Update minimum distance if an obstacle is detected
            //ROS_INFO("Obstacle detected on the right side at distance: %f", min_front_distance);
        }
    }

    // Scanning the left side
    for (int i = 0; i <= range_offset; i++){
        float range = msg->ranges[i];
        //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_front_distance = std::min(min_front_distance, range); // Update minimum distance if an obstacle is detected
            //ROS_INFO("Obstacle detected on the left side at distance: %f", min_front_distance);
        }
    }
    
    //ROS_INFO("Minimum distance to obstacle in front: %f", min_distance);

    ROS_INFO("Scanning for obstacles behind...");

    for (int i = back_center - range_offset; i <= back_center + range_offset; i++) {
        if (i >= 0 && i < num_ranges) { // Ensure index is within bounds
            float range = msg->ranges[i];
            //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
            if (range < threshold && range > msg->range_min) {
                min_back_distance = std::min(min_back_distance, range); // Update minimum distance if an obstacle is detected
                //ROS_INFO("Obstacle detected behind at distance: %f", min_back_distance);
            }
        }
    }
    ROS_INFO("Front distance: %f, Back distance: %f", min_front_distance, min_back_distance);
    geometry_msgs::Twist cmd_vel_msg;
    float diff = 0.0; // Difference between desired and actual distance

    // If no valid obstacle readings are detected
    if (min_front_distance >= threshold) {
        ROS_WARN("No obstacles detected. Stopping the robot.");
        linear_speed = 0.0;  // Let the rest of the logic publish a stop
    }
    else if (min_front_distance < desired_distance - stop_threshold) {
        diff = min_front_distance - desired_distance;
        linear_speed = Kp * diff;  // Move backward (diff is negative)
        ROS_WARN("Too close to front obstacle, moving back...");
        ROS_INFO("Distance from obstacle: %f", min_front_distance);
    }
    else if (min_front_distance > desired_distance + stop_threshold) {
        diff = min_front_distance - desired_distance;
        linear_speed = Kp * diff;  // Move forward
        ROS_INFO("Too far from front obstacle, moving forward...");
        ROS_INFO("Distance from obstacle: %f", min_front_distance);
    }
    // else if (min_back_distance < desired_distance - stop_threshold) {
    //     diff = desired_distance - min_back_distance;
    //     linear_speed = Kp * diff;  // Move forward
    //     ROS_WARN("Too close to back obstacle, moving forward...");
    //     ROS_INFO("Distance from obstacle: %f", min_back_distance);
    // }
    // else if (min_back_distance > desired_distance + stop_threshold) {
    //     diff = desired_distance - min_back_distance;
    //     linear_speed = Kp * diff;  // Move backward
    //     ROS_INFO("Too far from back obstacle, moving back...");
    //     ROS_INFO("Distance from obstacle: %f", min_back_distance);
    // }
    else {
        linear_speed = 0.0;
        ROS_INFO("Within desired distance range. Stopping.");
    }

    linear_speed = std::clamp(linear_speed, -max_linear_speed, max_linear_speed); // Clamp the speed to the maximum limits
    // Limit the linear speed to the maximum allowed speed

    // if (linear_speed > max_linear_speed) linear_speed = max_linear_speed;
    // else if (linear_speed < -max_linear_speed) linear_speed = -max_linear_speed;


    if (std::abs(diff) < stop_threshold) {
        linear_speed = 0.0; // Stop if within the threshold
        ROS_INFO("Stopping the robot within threshold distance.");
    }

    cmd_vel_msg.linear.x = linear_speed; // Set the linear speed
    cmd_vel_msg.angular.z = 0.0; // No angular movement 
    cmd_vel_pub.publish(cmd_vel_msg); // Publish the velocity command
    ROS_INFO("Published cmd_vel: linear.x = %f", cmd_vel_msg.linear.x);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "obstacle_avoider_node");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ROSBOT2/cmd_vel", 10);
    ROS_INFO("Node initialized, ready to maintain distance from obstacles.");
    
    // Subscribe to obstacle detector
    ros::Subscriber lidar_sub = nh.subscribe("/ROSBOT2/scan", 10, maintainDist);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}