#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <algorithm>

ros::Publisher cmd_vel_pub;

void maintainDist(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Initialising...");
    int range_offset = 25;
    float threshold = 0.5; 
    float desired_distance = 0.5; // Desired distance from obstacles
    float linear_speed = 0.0; // Linear speed of the robot
    int min_front_distance = msg->range_min; // Minimum distance the LiDAR can measure
    int min_back_distance = msg->range_min; // Minimum distance the LiDAR can measure behind
    float stop_threshold = 0.05; // Threshold to stop the robot if within this distance of an obstacle

    int num_ranges = msg->ranges.size();

    ROS_INFO("Scanning for obstacles in front...");
    // Scanning the right side
    for (int i = num_ranges-range_offset; i < num_ranges; i++){
        float range = msg->ranges[i];
        //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_front_distance = std::min(min_front_distance, range); // Update minimum distance if an obstacle is detected
            ROS_INFO("Obstacle detected on the right side at distance: %f", min_front_distance);
        }
    }

    // Scanning the left side
    for (int i = 0; i <= range_offset; i++){
        float range = msg->ranges[i];
        //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_front_distance = std::min(min_front_distance, range); // Update minimum distance if an obstacle is detected
            ROS_INFO("Obstacle detected on the left side at distance: %f", min_front_distance);
        }
    }
    
    ROS_INFO("Minimum distance to obstacle in front: %f", min_distance);

    int back_center = num_ranges / 2; // Center index for back scanning
    ROS_INFO("Scanning for obstacles behind...");

    for (int i = back_center - range_offset; i <= back_center + range_offset; i++) {
        if (i <= && i < num_ranges) { // Ensure index is within bounds
            float range = msg->ranges[i];
            //ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
            if (range < threshold && range > msg->range_min) {
                min_back_distance = std::min(min_back_distance, range); // Update minimum distance if an obstacle is detected
                ROS_INFO("Obstacle detected behind at distance: %f", min_back_distance);
            }
        }
    }
    geometry_msgs::Twist cmd_vel_msg;

    //float diff = desired_distance - min_distance;

    float Kp = 0.5; // Proportional gain for distance control
    linear_speed = Kp * diff;

    float max_linear_speed = 0.2; // Maximum linear speed

    bool front_too_close = (min_front_distance < desired_distance - stop_threshold);
    bool back_too_close = (min_back_distance < desired_distance - stop_threshold);
    bool front_too_far = (min_front_distance > desired_distance + stop_threshold);
    bool back_too_far = (min_back_distance > desired_distance + stop_threshold);
    if(front_too_close || back_too_close) {
        ROS_WARN("Obstacle too close! Adjusting speed to maintain distance.");
    } else if(front_too_far || back_too_far) {
        ROS_INFO("Obstacle too far away, maintaining speed.");
    } else {
        ROS_INFO("No obstacles detected within the desired distance.");
    }

    if (linear_speed > max_linear_speed) {
        linear_speed = max_linear_speed; // Limit the speed to the maximum
    } else if (linear_speed < -max_linear_speed) {
        linear_speed = -max_linear_speed; // Allow for reverse movement if needed
    }

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