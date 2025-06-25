#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_vel_pub;

void maintainDist(const sensor_msgs::LaserScan::ConstPtr& msg) {
    ROS_INFO("Initialising...");
    int range_offset = 25;
    float threshold = 0.5; 
    float desired_distance = 0.5; // Desired distance from obstacles
    float linear_speed = 0.0; // Linear speed of the robot
    int min_distance = msg->range_min; // Minimum distance the LiDAR can measure

    int num_ranges = msg->ranges.size();

    ROS_INFO("Scanning for obstacles...");
    // Scanning the right side
    for (int i = num_ranges-range_offset; i < num_ranges; i++){
        float range = msg->ranges[i];
        ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_distance = range; // Update minimum distance if an obstacle is detected
            ROS_INFO("Obstacle detected on the right side at distance: %f", min_distance);
        }
    }

    // Scanning the left side
    for (int i = 0; i <= range_offset; i++){
        float range = msg->ranges[i];
        ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            min_distance = range; // Update minimum distance if an obstacle is detected
            ROS_INFO("Obstacle detected on the left side at distance: %f", min_distance);
        }
    }
    
    ROS_INFO("Minimum distance to obstacle: %f", min_distance);

    geometry_msgs::Twist cmd_vel_msg;

    float diff = desired_distance - min_distance;

    float Kp = 0.5; // Proportional gain for distance control
    linear_speed = Kp * diff;

    float max_linear_speed = 0.2; // Maximum linear speed
    if (linear_speed > max_linear_speed) {
        linear_speed = max_linear_speed; // Limit the speed to the maximum
    } else if (linear_speed < -max_linear_speed) {
        linear_speed = -max_linear_speed; // Allow for reverse movement if needed
    }

    float stop_threshold = 0.05;
    if (std::abs(diff) < stop_threshold) {
        linear_speed = 0.0; // Stop if within the threshold
        ROS_INFO("Stopping the robot within threshold distance.");
    }

    cmd_vel_msg.linear.x = linear_speed; // Set the linear speed
    cmd_vel_msg.angular.z = 0.0; // No angular movement 
    cmd_vel_pub.publish(cmd_vel_msg); // Publish the velocity command
    ROS_INFO("Published cmd_vel: linear.x = %f", cmd_vel_msg.linear.x);

    if (min_distance < threshold) {
        ROS_WARN("Obstacle too close! Adjusting speed to maintain distance.");
    } else if (min_distance >= threshold) {
        ROS_INFO("No obstacles detected within threshold distance.");
    } else{
        ROS_INFO("Maintaining distance from obstacles.");
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "subscribe_lidar_data");
    ros::NodeHandle nh;

    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/ROSBOT2/cmd_vel", 10);
    ROS_INFO("Node initialized, ready to maintain distance from obstacles.");
    
    // Subscribe to obstacle detector
    ros::Subscriber lidar_sub = nh.subscribe("/ROSBOT2/scan", 10, maintainDist);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}