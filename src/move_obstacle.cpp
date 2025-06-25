#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void()

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "subscribe_lidar_data");
    ros::NodeHandle nh;
    
    // Subscribe to obstacle detector
    ros::Subscriber lidar_sub = nh.subscribe("/ROSBOT2/scan", 10, obstacleDetector);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}