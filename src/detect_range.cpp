#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <vector
#include <algorithm>
#include <limits>

void rangeDetector(const sensor_msgs::LaserScan::ConstPtr& msg){
    
}

int main(int argc, char** argv){
    // Initialize the ROS node
    ros::init(argc, argv, "range_detector_node");
    ros::NodeHandle nh;
    
    // Subscribe to range detector
    ros::Subscriber lidar_sub = nh.subscribe("/ROSBOT2/scan", 10, rangeDetector);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}