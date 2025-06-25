#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    // Callback function to process LiDAR data
    // Here you can access the LiDAR scan data from msg->ranges
    ROS_INFO("Received LiDAR data with %zu ranges", msg->ranges.size());
    
    // Example: Print the first 10 range values
    for (size_t i = 0; i < std::min(msg->ranges.size(), size_t(10)); ++i) {
        ROS_INFO("Range[%zu]: %f", i, msg->ranges[i]);
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "subscribe_lidar_data");
    ros::NodeHandle nh;

    // Subscribe to the LiDAR data topic
    ros::Subscriber lidar_sub = nh.subscribe("scan", 10, lidarCallback);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}