#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

void obstacleDetector(const sensor_msgs::LaserScan::ConstPtr& msg){
    ROS_INFO("Initialising...");
    //int center_index = msg->ranges.size() / 2;
    int range_offset = 25;
    float threshold = 0.5; 

    // Scanning a range of 20 scan beams centered at the middle 
    int center = 0; //center_index - range_offset;
    //int right = 20; //center_index + range_offset;
   
    int num_ranges = msg->ranges.size();
    
    bool obstacle_present = false;

    ROS_INFO("Scanning for obstacles...");
    // Scanning the right side
    for (int i = num_ranges-range_offset; i < num_ranges; i++){
        float range = msg->ranges[i];
        ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            obstacle_present = true;
            break;
        }
    }

    // Scanning the left side
    for (int i = center; i <= center+range_offset; i++){
        float range = msg->ranges[i];
        ROS_INFO("Range[%d]: %f", i, msg->ranges[i]);
        if(range < threshold && range > msg->range_min){
            obstacle_present = true;
            break;
        }
    }

    if(obstacle_present){
        ROS_INFO("There is an obstacle within 0.5m of the rosbot");
    }
    else{
        ROS_INFO("There is no obstacle detected");
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "obstacle_detector_node");
    ros::NodeHandle nh;
    
    // Subscribe to obstacle detector
    ros::Subscriber lidar_sub = nh.subscribe("/ROSBOT2/scan", 10, obstacleDetector);

    // Spin to keep the node running and processing callbacks
    ros::spin();

    return 0;
}