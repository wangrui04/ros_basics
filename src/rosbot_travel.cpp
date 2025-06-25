// Description: This code is a ROS node that controls a robot to travel towards a specified goal position.
// It subscribes to the robot's position from Vicon and publishes velocity commands to move towards the goal.   
#include "ros/ros.h"
#include <geometry_msgs/Twist.h> //velocity message type 
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <cmath>

double robot_x;
double robot_y;
double robot_theta;
double goal_x;
double goal_y;
geometry_msgs::Twist vel;

void TransformStampedCallback(const geometry_msgs::TransformStamped::ConstPtr& data){
    // This callback function is not used in this code, but it can be used to print the transform data.
    ROS_INFO("x-coordinate: %f", data->transform.translation.x);
    ROS_INFO("y-coordinate: %f", data->transform.translation.y);
    ROS_INFO("theta: %f", tf::getYaw(data->transform.rotation));
}

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    // Callback function to update the robot's position and orientation based on Vicon data.
    // It extracts the robot's x, y coordinates and orientation (theta) from the TransformStamped message.
    robot_x = msg->transform.translation.x;
    robot_y = msg->transform.translation.y;
    robot_theta = tf::getYaw(msg->transform.rotation);
}

void controlInput(){ //convert to velocity cmd to drive the robot
    // This function calculates the linear and angular velocity commands for the robot to move towards the goal position.
    // It computes the distance to the goal and the required angular adjustment based on the robot's current position and orientation.
    double velocity = 0.5*(sqrt(pow((goal_x-robot_x), 2) + pow((goal_y-robot_y), 2)));
    double theta_d = atan2((goal_y-robot_y),(goal_x-robot_x));
    double t = (theta_d - robot_theta); 
    double gamma = 2*(atan2(sin(t), cos(t)));

    vel.linear.x = velocity;
    vel.angular.z = gamma;
}

int main(int argc, char** argv){
    // Main function to initialize the ROS node, set up publishers and subscribers, and control the robot's movement.
    // It continuously checks the robot's position and publishes velocity commands to move towards the goal.
    // Initialize the ROS node
    // Set up the node handle, publisher, and subscriber
    ros::init(argc, argv, "rosbot_travel");
    ros::NodeHandle nh;
    ros::Publisher pos_pub;

    // Advertise the cmd_vel topic to publish velocity commands
    // The topic name is "ROSBOT2/cmd_vel", with a queue size of 1 and latch set to true
    pos_pub = nh.advertise<geometry_msgs::Twist>("ROSBOT2/cmd_vel", 1, true);

    ros::Subscriber sub = nh.subscribe("/vicon/rosbot_2/rosbot_2", 10, viconCallback);
    ros::Rate loop_rate(10);

    goal_x = 0.5;
    goal_y = 1.5; 

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        controlInput();
        pos_pub.publish(vel);

        ROS_INFO("x-coordinate: %f", robot_x);
        ROS_INFO("y-coordinate: %f", robot_y);
        ROS_INFO("theta: %f", robot_theta);
    }
    return 0;
}
