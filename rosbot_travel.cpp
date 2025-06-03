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
    ROS_INFO("x-coordinate: %f", data->transform.translation.x);
    ROS_INFO("y-coordinate: %f", data->transform.translation.y);
    ROS_INFO("theta: %f", tf::getYaw(data->transform.rotation));
}

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    robot_x = msg->transform.translation.x;
    robot_y = msg->transform.translation.y;
    robot_theta = tf::getYaw(msg->transform.rotation);
}

void controlInput(){ //convert to velocity cmd to drive the robot
    double velocity = 0.5*(sqrt(pow((goal_x-robot_x), 2) + pow((goal_y-robot_y), 2)));
    double theta_d = atan2((goal_y-robot_y),(goal_x-robot_x));
    double t = (theta_d - robot_theta); 
    double gamma = 2*(atan2(sin(t), cos(t)));

    vel.linear.x = velocity;
    vel.angular.z = gamma;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "rosbot_travel");
    ros::NodeHandle nh;
    ros::Publisher pos_pub;

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