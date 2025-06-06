#include "ros/ros.h"
#include <geometry_msgs/Twist.h> //velocity message type 
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <cmath>

double robot_x;
double robot_y; 
double robot_theta;
geometry_msgs::Twist vel;

// Circle parameters
double circle_x = 0.0; // Center x-coordinate of the circle
double circle_y = 0.0; // Center y-coordinate of the circle
double radius = 1.0; // Radius of the circle
double angular_velocity = 0.5; // Angular velocity in radians per second
double desired_distance = 0.5; // Desired distance from the center of the circle

void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){
    // """
    // Callback function to update the robot's position and orientation based on Vicon data.
    // It extracts the robot's x, y coordinates and orientation (theta) from the TransformStamped message.
    // """
    robot_x = msg->transform.translation.x;
    robot_y = msg->transform.translation.y;
    robot_theta = tf::getYaw(msg->transform.rotation);
}

void getCircularPursuitPoint(double& desired_x, double& desired_y, double t) {
    // """
    // This function calculates the desired point on the circular path based on the time t.
    // It uses the center coordinates, radius, and angular velocity to compute the x and y coordinates.
    // """
    desired_x = circle_x + radius * cos(angular_velocity * t);
    desired_y = circle_y + radius * sin(angular_velocity * t);
}

void controlInput(double& desired_x, double& desired_y) {
    double error = hypot(desired_x - robot_x, desired_y - robot_y) - desired_distance; // Distance from the center to the desired point
    double velocity = 0.5*(sqrt(pow((desired_x-robot_x), 2) + pow((desired_y-robot_y), 2)));
    double theta_d = atan2((desired_y-robot_y),(desired_x-robot_x));// Desired angle to the point
    double t = (theta_d - robot_theta); 
    double gamma = 2*(atan2(sin(t), cos(t)));

    vel.linear.x = velocity;
    vel.angular.z = gamma;
}

int main(int argc, char** argv){
    // """
    // Main function to initialize the ROS node, set up publishers and subscribers, and control the robot's movement.
    // It continuously checks the robot's position and publishes velocity commands to move towards the goal.
    // """
    // Initialize the ROS node
    // Set up the node handle, publisher, and subscriber
    ros::init(argc, argv, "rosbot_circle");
    ros::NodeHandle nh;
    ros::Publisher pos_pub;

    // Advertise the cmd_vel topic to publish velocity commands
    // The topic name is "ROSBOT2/cmd_vel", with a queue size of 1 and latch set to true
    pos_pub = nh.advertise<geometry_msgs::Twist>("ROSBOT2/cmd_vel", 1, true);

    ros::Subscriber sub = nh.subscribe("/vicon/rosbot_2/rosbot_2", 10, viconCallback);
    ros::Rate loop_rate(10);

    // goal_x = 0.5;
    // goal_y = 1.5; 

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
       // controlInput();
        pos_pub.publish(vel);

        ROS_INFO("x-coordinate: %f", robot_x);
        ROS_INFO("y-coordinate: %f", robot_y);
        ROS_INFO("theta: %f", robot_theta);
    }
    return 0;
}
