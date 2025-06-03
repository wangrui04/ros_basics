// Description: This code is a ROS node that controls a robot to travel with a specified angular velocity.
// It subscribes to the robot's IMU data and publishes velocity commands to turn the robot.
#include "ros/ros.h"
#include <geometry_msgs/Twist.h> //velocity message type 
#include <sensor_msgs/Imu.h> //publish imu message that we can listen to 

void imuCallback(const sensor_msgs::Imu::ConstPtr& data){
    ROS_INFO("Angular Velocity = %f", data->angular_velocity.z); //prints out the angular velocity 
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rosbot_turn"); //create a new node called rosbot_turn, register itself with the ros master 
    ros::NodeHandle nh; //use this to generate new ros objects 
    ros::Publisher vel_pub; //publisher 

    vel_pub = nh.advertise<geometry_msgs::Twist>("/ROSBOT2/cmd_vel", 1, true); //register publisher with ros master, things within <> is the type of msgs 
    //cmd_vel is the topic name --> publishing twist messages; 1 is the buffer length, if you're publishing faster than the master can handle, it will start deleting data 
    ros::Subscriber sub = nh.subscribe("/ROSBOT2/imu", 10, imuCallback); //subscribe to a topic; callback function: function thats called whenever node receives a msg 
    ros::Rate loop_rate(10); //10Hz, every 10s run the loop once 

    int count = 0;
    geometry_msgs::Twist vel;
    vel.linear.x = 0.5; //linear velocity 
    vel.angular.z = 2.0; //angular velocity 

    while(ros::ok()){ //something like a while(true)
        vel_pub.publish(vel); //publish one instance of velocity 
        ros::spinOnce(); //any msgs received it will call the callbacks on these msgs 
        loop_rate.sleep();
        count++;
        if(count == 50){ // e.g. freq = 10s, i want it run for 2s
            vel.linear.x = 0;
            vel.angular.z = 0;
        }
    }
    return 0;
}