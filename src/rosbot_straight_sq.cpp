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

std::vector<std::pair<double, double>> square_goals;
int current_goal_index = 0;
const double goal_tolerance = 0.1; // Tolerance for reaching the goal   

enum State { ROTATING, MOVING };
State current_state = ROTATING;


void viconCallback(const geometry_msgs::TransformStamped::ConstPtr& msg){

    //Callback function to update the robot's position and orientation based on Vicon data.
    //It extracts the robot's x, y coordinates and orientation (theta) from the TransformStamped message.
   
    robot_x = msg->transform.translation.x;
    robot_y = msg->transform.translation.y;
    robot_theta = tf::getYaw(msg->transform.rotation);
}

void resetGoal(){
    
    //This function resets the goal position to the next corner of a square path.
    //It cycles through the predefined square goals stored in the vector.
    
    double start_x = robot_x;
    double start_y = robot_y;

    square_goals.push_back({start_x + 1.5, start_y}); // Move right
    square_goals.push_back({start_x + 1.5, start_y + 1.5}); // Move up  
    square_goals.push_back({start_x, start_y + 1.5}); // Move left
    square_goals.push_back({start_x, start_y}); // Move down
}

void controlInput(){ //convert to velocity cmd to drive the robot
    
    //This function calculates the linear and angular velocity commands for the robot to move towards the goal position.
    //It computes the distance to the goal and the required angular adjustment based on the robot's current position and orientation.
    
    double velocity = 0.3*(sqrt(pow((goal_x-robot_x), 2) + pow((goal_y-robot_y), 2)));
    double theta_d = atan2((goal_y-robot_y),(goal_x-robot_x));
    double t = (theta_d - robot_theta); 
    double gamma = 3*(atan2(sin(t), cos(t)));

    vel.linear.x = velocity;
    vel.angular.z = gamma;
}

bool goal_reached(double goal_x, double goal_y){
    
    //This function checks if the robot has reached the current goal position within a specified tolerance.
    //It returns true if the distance to the goal is less than or equal to the goal tolerance.
    
    double distance = sqrt(pow((goal_x - robot_x), 2) + pow((goal_y - robot_y), 2));
    return distance <= goal_tolerance;
}

int main(int argc, char** argv){
    
    //Main function to initialize the ROS node, set up publishers and subscribers, and control the robot's movement.
    //It continuously checks the robot's position and publishes velocity commands to move towards the goal.
    
    // Initialize the ROS node
    // Set up the node handle, publisher, and subscriber
    ros::init(argc, argv, "rosbot_square");
    ros::NodeHandle nh;
    ros::Publisher pos_pub;

    // Advertise the cmd_vel topic to publish velocity commands
    // The topic name is "ROSBOT2/cmd_vel", with a queue size of 1 and latch set to true
    pos_pub = nh.advertise<geometry_msgs::Twist>("ROSBOT2/cmd_vel", 1, true);

    ros::Subscriber sub = nh.subscribe("/vicon/rosbot_2/rosbot_2", 10, viconCallback);
    ros::Rate loop_rate(10);

    // goal_x = 0.5;
    // goal_y = 1.5; 

    resetGoal();

    goal_x = square_goals[current_goal_index].first;
    goal_y = square_goals[current_goal_index].second;

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
//        resetGoal(); // Reset the goal position every iteration
//	goal_x = square_goals[0].first;
//	goal_y = square_goals[0].second;
        if(goal_reached(goal_x,goal_y)){
           ROS_INFO("Reached goal %d: (%f, %f)", current_goal_index, goal_x, goal_y);
           current_goal_index = (current_goal_index + 1) % square_goals.size();
	   goal_x = square_goals[current_goal_index].first;
	   goal_y = square_goals[current_goal_index].second;
        }
        controlInput();
        pos_pub.publish(vel);

        ROS_INFO("x-coordinate: %f", robot_x);
        ROS_INFO("y-coordinate: %f", robot_y);
        ROS_INFO("theta: %f", robot_theta);

	loop_rate.sleep();
    }
    return 0;
}
