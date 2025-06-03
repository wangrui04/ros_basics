# ros_basics
Code used for introductory practices to ROS

# rosbot_turn.cpp
* This code is a ROS node that controls a robot to travel with a specified angular velocity.
* It subscribes to the robot's IMU data and publishes velocity commands to turn the robot.
* Formula from slide 43 and 46

# rosbot_travel.cpp
* Description: This code is a ROS node that controls a robot to travel towards a specified goal position.
* It subscribes to the robot's position from Vicon and publishes velocity commands to move towards the goal.

# Running commands on ubuntu
* ssh into the robot
* launch the vicon bridge: roslaunch vicon_bridge vicon.launch
* launch the file that is governing the robot movement e.g. rosrun rosbot_amr rosbot_travel

# Some notes
* run roscore to run the following
* use rostopic type /vicon/rosbot_2/rosbot_2 to get the positional data