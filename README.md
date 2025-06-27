Adding a line
Adding a second line
# ros_basics
Code used for introductory practices to ROS

# rosbot_turn.cpp
* This code is a ROS node that controls a robot to travel with a specified angular velocity.
* It subscribes to the robot's IMU data and publishes velocity commands to turn the robot.
* Formula from slide 43 and 46

# rosbot_travel.cpp
* This code is a ROS node that controls a robot to travel towards a specified goal position.
* It subscribes to the robot's position from Vicon and publishes velocity commands to move towards the goal.

# rosbot_square.cpp
* This code allows the rosbot to travel in a perfect square. 
* Future work: allow the robot to turn on the spot when it reaches a goal instead of travelling in a curvature.

# lidar_subscriber.cpp
* This code enables ROS to subscribe data from the lidar of the rosbot.

# obstacle_detector.cpp
* This code enables the rosbot to detect obstacles based on lidar data.
* The range of detection is 40 scan ranges with the direct front being the center (20 to the right and 20 to the left). 

# move_obstacle.cpp
* This code allows the rosbot to maintain a fixed distance from the obstacle in front of it e.g. 0.5m.
* If the obstacle moves away from the rosbot, it will move forward until it is 0.5m away from the obstacle.
* If the obstacle moves towards the rosbot, it will move backwards until it is 0.5m away from the obstacle.

# Running commands on ubuntu
* ssh into the robot
* launch the vicon bridge: roslaunch vicon_bridge vicon.launch
* launch the file that is governing the robot movement e.g. rosrun rosbot_amr rosbot_travel

# Some notes
* run roscore to run the following
* use rostopic type /vicon/rosbot_2/rosbot_2 to get the positional data
