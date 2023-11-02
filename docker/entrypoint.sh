#!/bin/bash

# Source the ROS setup script and set up the environment
#source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

# Run your ROS nodes or desired commands

roslaunch kortex_gazebo spawn_kortex_robot.launch
