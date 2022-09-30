#!/usr/bin/env bash
source /opt/ros/noetic/setup.bash;
cd ~/simulation_ws;
catkin_make;
source ~/simulation_ws/devel/setup.bash  
roslaunch rb1_base_gazebo warehouse_rb1.launch