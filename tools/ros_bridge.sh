#!/usr/bin/env bash

echo "Bridging topics from ros1-ros2";
source /opt/ros/noetic/setup.bash; 
cd ~/catkin_ws; 
rm -rf build;
rm -rf devel;
catkin_make; 
source devel/setup.bash; 
roslaunch load_params load_params.launch; 
source /opt/ros/noetic/setup.bash; 
source /opt/ros/foxy/setup.bash; 
ros2 run ros1_bridge parameter_bridge;