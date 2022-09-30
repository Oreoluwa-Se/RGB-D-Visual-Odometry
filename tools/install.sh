#!/usr/bin/env bash

cd ~/tools

sudo apt update
sudo apt-get install libqglviewer-dev-qt5 -y

cd ~/tools/Sophus-master/build
sudo make install 

cd ~/tools/fmt-8.1.1/build
sudo make install

cd ~/tools/g2o/build
sudo make install

cd ~/tools/opencv-4.5.4/build
sudo make install

sudo apt install ros-foxy-image-transport-plugins -y
sudo apt-get upgrade -y
sudo ldconfig


cd ~/ros2_ws; 
rm -rf build;
rm -rf install;
rm -rf log;
clear; 
colcon build; 
source install/setup.bash;
clear;
