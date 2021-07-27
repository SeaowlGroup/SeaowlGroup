#!/bin/sh

# Configure Ubuntu to allow restricted, universe and multiverse repositories

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt-get update && sudo apt-get upgrade
sudo apt install ros-noetic-desktop-full

source /opt/ros/noetic/setup.bash

sudo apt install ros-noetic-tf2
sudo apt install libeigen3-dev
sudo apt install python3-pip
pip3 install pandas
pip3 install openpyxl
pip3 install scipy
pip3 install datetime

mkdir -p ws/src/
cd ws/src/
git clone https://github.com/Straccia11/seaowl.git
