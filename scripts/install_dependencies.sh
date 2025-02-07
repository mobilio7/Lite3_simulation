#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install -y build-essential
sudo apt-get install -y cmake
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y liblcm-dev
sudo apt-get install -y libglm-dev

sudo apt-get install -y ros-noetic-robot-state-publisher
sudo apt-get install -y ros-noetic-joint-state-publisher
sudo apt-get install -y ros-noetic-gazebo-ros
sudo apt-get install -y ros-noetic-gazebo-ros-pkgs
sudo apt-get install -y ros-noetic-rosbash
sudo apt-get install -y ros-noetic-xacro
sudo apt-get install -y ros-noetic-rviz
sudo apt-get install -y ros-noetic-controller-interface
sudo apt-get install -y ros-noetic-gazebo-ros-control
sudo apt-get install -y ros-noetic-joint-state-controller
sudo apt-get install -y ros-noetic-effort-controllers
sudo apt-get install -y ros-noetic-joint-trajectory-controller

# Install Velodyne Simulator
sudo apt-get install -y git
cd src && git clone https://bitbucket.org/DataspeedInc/velodyne_simulator.git