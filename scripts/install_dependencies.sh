#!/bin/bash
sudo apt-get update
sudo apt-get upgrade -y

sudo apt-get install -y build-essential
sudo apt-get install -y cmake
#########################################3
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y libeigen3-dev
sudo apt-get install -y liblcm-dev
sudo apt-get install -y libglm-dev
##########################################

sudo apt-get install -y ros-${ROS_DISTRO}-robot-state-publisher
sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-publisher
sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros
sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-pkgs
################################################################
sudo apt-get install -y ros-${ROS_DISTRO}-rosbash
sudo apt-get install -y ros-${ROS_DISTRO}-xacro
sudo apt-get install -y ros-${ROS_DISTRO}-rviz
sudo apt-get install -y ros-${ROS_DISTRO}-controller-interface
sudo apt-get install -y ros-${ROS_DISTRO}-gazebo-ros-control
sudo apt-get install -y ros-${ROS_DISTRO}-joint-state-controller
sudo apt-get install -y ros-${ROS_DISTRO}-effort-controllers
sudo apt-get install -y ros-${ROS_DISTRO}-joint-trajectory-controller
################################################################