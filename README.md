# DEEP Robotics Jueying Lite3 Simulator
DEEP Robotics Jueying Lite3 Gazebo simulation with high-level motion control \
Original Source Code : https://github.com/DeepRoboticsLab/Lite3_Model_Control

## System Requirements
- Ubuntu 20.04
- ROS Noetic

[Note] Before proceeding, check if you get any outputs when running the following command:
```bash
echo $ROS_DISTRO
```
This command is expected to print 'noetic'. Otherwise, follow the steps below and try again. \
(1) Add the following line to the ~/.bashrc file
```
source /opt/ros/noetic/setup.bash
```
(2) Run the following command on the terminal:
```bash
source ~/.bashrc
```

## Install Dependencies
Run the following command at this directory to install dependencies:
```bash
chmod +x ./scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

## How to Build?
To build the relevant packages, run the following command at this directory:
```bash
catkin_make
```

## Running Simulation
All the commands are run at this directory. To terminate programs, press [ctrl + C].
- [Terminal 1] Start gazebo and load the quadruped robot. The robot will be standing on the ground after a few seconds.
```bash
chmod +x ./scripts/start_sim.sh && ./scripts/start_sim.sh
```
- [Terminal 2] Run Keyboard Teleoperation. \
To move robot, press L (torque-stance) and J (Gait Change) keys sequentially, then use A S D W Q E keys to control robot velocity. To stop, press L key. To restart, press J key.
```bash
chmod +x ./scripts/teleop_key.sh && ./scripts/teleop_key.sh
```