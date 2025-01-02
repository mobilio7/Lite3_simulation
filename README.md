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
```bash
source /opt/ros/noetic/setup.bash
```
(2) Run the following command on the terminal:
```bash
source ~/.bashrc
```

################################################################
##  1. docker 사용하는 경우
## [Option 1] Using Docker Container
################################################################
In the case the system requirements cannot be satisfied (e.g. you are using a different version of Ubuntu), \
you can work inside a Docker container created from a ROS Noetic image. \
Follow the steps below to create the Docker container inside a Ubuntu system.

(1) If Docker Engine is not installed on your system, please follow the instructions from the link below to install Docker Engine.

https://docs.docker.com/engine/install/ubuntu/

(2) Build a Docker image from the Dockerfile. The image name will be lite3-sim-ros1-image
```bash
sudo docker build -t lite3-sim-ros1-image .
```
(3) Create a Docker container from the image. The container name will be lite3_sim_ros1
```bash
sudo docker run -it --name lite3_sim_ros1 --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY lite3-sim-ros1-image
```
(4) To interact with the container, run the following command:
```bash
sudo docker exec -it lite3_sim_ros1 bash
```
If the container is not running, use the following command to start the container, then try again.
```bash
sudo docker start lite3_sim_ros1
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
### Running on Docker Container
If you are trying to run simulation on a Docker container, rendering may fail. In such cases, open a new terminal, run the following command, and try again.
```bash
xhost +local:docker
```

################################################################################################################################################
##  2. ros Server 사용하는 경우
## [Option 2] ROS Server
################################################################################################################################################
(1) This command is for modifying the CMakeLists.txt file located in Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/
```bash
sed -i '53s/.*/set(Python3_EXECUTABLE "\/usr\/bin\/python3.8")/' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a find_package(Python3 REQUIRED COMPONENTS Interpreter Development)' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a set(Python3_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a set(Python3_INCLUDE_DIR "/usr/include/python3.8")' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt
```

(2) Install required libraries
```bash
sudo apt install liblcm-dev
sudo apt install libglm-dev
```
