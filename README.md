# DEEP Robotics Jueying Lite3 Simulator
DEEP Robotics Jueying Lite3 Gazebo simulation with high-level motion control \
[Note] Original Source Code : https://github.com/DeepRoboticsLab/Lite3_Model_Control \
**[Warning] The simulated robot may have different parameters from the real robot.** \
**[Warning] The sensors (e.g. LiDAR, IMU), actuators, etc. and their properties may be different from the ones in the real robot.** \
**[Warning] The simulated robot's velocity control may not be accurate.**

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

## [Optional] Using Docker Container

In the case the system requirements cannot be satisfied (e.g. you are using a different version of Ubuntu), \
you can work inside a Docker container created from a ROS Noetic image. \
Follow the steps below to create the Docker container inside a Ubuntu system.

### Step 1 : Install Docker Engine
If Docker Engine is not installed on your system, please follow the instructions from the link below to install Docker Engine.

https://docs.docker.com/engine/install/ubuntu/

### Step 2 : Install NVIDIA Container Toolkit (Optional, but strongly recommended)
If your PC is equipped with NVIDIA graphics card, then follow the instructions from the link below to install NVIDIA Container Toolkit.

https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

Creating a Docker container with GPU enabled is **essential if you want to try robot simulation with 3D LiDAR plugin**, which may slow down the simulation speed significantly when only CPU is used.

### Step 3 : Build a Docker Image
Build a Docker image from the provided Dockerfile. The image name will be lite3-sim-ros1-image
```bash
sudo docker build -t lite3-sim-ros1-image .
```

### Step 4 : Create a Docker Container
Create a Docker container using the image from [Step 3]. The container name will be lite3_sim_ros1
#### With GPU
```bash
sudo docker run -it --name lite3_sim_ros1 --gpus all --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY lite3-sim-ros1-image
```
#### Without GPU
```bash
sudo docker run -it --name lite3_sim_ros1 --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY lite3-sim-ros1-image
```

### Step 5 : Interact with the Docker Container
To interact with the container, run the following command:
```bash
sudo docker exec -it lite3_sim_ros1 bash
```
If the container is not running, use the following command to start the container, then try again.
```bash
sudo docker start lite3_sim_ros1
```

## [Optional] Using ROS Server

This command is for modifying the CMakeLists.txt file for "quadruped" package.
[Note] Please enter this command all at once
```bash
sed -i '53s/.*/set(Python3_EXECUTABLE "\/usr\/bin\/python3.8")/' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a find_package(Python3 REQUIRED COMPONENTS Interpreter Development)' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a set(Python3_LIBRARIES "/usr/lib/x86_64-linux-gnu/libpython3.8.so")' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt && \
sed -i '53a set(Python3_INCLUDE_DIR "/usr/include/python3.8")' ~/Lite3_simulation/src/Lite3_Model_Control/high_level_sim/src/quadruped/CMakeLists.txt
```


## Install Dependencies
Run the following command at this directory to install dependencies:
```bash
chmod +x ./scripts/install_dependencies.sh
./scripts/install_dependencies.sh
```

## Build Packages
To build the relevant packages, run the following commands at this directory:
```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

## Run Simulation
All the commands are run at this directory. To terminate programs, press [ctrl + C].

### Terminal 1 : Robot Simulation
You can either run robot simulation with or without a 3D LiDAR plugin. \
To run without 3D LiDAR, run: \
**(With GPU)**
```bash
chmod +x ./scripts/start_sim.sh && ./scripts/start_sim.sh
```
**(Without GPU)**
```bash
chmod +x ./scripts/start_sim.sh && LIBGL_ALWAYS_SOFTWARE=1 ./scripts/start_sim.sh
```
To run with 3D LiDAR enabled, run: \
**(With GPU)**
```bash
chmod +x ./scripts/start_sim.sh && ./scripts/start_sim.sh lidar:=true
```
**(Without GPU)**
```bash
chmod +x ./scripts/start_sim.sh && LIBGL_ALWAYS_SOFTWARE=1 ./scripts/start_sim.sh lidar:=true
```
Then Gazebo simulation begins and loads the quadruped robot. The robot will stand on the ground after a few seconds. (If the robot fails to stand, then stop the process and try again.)

### Terminal 2 : Keyboard Teleoperation
Run the following command to start keyboard Teleoperation.
```bash
chmod +x ./scripts/teleop_key.sh && ./scripts/teleop_key.sh
```
To move the robot, press L (torque-stance) and J (Gait Change) keys sequentially, then use W A S D Q E keys to control robot velocity. \
To stop, press L key. To restart, press J key.

[Note] W : front, A : left, S : back, D : right, Q : diagonal left, E : diagonal right

### Terminal 3 : RViz Visualization
Run the following command to visualize robot states. 
```bash
chmod +x ./scripts/lite3_rviz.sh && ./scripts/lite3_rviz.sh
```
If 3D LiDAR is enabled, then the 3D point cloud will be also displayed on RViz screen.

### (Note) Running on Docker Container
If you are trying to run simulation on a Docker container, rendering may fail. In such cases, open a new terminal, run the following command, and try again.
```bash
xhost +local:docker
```

## Compatibility with ROS2 Applications (ROS1 Bridge)
Probably you want to apply some algorithms (e.g. SLAM, navigation) written in ROS2 framework to this virtual robot. The package "ROS1 Bridge" enables communication between ROS1 and ROS2 applications. For detailed explanation about ROS1 Bridge, please visit: \
 https://github.com/ros2/ros1_bridge

### Before Proceeding
- If you use Docker, it is recommended to create a separate Docker container for ROS2 applications because of the root privilege issue. Also, all relevant Docker containers should be created with "--network host". 

  Create the Docker container for simulation by running the following command: \
  **(With GPU)**
  ```bash
  sudo docker run -it --name lite3_sim_ros1 --gpus all --network host --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY lite3-sim-ros1-image
  ```
  **(Without GPU)**
  ```bash
  sudo docker run -it --name lite3_sim_ros1 --network host --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY lite3-sim-ros1-image
  ```

  Create the Docker container for ROS2 applications as below: \
  **(With GPU)**
  ```bash
  sudo docker run -it --name applications_ros2 --gpus all --network host --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY ros:humble-ros-core
  ```
  **(Without GPU)**
  ```bash
  sudo docker run -it --name applications_ros2 --network host --volume /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY=unix$DISPLAY ros:humble-ros-core
  ```
- Check whether "ROS2 Foxy" is installed on the environment where you run simulation. If not installed, then run the following command to install ROS2 Foxy.
  ```bash
  chmod +x ./scripts/install_ros2_foxy.sh && ./scripts/install_ros2_foxy.sh
  ``` 
  Then, at the same environment, install ROS1 Bridge running:
  ```bash
  sudo apt-get install ros-foxy-ros1-bridge
  ```

### Enable ROS1 - ROS2 Communication

To enable ROS1<->ROS2 communication, open **a separate terminal** and run the following command. Note that, when you use Docker, the terminal should be opened inside the Docker container where you run simulation.
```bash
chmod +x ./scripts/run_ros1_bridge.sh && ./scripts/run_ros1_bridge.sh
```

While the script is running, you can run your ROS2 nodes to publish/subscribe to the topics of common types from the robot simulation.
