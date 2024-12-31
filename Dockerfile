FROM ros:noetic-ros-core
WORKDIR /root/catkin_ws
RUN apt-get update && apt-get install sudo
COPY . /root/catkin_ws
CMD ["bash"]
