#include "utils.hpp"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

void gazeboThreadFunc() {
  const std::string cmd = "source ./devel/setup.bash && roslaunch gazebo_model_spawn gazebo_startup.launch wname:=earth";
  runBashCommand(cmd);
}

void spawnRobotThreadFunc() {
  const std::string cmd = "source ./devel/setup.bash && roslaunch gazebo_model_spawn model_spawn.launch rname:=lite3 use_xacro:=true use_camera:=false";
  runBashCommand(cmd);
}

void robotStandupThreadFunc() {
  const std::string cmd = "source ./devel/setup.bash && rosrun examples example_lite3_sim ";
  runBashCommand(cmd); 
}


int main() {
  std::vector<std::thread> threads;

  // Add gazebo thread
  threads.emplace_back(&gazeboThreadFunc);
  std::this_thread::sleep_for(1s);

  // Add spawn_robot thread
  threads.emplace_back(&spawnRobotThreadFunc);
  std::this_thread::sleep_for(5s);

  // Add robot standup thread
  threads.emplace_back(&robotStandupThreadFunc);
  std::this_thread::sleep_for(1s);

  for (auto & t : threads) {
    t.join();
  }

  return 0;
}