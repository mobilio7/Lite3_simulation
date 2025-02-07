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

void spawnRobotThreadFunc(const bool lidar = false) {
  const std::string cmd = std::string("source ./devel/setup.bash && roslaunch gazebo_model_spawn model_spawn.launch lidar:=") + (lidar ? "true" : "false");
  std::cout << "cmd = " << cmd << std::endl;
  runBashCommand(cmd);
}

void robotStandupThreadFunc() {
  const std::string cmd = "source ./devel/setup.bash && rosrun examples example_lite3_sim ";
  runBashCommand(cmd); 
}


int main(int argc, char * argv[]) {
  std::vector<std::thread> threads;

  bool lidar = false;
  if (argc > 1 && std::string(argv[1]) == "lidar:=true") {
    lidar = true;
  }

  // Add gazebo thread
  threads.emplace_back(&gazeboThreadFunc);
  std::this_thread::sleep_for(1s);

  // Add spawn_robot thread
  threads.emplace_back(&spawnRobotThreadFunc, lidar);
  std::this_thread::sleep_for(5s);

  // Add robot standup thread
  threads.emplace_back(&robotStandupThreadFunc);
  std::this_thread::sleep_for(1s);

  for (auto & t : threads) {
    t.join();
  }

  return 0;
}