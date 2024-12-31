cmake_minimum_required(VERSION 3.0.2)
project(run_high_level)
add_compile_options(-std=c++14)
find_package(catkin REQUIRED)
find_package(Threads REQUIRED)
catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(start_simulator src/start_simulator.cpp src/utils.cpp)
target_link_libraries(start_simulator Threads::Threads ${catkin_LIBRARIES})
