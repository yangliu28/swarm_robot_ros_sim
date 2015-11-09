cmake_minimum_required(VERSION 2.8.3)
project(swarm_robot_controller)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
cs_add_executable(swarm_robot_pos_publisher src/swarm_robot_pos_publisher.cpp)
cs_add_executable(two_wheel_pos_publisher src/two_wheel_pos_publisher.cpp)
cs_add_executable(two_wheel_pos_minimal_controller src/two_wheel_pos_minimal_controller.cpp)

target_link_libraries(swarm_robot_pos_publisher ${catkin_LIBRARIES})
target_link_libraries(two_wheel_pos_publisher ${catkin_LIBRARIES})
target_link_libraries(two_wheel_pos_minimal_controller ${catkin_LIBRARIES})

cs_install()
cs_export()
