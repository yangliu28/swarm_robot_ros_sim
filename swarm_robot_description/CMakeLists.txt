cmake_minimum_required(VERSION 2.8.3)
project(swarm_robot_description)

find_package(catkin_simple REQUIRED)

catkin_simple()

# find_package(cmake_modules REQUIRED)
# find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR})
# add_definitions(${EIGEN_DEFINITIONS})


# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(example src/example.cpp)
# target_link_library(example my_lib)
# cs_add_executable(swarm_spawner_gazebo_client src/swarm_spawner_gazebo_client.cpp)
cs_add_executable(two_wheel_robot_manager src/two_wheel_robot_manager.cpp)
cs_add_executable(two_wheel_robot_batch_add src/two_wheel_robot_batch_add.cpp)
cs_add_executable(two_wheel_robot_line_distribution src/two_wheel_robot_line_distribution.cpp)

cs_install()
cs_export()
