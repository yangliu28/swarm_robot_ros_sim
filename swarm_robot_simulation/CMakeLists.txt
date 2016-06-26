cmake_minimum_required(VERSION 2.8.3)
project(swarm_robot_simulation)

find_package(catkin_simple REQUIRED)

catkin_simple()

# example boost usage
# find_package(Boost REQUIRED COMPONENTS system thread)

# C++0x support - not quite the same as final C++11!
# SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Libraries
# cs_add_library(my_lib src/my_lib.cpp)   

# Executables
# cs_add_executable(two_wheel_dispersion src/two_wheel_dispersion.cpp)
# cs_add_executable(two_wheel_line_formation src/two_wheel_line_formation.cpp)
cs_add_executable(two_wheel_robot_dispersion src/two_wheel_robot_dispersion.cpp)
cs_add_executable(two_wheel_robot_line_formation src/two_wheel_robot_line_formation.cpp)
cs_add_executable(two_wheel_robot_aggregation src/two_wheel_robot_aggregation.cpp)

# target_link_libraries(two_wheel_dispersion ${catkin_LIBRARIES})
# target_link_libraries(two_wheel_line_formation ${catkin_LIBRARIES})

cs_install()
cs_export()
