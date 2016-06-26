// this node is the aggregation simulation of two wheel robots

// aggregation algorithm:


// subscribe to topic "/swarm_sim/two_wheel_robot"
// service client to service "/gazebo/set_joint_properties"

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <math.h>

#include <iostream>  // debug
#include <iomanip>

