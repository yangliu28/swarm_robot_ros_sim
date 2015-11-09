// 

// Communication includes: listening to topic "two_wheel_poses" and "two_wheel_poses_cmd".
// target is to eliminate the errors between command and real-time positions

#include <ros/ros.h>
// important messages
#include <swarm_robot_msgs/two_wheel_poses.h>  // for the topic communication
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>



