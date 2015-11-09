// two_wheel_pos_action_server.h header file //
/// yangliu; Nov, 2015.
/// This is a library for low-level control of wheel positions of two wheel robot.
/// Communication includes: listening to topic 

#ifndef TWO_WHEEL_POS_ACTION_SERVER_H_
#define TWO_WHEEL_POS_ACTION_SERVER_H_

#include <ros/ros.h>
// important messages
#include <swarm_robot_msgs/two_wheel_poses.h>  // for the topic communication
#include <gazebo_msgs/ApplyJointEffort.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <sensor_msgs/JointState.h>

class TwoWheelPos {
public:
    TwoWheelPos();  // constructor
    void get
};

