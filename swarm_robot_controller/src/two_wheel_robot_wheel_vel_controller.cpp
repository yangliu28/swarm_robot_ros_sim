// this node control the wheel velocity of all present two wheel robots

// this node should be able to run independently of the manager node

// ros communication:
    // subscribe to two wheel robot information topic "/swarm_sim/two_wheel_robot"
    // service client to gazebo service "/gazebo/apply_joint_effort"
    // subscribe to wheel vel command topic "/swarm_sim/two_wheel_robot_wheel_vel_cmd"

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <swarm_robot_msg/two_wheel_robot_wheel_vel_cmd.h>
#include <gazebo_msgs/ApplyJointEffort.h>

// global variables
swarm_robot_msg::two_wheel_robot current_robots;


// the callback for getting two wheel robot information
void twoWheelRobotCallback(const swarm_robot_msg::two_wheel_robot& current_two_wheel_robot) {
    // 
}

// the callback for receiving wheel velocity command
void twoWheelRobotVelCmdCallback(const swarm_robot_msg::two_wheel_robot_wheel_vel_cmd& current_vel_cmd) {
    // 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_robot_wheel_vel_controller");
    ros::NodeHandle nh;

    // handshake with robot name in parameter server
    std::string robot_name;
    bool get_name;
    get_name = nh.getParam("/swarm_sim/robot_name", robot_name);
    if (!get_name) {
        ROS_ERROR("simulation environment(parameters) is not set");
        return 0;
    }
    if (robot_name != "two_wheel_robot") {
        ROS_ERROR("wrong robot according to parameter server");
        return 0;
    }

    // instantiate a subscriber for "/swarm_sim/two_wheel_robot"
    ros::Subscriber two_wheel_robot_subscriber
        = nh.subscribe("/swarm_sim/two_wheel_robot", 1, twoWheelRobotCallback);

    // instantiate a service client for "/gazebo/apply_joint_effort"
    ros::ServiceClient apply_joint_effort_client
        = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_joint_effort_srv_msg;

    // instantiate a subscriber for "/swarm_sim/two_wheel_robot_wheel_vel_cmd"
    ros::Subscriber two_wheel_robot_wheel_vel_cmd_subscriber
        = nh.subscribe("/swarm_sim/two_wheel_robot_wheel_vel_cmd", 1, twoWheelRobotVelCmdCallback);




}



