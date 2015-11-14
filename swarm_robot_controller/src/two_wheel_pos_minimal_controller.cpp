// the minimal wheel pos controller for two wheel swarm robot

// Communication includes: listening to topics "two_wheel_poses" and "two_wheel_poses_cmd".
// target is to eliminate the errors between command and real-time positions
// by apply joint effort to the wheel joint in gazebo

// using a service to tune the kp&kv from the terminal

// the robot model of two_wheel_robot has been modified that positive wheel increment 
// of two wheels will both pushing the robot forward, and vice versa

#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <swarm_robot_msgs/two_wheel_poses.h>
#include <gazebo_msgs/ApplyJointEffort.h>
#include <swarm_robot_srv/kpkv_msg.h>

// constants, change here
double dt = 0.01; // sample time for the controller
// used in the frequency control of the torque controller
// used in the apply joint effort service message as duration

// global variable
double g_kp = 0.0001;  // already tuned for two_wheel_robot
double g_kv = 0.00003;  // already tuned for two_wheel_robot
std::vector<double> g_left_wheel_poses;
std::vector<double> g_left_wheel_vels;
std::vector<double> g_left_wheel_poses_cmd;  // no need to specify velocity in command
std::vector<double> g_right_wheel_poses;
std::vector<double> g_right_wheel_vels;
std::vector<double> g_right_wheel_poses_cmd;
bool g_poses_callback_started = false;  // poses callback has been invoked at least once
bool g_poses_cmd_callback_started = false;  // poses cmd callback has been invoked at least once

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

// callback for message from topic "two_wheel_poses"
void twoWheelPosesCallback(const swarm_robot_msgs::two_wheel_poses& message_holder) {
    if (!g_poses_callback_started)  // first time to be invoked
        g_poses_callback_started = true;
    g_left_wheel_poses = message_holder.left_wheel_pos;
    g_left_wheel_vels = message_holder.left_wheel_vel;
    g_right_wheel_poses = message_holder.right_wheel_pos;
    g_right_wheel_vels = message_holder.right_wheel_vel;
    // check if cmd callback started yet
    if (!g_poses_cmd_callback_started) {
        // let the cmd poses be the same as current poses
        g_left_wheel_poses_cmd = g_left_wheel_poses;
        g_right_wheel_poses_cmd = g_right_wheel_poses;
    }
}

// callback for message from topic "two_wheel_poses_cmd"
void twoWheelPosesCmdCallback(const swarm_robot_msgs::two_wheel_poses& message_holder) {
    if (!g_poses_cmd_callback_started)  // first time to be invoked
        g_poses_cmd_callback_started = true;
    g_left_wheel_poses_cmd = message_holder.left_wheel_pos;
    g_right_wheel_poses_cmd = message_holder.right_wheel_pos;
}

// callback function for the service "two_wheel_kpkv"
bool twoWheelKpkvCallback(swarm_robot_srv::kpkv_msgRequest& request, 
    swarm_robot_srv::kpkv_msgResponse& response)
{
    ROS_INFO("kpkvCallback activated");
    g_kp = request.kp;
    g_kv = request.kv;
    response.setting_is_done = true;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_pos_minimal_controller");
    ros::NodeHandle nh;
    ros::Duration duration(dt);  // duration for the torque to be exerted in the controller

    // get initialization message of robot swarm from parameter server
    std::string robot_model_name;
    int robot_quantity;
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // resize global variables
    g_left_wheel_poses.resize(robot_quantity);
    g_left_wheel_vels.resize(robot_quantity);
    g_left_wheel_poses_cmd.resize(robot_quantity);
    g_right_wheel_poses.resize(robot_quantity);
    g_right_wheel_vels.resize(robot_quantity);
    g_right_wheel_poses_cmd.resize(robot_quantity);

    // initialize a subscriber for "two_wheel_poses"
    ros::Subscriber two_wheel_poses_subscriber = nh.subscribe("two_wheel_poses", 1, twoWheelPosesCallback);
    // initialize a subscriber for "two_wheel_poses_cmd"
    ros::Subscriber two_wheel_poses_cmd_subscriber =
        nh.subscribe("two_wheel_poses_cmd", 1, twoWheelPosesCmdCallback);

    while (!g_poses_callback_started) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    ROS_INFO("topic message from two_wheel_poses is ready");

    // initialize a service client to apply joint effort
    ros::ServiceClient set_wheel_torque_client = nh.serviceClient<gazebo_msgs::ApplyJointEffort>(
        "/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort set_wheel_torque_srv_msg;  // service message
    set_wheel_torque_srv_msg.request.effort = 0.0;
    set_wheel_torque_srv_msg.request.duration = duration;

    // make sure apply_joint_effort service is ready
    ros::Duration half_sec(0.5);
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/apply_joint_effort",true);
        ROS_INFO("waiting for apply_joint_effort service");
        half_sec.sleep();
    }
    ROS_INFO("apply_joint_effort service is ready");

    // initialize a service to change g_kp & g_kv, service name is "two_wheel_kpkv"
    ros::ServiceServer kpkv_service = nh.advertiseService("two_wheel_kpkv", twoWheelKpkvCallback);

    // prepare control variables
    ros::Rate rate_timer(1/dt);
    double left_wheel_pos_err;
    double left_wheel_torque;
    double right_wheel_pos_err;
    double right_wheel_torque;
    std::string left_motor_name;  // name of left motor
    std::string right_motor_name;  // name of right motor
    // control loop
    while (ros::ok()) {
        // ROS_INFO("in the loop...");  // for debug
        // apply joint effort loop
        for (int i=0; i<robot_quantity; i++) {
            // calculate the error
            left_wheel_pos_err = g_left_wheel_poses_cmd[i] - g_left_wheel_poses[i];
            right_wheel_pos_err = g_right_wheel_poses_cmd[i] - g_right_wheel_poses[i];

            // do not watch for periodicity, that is, making error in (-M_PI, M_PI)
            // because wheels are continuous rotating, angle could not always be in that range

            // calculate the torque
            left_wheel_torque = g_kp * left_wheel_pos_err - g_kv * g_left_wheel_vels[i];
            right_wheel_torque = g_kp * right_wheel_pos_err - g_kv * g_right_wheel_vels[i];
            // left_wheel_torque = 0.00001;  // for debug

            std::string s_index = intToString(i);
            // actually cheat here, no better way to get the key words "left_motor" and "right_motor"
            left_motor_name = robot_model_name + "_" + s_index + "::left_motor";
            right_motor_name = robot_model_name + "_" + s_index + "::right_motor";

            // set left wheel torque
            set_wheel_torque_srv_msg.request.joint_name = left_motor_name;
            set_wheel_torque_srv_msg.request.effort = left_wheel_torque;
            // ROS_INFO_STREAM("left_wheel_torque " << left_wheel_torque);  // for debug
            set_wheel_torque_client.call(set_wheel_torque_srv_msg);

            // set right wheel torque
            set_wheel_torque_srv_msg.request.joint_name = right_motor_name;
            set_wheel_torque_srv_msg.request.effort = right_wheel_torque;
            // ROS_INFO_STREAM("right_wheel_torque " << right_wheel_torque);  // for debug
            set_wheel_torque_client.call(set_wheel_torque_srv_msg);
        }

        // frequency control
        ros::spinOnce();  // update pos_cmd
        rate_timer.sleep();
    }
}

