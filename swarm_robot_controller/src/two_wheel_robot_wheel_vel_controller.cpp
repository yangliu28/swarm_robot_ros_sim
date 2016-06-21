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
#include <string>

const double TOPIC_ACTIVE_PERIOD = 1.0;  // threshold to judge if topics are active
const double CONTROL_PERIOD = 0.001;
const double KP = 1.0;  // proportional control feedback ratio

// global variables
swarm_robot_msg::two_wheel_robot current_robots;
swarm_robot_msg::two_wheel_robot_wheel_vel_cmd current_vel_cmd;
// time stamps for callbacks, used to check topic activity
ros::Time two_wheel_robot_topic_timer, vel_cmd_topic_timer;
bool two_wheel_robot_topic_active, vel_cmd_topic_active;

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

// the callback for getting two wheel robot information
    // the update-to-date robot status will be decided only by this topic
    // because the publisher on velocity command is also rely on this topic
void twoWheelRobotCallback(const swarm_robot_msg::two_wheel_robot& input_two_wheel_robot) {
    // update the time stamp when this callback is invoked the last time
    two_wheel_robot_topic_timer = ros::Time::now();
    current_robots = input_two_wheel_robot;  // update msg in global variable
}

// the callback for receiving wheel velocity command
void twoWheelRobotVelCmdCallback(const swarm_robot_msg::two_wheel_robot_wheel_vel_cmd&
    input_vel_cmd) {
    // update the time stamp when this callback is invoked the last time
    vel_cmd_topic_timer = ros::Time::now();
    current_vel_cmd = input_vel_cmd;  // update msg in global variable
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

    // check if gazebo service apply_joint_effort is ready
    ros::Duration half_sec(0.5);
    bool service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
    if (!service_ready) {
        // service not ready
        while (!service_ready) {
            ROS_INFO("waiting for gazebo service apply_joint_effort");
            half_sec.sleep();
            service_ready = ros::service::exists("/gazebo/apply_joint_effort", true);
        }
    }
    ROS_INFO("gazebo service apply_joint_effort is ready");

    // instantiate a subscriber for "/swarm_sim/two_wheel_robot"
    ros::Subscriber two_wheel_robot_subscriber
        = nh.subscribe("/swarm_sim/two_wheel_robot", 1, twoWheelRobotCallback);

    // instantiate a subscriber for "/swarm_sim/two_wheel_robot_wheel_vel_cmd"
    ros::Subscriber two_wheel_robot_wheel_vel_cmd_subscriber
        = nh.subscribe("/swarm_sim/two_wheel_robot_wheel_vel_cmd", 1, twoWheelRobotVelCmdCallback);

    // instantiate a service client for "/gazebo/apply_joint_effort"
    ros::ServiceClient apply_joint_effort_client
        = nh.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");
    gazebo_msgs::ApplyJointEffort apply_joint_effort_srv_msg;

    // initialize callback timer
    two_wheel_robot_topic_timer = ros::Time::now();
    vel_cmd_topic_timer = ros::Time::now();
    ros::Duration(TOPIC_ACTIVE_PERIOD + 0.1).sleep();  // avoid false judgement of active

    // control wheel velocity loop
    ros::Time timer_now;
    ros::Rate loop_rate(1/CONTROL_PERIOD);
    ros::Duration torque_duration(CONTROL_PERIOD);  // duration for torque to be exerted on wheel
    apply_joint_effort_srv_msg.request.duration = torque_duration;
    while (ros::ok()) {
        // check if two wheel robot and wheel vel cmd topics are active
        timer_now = ros::Time::now();
        if ((timer_now - two_wheel_robot_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD)
            two_wheel_robot_topic_active = true;
        else
            two_wheel_robot_topic_active = false;
        if ((timer_now - vel_cmd_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD)
            vel_cmd_topic_active = true;
        else
            vel_cmd_topic_active = false;

        // 
        if (two_wheel_robot_topic_active) {
            // control wheel velocity only when this topic is ready
            // prepare the velocity commands to be used in the control
            int robot_quantity = current_robots.index.size();
            swarm_robot_msg::two_wheel_robot_wheel_vel_cmd processed_vel_cmd;
            processed_vel_cmd.index = current_robots.index;
            processed_vel_cmd.left_wheel_vel_cmd.resize(robot_quantity);
            processed_vel_cmd.right_wheel_vel_cmd.resize(robot_quantity);
            // get data for processed_vel_cmd from current_vel_cmd
            if (vel_cmd_topic_active) {
                // the velocity command topic is active
                // start mapping for each other
                for (int i=0; i<robot_quantity; i++) {
                    // searching in current_vel_cmd
                    bool found_vel_cmd = false;
                    for (int j=0; j<current_vel_cmd.index.size(); j++) {
                        if (processed_vel_cmd.index[i] == current_vel_cmd.index[j]) {
                            processed_vel_cmd.left_wheel_vel_cmd[i]
                                = current_vel_cmd.left_wheel_vel_cmd[j];
                            processed_vel_cmd.right_wheel_vel_cmd[i]
                                = current_vel_cmd.right_wheel_vel_cmd[j];
                            found_vel_cmd = true;
                            break;
                        }
                    }
                    // fail to find corresponding velocity command
                    // this happens when new robot is added in gazebo, and vel_cmd is not updated
                    if (!found_vel_cmd) {
                        // set vel cmd to zero, not moving
                        processed_vel_cmd.left_wheel_vel_cmd[i] = 0.0;
                        processed_vel_cmd.right_wheel_vel_cmd[i] = 0.0;
                    }
                }
            }
            else {
                // the velocity command topic is not active
                for (int i=0; i<robot_quantity; i++) {
                    // set vel cmd to zero
                    processed_vel_cmd.left_wheel_vel_cmd[i] = 0.0;
                    processed_vel_cmd.right_wheel_vel_cmd[i] = 0.0;
                }
            }
            // control wheel velocity by apply joint effort
            for (int i=0; i<robot_quantity; i++) {
                apply_joint_effort_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::left_motor";
                apply_joint_effort_srv_msg.request.effort = 
            }


        }




// not finished, turn to another velocity control method
// service server to tune proportional feedback parameter





        loop_rate.sleep();
        ros::spinOnce();  // let the global variables update
    }

    return 0;
}




