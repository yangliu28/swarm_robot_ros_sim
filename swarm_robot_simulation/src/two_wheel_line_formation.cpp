// this program is for the line formation simulation of two wheel robot as a swarm robot

// communication includes:
// subscribe to topic "swarm_robot_poses" to get current robot swarm_robot_poses
// create an action client to "two_wheel_traj_action"

// line formation algorithm:
// 

#include <ros/ros.h>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
bool g_robot_poses_cb_started = false;

// simulation control parameters
const double sensing_range = 3.0;  // the upper limit for distance with another robot


// two wheel robot specification, really should get these values in another way
const double half_wheel_dist = 0.0177;
const double wheel_radius = 0.015;
const double wheel_speed = 2.0;  // rad*s-1, for the calculation of time cost of the action

// callback for message from topic "swarm_robot_poses"
void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!g_robot_poses_cb_started)  // first time to be invoked
        g_robot_poses_cb_started = true;
    g_robot_x = message_holder.x;
    g_robot_y = message_holder.y;
    g_robot_angle = message_holder.angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "dispersion");
    ros::NodeHandle nh;

    // get initialization message of robot swarm from parameter server
    std::string robot_model_name;
    int robot_quantity;
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // get settings for this simulation from private parameter
    // parameter: wheel_speed
    bool get_wheel_speed = nh.getParam("wheel_speed", wheel_speed);
    if (get_wheel_speed)
        ROS_INFO_STREAM("using wheel_speed passed in: " << wheel_speed);
    else
        ROS_INFO_STREAM("using default wheel_speed: " << wheel_speed);

    // initialize a subscriber to topic "swarm_robot_poses"
    ros::Subscriber swarm_robot_poses_subscriber = nh.subscribe("swarm_robot_poses", 1, swarmRobotPosesCb);

    // make sure topics "swarm_robot_poses" are active
    while (!g_robot_poses_cb_started) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    std::cout << "topic message from swarm_robot_poses is ready" << std::endl;

    // initialize an action client
    actionlib::SimpleActionClient<swarm_robot_action::swarm_robot_trajAction> action_client(
        "two_wheel_traj_action", true);
    swarm_robot_action::swarm_robot_trajGoal goal;  // instantiate a goal message

    // try to connect the client to action server
    bool server_exist = action_client.waitForServer(ros::Duration(5.0));
    ros::Duration(1.0).sleep();
    while (!server_exist) {
        ROS_WARN("could not connect to server; retrying");
        bool server_exist = action_client.waitForServer(ros::Duration(1.0));
        ros::Duration(1.0).sleep();
    }
    // if here, then connected to the server
    ROS_INFO("connected to action server");








    return 0;
}

