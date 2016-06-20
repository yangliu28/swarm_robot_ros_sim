// the test program for two_wheel_traj_interpolator_as

// subscribe to topic "swarm_robot_poses" to get current robot poses
// get robot poses cmd manually from terminal
// sending these messages as a goal to action server "two_wheel_traj_action"

#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
std::vector<double> g_robot_angle;
bool g_robot_poses_cb_started = false;

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

// callback for message from topic "swarm_robot_poses"
void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!g_robot_poses_cb_started)  // first time to be invoked
        g_robot_poses_cb_started = true;
    g_robot_x = message_holder.x;
    g_robot_y = message_holder.y;
    g_robot_angle = message_holder.angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_wheel_traj_interpolator_ac");
    ros::NodeHandle nh;

    // get initialization message of robot swarm from parameter server
    std::string robot_model_name;
    int robot_quantity;
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // resize global variable
    g_robot_x.resize(robot_quantity);
    g_robot_y.resize(robot_quantity);
    g_robot_angle.resize(robot_quantity);

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

    // variables that will be used in the loop
    std::string in_index;
    int in_index_int;
    double in_x_incre;
    double in_y_incre;
    double in_time_cost;
    while (ros::ok()) {
        std::cout << std::endl;
        // get the index of the robot, chance to quit
        std::cout << "enter the index of the robot (0-based)" << std::endl
            << "(should be between 0 and " << intToString(robot_quantity - 1) << ", x to quit): ";
        std::cin >> in_index;
        if (in_index.compare("x") == 0)
            return 0;
        in_index_int = std::atoi(in_index.c_str());  // convert string to int

        ros::spinOnce();  // allow robot poses updated
        // print out specified robot poses
        std::cout << "    x coordinate is " << g_robot_x[in_index_int] << std::endl;
        std::cout << "    y coordinate is " << g_robot_y[in_index_int] << std::endl;
        // get position command of this specified robot
        std::cout << "enter the increment of x coordinate: ";
        std::cin >> in_x_incre;
        std::cout << "enter the increment of y coordinate: ";
        std::cin >> in_y_incre;
        // get time cost for this movement action
        std::cout << "enter the time cost for the action (second): ";
        std::cin >> in_time_cost;

        // prepare goal message
        goal.x = g_robot_x;
        goal.y = g_robot_y;
        goal.x[in_index_int] = goal.x[in_index_int] + in_x_incre;
        goal.y[in_index_int] = goal.y[in_index_int] + in_y_incre;
        goal.time_cost = in_time_cost;
        // send out the goal
        action_client.sendGoal(goal);
        // wait for expected duration plus some tolerance (2 seconds)
        bool finish_before_timeout = action_client.waitForResult(ros::Duration(in_time_cost + 2.0));
        if (!finish_before_timeout) {
            ROS_WARN("this action is not done...timeout");
            return 0;
        }
        else {
            ROS_INFO("this action is done.");
        }
    }
    return 0;
}

