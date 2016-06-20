// the test program for the minimal wheel pos controller

// get wheel poses cmd from terminal
// publishing these repacked information to topic "two_wheel_poses_cmd"
// (because there is too much typing by publish on this topic from terminal)
// also subscribe to topic "two_wheel_poses" to get current wheel poses

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <swarm_robot_msgs/two_wheel_poses.h>

// global variables
std::vector<double> g_left_wheel_poses;
std::vector<double> g_right_wheel_poses;
bool g_poses_callback_started = false;  // whether twoWheelPosesCallback has been invoked the first time

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

// callback for message from topic "two_wheel_poses"
void twoWheelPosesCallback(const swarm_robot_msgs::two_wheel_poses& message_holder) {
    if (!g_poses_callback_started)
        g_poses_callback_started = true;
    g_left_wheel_poses = message_holder.left_wheel_pos;
    g_right_wheel_poses = message_holder.right_wheel_pos;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "example_wheel_pos_cmd_service");
    ros::NodeHandle nh;

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
    g_right_wheel_poses.resize(robot_quantity);

    // initialize a subscriber for topic "two_wheel_poses"
    // this node should able to know the current wheel position
    ros::Subscriber two_wheel_poses_subscriber = nh.subscribe("two_wheel_poses", 1, twoWheelPosesCallback);

    // initialize a publisher for topic "two_wheel_poses_cmd"
    ros::Publisher two_wheel_poses_cmd_publisher = 
        nh.advertise<swarm_robot_msgs::two_wheel_poses>("two_wheel_poses_cmd", 1);
    swarm_robot_msgs::two_wheel_poses cmd_msg;

    // wait for the twoWheelPosesCallback to be invoked first time
    while (!g_poses_callback_started) {
        ros::Duration(0.5).sleep();  // sleep for 0.5 second
        ros::spinOnce();  // let the global variables to be updated
    }
    std::cout << "topic message from two_wheel_poses is ready" << std::endl;  // message is ready

    // variables that will be used in the loop
    std::string in_index;
    int in_index_int;
    double in_left_wheel_incre;
    double in_right_wheel_incre;
    // get command loop
    while (ros::ok()) {
        std::cout << std::endl;
        // get the index of the robot, chance to quit
        std::cout << "enter the index of the robot (0-based)" << std::endl
            << "(should be between 0 and " << intToString(robot_quantity - 1) << ", x to quit): ";
        std::cin >> in_index;
        if (in_index.compare("x") == 0)
            return 0;
        in_index_int = std::atoi(in_index.c_str());  // convert string to int

        ros::spinOnce();  // necessary here, left wheel poses update again
        // print out wheel position of specified robot
        std::cout << "    left wheel position is " << g_left_wheel_poses[in_index_int] << std::endl;
        std::cout << "    right wheel position is " << g_right_wheel_poses[in_index_int] << std::endl;
        // get the position command of this indexed robot
        std::cout << "enter the increment of left wheel position: ";
        std::cin >> in_left_wheel_incre;
        std::cout << "enter the increment of right wheel position: ";
        std::cin >> in_right_wheel_incre;

        // prepare topic message
        cmd_msg.left_wheel_pos = g_left_wheel_poses;
        cmd_msg.right_wheel_pos = g_right_wheel_poses;
        cmd_msg.left_wheel_pos[in_index_int] = cmd_msg.left_wheel_pos[in_index_int] + in_left_wheel_incre;
        cmd_msg.right_wheel_pos[in_index_int] = cmd_msg.right_wheel_pos[in_index_int] + in_right_wheel_incre;
        // publish this topic message
        two_wheel_poses_cmd_publisher.publish(cmd_msg);

        ros::spinOnce();
    }
    return 0;
}

