// the test program for two_wheel_traj_interpolator_as

// get robot poses cmd from terminal
// sending these messages as a goal to action server "two_wheel_traj_action"
// also subscribe to topic "swarm_robot_poses" to get current wheel poses

#include <ros/ros.h>
#include <vector>
#include <swarm_robot_msgs/swarm_robot_poses.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
std::vector<double> g_robot_angle;



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



}