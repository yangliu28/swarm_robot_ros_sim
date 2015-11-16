// this program is for the dispersion simulation of two wheel robot as a swarm robot

// communication including:
// subscribe to topic "swarm_robot_poses" to get current robot poses
// create an action client to "two_wheel_traj_action"

#include <ros/ros.h>
#include <math.h>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
std::vector<double> g_robot_angle;
bool g_robot_poses_cb_started = false;

// simulation control parameters
const double spring_length = 0.5;  // spring length when not compressed or extended
const double upper_limit_ratio = 0.50;  // upper limit part relative to spring length
const double feedback_ratio = 0.382;  // smaller than 1 to make it stable, golden ratio ;)

const double upper_limit = spring_length * (1 + upper_limit_ratio);

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

    // all the above are copied from example_wheel_traj_interpolator_ac

    double distance[robot_quantity][robot_quantity];  // two dimensional array to store distance
    double distance_sort[robot_quantity][robot_quantity];
    int index_sort[robot_quantity][robot_quantity];
    while (ros::ok()) {
        ros::spinOnce();  // update robot positions

        // calculate the distance between robots
        for (int i=0; i<robot_quantity; i++) {
            for (int j=i; j<robot_quantity; j++) {
                distance[i][j] = sqrt(pow(g_robot_x[i] - g_robot_x[j], 2), 
                    pow(g_robot_y[i] - g_robot_y[j], 2));
                if (i != j)  // not in diagonal axis
                    distance[j][i] = distance[i][j];  // symmetrical matrix
            }
        }

        // sort these distance
        distance_sort = distance;
    }

}

