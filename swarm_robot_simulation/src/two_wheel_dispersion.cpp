// this program is for the dispersion simulation of two wheel robot as a swarm robot

// communication including:
// subscribe to topic "swarm_robot_poses" to get current robot poses
// create an action client to "two_wheel_traj_action"

// impact avoiding is not implemented here, what's the better way to do that?

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
double spring_length = 0.7;  // spring length, may change from parameter server
const double upper_limit_ratio = 0.30;  // upper limit part relative to spring length
const double upper_limit = spring_length * (1 + upper_limit_ratio);
const double feedback_ratio = 0.382/2.0;  // smaller than 1 to make it stable, golden ratio ;)
// half the feedback_ratio to alleviate vibration

// two wheel robot specification, really should get these values in another way
const double half_wheel_dist = 0.0177;
const double wheel_radius = 0.015;
double wheel_speed = 2.0;  // rad*s-1, for time cost of the action, may change also

// callback for message from topic "swarm_robot_poses"
void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!g_robot_poses_cb_started)  // first time to be invoked
        g_robot_poses_cb_started = true;
    g_robot_x = message_holder.x;
    g_robot_y = message_holder.y;
    g_robot_angle = message_holder.angle;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_dispersion");
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
    // parameter: spring_length
    bool get_spring_length = nh.getParam("spring_length", spring_length);
    if (get_spring_length)
        ROS_INFO_STREAM("using spring_length passed in: " << spring_length);
    else
        ROS_INFO_STREAM("using default spring_length: " << spring_length);
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

    // variables for the loop
    double distance[robot_quantity][robot_quantity];  // two dimensional array to store distance
    double distance_sort[robot_quantity][robot_quantity];
    int index_sort[robot_quantity][robot_quantity];
    // the loop of optimizing robot position for dispersion
    int iteration_index = 0;
    while (ros::ok()) {
        iteration_index = iteration_index + 1;
        ROS_INFO_STREAM("");  // blank line
        ROS_INFO_STREAM("iteration index: " << iteration_index);  // iteration index

        ros::spinOnce();  // let robot positions update, will be used later

        // calculate the distance between robots
        for (int i=0; i<robot_quantity; i++) {
            for (int j=i; j<robot_quantity; j++) {
                distance[i][j] = sqrt(pow(g_robot_x[i] - g_robot_x[j], 2) +
                    pow(g_robot_y[i] - g_robot_y[j], 2));
                if (i != j)  // not in diagonal axis
                    distance[j][i] = distance[i][j];  // symmetrical matrix
            }
        }

        // initialize distance_sort and index_sort
        for (int i=0; i<robot_quantity; i++)
            for (int j=0; j<robot_quantity; j++) {
                distance_sort[i][j] = distance[i][j];
                index_sort[i][j] = j;
            }
        // sort the distances and get sorted index
        double distance_temp;
        int index_temp;
        for (int i=0; i<robot_quantity; i++) {
            for (int j=0; j<robot_quantity-1; j++) {
                for (int k=0; k<robot_quantity-1-j; k++) {
                    if (distance_sort[i][k] > distance_sort[i][k+1]) {
                        // switch value between distance_sort[i][k] and distance_sort[i][k+1] 
                        distance_temp = distance_sort[i][k];
                        distance_sort[i][k] = distance_sort[i][k+1];
                        distance_sort[i][k+1] = distance_temp;
                        // switch value between index_sort[i][k] and index[i][k+1]
                        index_temp = index_sort[i][k];
                        index_sort[i][k] = index_sort[i][k+1];
                        index_sort[i][k+1] = index_temp;
                    }
                }
            }
        }

        // check whether the closest is itself
        for (int i=0; i<robot_quantity; i++) {
            if (index_sort[i][0] != i) {
                ROS_WARN("error in the sorting of robot distance");
                return 0;
            }
        }

        // find all the neighbors that will be used in the force feedback control
        // the algorithm used here is the same with the algorithm implemented in matlab before
        // no matter how far away one robot is with others, 3 closest neighbors will be counted
        // no matter how close one robot is with others, 6 closest neighbors at most will be counted
        // which means there should be at least 6 robots
        int neighbor_num[robot_quantity];  // the number of valid neighbors
        for (int i=0; i<robot_quantity; i++) {
            neighbor_num[i] = 3;  // there are at least 3 neighbors
            while (true) {
                neighbor_num[i] = neighbor_num[i] + 1;
                if (neighbor_num[i] <= 6)
                    if (distance_sort[i][neighbor_num[i]] < upper_limit)
                        continue;
                    else {
                        neighbor_num[i] = neighbor_num[i] - 1;
                        break;
                    }
                else {
                    neighbor_num[i] = neighbor_num[i] - 1;
                    break;
                }
            }
            // ROS_INFO_STREAM("neighbor number of " << i << ": " << neighbor_num[i]);
        }
        // for (int i=0; i<robot_quantity; i++) {
        //     neighbor_num[i] = 3;  // there are at least 3 neighbors
        //     while (distance_sort[i][neighbor_num[i]+1] < upper_limit && neighbor_num[i] <= 6)
        //         neighbor_num[i] = neighbor_num[i] + 1;
        // }

        // calculate displacement of each robot
        double displacement_x[robot_quantity];  // displacement in x
        double displacement_y[robot_quantity];  // displacement in y
        double distance_diff;
        for (int i=0; i<robot_quantity; i++) {
            displacement_x[i] = 0.0;
            displacement_y[i] = 0.0;
            double distance_diff_sum = 0.0;
            for (int j=1; j<=neighbor_num[i]; j++) {
                distance_diff = distance_sort[i][j] - spring_length;
                // add the spring effect of all the neighbors
                displacement_x[i] = displacement_x[i] + feedback_ratio * distance_diff * 
                    (g_robot_x[index_sort[i][j]] - g_robot_x[i]) / distance_sort[i][j];
                displacement_y[i] = displacement_y[i] + feedback_ratio * distance_diff * 
                    (g_robot_y[index_sort[i][j]] - g_robot_y[i]) / distance_sort[i][j];
            }
        }

        // prepare the goal message
        double displacement_average = 0.0;  // for the calculation of time cost
        goal.x.resize(robot_quantity);  // important here, otherwise runtime error
        goal.y.resize(robot_quantity);  // important here, otherwise runtime error
        for (int i=0; i<robot_quantity; i++) {
            goal.x[i] = g_robot_x[i] + displacement_x[i];
            goal.y[i] = g_robot_y[i] + displacement_y[i];
            displacement_average = displacement_average +
                sqrt(pow(displacement_x[i], 2) + pow(displacement_y[i], 2));
        }
        // this is the real average displacement
        displacement_average = displacement_average / robot_quantity;
        // use displacement_average to represent time spent on line movement
        // use M_PI/3 to represent time spent on self rotating
        goal.time_cost = displacement_average / wheel_radius / wheel_speed + 
            M_PI/3 * half_wheel_dist / wheel_radius / wheel_speed;
        ROS_INFO_STREAM("time cost for action: " << goal.time_cost << "(second)");

        // send out goal
        action_client.sendGoal(goal);
        // wait for expected duration plus some tolerance (2 seconds)
        bool finish_before_timeout = action_client.waitForResult(ros::Duration(goal.time_cost + 2.0));
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

