// this node is the line formation of two wheel robots

// subscribe to topic "/swarm_sim/two_wheel_robot"
// service client to service "/gazebo/set_joint_properties"

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <math.h>

// flow control parameters
const double TOPIC_ACTIVE_PERIOD = 1.0;  // threshold to tell if a topic is active
const double CONTROL_PERIOD = 0.001;

// global variables
swarm_robot_msg::two_wheel_robot current_robots;
// time stamp for callback, used to check topic activity
ros::Time two_wheel_robot_topic_timer;

// callback for getting two wheel robot information
void twoWheelRobotCallback(const swarm_robot_msg::two_wheel_robot& input_msg) {
    // update the time stamp every time the callback is invoked
    two_wheel_robot_topic_timer = ros::Time::now();
    current_robots = input_msg;  // update in global variables
}

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_robot_line_formation");
    ros::NodeHandle nh("~");

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

    // check if gazebo service set_joint_properties is ready
    ros::Duration half_sec(0.5);
    bool service_ready = ros::service::exists("/gazebo/set_joint_properties", true);
    if (!service_ready) {
        // service not ready
        while (!service_ready) {
            ROS_INFO("waiting for gazebo service set_joint_properties");
            half_sec.sleep();
            service_ready = ros::service::exists("/gazebo/set_joint_properties", true);
        }
    }
    ROS_INFO("gazebo service set_joint_properties is ready");

    // get the settings for this simulation from private parameter

    // instantiate a subscriber to topic "/swarm_sim/two_wheel_robot"
    ros::Subscriber two_wheel_robot_subscriber
        = nh.subscribe("/swarm_sim/two_wheel_robot", 1, twoWheelRobotCallback);

    // instantiate a service client for "/gazebo/set_joint_properties"
    ros::ServiceClient set_joint_properties_client
        = nh.serviceClient<gazebo_msgs::SetJointProperties>("/gazebo/set_joint_properties");
    gazebo_msgs::SetJointProperties set_joint_properties_srv_msg;
    // usually this is enough for fast response
    set_joint_properties_srv_msg.request.ode_joint_config.fmax.resize(1);  // in case of segmentation error
    set_joint_properties_srv_msg.request.ode_joint_config.vel.resize(1);
    set_joint_properties_srv_msg.request.ode_joint_config.fmax[0] = 1.0;

    // line formation loop
    ros::Time timer_now;
    ros::Time loop_last_timer = ros::Time::now();  // use to examine loop rate
    double loops_sum_period;
    int loop_count = 0;
    ros::Rate loop_rate(1.0/CONTROL_PERIOD);  // use to control loop rate
    bool stop_all_robot_once = false;  // stop all robots for one time when topic is inactive
    while (ros::ok()) {
        int robot_quantity = current_robots.index.size();

        // check if two wheel robot topic is active
        timer_now = ros::Time::now();
        if ((timer_now - two_wheel_robot_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD) {
            // the topic is been actively published
            // ROS_WARN("topic is active");

            // examine loop rate
            bool print_debug_msg = false;  // whether to print debug msg elsewhere
            loop_count = loop_count + 1;
            if (loop_count >= 10) {
                // at least 10 counts when averaging frequency
                loops_sum_period = (timer_now - loop_last_timer).toSec();
                if (loops_sum_period > 1.0) {
                    // interval should also be bigger than 1.0 sec
                    // average the frequency on past loops
                    std::cout << "loop rate when controller is on is "
                        << ((double)loop_count)/loops_sum_period << std::endl;
                    loop_count = 0;  // reset loop count
                    loop_last_timer = timer_now;  // update last timer
                    print_debug_msg = true;
                }
            }

            // set the stop_all_robot_once flag, prepare when topic out of active state
            stop_all_robot_once = true;

            // 1.calculate distance between any two robots
            double distance[robot_quantity][robot_quantity];
            for (int i=0; i<robot_quantity; i++) {
                for (int j=i; j<robot_quantity; j++) {
                    if (i == j) {
                        // zero for the diagonal
                        distance[i][j] = 0;
                    }
                    else {
                        distance[i][j] = sqrt(pow(current_robots.x[i] - current_robots.x[j], 2)
                            + pow(current_robots.y[i] - current_robots.y[j], 2));
                        // symmetrical matrix, copy the other side
                        distance[j][i] = distance[i][j];
                    }
                }
            }







        }
        else {
            // the topic is not active, meaning two wheel robot manager node is down
            // ROS_WARN("when topic is inactive");

            if (stop_all_robot_once) {
                // set wheel velocity of all robots to zero according to last topic update
                bool call_service;
                for (int i=0; i<robot_quantity; i++) {
                    // left wheel
                    set_joint_properties_srv_msg.request.joint_name
                        = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::left_motor";
                    set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = 0.0;
                    call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                    if (call_service) {
                        if (!set_joint_properties_srv_msg.response.success)
                            // possibly the robot not found
                            ROS_WARN("the robot model not found when reset left wheel vel");
                    }
                    else
                        ROS_ERROR("fail to connect with gazebo server when reset left wheel vel");
                    // right wheel
                    set_joint_properties_srv_msg.request.joint_name
                        = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::right_motor";
                    set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = 0.0;
                    call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                    if (call_service) {
                        if (!set_joint_properties_srv_msg.response.success)
                            // possibly the robot not found
                            ROS_WARN("the robot model not found when reset right wheel vel");
                    }
                    else
                        ROS_ERROR("fail to connect with gazebo server when reset right wheel vel");
                }
                // reset the stop_all_robot_once flag
                stop_all_robot_once = false;
            }
        }

        loop_rate.sleep();
        ros::spinOnce();  // let the global variables update
    }

    return 0;
}

