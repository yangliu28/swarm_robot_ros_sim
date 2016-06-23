// this node is the dispersion simulation of two wheel robots

// ros communication:
    // subscribe to topic "/swarm_sim/two_wheel_robot"
    // service client to service "/gazebo/set_joint_properties"

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <math.h>

#include <iostream>  // debug

// flow control parameters
const double TOPIC_ACTIVE_PERIOD = 1.0;  // threshold to tell if a topic is active
const double CONTROL_PERIOD = 0.001;
// simulation control parameters
double spring_length = 0.7;  // default spring length, may change from private parameter
double upper_limit_ratio = 0.30;
const int NEIGHBOR_NUM_L_LIMIT = 3;
const int NEIGHBOR_NUM_H_LIMIT = 6;
const double DISTANCE_FEEDBACK_RATIO = 0.382/2.0;
const double VEL_RATIO = 10.0;  // the ratio of robot velocity relative to feedback vector
const double STABLE_THRESHOLD = 0.02;  // temperarily not used
const double LEFT_WHEEL_POSITION = -0.0157;
const double RIGHT_WHEEL_POSITION = 0.0157;  // right is positive direction

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
    ros::init(argc, argv, "two_wheel_robot_dispersion");
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

    // get settings for this simulation from private parameter
    bool get_spring_length = nh.getParam("spring_length", spring_length);
    if (get_spring_length) {
        ROS_INFO_STREAM("using spring length passed in: " << spring_length);
        // delete parameter
        nh.deleteParam("spring_length");
    }
    else
        ROS_INFO_STREAM("using default spring length: 0.7");
    // calculate other parameter depending on spring length
    double upper_limit = spring_length * (1 + upper_limit_ratio);

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

    // initialize callback timer
    two_wheel_robot_topic_timer = ros::Time::now();
    // delay for a while to avoid false judgement of topic activity
    ros::Duration(TOPIC_ACTIVE_PERIOD + 0.1).sleep();

    // dispersion control loop
    ros::Time timer_now;
    ros::Rate loop_rate(1/CONTROL_PERIOD);
    bool stop_all_robot_once = false;  // stop all robots for one time when topic is inactive
    while (ros::ok()) {
        int robot_quantity = current_robots.index.size();

        // check if two wheel robot topic is active
        timer_now = ros::Time::now();
        if ((timer_now - two_wheel_robot_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD) {
            // the topic is been actively published
            ROS_WARN("topic is active");

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

            // 2.sort the distance and record the index change
            double distance_sort[robot_quantity][robot_quantity];
            int index_sort[robot_quantity][robot_quantity];
            // initialize distance_sort and index_sort
            for (int i=0; i<robot_quantity; i++) {
                for (int j=0; j<robot_quantity; j++) {
                    distance_sort[i][j] = distance[i][j];
                    index_sort[i][j] = j;
                }
            }
            // start sorting, bubble sort method
            double distance_temp;
            int index_temp;
            for (int i=0; i<robot_quantity; i++) {
                for (int j=0; j<robot_quantity-1; j++) {
                    for (int k=0; k<robot_quantity-1-j; k++) {
                        if (distance_sort[i][k] > distance_sort[i][k+1]) {
                            // switch between these two distances
                            distance_temp = distance_sort[i][k];
                            distance_sort[i][k] = distance_sort[i][k+1];
                            distance_sort[i][k+1] = distance_temp;
                            // also switch corresponding indices
                            index_temp = index_sort[i][k];
                            index_sort[i][k] = index_sort[i][k+1];
                            index_sort[i][k+1] = index_temp;
                        }
                    }
                }
            }

            // 3.find all neighbors that will be used in force feedback control
            // find all the neighbors within the upper limit, choose closest 6 if exceed 6
            // if neighbors are sparse, override upper limit and choose cloest 3
            // i.e., there are 3 ~ 6 neighbors for each robot
            int neighbor_num[robot_quantity];  // the number of vallid neighbors
            // no need to record index
            // they are the first neighbor_num robots in the sorted list
            for (int i=0; i<robot_quantity; i++) {
                // compare robot quantity with neighbor number limits
                if ((robot_quantity - 1) <= NEIGHBOR_NUM_L_LIMIT) {
                    // (robot_quantity - 1) is the largest neighbor number for each robot
                    // set neighbor number to the largest possible
                    neighbor_num[i] = robot_quantity - 1;
                }
                else {
                    neighbor_num[i] = NEIGHBOR_NUM_L_LIMIT;  // initialize with lower limit
                    for (int j=NEIGHBOR_NUM_L_LIMIT+1; j<robot_quantity; j++) {
                        // (NEIGHBOR_NUM_L_LIMIT+1) is the index in the distance_sort
                        // of the first robot except itself and the neighbors
                        if (distance_sort[i][j] < upper_limit) {
                            neighbor_num[i] = neighbor_num[i] + 1;
                            if (neighbor_num[i] == NEIGHBOR_NUM_H_LIMIT)
                                break;
                        }
                    }
                }
            }

            // 4.calculate feedback vector for each robot
            double feedback_vector[robot_quantity][2];  // vector in x and y for each robot
            double distance_diff;
            for (int i=0; i<robot_quantity; i++) {
                feedback_vector[i][0] = 0.0;
                feedback_vector[i][1] = 0.0;
                for (int j=1; j<=neighbor_num[i]; j++) {
                    distance_diff = distance_sort[i][j] - spring_length;
                    // feedback on x
                    feedback_vector[i][0] = feedback_vector[i][0] + DISTANCE_FEEDBACK_RATIO * distance_diff
                        * (current_robots.x[index_sort[i][j]] - current_robots.x[i]) / distance_sort[i][j];
                    // feedback on y
                    feedback_vector[i][1] = feedback_vector[i][1] + DISTANCE_FEEDBACK_RATIO * distance_diff
                        * (current_robots.y[index_sort[i][j]] - current_robots.y[i]) / distance_sort[i][j];
                }
            }
            // calculate the length and direction of feedback vector
            double feedback_vector_length[robot_quantity];
            double feedback_vector_direction[robot_quantity];
            for (int i=0; i<robot_quantity; i++) {
                feedback_vector_length[i]
                    = sqrt(pow(feedback_vector[i][0], 2) + pow(feedback_vector[i][1], 2));
                feedback_vector_direction[i] = atan2(feedback_vector[i][1], feedback_vector[i][0]);
            }

            // 5.calculate the wheel velocities
            // the wheel velocities are calculate so that the robot will move
            // in a constant radius curve to the destination defined by the feedback vector            
            double wheel_vel[robot_quantity][2];  // vel of left and right wheels
            // relative angle between the feedback vector and the robot orientation
            double relative_direction[robot_quantity];
            double wheel_rotation_center[robot_quantity];
            bool rotation_direction_ccw[robot_quantity];  // true is ccw, false is cw
            for (int i=0; i<robot_quantity; i++) {
                relative_direction[i] = feedback_vector_direction[i] - current_robots.orientation[i];
                // both feedback vector direction and robot orientation are in range of [-M_PI, M_PI]
                // the difference is in range of [-2*M_PI, 2*M_PI], will convert to [-M_PI, M_PI]
                if (relative_direction[i] > M_PI)
                    relative_direction[i] = relative_direction[i] - 2*M_PI;
                if (relative_direction[i] < -M_PI)
                    relative_direction[i] = relative_direction[i] + 2*M_PI;
                // now we have relative direction and length of feedback vector
                // next step is converting them to wheel rotation center and rotation direction
                    // wheel rotation center is on the axis of two rotation wheels
                        // left side is negative, right side is positive
                    // rotation direction is either ccw or cw, relative to wheel rotation center
                // divide relative direction into four quarters
                if (relative_direction[i] > 0 && relative_direction[i] <= M_PI/2) {
                    // going forward and rotate ccw
                    wheel_rotation_center[i]
                        = -feedback_vector_length[i]/2 / cos(M_PI/2 - relative_direction[i]);
                    rotation_direction_ccw[i] = true;
                }
                else if (relative_direction[i] > M_PI/2 && relative_direction[i] < M_PI) {
                    // going backward and rotate cw
                    wheel_rotation_center[i]
                        = -feedback_vector_length[i]/2 / cos(relative_direction[i] - M_PI/2);
                    rotation_direction_ccw[i] = false;
                }
                else if (relative_direction[i] >=-M_PI/2 && relative_direction[i] < 0) {
                    // going forward and rotate cw
                    wheel_rotation_center[i]
                        = feedback_vector_length[i]/2 / cos(relative_direction[i] + M_PI/2);
                    rotation_direction_ccw[i] = false;
                }
                else if (relative_direction[i] > -M_PI && relative_direction[i] < -M_PI/2) {
                    // going backward and rotate ccw
                    wheel_rotation_center[i]
                        = feedback_vector_length[i]/2 / cos(-M_PI/2 - relative_direction[i]);
                    rotation_direction_ccw[i] = true;
                }
                else if (relative_direction[i] == 0){
                    // very unlikely
                    ROS_WARN("feedback vector relative direction is 0");
                    wheel_rotation_center[i] = -100;  // a very large number
                    rotation_direction_ccw[i] = true;
                }
                else if (relative_direction[i] == -M_PI || relative_direction[i] == M_PI) {
                    // very unlikely
                    ROS_WARN("feedback vector relative direction is -M_PI or M_PI");
                    wheel_rotation_center[i] = -100;
                    rotation_direction_ccw[i] = false;
                }
                // calculate wheel velocity here
                if (wheel_rotation_center[i] <= 0 && rotation_direction_ccw[i] == true) {
                    // rotation center at left side, and rotate ccw
                    // the velocity of the center of the robot: feedback_vector_length[i] * VEL_RATIO
                    // left wheel
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO
                        * (LEFT_WHEEL_POSITION - wheel_rotation_center[i]) / (0 - wheel_rotation_center[i]);
                    // right wheel
                    wheel_vel[i][1] = feedback_vector_length[i] * VEL_RATIO
                        * (RIGHT_WHEEL_POSITION - wheel_rotation_center[i]) / (0 - wheel_rotation_center[i]);
                }
                else if (wheel_rotation_center[i] <= 0 && rotation_direction_ccw[i] == false) {
                    // rotation center at left side, and rotate cw
                    // left wheel
                    wheel_vel[i][0] = -feedback_vector_length[i] * VEL_RATIO
                        * (LEFT_WHEEL_POSITION - wheel_rotation_center[i]) / (0 - wheel_rotation_center[i]);
                    // right wheel
                    wheel_vel[i][1] = -feedback_vector_length[i] * VEL_RATIO
                        * (RIGHT_WHEEL_POSITION - wheel_rotation_center[i]) / (0 - wheel_rotation_center[i]);
                }
                else if (wheel_rotation_center[i] > 0 && rotation_direction_ccw[i] == false) {
                    // rotation center at right side, and rotate cw
                    // left wheel
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center[i] - LEFT_WHEEL_POSITION) / (wheel_rotation_center[i] - 0);
                    // right wheel
                    wheel_vel[i][1] = feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center[i] - RIGHT_WHEEL_POSITION) / (wheel_rotation_center[i] - 0);
                }
                else if (wheel_rotation_center[i] > 0 && rotation_direction_ccw[i] == true) {
                    // rotation center at right side, and rotate ccw
                    // left wheel
                    wheel_vel[i][0] = -feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center[i] - LEFT_WHEEL_POSITION) / (wheel_rotation_center[i] - 0);
                    // right wheel
                    wheel_vel[i][1] = -feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center[i] - RIGHT_WHEEL_POSITION) / (wheel_rotation_center[i] - 0);
                }
            }

            // ***********************************
            // print out the wheel vel data
            for (int i=0; i<robot_quantity; i++) {
                std::cout << "robot index " << current_robots.index[i] << "\tleft vel\t"
                    << wheel_vel[i][0] << "\tright vel\t" << wheel_vel[i][1] << std::endl;
            }
            std::cout << std::endl;

            // 6. send service request of wheel velocities
            bool call_service;
            for (int i=0; i<robot_quantity; i++) {
                // left wheel
                set_joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::left_motor";
                set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = wheel_vel[i][0];
                call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                if (call_service) {
                    if (!set_joint_properties_srv_msg.response.success)
                        // possibly the robot not found
                        ROS_WARN("the robot model not found when set left wheel vel");
                }
                else
                    ROS_ERROR("fail to connect with gazebo server when set left wheel vel");
                // right wheel
                set_joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::right_motor";
                set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = wheel_vel[i][1];
                call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                if (call_service) {
                    if (!set_joint_properties_srv_msg.response.success)
                        // possibly the robot not found
                        ROS_WARN("the robot model not found when set right wheel vel");
                }
                else
                    ROS_ERROR("fail to connect with gazebo server when set right wheel vel");
            }

            // break;  // debug purpose

        }
        else {
            // the topic is not active
            ROS_WARN("when topic is inactive");

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

