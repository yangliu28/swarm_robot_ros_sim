// this node is the aggregation simulation of two wheel robots

// aggregation algorithm: (geometric center is replaced with center of minimum covering circle)
// This simulation aggregates a swarm of two wheel robots by a geometric center method. Each robot
// calculates the geometric center of all its neighbors in the sensing range. The first feedback vector
// starts from the robot to the geometric center. In order to avoid collision, a spring model with only
// propulsional force (the spring only pushes two robots away, not drag them together) is used for all
// neighbors with distance smaller than spring length. This is the second feedback vector. Then two feedback
// vector will be fused into one with weighted ratio.

// subscribe to topic "/swarm_sim/two_wheel_robot"
// service client to service "/gazebo/set_joint_properties"

// add exit criteria when goal is achieved (07/10/2016)
// exit when the minimum number of robots in collision range is equal or larger than 3
// the robots are pushed into the collision range while driven by the driving vector
// 3 when robots on the rim, 6 or higher for robots inside, highest of 9 is observed
// (3 might be a little strict, but it works well robot quantity is large)
// (robot quantity smaller than 20 may end up with neighbor number of 2)
// exit only when the two wheel robot topic is active
// not compatible when there are seperated robot or group of robots:
    // no isolated group(desired situation), exit when goal is acheved
    // isolated single robot, neighbor number is 0, will not exit
    // isolated group of robots<=3, minimal neighbor number is 2, will not exit
    // isolated group of robots>3, minimal neighbor number is 3, will exit
    // robot quantity smaller than 20, may stabilize on neighbor number of 2, will not exit

// replace the geometric center with center of minimum covering circle (07/11/2016)
// the algorithm has been tested in matlab, see "minimum_covering_circle.m"
// the method is first generate convex hull of all neighbors using gift wrappign algorithm
// then find the minimum covering circle using the convex hull

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <cmath>
#include <vector>

#include <iostream>  // debug
#include <iomanip>

// flow control parameters
const double TOPIC_ACTIVE_PERIOD = 1.0;  // threshold to tell if a topic is active
const double CONTROL_PERIOD = 0.001;
// simulation control parameters
const double SPRING_LENGTH = 0.1;  // the distance to maintain when aggregation
double sensing_range = 2.0;
const double COLLISION_VECTOR_PERCENTAGE = 85.0/100.0;
const double DRIVING_VECTOR_PERCENTAGE = 1.0 - COLLISION_VECTOR_PERCENTAGE;
const double VEL_RATIO = 50.0;  // the ratio of wheel velocity to the feedback vector
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
    ros::init(argc, argv, "two_wheel_robot_aggregation");
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
    // get sensing range
    bool get_sensing_range = nh.getParam("sensing_range", sensing_range);
    if (get_sensing_range) {
        ROS_INFO_STREAM("using sensing range passed in: " << sensing_range);
        // delete parameter
        nh.deleteParam("sensing_range");
    }
    else
        ROS_INFO_STREAM("using default sensing range: 2.0");

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

    // aggregation loop
    ros::Time timer_now;
    ros::Time loop_last_timer = ros::Time::now();  // use to examine loop rate
    ros::Time timer_program_start;  // for calculating time consumption of simulation
    bool timer_program_start_initialized = false;
    double loops_sum_period;
    int loop_count = 0;
    ros::Rate loop_rate(1.0/CONTROL_PERIOD);  // use to control loop rate
    bool stop_all_robot_once = false;  // stop all robots for one time when topic is inactive
    while (ros::ok()) {
        int robot_quantity = current_robots.index.size();

        // get current time stamp
        timer_now = ros::Time::now();
        // timer_now is zero when it doesn't get simulation timer under topic /clock
        // it happens in the first few moments when this node starts, not sure why
        // initialize program start timer
        if (!timer_program_start_initialized) {
            if (timer_now.toSec() > 0) {
                timer_program_start = timer_now;
                timer_program_start_initialized = true;
            }
        }
        // check if two wheel robot topic is active
        if ((timer_now - two_wheel_robot_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD
            && timer_now.toSec() > 0) {
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
            // full sorting, bubble sorting method
            double distance_temp;
            int index_temp;
            for (int i=0; i<robot_quantity; i++) {
                for (int j=0; j<robot_quantity-1; j++) {
                    // j control the loops of bubble sorting
                    for (int k=0; k<robot_quantity-1-j; k++) {
                        // bubble from back to front to pop up the smaller ones
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

            // 3.find all neighbors in sensing range
            int neighbor_num_in_range[robot_quantity];
            for (int i=0; i<robot_quantity; i++) {
                neighbor_num_in_range[i] = 0;
                for (int j=1; j<robot_quantity; j++) {
                    if (distance_sort[i][j] < sensing_range) {
                        neighbor_num_in_range[i] = neighbor_num_in_range[i] + 1;
                    }
                    else {
                        break;
                    }
                }
            }

            // 4.find all neighbors in spring range
            int neighbor_num_in_spring[robot_quantity];
            int min_neighbor_num_in_spring = 1000;  // for simulation exiting control
            for (int i=0; i<robot_quantity; i++) {
                neighbor_num_in_spring[i] = 0;
                for (int j=1; j<robot_quantity; j++) {
                    if (distance_sort[i][j] < SPRING_LENGTH) {
                        neighbor_num_in_spring[i] = neighbor_num_in_spring[i] + 1;
                    }
                    else {
                        break;
                    }
                }
                // update minimum neighbor num in spring range
                if (min_neighbor_num_in_spring > neighbor_num_in_spring[i]) {
                    min_neighbor_num_in_spring = neighbor_num_in_spring[i];
                }
            }

            // print out number of neighbors in sensing range and spring range
            if (print_debug_msg) {
                std::cout << "number of neighbors in sensing range and spring range:" << std::endl;
                // print multiple colomns in the terminal
                int colomn_count = -1;  // increment by 1 first thing in the loop
                int i=0;  // robot count
                while (i < robot_quantity) {
                    // colomn control
                    colomn_count = colomn_count + 1;
                    if (colomn_count == 3) {
                        colomn_count = 0;
                        std::cout << std::endl;
                    }
                    // print message
                    std::cout << std::setw(10) << current_robots.index[i]
                        << std::setw(5) << neighbor_num_in_range[i]
                        << std::setw(5) << neighbor_num_in_spring[i];
                    i = i + 1;
                }
                std::cout << std::endl;
            }

            // 5.calculate driving feedback vector based on geometric center of in-range neighbors
            // double driving_feedback_vector[robot_quantity][2];
            // double geometric_center[2];
            // for (int i=0; i<robot_quantity; i++) {
            //     // check the in sensor range neighbor numbers
            //     if (neighbor_num_in_range[i] > 0) {
            //         // there is at least one neighbor in sensor range
            //         geometric_center[0] = 0.0;  // initialize the geometric center
            //         geometric_center[1] = 0.0;
            //         // solve for the geometric center
            //         for (int j=1; j<neighbor_num_in_range[i]+1; j++) {
            //             geometric_center[0] = geometric_center[0] + current_robots.x[index_sort[i][j]];
            //             geometric_center[1] = geometric_center[1] + current_robots.y[index_sort[i][j]];
            //         }
            //         geometric_center[0] = geometric_center[0] / double(neighbor_num_in_range[i]);
            //         geometric_center[1] = geometric_center[1] / double(neighbor_num_in_range[i]);
            //         // calculate the driving feedback vector
            //         driving_feedback_vector[i][0] = geometric_center[0] - current_robots.x[i];
            //         driving_feedback_vector[i][1] = geometric_center[1] - current_robots.y[i];
            //     }
            //     else {
            //         // no neighbor in sensor range
            //         driving_feedback_vector[i][0] = 0.0;
            //         driving_feedback_vector[i][1] = 0.0;
            //     }
            // }

            ///////////////////////////////////////////////////////////
            // core algorithm, find minimum covering circle, start here
            ///////////////////////////////////////////////////////////

            // 5.calculate driving feedback vector based on minimum covering circle of in-range neighbors
            double driving_feedback_vector[robot_quantity][2];
            double circle_center[2];  // center of minimum covering circle
            // prepare the neighbor positions in an easy to use dataset
            std::vector<double> pos_x, pos_y;
            std::vector<int> convex_index;
            for (int i=0; i<robot_quantity; i++) {
                // discuss base on different number of neighbor robots
                if (neighbor_num_in_range[i] == 0) {
                    // 0 neighbors in range, isolated, feedback vector zero
                    driving_feedback_vector[i][0] = 0.0;
                    driving_feedback_vector[i][1] = 0.0;
                }
                else if (neighbor_num_in_range[i] == 1) {
                    // 1 neighbor, feedback vector pointing to this robot
                    driving_feedback_vector[i][0] = current_robots.x[index_sort[i][1]] - current_robots.x[i];
                    driving_feedback_vector[i][1] = current_robots.y[index_sort[i][1]] - current_robots.y[i];
                }
                else if (neighbor_num_in_range[i] == 2) {
                    // 2 neighbor, feedback vector pointing to middle point of the two
                    circle_center[0] = (current_robots.x[index_sort[i][1]]
                        + current_robots.x[index_sort[i][2]])/2.0;
                    circle_center[1] = (current_robots.y[index_sort[i][1]]
                        + current_robots.y[index_sort[i][2]])/2.0;
                    driving_feedback_vector[i][0] = circle_center[0] - current_robots.x[i];
                    driving_feedback_vector[i][1] = circle_center[1] - current_robots.y[i];
                }
                else {
                    // 3 or more neighbors, prepare dataset for this situation
                    pos_x.resize(neighbor_num_in_range[i]);
                    pos_y.resize(neighbor_num_in_range[i]);
                    convex_index.clear();  // index of the pos_x, pos_y
                    for (int j=1; j<neighbor_num_in_range[i]+1; j++) {
                        pos_x[j-1] = current_robots.x[index_sort[i][j]];
                        pos_y[j-1] = current_robots.y[index_sort[i][j]];
                    }

                    // find the leftmost robot as the starting point of the convex hull
                    // leftmost robot will always be on the convex hull
                    convex_index.push_back(0);  // initialize the first robot position as leftmost
                    for (int j=1; j<neighbor_num_in_range[i]; j++) {
                        if (pos_x[j] < pos_x[convex_index[0]]) {
                            convex_index[0] = j;
                        }
                    }

                    // find second point on the convex hull
                    std::vector<double> base_vector(2,0);  // 2 number of 0
                    std::vector<double> probe_vector(2,0);
                    double max_angle = 0.0;
                    double probe_angle;
                    convex_index.push_back(0);  // initialize the second point
                    for (int j=0; j<neighbor_num_in_range[i]; j++) {
                        if (j == convex_index[0]) {
                            // jump over this point, it's itself
                            continue;
                        }
                        // ROS_INFO("program goes here: 5.2");
                        base_vector[0] = 0.0;
                        base_vector[1] = -1.0;
                        probe_vector[0] = pos_x[j] - pos_x[convex_index[0]];
                        probe_vector[1] = pos_y[j] - pos_y[convex_index[0]];
                        probe_angle = acos((base_vector[0]*probe_vector[0] + base_vector[1]*probe_vector[1])
                            / sqrt(pow(probe_vector[0],2)+pow(probe_vector[1],2)));
                        if (probe_angle > max_angle) {
                            max_angle = probe_angle;
                            convex_index[1] = j;  // update the second point
                        }
                    }

                    // continue find the rest convex points
                    // the only difference with finding the first one is the base_vector
                    int convex_index_index = 1;  // current index of convex_index
                    int convex_index_next = convex_index[1];  // so that it's not equal to the first index
                    while (convex_index_next != convex_index[0]) {
                        // next convex index not equal to first index, convex not closed
                        max_angle = 0.0;
                        convex_index.push_back(0);  // initialize the next index
                        base_vector[0] = pos_x[convex_index[convex_index_index-1]]
                            - pos_x[convex_index[convex_index_index]];
                        base_vector[1] = pos_y[convex_index[convex_index_index-1]]
                            - pos_y[convex_index[convex_index_index]];
                        for (int j=0; j<neighbor_num_in_range[i]; j++) {
                            if (j == convex_index[convex_index_index]) {
                                // exclude itself
                                continue;
                            }
                            probe_vector[0] = pos_x[j] - pos_x[convex_index[convex_index_index]];
                            probe_vector[1] = pos_y[j] - pos_y[convex_index[convex_index_index]];
                            probe_angle = acos((base_vector[0]*probe_vector[0] + base_vector[1]*probe_vector[1])
                                / sqrt(pow(base_vector[0],2)+pow(base_vector[1],2))
                                / sqrt(pow(probe_vector[0],2)+pow(probe_vector[1],2)));
                            if (probe_angle > max_angle) {
                                max_angle = probe_angle;
                                convex_index[convex_index_index+1] = j;  // update next convex index
                            }
                        }
                        convex_index_next = convex_index[convex_index_index+1];  // the found index
                        convex_index_index = convex_index_index + 1;
                    }
                    // if here, then the convex if closed
                    // the last index is equal to the first index of convex_index
                    convex_index.pop_back();  // delete the repeated index

                    // find the minimum covering circle based on the convex hull
                    std::vector<int> circle_points_index;  // contain the index of the points of the circle
                    // possible two points connecting the diameter, or three points on the icrcle
                    // start with a random side of the convex
                    int side_index[2] = {0,1};  // the first side of the convex, they are index of convex_index
                    double vector_1[2], vector_2[2];  // vectors for computing angle
                    while (true) {
                        // find the smallest angle subtended the side
                        double smallest_angle = M_PI;
                        double subtended_angle;
                        // int convex_index_index;  // change role to the vertex subtend the side
                        for (int j=0; j<convex_index.size(); j++) {
                            if (j == side_index[0] || j == side_index[1]) {
                                // exclude the vertices from the side
                                continue;
                            }
                            vector_1[0] = pos_x[convex_index[side_index[0]]] - pos_x[convex_index[j]];
                            vector_1[1] = pos_y[convex_index[side_index[0]]] - pos_y[convex_index[j]];
                            vector_2[0] = pos_x[convex_index[side_index[1]]] - pos_x[convex_index[j]];
                            vector_2[1] = pos_y[convex_index[side_index[1]]] - pos_y[convex_index[j]];
                            subtended_angle = acos((vector_1[0]*vector_2[0]+vector_1[1]*vector_2[1])
                                / sqrt(pow(vector_1[0],2)+pow(vector_1[1],2))
                                / sqrt(pow(vector_2[0],2)+pow(vector_2[1],2)));
                            if (subtended_angle < smallest_angle) {
                                smallest_angle = subtended_angle;
                                convex_index_index = j;
                            }
                        }
                        if (smallest_angle >= M_PI/2) {
                            // return the side as the diameter of the circle
                            circle_points_index.resize(2);
                            circle_points_index[0] = convex_index[side_index[0]];
                            circle_points_index[1] = convex_index[side_index[1]];
                            break;
                        }
                        else {
                            // check the other two angles of the triangle from the side and vertex
                            // continue new iteration if one angle is obtuse
                            // check the angle at vertex side_index[0]
                            vector_1[0] = pos_x[convex_index[convex_index_index]]
                                - pos_x[convex_index[side_index[0]]];
                            vector_1[1] = pos_y[convex_index[convex_index_index]]
                                - pos_y[convex_index[side_index[0]]];
                            vector_2[0] = pos_x[convex_index[side_index[1]]]
                                - pos_x[convex_index[side_index[0]]];
                            vector_2[1] = pos_y[convex_index[side_index[1]]]
                                - pos_y[convex_index[side_index[0]]];
                            subtended_angle = acos((vector_1[0]*vector_2[0]+vector_1[1]*vector_2[1])
                                / sqrt(pow(vector_1[0],2)+pow(vector_1[1],2))
                                / sqrt(pow(vector_2[0],2)+pow(vector_2[1],2)));
                            if (subtended_angle > M_PI/2) {
                                // continue new iteration with the subtended side
                                side_index[0] = convex_index_index;
                                continue;
                            }
                            // check the angle at vertex side_index[1]
                            vector_1[0] = pos_x[convex_index[convex_index_index]]
                                - pos_x[convex_index[side_index[1]]];
                            vector_1[1] = pos_y[convex_index[convex_index_index]]
                                - pos_y[convex_index[side_index[1]]];
                            vector_2[0] = pos_x[convex_index[side_index[0]]]
                                - pos_x[convex_index[side_index[1]]];
                            vector_2[1] = pos_y[convex_index[side_index[0]]]
                                - pos_y[convex_index[side_index[1]]];
                            subtended_angle = acos((vector_1[0]*vector_2[0]+vector_1[1]*vector_2[1])
                                / sqrt(pow(vector_1[0],2)+pow(vector_1[1],2))
                                / sqrt(pow(vector_2[0],2)+pow(vector_2[1],2)));
                            if (subtended_angle > M_PI/2) {
                                // continue new iteration with the subtended side
                                side_index[1] = convex_index_index;
                                continue;
                            }
                            // if here, no vertice are obtuse
                            // return the three points that the circle passes
                            circle_points_index.resize(3);
                            circle_points_index[0] = convex_index[side_index[0]];
                            circle_points_index[1] = convex_index[side_index[1]];
                            circle_points_index[2] = convex_index[convex_index_index];
                            break;
                        }
                    }

                    // the points for the circle are found
                    // calculate circle center based on circle type
                    if (circle_points_index.size() == 2) {
                        // two points connecting the diameter of the circle
                        int ii[2] = {circle_points_index[0],circle_points_index[1]};
                        circle_center[0] = (pos_x[ii[0]] + pos_x[ii[1]])/2;
                        circle_center[1] = (pos_y[ii[0]] + pos_y[ii[1]])/2;
                    }
                    else if (circle_points_index.size() == 3) {
                        // three points the circle passes
                        int iii[3] = {circle_points_index[0],circle_points_index[1],circle_points_index[2]};
                        double y_delta_a = pos_y[iii[1]] - pos_y[iii[0]];
                        double x_delta_a = pos_x[iii[1]] - pos_x[iii[0]];
                        double y_delta_b = pos_y[iii[2]] - pos_y[iii[1]];
                        double x_delta_b = pos_x[iii[2]] - pos_x[iii[1]];
                        double a_slope = y_delta_a/x_delta_a;
                        double b_slope = y_delta_b/x_delta_b;
                        circle_center[0] = (a_slope*b_slope*(pos_y[iii[0]]-pos_y[iii[2]])
                            + b_slope*(pos_x[iii[0]]+pos_x[iii[1]]) - a_slope*(pos_x[iii[1]]+pos_x[iii[2]]))
                            / (2*(b_slope-a_slope));
                        circle_center[1] = -1.0*(circle_center[0] - (pos_x[iii[0]]+pos_x[iii[1]])/2)/a_slope
                            + (pos_y[iii[0]]+pos_y[iii[1]])/2;
                    }

                    // calculate the feedback vector according to the circle center
                    driving_feedback_vector[i][0] = circle_center[0] - current_robots.x[i];
                    driving_feedback_vector[i][1] = circle_center[1] - current_robots.y[i];
                }
            }

            ///////////////////////////////////////////////////////////
            // core algorithm, find minimum covering circle, end here
            ///////////////////////////////////////////////////////////

            // 6.calculate collision feedback vector from in spring range neighbors
            double collision_feedback_vector[robot_quantity][2];
            double distance_diff;
            for (int i=0; i<robot_quantity; i++) {
                // initialize the collision feedback vector
                collision_feedback_vector[i][0] = 0.0;
                collision_feedback_vector[i][1] = 0.0;
                for (int j=1; j<neighbor_num_in_spring[i]+1; j++) {
                    distance_diff = SPRING_LENGTH - distance_sort[i][j];
                    collision_feedback_vector[i][0] = collision_feedback_vector[i][0] + (current_robots.x[i]
                        - current_robots.x[index_sort[i][j]]) * distance_diff / distance_sort[i][j];
                    collision_feedback_vector[i][1] = collision_feedback_vector[i][1] + (current_robots.y[i]
                        - current_robots.y[index_sort[i][j]]) * distance_diff / distance_sort[i][j];
                }
            }

            // 7.sensor fusion of driving and collision feedback vector
            double feedback_vector[robot_quantity][2];
            for (int i=0; i<robot_quantity; i++) {
                for (int j=0; j<2; j++) {
                    feedback_vector[i][j] = driving_feedback_vector[i][j] * DRIVING_VECTOR_PERCENTAGE
                        + collision_feedback_vector[i][j] * COLLISION_VECTOR_PERCENTAGE;
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

            // 8.calculate the wheel velocity based on the feedback vector
            // the wheel velocities are calculate so that the robot will move
            // in a constant radius curve to the destination defined by the feedback vector            
            double wheel_vel[robot_quantity][2];  // vel of left and right wheels
            // relative angle between the feedback vector and the robot orientation
            double relative_direction;
            double wheel_rotation_center;
            for (int i=0; i<robot_quantity; i++) {
                relative_direction = feedback_vector_direction[i] - current_robots.orientation[i];
                // both feedback vector direction and robot orientation are in range of [-M_PI, M_PI]
                // the difference is in range of [-2*M_PI, 2*M_PI], will convert to [-M_PI, M_PI]
                if (relative_direction > M_PI)
                    relative_direction = relative_direction - 2*M_PI;
                if (relative_direction < -M_PI)
                    relative_direction = relative_direction + 2*M_PI;
                // now we have relative direction and length of feedback vector
                // next step is converting them to wheel rotation center and rotation direction
                    // wheel rotation center is on the axis of two rotation wheels
                        // left side is negative, right side is positive
                    // rotation direction is either ccw or cw, relative to wheel rotation center
                // divide relative direction into four quarters
                if (relative_direction > 0 && relative_direction <= M_PI/2) {
                    // going forward and rotate ccw, rotation center at left side
                    wheel_rotation_center
                        = -feedback_vector_length[i]/2 / cos(M_PI/2 - relative_direction);
                    // the velocity of the center of the robot: feedback_vector_length[i] * VEL_RATIO
                    // left wheel velocity
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO
                        * (LEFT_WHEEL_POSITION - wheel_rotation_center) / (0 - wheel_rotation_center);
                    // right wheel velocity
                    wheel_vel[i][1] = feedback_vector_length[i] * VEL_RATIO
                        * (RIGHT_WHEEL_POSITION - wheel_rotation_center) / (0 - wheel_rotation_center);
                }
                else if (relative_direction > M_PI/2 && relative_direction < M_PI) {
                    // going backward and rotate cw, rotation center at left side
                    wheel_rotation_center
                        = -feedback_vector_length[i]/2 / cos(relative_direction - M_PI/2);
                    // wheel velocities
                    wheel_vel[i][0] = -feedback_vector_length[i] * VEL_RATIO
                        * (LEFT_WHEEL_POSITION - wheel_rotation_center) / (0 - wheel_rotation_center);
                    wheel_vel[i][1] = -feedback_vector_length[i] * VEL_RATIO
                        * (RIGHT_WHEEL_POSITION - wheel_rotation_center) / (0 - wheel_rotation_center);
                }
                else if (relative_direction >=-M_PI/2 && relative_direction < 0) {
                    // going forward and rotate cw, rotation center at right side
                    wheel_rotation_center
                        = feedback_vector_length[i]/2 / cos(relative_direction + M_PI/2);
                    // wheel velocities
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center - LEFT_WHEEL_POSITION) / (wheel_rotation_center - 0);
                    wheel_vel[i][1] = feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center - RIGHT_WHEEL_POSITION) / (wheel_rotation_center - 0);
                }
                else if (relative_direction > -M_PI && relative_direction < -M_PI/2) {
                    // going backward and rotate ccw, rotation center at right side
                    wheel_rotation_center
                        = feedback_vector_length[i]/2 / cos(-M_PI/2 - relative_direction);
                    // wheel velocities
                    wheel_vel[i][0] = -feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center - LEFT_WHEEL_POSITION) / (wheel_rotation_center - 0);
                    wheel_vel[i][1] = -feedback_vector_length[i] * VEL_RATIO
                        * (wheel_rotation_center - RIGHT_WHEEL_POSITION) / (wheel_rotation_center - 0);
                }
                else if (relative_direction == 0){
                    // very unlikely here, just in case
                    ROS_WARN("feedback vector relative direction is 0");
                    wheel_rotation_center = -100;  // a very large number
                    // wheel velocities
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO;
                    wheel_vel[i][1] = wheel_vel[i][0];
                }
                else if (relative_direction == -M_PI || relative_direction == M_PI) {
                    // very unlikely here, just in case
                    ROS_WARN("feedback vector relative direction is -M_PI or M_PI");
                    wheel_rotation_center = -100;
                    // wheel velocities
                    wheel_vel[i][0] = feedback_vector_length[i] * VEL_RATIO;
                    wheel_vel[i][1] = wheel_vel[i][0];
                }
            }

            // 9.send service request of wheel velocities
            bool call_service;
            for (int i=0; i<robot_quantity; i++) {
                // left wheel
                set_joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::left_motor";
                set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = wheel_vel[i][0];
                call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                if (call_service) {
                    if (!set_joint_properties_srv_msg.response.success) {
                        // possibly the robot not found
                        ROS_WARN("the robot model not found when set left wheel vel");
                    }
                }
                else
                    ROS_ERROR("fail to connect with gazebo server when set left wheel vel");
                // right wheel
                set_joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::right_motor";
                set_joint_properties_srv_msg.request.ode_joint_config.vel[0] = wheel_vel[i][1];
                call_service = set_joint_properties_client.call(set_joint_properties_srv_msg);
                if (call_service) {
                    if (!set_joint_properties_srv_msg.response.success) {
                        // possibly the robot not found
                        ROS_WARN("the robot model not found when set right wheel vel");
                    }
                }
                else
                    ROS_ERROR("fail to connect with gazebo server when set right wheel vel");
            }

            // 10.exit program when minimum neighbor number in collision range is >= 3
            if (min_neighbor_num_in_spring >= 3) {
                // prepare to exit, set wheel velocity of all robots to zero
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
                // exit message
                ROS_INFO_STREAM("minimum neighbor number in collision range: "
                    << min_neighbor_num_in_spring);
                ROS_INFO("two wheel robot aggregation program exit: criteria satisfied");
                ROS_INFO_STREAM("time consumption for this simulation: "
                    << (ros::Time::now() - timer_program_start).toSec());
                break;  // exit this program
            }

        }
        else {
            // the topic is not active, meaning two wheel robot manager node is down
            // ROS_WARN("topic is inactive");

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

