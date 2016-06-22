// this node is the dispersion simulation of two wheel robots

// ros communication:
    // subscribe to topic "/swarm_sim/two_wheel_robot"
    // service client to service "/gazebo/set_joint_properties"

#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <gazebo_msgs/SetJointProperties.h>
#include <math.h>

// flow control parameters
const double TOPIC_ACTIVE_PERIOD = 1.0;  // threshold to tell if a topic is active
const double CONTROL_PERIOD = 0.001;
// simulation control parameters
double spring_length = 0.7;  // default spring length, may change from private parameter
double upper_limit_ratio = 0.30;
const int NEIGHBOR_NUM_L_LIMIT = 3;
const int NEIGHBOR_NUM_H_LIMIT = 6;
const double FEEDBACK_RATIO = 0.382/2.0;
const double STABLE_THRESHOLD = 0.02;

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
    set_joint_properties_srv_msg.request.ode_joint_config = 1.0;

    // initialize callback timer
    two_wheel_robot_topic_timer = ros::Time::now();
    // delay for a while to avoid false judgement of topic activity
    ros::Duration(TOPIC_ACTIVE_PERIOD + 0.1).sleep();

    // dispersion control loop
    ros::Time timer_now;
    ros::Rate loop_rate(1/CONTROL_PERIOD);
    bool stop_all_robot_once = false;  // stop all robots for one time when topic is inactive
    while (ros::ok()) {
        // check if two wheel robot topic is active
        timer_now = ros::Time::now();
        if ((timer_now - two_wheel_robot_topic_timer).toSec() < TOPIC_ACTIVE_PERIOD) {
            // the topic is been actively published
            // set the stop_all_robot_once flag, prepare when topic out of active state
            stop_all_robot_once = true;

            // 1.calculate distance between any two robots
            int robot_quantity = current_robots.index.size();
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
                for (int j=0; j<=neighbor_num[i]; j++) {
                    distance_diff = distance_sort[i][j] - spring_length;
                    // feedback on x
                    feedback_vector[i][0] = feedback_vector[i][0] + FEEDBACK_RATIO * distance_diff
                        * (current_robots.x[index_sort[i][j]] - current_robots.x[i]) / distance_sort[i][j];
                    // feedback on y
                    feedback_vector[i][1] = feedback_vector[i][1] + FEEDBACK_RATIO * distance_diff
                        * (current_robots.y[index_sort[i][j]] - current_robots.y[i]) / distance_sort[i][j];
                }
            }

            // 5.calculate the wheel velocities and send service request
            // the wheel velocities are calculate so that the robot will move
            // in a constant radius curve to the destination defined by the feedback vector
            
            double wheel_vel[robot_quantity][2];

        }
        else {
            // the topic is not active
            if (stop_all_robot_once) {
                // set wheel speed of all robots to zero according to last topic update

                // reset the stop_all_robot_once flag
                stop_all_robot_once = false;
            }
        }




        loop_rate.sleep();
        ros::spinOnce();  // let the global variables update
    }

    return 0;
}

