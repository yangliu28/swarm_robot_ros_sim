// this program is for the line formation simulation of two wheel robot as a swarm robot

// communication includes:
// subscribe to topic "swarm_robot_poses" to get current robot swarm_robot_poses
// create an action client to "two_wheel_traj_action"

// line formation algorithm:
// there is a sensing range for each robot, should be relatively large
// the position of in-range robots will be used to fitted in a line (line formation)
// then the robot will move in two directions added together
// one direction is perpendicular to the line and heading to it
// the feedback is the distance between the robot and line, the ratio is less than 1
// the other direction is parallel to the line, feedback is the force from spring model
// derived the same way in dispersion simulation, except we only take the parallel part

// how to understand?
// with feedback in two direction to control the robot
// the one perpendicular to the line aims to make all robot move closer to the line
// the one parallel to the line aims to make all robot uniformly distributed on the line

#include <ros/ros.h>
#include <algorithm>
#include <numeric>
#include <vector>
#include <math.h>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <actionlib/client/simple_action_client.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// global variables
std::vector<double> g_robot_x;
std::vector<double> g_robot_y;
bool g_robot_poses_cb_started = false;

// simulation control parameters
// for getting which robots will be included in line fitting
double sensing_range = 3.0;  // sensing range, may change from parameter
// for perpendicular displacement calculation
const double perpendicular_feedback_ratio = 0.618;  // golden ratio
// for parallel displacement calculation
double spring_length = 0.7;  // spring length, may change
const double upper_limit_ratio = 0.30;  // upper limit part relative to spring length
const double upper_limit = spring_length * (1 + upper_limit_ratio);
const double parallel_feedback_ratio = 0.382/2.0;  // verse golden ratio
// half the feedback_ratio to alleviate vibration

// two wheel robot specification, really should get these values in another way
const double half_wheel_dist = 0.0177;
const double wheel_radius = 0.015;
double wheel_speed = 2.0;  // rad*s-1, for time cost of the action, may change

// callback for message from topic "swarm_robot_poses"
void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!g_robot_poses_cb_started)  // first time to be invoked
        g_robot_poses_cb_started = true;
    g_robot_x = message_holder.x;
    g_robot_y = message_holder.y;
}

// linear fitting function, revised from C++11 implementation
// linear fitting here using y=kx+b form, which means vertical line is not supported
// but it's very unlikely to happen with double precision data, we'll see
std::vector<double> linear_fitting(const std::vector<double>& x, const std::vector<double>& y) {
    const double n = x.size();
    const double s_x = std::accumulate(x.begin(), x.end(), 0.0);
    const double s_y = std::accumulate(y.begin(), y.end(), 0.0);
    const double s_xx = std::inner_product(x.begin(), x.end(), x.begin(), 0.0);
    const double s_xy = std::inner_product(x.begin(), x.end(), y.begin(), 0.0);
    const double slope = (n * s_xy - s_x * s_y) / (n * s_xx - s_x * s_x);
    const double intercept = (s_y - slope * s_x) / n;
    std::vector<double> a;
    a.push_back(slope);
    a.push_back(intercept);
    return a;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_line_formation");
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
    // parameter: sensing_range
    bool get_sensing_range = nh.getParam("sensing_range", sensing_range);
    if (get_sensing_range)
        ROS_INFO_STREAM("using sensing_range passed in: " << sensing_range);
    else
        ROS_INFO_STREAM("using default sensing_range: " << sensing_range);
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
    ROS_INFO("topic message from swarm_robot_poses is ready");

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
    double distance[robot_quantity][robot_quantity];
    double distance_sort[robot_quantity][robot_quantity];
    int index_sort[robot_quantity][robot_quantity];
    // the loop of optimizing robot position for line formation
    int iteration_index = 0;
    while (ros::ok()) {
        iteration_index = iteration_index + 1;
        ROS_INFO_STREAM("");  // blank line
        ROS_INFO_STREAM("iteration_index: " << iteration_index);  // iteration index

        ros::spinOnce();  // let the robot positions update

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

        // find all the neighbors in the sensing range
        int neighbor_num_in_range[robot_quantity];  // number of neighbors in range
        for (int i=0; i<robot_quantity; i++) {
            // with at least 2 neighbors and itself to do line fitting
            neighbor_num_in_range[i] = 2;  // there will be at least 2 neighbors
            // following statement is a bit different from dispersion simulation
            // avoiding visiting element in distance_sort that is out of index
            // really not nice code below, any better way to do this?
            while (true) {
                neighbor_num_in_range[i] = neighbor_num_in_range[i] + 1;
                if (neighbor_num_in_range[i] <= robot_quantity-1)
                    if (distance_sort[i][neighbor_num_in_range[i]] < sensing_range)
                        continue;
                    else {
                        neighbor_num_in_range[i] = neighbor_num_in_range[i] - 1;
                        break;
                    }
                else {
                    neighbor_num_in_range[i] = neighbor_num_in_range[i] - 1;
                    break;
                }
            }
            // ERROR below: neighbor_num will increase by 1 when out of while, not wanted
            // while (neighbor_num_in_range[i] <= robot_quantity-1) {
            //     if (distance_sort[i][neighbor_num_in_range[i]+1] < sensing_range)
            //         neighbor_num_in_range[i] = neighbor_num_in_range[i] + 1;
            //     else
            //         break;
            // }
            
            // ROS_INFO_STREAM("neighbor number of " << i << ": " << neighbor_num_in_range[i]);  // debug
        }

        // find out the fitted line from neighbors in sensing range
        std::vector<double> fit_temp;  // // for result from linear fitting
        double fitting_result[robot_quantity][2];  // store the fitting result
        for (int i=0; i<robot_quantity; i++) {
            std::vector<double> x;
            std::vector<double> y;
            x.resize(neighbor_num_in_range[i]+1);  // neighbors plus itself
            y.resize(neighbor_num_in_range[i]+1);
            for (int j=0; j<neighbor_num_in_range[i]+1; j++) {
                x[j] = g_robot_x[index_sort[i][j]];
                y[j] = g_robot_y[index_sort[i][j]];
            }
            fit_temp = linear_fitting(x, y);
            // ROS_INFO_STREAM("fit of " << i << ": " << fit_temp[0] << ", " << fit_temp[1]);
            fitting_result[i][0] = fit_temp[0];  // the slope
            fitting_result[i][1] = fit_temp[1];  // the intercept
        }

        // calculate displacement for the perpendicular part
        double perpendicular_displacement[robot_quantity][2];  // for x and y
        for (int i=0; i<robot_quantity; i++) {
            // calculate the vector from robot position to pedal on the line
            // do some geometry you will find out
            // (-(ax+b-y)/(2a), (ax+b-y)/2)
            perpendicular_displacement[i][0] = perpendicular_feedback_ratio *
                (-(fitting_result[i][0] * g_robot_x[i] + fitting_result[i][1] - g_robot_y[i]) /
                    (2 * fitting_result[i][0]));
            perpendicular_displacement[i][1] = perpendicular_feedback_ratio *
                (fitting_result[i][0] * g_robot_x[i] + fitting_result[i][1] - g_robot_y[i]) / 2;
        }

        // find all the neighbors that will be used in the force feedback control
        int neighbor_num_spring[robot_quantity];  // neighbor numbers for force feedback
        for (int i=0; i<robot_quantity; i++) {
            // least number of neighbors is 2, situation different from dispersion
            neighbor_num_spring[i] = 2;
            while (neighbor_num_spring[i] <= robot_quantity-1) {
                if (distance_sort[i][neighbor_num_spring[i]+1] < upper_limit)
                    neighbor_num_spring[i] = neighbor_num_spring[i] + 1;
                else
                    break;
            }
        }

        // calculate displacement for the parallel part
        double parallel_displacement[robot_quantity][2];  // for x and y
        double distance_diff;
        double fitting_unit_vector[2];  // a unit vector parallel to the fitted line
        double parallel_distance;
        for (int i=0; i<robot_quantity; i++) {
            parallel_displacement[i][0] = 0.0;
            parallel_displacement[i][1] = 0.0;
            for (int j=1; j<=neighbor_num_spring[i]; j++) {
                distance_diff = distance_sort[i][j] - spring_length;
                // adding the spring effect of all the neighbors
                parallel_displacement[i][0] = parallel_displacement[i][0] +
                    parallel_feedback_ratio * distance_diff * (g_robot_x[index_sort[i][j]] -
                        g_robot_x[i]) / distance_sort[i][j];
                parallel_displacement[i][1] = parallel_displacement[i][1] +
                    parallel_feedback_ratio * distance_diff * (g_robot_y[index_sort[i][j]] -
                        g_robot_y[i]) / distance_sort[i][j];
            }
            // get the parallel part of this displacement
            fitting_unit_vector[0] = 1 / sqrt(1 + fitting_result[i][0]);
            fitting_unit_vector[1] = fitting_result[i][0] / sqrt(1 + fitting_result[i][0]);
            if (abs(atan(fitting_result[i][0]) - atan(parallel_displacement[i][1] /
                parallel_displacement[i][0])) < M_PI/2) {
                // angle between displacement vector and unit vector is less than M_PI/2
                parallel_distance = fitting_unit_vector[0] * parallel_displacement[i][0] +
                    fitting_unit_vector[1] * parallel_displacement[i][1];
                parallel_displacement[i][0] = parallel_distance * fitting_unit_vector[0];
                parallel_displacement[i][1] = parallel_distance * fitting_unit_vector[1];
            }
            else {
                // angle is more than M_PI/2
                fitting_unit_vector[0] = -fitting_unit_vector[0];  // reverse the direction
                fitting_unit_vector[1] = -fitting_unit_vector[1];
                parallel_distance = fitting_unit_vector[0] * parallel_displacement[i][0] +
                    fitting_unit_vector[1] * parallel_displacement[i][1];
                parallel_displacement[i][0] = parallel_distance * fitting_unit_vector[0];
                parallel_displacement[i][1] = parallel_distance * fitting_unit_vector[1];
            }
        }

        // prepare goal message
        double displacement_average = 0.0;  // for the calculation of time cost
        goal.x.resize(robot_quantity);  // important here, otherwise runtime error
        goal.y.resize(robot_quantity);
        for (int i=0; i<robot_quantity; i++) {
            // combine two displacement together
            goal.x[i] = g_robot_x[i] + perpendicular_displacement[i][0] +
                parallel_displacement[i][0];
            goal.y[i] = g_robot_y[i] + perpendicular_displacement[i][1] +
                parallel_displacement[i][1];
            displacement_average = displacement_average +
                sqrt(pow(perpendicular_displacement[i][0] + parallel_displacement[i][0], 2) +
                    pow(perpendicular_displacement[i][1] + parallel_displacement[i][1], 2));
        }
        // this is the real average displacement
        displacement_average = displacement_average / robot_quantity;
        // use displacement_average to represent time spent on line movement
        // use M_PI/3 to represent time spent on self rotating
        goal.time_cost = displacement_average / wheel_radius / wheel_speed + 
            M_PI/3 * half_wheel_dist / wheel_radius / wheel_speed;
        ROS_INFO_STREAM("time cost for this action: " << goal.time_cost << "(second)");

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

