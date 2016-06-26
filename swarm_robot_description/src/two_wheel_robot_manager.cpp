// this node manage the number and index of present two wheel robot

// publish topic on two wheel robot information, including:
    // robot index, 2D position, orientation and speed of two wheels
// use gazebo service to add and delete model in gazebo
    // accept service request of adding or deleting and quantity
    // compatible when a robot model is deleted directly from gazebo gui
    // can delete a number of robots, delete all robots, but one robot a time when adding
    // because the service callback function will check collision when adding a robot
    // if adding multiple robots in one service call, after the first robot is added
    // its position is unknown and needs model states callback funtion to update it
    // so one robot a time when adding, either random or specific position

// both low and high level robot control subscribe to same robot information topic
// although it's better to divide the message into two, and publish at different frequency
// this one topic method can be reused when it comes to other robot models


#include <ros/ros.h>
#include <swarm_robot_msg/two_wheel_robot.h>
#include <swarm_robot_srv/two_wheel_robot_update.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <geometry_msgs/Quaternion.h>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <stdlib.h>  // rand()

// the distance unit in gazebo is meter
const double SAFE_DIST = 0.0465;  // the collision free distance between two robots

// global variables
// THE container maintained by this node
swarm_robot_msg::two_wheel_robot current_robots;
bool robot_position_updated = false;


// quaternion => rotation angle of two wheel robot
double quaternion_to_angle(geometry_msgs::Quaternion input_quaternion) {
    // this assume the x and y element of the quaternion is close to zero
    return atan(input_quaternion.z/input_quaternion.w) * 2;
}

// generate a quaternion for the random heading of the two wheel robot
geometry_msgs::Quaternion random_quaternion() {
    // alpha is the rotation angle along z axis
    double half_alpha = (float)(rand() % 360 - 180)/2.0 * M_PI/180.0;
    geometry_msgs::Quaternion output_quaternion;
    output_quaternion.x = 0;
    output_quaternion.y = 0;
    output_quaternion.z = sin(half_alpha);
    output_quaternion.w = cos(half_alpha);
    return output_quaternion;
}

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

// generate a pair of random float numbers with in half_range
std::vector<double> random_position(double half_range) {
    std::vector<double> position_2d;
    position_2d.resize(2);  // contains 2 numbers
    // generate the pseudorandom float number between [-half_range, half_range]
    position_2d[0] = (((double)rand())/(double)RAND_MAX * 2.0 - 1.0) * half_range;
    position_2d[1] = (((double)rand())/(double)RAND_MAX * 2.0 - 1.0) * half_range;
    return position_2d;
}

// position check for availbility, avoid collision when adding new robot model in gazebo
bool position_availibility(std::vector<double> position_2d) {
    // return true if input position is ok to add a robot
    int current_robot_quantity = current_robots.index.size();
    for (int i=0; i<current_robot_quantity; i++) {
        double dist = sqrt(pow(position_2d[0] - current_robots.x[i], 2)
            + pow(position_2d[1] - current_robots.y[i], 2));
        if (dist < SAFE_DIST) return false;
    }
    // if here, the position is safe
    return true;
}

// callback for getting robot positions
// also check and update if there is any addition or deletion of robots
void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // parsing the two wheel robots from all the models in gazebo
    // possible that there are models for obstacles or other environments
    int model_quantity = current_model_states.name.size();
    // the index of robots in the container
    std::vector<int32_t> container_index = current_robots.index;
    int container_size = container_index.size();
    for (int i=0; i<model_quantity; i++) {
        // check if it is a two wheel robot
        // there is a underscore between the name and the index
        std::size_t found = current_model_states.name[i].find("two_wheel_robot");
        if (found != std::string::npos) {
            // a two_wheel_robot has been found
            // get the robot index
            // 16 = 15 + 1, 15 is the length of "two_wheel_robot"
            std::string index_str = current_model_states.name[i].substr(16);
            int index_parsed = std::atoi(index_str.c_str());
            // search in the container
            bool parsed_index_found = false;
            for (int j=0; j<container_size; j++) {
                // the size of container_index may change for each i
                if (index_parsed == container_index[j]) {
                    // update the 2D position of two wheel robots
                    current_robots.x[j] = current_model_states.pose[i].position.x;
                    current_robots.y[j] = current_model_states.pose[i].position.y;
                    current_robots.orientation[j]
                        = quaternion_to_angle(current_model_states.pose[i].orientation);
                    // mark as -1, meaning position is updated
                    // will check later if there is any element not marked, i.e., not in gazebo
                    container_index[j] = -1;
                    parsed_index_found = true;
                    break;
                }
            }
            if (!parsed_index_found) {
                // parsed index not found in the container, ADDITION found!
                // update a new robot in the container
                current_robots.index.push_back(index_parsed);
                current_robots.x.push_back(current_model_states.pose[i].position.x);
                current_robots.y.push_back(current_model_states.pose[i].position.y);
                current_robots.orientation.push_back(quaternion_to_angle(current_model_states.pose[i].orientation));
                current_robots.left_wheel_vel.push_back(0);
                current_robots.right_wheel_vel.push_back(0);
                ROS_INFO_STREAM("robot addition detected: two_wheel_robot_" << index_str);
            }
        }
    }
    // update the container if there is any deletion in gazebo
    for (int i=0; i<container_size; i++) {
        if (container_index[i] != -1) {
            // not marked with -1, there is a deletion
            int current_robots_index_size = current_robots.index.size();
            for (int j=0; j<current_robots_index_size; j++) {
                // the index should be found before this loop ends
                if (container_index[i] == current_robots.index[j]) {
                    // erase the robot
                    current_robots.index.erase(current_robots.index.begin() + j);
                    current_robots.x.erase(current_robots.x.begin() + j);
                    current_robots.y.erase(current_robots.y.begin() + j);
                    current_robots.orientation.erase(current_robots.orientation.begin() + j);
                    current_robots.left_wheel_vel.erase(current_robots.left_wheel_vel.begin() + j);
                    current_robots.right_wheel_vel.erase(current_robots.right_wheel_vel.begin() + j);
                    ROS_INFO_STREAM("robot deletion detected: two_wheel_robot_"
                        << intToString(container_index[i]));
                    break;
                }
            }
        }
    }
    // reset robot position updated flag
    robot_position_updated = true;
}

// callback for service to change robot models in gazebo
// the return value of this function also shows whether the operation is successful
bool twoWheelRobotUpdateCallback(swarm_robot_srv::two_wheel_robot_updateRequest& request
    , swarm_robot_srv::two_wheel_robot_updateResponse& response
    , ros::ServiceClient add_model_client
    , ros::ServiceClient delete_model_client
    , std::string two_wheel_robot_urdf) {
    ROS_INFO("in the two wheel robot update callback");
    // pre-set the response code to success, it changes to false when at least one false detected
    response.response_code = swarm_robot_srv::two_wheel_robot_updateResponse::SUCCESS;
    // add or delete models in gazebo
    if (request.update_code <= swarm_robot_srv::two_wheel_robot_updateRequest::CODE_DELETE) {
        // update code is a negative number, meaning delete a number of robots
        std::vector<int32_t> container_index = current_robots.index;
        int delete_robot_quantity = std::abs(request.update_code);
        int current_robot_quantity = container_index.size();
        if (delete_robot_quantity > current_robot_quantity) {
            ROS_ERROR("requested deletion quantity exceeds existed robots");
            response.response_code
                = swarm_robot_srv::two_wheel_robot_updateResponse::DELETE_FAIL_EXCEED_QUANTITY;
            return true;
        }
        else {
            // start randomly choose robot to delete
            gazebo_msgs::DeleteModel delete_model_srv_msg;
            for (int i=0; i<delete_robot_quantity; i++) {
                // repeat random deletion for "delete_robot_quantity" times
                // index of the vector "container_index"
                int container_index_index = rand() % container_index.size();
                int delete_index = container_index[container_index_index];
                // avoid deletion on same robot
                container_index.erase(container_index.begin() + container_index_index);
                delete_model_srv_msg.request.model_name = "two_wheel_robot_" + intToString(delete_index);
                bool call_service = delete_model_client.call(delete_model_srv_msg);
                if (call_service) {
                    if (delete_model_srv_msg.response.success) {
                        ROS_INFO_STREAM("two_wheel_robot_" << intToString(delete_index)
                            << " has been deleted");
                    }
                    else {
                        ROS_INFO_STREAM("two_wheel_robot_" << intToString(delete_index)
                            << " deletion failed");
                        // this error is categorized to others
                        response.response_code
                            = swarm_robot_srv::two_wheel_robot_updateResponse::FAIL_OTHER_REASONS;
                    }
                }
                else {
                    ROS_ERROR("fail to connect with gazebo server");
                    response.response_code
                        = swarm_robot_srv::two_wheel_robot_updateResponse::DELETE_FAIL_NO_RESPONSE;
                    return true;
                }
            }
            // if here, the operation is either successful or fail for other reasons
            return true;
        }
    }
    else if (request.update_code == swarm_robot_srv::two_wheel_robot_updateRequest::CODE_DELETE_ALL) {
        // delete all robots sequentially, no need to randomly choose
        gazebo_msgs::DeleteModel delete_model_srv_msg;
        std::vector<int32_t> container_index = current_robots.index;
        int delete_robot_quantity = container_index.size();
        for (int i=0; i<delete_robot_quantity; i++) {
            int delete_index = container_index[i];
            delete_model_srv_msg.request.model_name = "two_wheel_robot_" + intToString(delete_index);
            bool call_service = delete_model_client.call(delete_model_srv_msg);
            if (call_service) {
                if (delete_model_srv_msg.response.success) {
                    ROS_INFO_STREAM("two_wheel_robot_" << intToString(delete_index)
                        << " has been deleted");
                }
                else {
                    ROS_INFO_STREAM("two_wheel_robot_" << intToString(delete_index)
                        << " deletion failed");
                    response.response_code
                        = swarm_robot_srv::two_wheel_robot_updateResponse::FAIL_OTHER_REASONS;
                }
            }
            else {
                ROS_ERROR("fail to connect with gazebo server");
                response.response_code
                    = swarm_robot_srv::two_wheel_robot_updateResponse::DELETE_FAIL_NO_RESPONSE;
                return true;
            }
        }
        // if here, the operation is either successful or fail for other reasons
        return true;
    }
    else if (request.update_code >= swarm_robot_srv::two_wheel_robot_updateRequest::CODE_ADD) {
        // add one robot either at specified position or random position in range
        gazebo_msgs::SpawnModel add_model_srv_msg;
        std::vector<double> new_position;
        new_position.resize(2);
        // decide where the position comes from and check its effectiveness
        if (request.add_mode == swarm_robot_srv::two_wheel_robot_updateRequest::ADD_MODE_RANDOM) {
            // add a robot at random position in range defined
            new_position = random_position(request.half_range);
            int position_generating_count = 1;  // already generate once
            while (!position_availibility(new_position) && position_generating_count < 100) {
                // generate new position until there is no collision
                new_position = random_position(request.half_range);
                position_generating_count = position_generating_count + 1;
            }
            if (position_generating_count == 100) {
                // position generated is still not availabe after 100 trials
                ROS_ERROR("add robot at random fail because range is too crowded");
                response.response_code
                    = swarm_robot_srv::two_wheel_robot_updateResponse::ADD_FAIL_TOO_CROWDED;
                return true;
            }
            // if here, position randomly generated is ok to be used
        }
        else if (request.add_mode == swarm_robot_srv::two_wheel_robot_updateRequest::ADD_MODE_SPECIFIED) {
            // add a robot at specified position
            new_position = request.position_2d;
            if (!position_availibility(new_position)) {
                // passed in position is occupied
                ROS_ERROR("add robot at specified position fail because it is occupied");
                response.response_code
                    = swarm_robot_srv::two_wheel_robot_updateResponse::ADD_FAIL_OCCUPIED;
                return true;
            }
            // if here, position passed in is ok to be used
        }
        // add the robot using position either from random generated or specified
        std::string new_model_name;
        if (current_robots.index.size() == 0) {
            // the container is empty, initialize with index 0
            new_model_name = "two_wheel_robot_0";
        }
        else {
            // queue after the last robot in the container
            new_model_name = "two_wheel_robot_" + intToString(current_robots.index.back() + 1);
        }
        add_model_srv_msg.request.model_name = new_model_name;
        add_model_srv_msg.request.model_xml = two_wheel_robot_urdf;
        add_model_srv_msg.request.robot_namespace = new_model_name;
        add_model_srv_msg.request.initial_pose.position.x = new_position[0];
        add_model_srv_msg.request.initial_pose.position.y = new_position[1];
        add_model_srv_msg.request.initial_pose.position.z = 0.0;
        add_model_srv_msg.request.initial_pose.orientation = random_quaternion();
        bool call_service = add_model_client.call(add_model_srv_msg);
        if (call_service) {
            if (add_model_srv_msg.response.success) {
                ROS_INFO_STREAM(new_model_name << " has been spawned");
            }
            else {
                ROS_INFO_STREAM(new_model_name << " fail to be spawned");
                response.response_code
                    = swarm_robot_srv::two_wheel_robot_updateResponse::FAIL_OTHER_REASONS;
            }
        }
        else {
            ROS_ERROR("fail to connect with gazebo server");
            response.response_code
                = swarm_robot_srv::two_wheel_robot_updateResponse::ADD_FAIL_NO_RESPONSE;
            return true;
        }
        // if here, the operation is either successful or fail for other reasons
        return true;
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_robot_manager");
    ros::NodeHandle nh;

    // handshake with robot name in parameter server, and get model urdf
    std::string robot_name;
    std::string two_wheel_robot_urdf;
    bool get_name, get_urdf;
    get_name = nh.getParam("/swarm_sim/robot_name", robot_name);
    get_urdf = nh.getParam("/swarm_sim/two_wheel_robot_urdf", two_wheel_robot_urdf);
    if (!(get_name && get_urdf)) {
        ROS_ERROR("simulation environmnet(parameters) is not set");
        return 0;  // return when parameter server is not good
    }
    if (robot_name != "two_wheel_robot") {
        ROS_ERROR("wrong robot according to parameter server");
        return 0;  // return when wrong robot manager is called
    }

    // check if gazebo is up and running by check service "/gazebo/set_physics_properties"
    // this service seems like the last service hosted by gazebo
    ros::Duration half_sec(0.5);
    bool gazebo_ready = ros::service::exists("/gazebo/set_physics_properties", true);
    if (!gazebo_ready) {
        // gazebo not ready
        while (!gazebo_ready) {
            ROS_INFO("waiting for gazebo");
            half_sec.sleep();
            gazebo_ready = ros::service::exists("/gazebo/set_physics_properties", true);
        }
    }
    ROS_INFO("gazebo is ready");

    // instantiate a publisher for the maintained information of two wheel robot
    ros::Publisher two_wheel_robot_publisher
        = nh.advertise<swarm_robot_msg::two_wheel_robot>("/swarm_sim/two_wheel_robot", 1);

    // instantiate a subscriber for "/gazebo/model_states"
    ros::Subscriber model_states_subscriber
        = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // this topic publish at 1000hz rate

    // instantiate a service client to get wheel velocities
    ros::ServiceClient joint_properties_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
        "/gazebo/get_joint_properties");
    gazebo_msgs::GetJointProperties joint_properties_srv_msg;

    // instantiate a service client for "/gazebo/spawn_urdf_model"
    ros::ServiceClient add_model_client
        = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    // instantiate a service client for "/gazebo/delete_model"
    ros::ServiceClient delete_model_client
        = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

    // instantiate a service server to modify the robots in gazebo
    // add or delete robot models in gazebo
    ros::ServiceServer two_wheel_robot_service
        = nh.advertiseService<swarm_robot_srv::two_wheel_robot_update::Request
        , swarm_robot_srv::two_wheel_robot_update::Response>("/swarm_sim/two_wheel_robot_update"
        , boost::bind(twoWheelRobotUpdateCallback, _1, _2, add_model_client
        , delete_model_client, two_wheel_robot_urdf));

    // publish loop
    // the topic "/swarm_sim/two_wheel_robot" will publish at full speed, as fast as possible
    // but limited by model states callback (1000hz at most)
    while (ros::ok()) {
        // get the wheel speed for the maintained robots
        int current_robot_quantity = current_robots.index.size();
        if (robot_position_updated) {
            // only update wheel velocities when positions have been update by model states callback
            for (int i=0; i<current_robot_quantity; i++) {
                // get left wheel velocity
                joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::left_motor";
                bool call_service = joint_properties_client.call(joint_properties_srv_msg);
                // if call service not successful, it will leave the joint velocity unchanged
                if (call_service) {
                    if (!joint_properties_srv_msg.response.success) {
                        // joint not found, there is possible robot deletion the container doesn't realize
                        ROS_WARN("there is possible robot deletion not updated");
                    }
                    else {
                        // update the left wheel velocity
                        current_robots.left_wheel_vel[i] = joint_properties_srv_msg.response.rate[0];
                    }
                }
                else {
                    ROS_ERROR("fail to connect with gazebo server");
                    // do not return here
                }
                // get right wheel velocity
                joint_properties_srv_msg.request.joint_name
                    = "two_wheel_robot_" + intToString(current_robots.index[i]) + "::right_motor";
                call_service = joint_properties_client.call(joint_properties_srv_msg);
                // if call service not successful, it will leave the joint velocity unchanged
                if (call_service) {
                    if (!joint_properties_srv_msg.response.success) {
                        // joint not found, there is possible robot deletion the container doesn't realize
                        ROS_WARN("there is possible robot deletion not updated");
                    }
                    else {
                        // update the right wheel velocity
                        current_robots.right_wheel_vel[i] = joint_properties_srv_msg.response.rate[0];
                    }
                }
                else {
                    ROS_ERROR("fail to connect with gazebo server");
                }
            }
            // publish the two wheel robot information container
            two_wheel_robot_publisher.publish(current_robots);
            // reset the position update flag
            robot_position_updated = false;
        }
        // update global variables
        ros::spinOnce();
    }

    return 0;

}

