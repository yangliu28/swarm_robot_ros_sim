// spawn robot swarm at customized quantity, positions and orientation using gazebo service

// all parameters are from parameter server
    // robot model: /robot_model_path
    // robot quantity: /robot_quantity
    // robot distribution: /half_range

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

using namespace Eigen;

Matrix<double, Dynamic, 3> randomGenerator(int quantity, double half_range) {
    // input the quantity of swarm robots and position range, output the pose message

    MatrixXd random_matrix;  // output data
    random_matrix.resize(quantity, 3);
    // first colomn for position.x
    // second colomn for position.y
    // third colomn for rotation angle
    random_matrix = MatrixXd::Random(quantity, 3);  // generate random numbers in (-1, 1)

    // map data, 1st & 2nd colomns to (-half_range, half_range), 3rd to (-M_PI, M_PI)
    random_matrix.col(0) = random_matrix.col(0) * half_range;
    random_matrix.col(1) = random_matrix.col(1) * half_range;    
    random_matrix.col(2) = random_matrix.col(2) * M_PI;

    return random_matrix;
}

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_spawner_gazebo_client");
    ros::NodeHandle nh;
    // service client for service /gazebo/spawn_model
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg;  // service message
    geometry_msgs::Pose model_pose;  // model pose message for service message

    // get initialization information of robot swarm from parameter
    std::string robot_model_name;
    std::string robot_model_path;
    int robot_quantity;
    double half_range;
    bool get_name, get_path, get_quantity, get_range;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_path = nh.getParam("/robot_model_path", robot_model_path);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    get_range = nh.getParam("/half_range", half_range);
    if (!(get_name && get_path && get_quantity && get_range))
        return 0;  // return if fail to get parameters

    // prepare: the xml for service call, read urdf into string
    std::ifstream inXml;
    std::stringstream strStream;
    std::string xmlStr;
    // this is an absolute path, which means this node has to be invoked at ros_ws directory
    inXml.open(robot_model_path.c_str());  // why is c_str() needed?
    strStream << inXml.rdbuf();
    xmlStr = strStream.str();
    // prepare: the service message
    spawn_model_srv_msg.request.model_xml = xmlStr;
    spawn_model_srv_msg.request.initial_pose.position.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv_msg.request.reference_frame = "world";
    // prepare: the random numbers used in the service call, the position and orientation
    MatrixXd randomNumbers;
    randomNumbers = randomGenerator(robot_quantity, half_range);

    // begin spawn robot through gazebo service
    for (int i=0; i<robot_quantity; i++) {
        std::string index_string = intToString(i);
        // prepare service message for each swarm robot
        spawn_model_srv_msg.request.model_name = robot_model_name + "_" + index_string;
        spawn_model_srv_msg.request.robot_namespace = robot_model_name + "_" + index_string;
        spawn_model_srv_msg.request.initial_pose.position.x = randomNumbers(i, 0);
        spawn_model_srv_msg.request.initial_pose.position.y = randomNumbers(i, 1);
        // calculate the quaternion from random orientation angle
        AngleAxisf aaf(randomNumbers(i, 2), Vector3f::UnitZ());
        Quaternionf qf(aaf);  // convert to quaternion
        // only z & w need to be changed, because of rotation along z axis
        spawn_model_srv_msg.request.initial_pose.orientation.z = qf.z();
        spawn_model_srv_msg.request.initial_pose.orientation.w = qf.w();

        // call service and get response
        bool call_service = client.call(spawn_model_srv_msg);  // call the server
        if (call_service) {
            if (spawn_model_srv_msg.response.success)
                std::cout << robot_model_name << "_" << index_string << " has been spawned" << std::endl;
            else
                std::cout << robot_model_name << "_" << index_string << " spawn failed" << std::endl;
        }
        else {
            ROS_ERROR("Failed to connect with gazebo server");
            return 0;
        }
    }

    return 0;
}

