// spawn robot swarm at customized quantity, positions and orientation using gazebo service

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <Eigen/Geometry>

using namespace Eigen;

geometry_msgs::Pose[] poseGenerator(int quantity, double half_range) {
    // input the quantity of swarm robots and position range, output the pose message

    geometry_msgs::Pose[] random_pose;  // output data
    random_pose.resize(quantity);

    Matrix<float, quantity, 3> random_matrix;  // container for random numbers
    // first colomn for position.x
    // second colomn for position.y
    // third colomn for rotation angle
    random_matrix = MatrixXd::Random(quantity, 3);  // generate random numbers in (-1, 1)

    // map data, 1st & 2nd colomns to (-half_range, half_range), 3rd to (-M_PI, M_PI)
    random_matrix.col(0) = random_matrix.col(0) * half_range;
    random_matrix.col(1) = random_matrix.col(1) * half_range;    
    random_matrix.col(2) = random_matrix.col(2) * M_PI;



}



int main(int argc, char **argv) {
    ros::init(argc, argv, "swarm_spawner_gazebo_client");
    ros::NodeHandle nh;
    // service client for service /gazebo/spawn_model
    ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel spawn_model_srv_msg;  // service message
    geometry_msgs::Pose model_pose;  // model pose message for service message

    // read the urdf file into string
    std::ifstream inUrdf;
    std::stringstream strStream;
    std::string xmlStr;
    // this is an absolute path, which means this node has to be invoked at ros_ws directory
    inUrdf.open("src/swarm_robot_ros_sim/swarm_robot_description/urdf/two_wheel_robot.urdf");
    strStream << inUrdf.rdbuf();
    xmlStr = strStream.str();



    AngleAxisf randomRotation(, Vector3f::UnitZ())

    // prepare service message
    spawn_model_srv_msg.request.model_name = "two_wheel_robot0";
    spawn_model_srv_msg.request.model_xml = xmlStr;
    spawn_model_srv_msg.request.robot_namespace = "two_wheel_robot0";
    spawn_model_srv_msg.request.initial_pose.position.x = 0.5;
    spawn_model_srv_msg.request.initial_pose.position.y = 0.5;
    spawn_model_srv_msg.request.initial_pose.position.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0.0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 1.0;
    spawn_model_srv_msg.request.reference_frame = "world";

    ros::Duration(5.0).sleep();  // sleep for 5 second

    // call service and get response
    bool callService = client.call(spawn_model_srv_msg);
    if (callService) {
        if (spawn_model_srv_msg.response.success)
            std::cout << "model two_wheel_robot0 has been spawned in gazebo" << std::endl;
        else
            std::cout << "spawn model failed" << std::endl;
    }
    else {
        ROS_ERROR("Failed to call the gazebo service");
        return 1;
    }

    return 0;
}

