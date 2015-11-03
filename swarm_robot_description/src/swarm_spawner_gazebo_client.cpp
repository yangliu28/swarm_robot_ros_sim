// spawn robot swarm at customized number and positions using gazebo service

#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

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
    inUrdf.open("two_wheel_robot.urdf");
    strStream << inUrdf.rdbuf();
    xmlStr = strStream.str();

    // prepare service message
    spawn_model_srv_msg.request.model_name = "two_wheel_robot";
    spawn_model_srv_msg.request.model_xml = xmlStr;
    spawn_model_srv_msg.request.robot_namespace = "two_wheel_robot";
    spawn_model_srv_msg.request.initial_pose.position.x = 0;
    spawn_model_srv_msg.request.initial_pose.position.y = 0;
    spawn_model_srv_msg.request.initial_pose.position.z = 0;
    spawn_model_srv_msg.request.initial_pose.orientation.x = 0;
    spawn_model_srv_msg.request.initial_pose.orientation.y = 0;
    spawn_model_srv_msg.request.initial_pose.orientation.z = 0;
    spawn_model_srv_msg.request.initial_pose.orientation.w = 1;
    spawn_model_srv_msg.request.reference_frame = "world";

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

