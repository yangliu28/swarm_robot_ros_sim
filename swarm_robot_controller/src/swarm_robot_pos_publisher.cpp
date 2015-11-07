// this node will publish topic about the position and orientation of all the swarm robot
// by subscribe to the /gazebo/get_model_state service

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <gazebo_msgs/GetModelState.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_pos_publisher");
    ros::NodeHandle nh;

    // make sure /gazebo/get_model_state service is ready
    bool service_ready = false;
    ros::Duration half_sec(0.5);
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/get_model_state", true);
        ROS_INFO("waiting for /gazebo/get_model_state service");
        half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_model_state service exists");

    // initialize a service client to get positions of all the swarm robots
    ros::ServiceClient get_model_state_client = nh.serviceClient<gazebo_msgs::GetModelState>(
        "/gezebo/get_model_state");
    gazebo_msgs::GetModelState get_model_state_srv_msg;

    // call the service
    get_model_state_srv_msg.request.model_name = "two_wheel_robot_0";
    get_model_state_srv_msg.request.relative_entity_name = "world";
    get_model_state_client.call(get_model_state_srv_msg);
    

    while(ros::ok()) {
        // call the ser
    }




}

