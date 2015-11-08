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

    // get the number of swarm robots
    int robot_quantity;
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!get_quantity)
        return 0;  // return if fail to get parameter



    while(ros::ok()) {
        for (int i=0; i<robot_quantity; i++) {
            // prepare the service message
            std::string index_string = intToString(i);
            get_model_state_srv_msg.request.model_name = "two_wheel_robot_" + index_string;
            get_model_state_srv_msg.request.relative_entity_name = "world";
            // call the service
            get_model_state_client.call(get_model_state_srv_msg);
        }
        
    }




}

