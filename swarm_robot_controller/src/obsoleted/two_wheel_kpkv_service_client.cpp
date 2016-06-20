// service client to change kp&kv for two wheel robot in the minimal controller

#include <ros/ros.h>
#include <swarm_robot_srv/kpkv_msg.h>
#include <string>
#include <iostream>

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_kpkv_service_client");
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<swarm_robot_srv::kpkv_msg>("two_wheel_kpkv");
    swarm_robot_srv::kpkv_msg srv_msg;
    // cin variables
    double in_kp, in_kv;
    while(ros::ok()) {
        // get the value of kp and kv
        std::cout << "enter the value of kp: ";
        std::cin >> in_kp;
        std::cout << "enter the value of kv: ";
        std::cin >> in_kv;
        // set the service request
        srv_msg.request.kp = in_kp;
        srv_msg.request.kv = in_kv;
        // call service and get response
        if (client.call(srv_msg)) {
            if (srv_msg.response.setting_is_done)
                std::cout << "setting is done." << std::endl;
            else
                std::cout << "setting is not done." << std::endl;
        }
        else {
            ROS_ERROR("Failed to call service kpkv_service");
            return 0;
        }
    }
    return 0;
}
