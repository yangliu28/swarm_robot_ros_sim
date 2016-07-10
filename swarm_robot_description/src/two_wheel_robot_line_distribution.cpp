// this node add a line of two wheel robots by talking to the manager node
// through the "/swarm_sim/two_wheel_robot_update" service

// this program is used specifically for preparing a setup for testing aggregation
// alternatively it's possible by using the line formation program, but it takes much more time

// since this program is designed for exclusive purpose
// it may not be compatible with other add delete robot node like the batch add node
// use together with
    // roslaunch swarm_robot_description two_wheel_robot.launch robot_quantity:=0
    // rosservice call /swarm_sim/two_wheel_robot_update ... (delete all robots)

#include <ros/ros.h>
#include <swarm_robot_srv/two_wheel_robot_update.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_robot_batch_add");
    ros::NodeHandle nh("~");  // private namespace, to get private parameters

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

    // check if service is ready, "/swarm_sim/two_wheel_robot_update"
    ros::Duration half_sec(0.5);
    bool service_ready = ros::service::exists("/swarm_sim/two_wheel_robot_update", true);
    if (!service_ready) {
        while (!service_ready) {
            ROS_INFO("waiting for service /swarm_sim/two_wheel_robot_update");
            half_sec.sleep();
            service_ready = ros::service::exists("/swarm_sim/two_wheel_robot_update", true);
        }
    }
    ROS_INFO("/swarm_sim/two_wheel_robot_update service is ready");

    // instantiate a service client
    ros::ServiceClient two_wheel_robot_update_client
        = nh.serviceClient<swarm_robot_srv::two_wheel_robot_update>("/swarm_sim/two_wheel_robot_update");
    swarm_robot_srv::two_wheel_robot_update two_wheel_robot_update_srv_msg;  // service msg

    // get the private parameters passed to this node
    // parameter: robot_quantity
    int robot_quantity = 10;  // the default value
    bool get_robot_quantity = nh.getParam("robot_quantity", robot_quantity);
    if (get_robot_quantity) {
        ROS_INFO_STREAM("using robot quantity passed in: " << robot_quantity);
        // delete this parameter, in case it will be reused because it continues to exist
        nh.deleteParam("robot_quantity");
    }
    else
        ROS_INFO_STREAM("using default robot quantity: 10");
    // parameter: robot_space
    double robot_space = 0.5;  // the default value
    bool get_robot_space = nh.getParam("robot_space", robot_space);
    if (get_robot_space) {
        ROS_INFO_STREAM("using robot space passed in: " << robot_space);
        nh.deleteParam("robot_space");
    }
    else
        ROS_INFO_STREAM("using default robot space: 0.5");

    // prepare the service message
    two_wheel_robot_update_srv_msg.request.update_code = 1;
    two_wheel_robot_update_srv_msg.request.add_mode
        = swarm_robot_srv::two_wheel_robot_updateRequest::ADD_MODE_SPECIFIED;
    two_wheel_robot_update_srv_msg.request.position_2d.resize(2);
    two_wheel_robot_update_srv_msg.request.position_2d[1] = 0.0;  // y coordinate is 0.0

    // call service to add robot repeatedly
    bool call_service;
    ros::Duration minimal_delay(0.01);  // moderate waiting time
    // position of the robots
    double start_position = -double(robot_quantity-1) * robot_space / 2.0;
    double current_position;
    for (int i=0; i<robot_quantity; i++) {
        current_position = start_position + i * robot_space;
        two_wheel_robot_update_srv_msg.request.position_2d[0] = current_position;  // along x coordinate
        call_service = two_wheel_robot_update_client.call(two_wheel_robot_update_srv_msg);
        if (call_service) {
            switch (two_wheel_robot_update_srv_msg.response.response_code) {
                case swarm_robot_srv::two_wheel_robot_updateResponse::SUCCESS:
                    ROS_INFO("success");
                    break;
                case swarm_robot_srv::two_wheel_robot_updateResponse::ADD_FAIL_NO_RESPONSE:
                    ROS_WARN("add fail because no response from gazebo");
                    break;
                case swarm_robot_srv::two_wheel_robot_updateResponse::ADD_FAIL_TOO_CROWDED:
                    ROS_WARN("add fail because the range is too crowded");
                    break;
                case swarm_robot_srv::two_wheel_robot_updateResponse::FAIL_OTHER_REASONS:
                    // keep trying for this case
                    ROS_WARN("fail because of other reasons");
                    while (!(call_service
                        = two_wheel_robot_update_client.call(two_wheel_robot_update_srv_msg))) {
                        ROS_WARN("keep trying to add one robot");
                        minimal_delay.sleep();
                    }
                    ROS_INFO("success after retry");
                    break;
                default:
                    // there is no reason to be here, otherwise there is problems in service request
                    ROS_ERROR("wrong response code, check the service request for details");
            }
        }
        else {
            ROS_ERROR("fail to connect to service /swarm_sim/two_wheel_robot_update");
        }

        // delay for a minimal time, so that topic message in manager can update
        minimal_delay.sleep();
    }

    return 0;

}

