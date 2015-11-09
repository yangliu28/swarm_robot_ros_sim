// this node will publish the topic "two_wheel_poses"
// which are the current wheel positions of all robots

// by calling the service: /gazebo/get_joint_properties
    // (gazebo is publishing a topic: /gazebo/link_states, not handy to use)
// the purpose is to publish current wheel positions in a straightforward way

#include <ros/ros.h>
#include <string>
#include <gazebo_msgs/GetJointProperties.h>  // service message
#include <swarm_robot_msgs/two_wheel_poses.h>  // topic message

// constants
// publish two_wheel_poses at this frequency
// not really working, the frequency is limited by calling gazebo service
// the maximum publishing frequency is around 30Hz
// using the gazebo topic may be a better choice
const double g_publish_frequency = 100.0;

// int to string converter
std::string intToString(int a) {
    std::stringstream ss;
    ss << a;
    return ss.str();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "two_wheel_pos_publisher");
    ros::NodeHandle nh;

    // get initialization message of robot swarm from parameter server
    std::string robot_model_name;
    int robot_quantity;
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // initialize a service client to get wheel positions
    ros::ServiceClient get_wheel_position_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
        "/gazebo/get_joint_properties");
    gazebo_msgs::GetJointProperties get_wheel_position_srv_msg;  // service message

    // make sure get_joint_properties service is ready
    ros::Duration half_sec(0.5);
    bool service_ready = false;
    while (!service_ready) {
        service_ready = ros::service::exists("/gazebo/get_joint_properties",true);
        ROS_INFO("waiting for /gazebo/get_joint_properties service");
        half_sec.sleep();
    }
    ROS_INFO("/gazebo/get_joint_properties service exists");

    // initialize a publisher with topic name "two_wheel_poses", buffer size is 1
    ros::Publisher two_wheel_poses_publisher = 
        nh.advertise<swarm_robot_msgs::two_wheel_poses>("two_wheel_poses", 1);

    // prepare messages to be published
    swarm_robot_msgs::two_wheel_poses current_poses;
    current_poses.left_wheel_pos.resize(robot_quantity);
    current_poses.left_wheel_vel.resize(robot_quantity);
    current_poses.right_wheel_pos.resize(robot_quantity);
    current_poses.right_wheel_vel.resize(robot_quantity);

    // publishing loop
    ros::Rate naptime(g_publish_frequency);  // publishing frequency control
    std::string s_index;  // index as a string
    std::string left_motor_name;  // name of left motor
    std::string right_motor_name;  // name of right motor
    while (ros::ok()) {
        // calling service loop
        for (int i=0; i<robot_quantity; i++) {
            s_index = intToString(i);
            // actually cheat here, no better way to get the key words "left_motor" and "right_motor"
            left_motor_name = robot_model_name + "_" + s_index + "::left_motor";
            right_motor_name = robot_model_name + "_" + s_index + "::right_motor";

            // get left wheel positions
            get_wheel_position_srv_msg.request.joint_name = left_motor_name;
            get_wheel_position_client.call(get_wheel_position_srv_msg);
            // prepare left wheel message to be published
            current_poses.left_wheel_pos[i] = get_wheel_position_srv_msg.response.position[0];
            current_poses.left_wheel_vel[i] = get_wheel_position_srv_msg.response.rate[0];

            // get right wheel positions
            get_wheel_position_srv_msg.request.joint_name = right_motor_name;
            get_wheel_position_client.call(get_wheel_position_srv_msg);
            // prepare right wheel message to be published
            current_poses.right_wheel_pos[i] = get_wheel_position_srv_msg.response.position[0];
            current_poses.right_wheel_vel[i] = get_wheel_position_srv_msg.response.rate[0];
        }
        // publish data
        two_wheel_poses_publisher.publish(current_poses);
        naptime.sleep();
    }
}

