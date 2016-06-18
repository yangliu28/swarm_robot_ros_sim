// this node manage the number and index of present two wheel robot

// publish topic on two wheel robot information
    // robot index, 2D position and speed of wheels
// use gazebo service to spawn and delete model in gazebo
// (compatible when a robot model is deleted directly from gazebo gui)
// accept service request the command of adding or deleting robot model
// (adding or deleting action will be reflected directly from the topic msg)

// this node is not compatible with different robot models, for two wheel orbot only
// because only two wheel robot has wheel speed, may write another node for other robot models

// both low and high level robot control subscribe to same robot information topic
// although it's better to divide the message into two, and publish at different frequency
// this one topic method can be reused when it comes to other robot models




// check collision when adding robots




#include <ros/ros.h>
#include <swarm_robot_msgs/two_wheel_robot.h>
#include <swarm_robot_srv/swarm_robot_update.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetJointProperties.h>

// global variables
swarm_robot_msgs::two_wheel_robot current_robots;  // THE container

// callback for getting robot positions
void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {

}

// callback for service to change robot models in gazebo
bool twoWheelRobotUpdateCallback(swarm_robot_srv::swarm_robot_update& )



int main(int argc, char **argv) {
    ros::init(argc, argv, "/swarm_sim/two_wheel_robot_manager");
    ros::NodeHandle nh;

    // check if gazebo is up and running by check service "/gazebo/set_physics_properties"
    ros::Duration half_sec(0.5);
    bool gazebo_ready = ros::service::exist("/gazebo/set_physics_properties", true);
    if (!gazebo_ready) {
        // gazebo not ready
        while (!gazebo_ready) {
            ROS_INFO("waiting for gazebo");
            half_sec.sleep();
            gazebo_ready = ros::service::exist("/gazebo/set_physics_properties", true);
        }
    }

    // initialize a publisher for the managed information of two wheel robot
    ros::Publisher two_wheel_robot_publisher
        = nh.advertise<swarm_robot_msgs::two_wheel_robot>("/swarm_sim/two_wheel_robot", 1);

    // initialize a subscriber for "/gazebo/model_states"
    ros::Subscriber model_states_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // this topic publish at 1000hz rate

    // initialize a service client to get wheel velocities
    ros::ServiceClient joint_properties_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
        "/gazebo/get_joint_properties");
    gazebo_msgs::GetJointProperties joint_properties_srv_msg;

    // initialize a service server to modify the robots in gazebo
    // add or delete robot models in gazebo
    ros::ServiceServer two_wheel_robot_service
        = nh.advertiseService("/swarm_sim/two_wheel_robot_update", twoWheelRobotUpdateCallback);




}











