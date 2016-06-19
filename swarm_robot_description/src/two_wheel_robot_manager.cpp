// this node manage the number and index of present two wheel robot

// publish topic on two wheel robot information
    // robot index, 2D position and speed of wheels
// use gazebo service to spawn and delete model in gazebo
// (compatible when a robot model is deleted directly from gazebo gui)
// accept service request the command of adding or deleting robot model
// (adding or deleting action will be reflected directly in the topic msg)

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
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <string>
#include <vector>

// global variables
// THE container maintained by this node
swarm_robot_msgs::two_wheel_robot current_robots;
bool robot_position_updated = false;

// callback for getting robot positions
// also check and update if there is any addition or deletion of robots
void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // parsing the two wheel robots from all the robot models
    // possible that there is models for obstacles or other environments
    int model_quantity = current_model_states.name.size();
    // the index of robots in the container
    std::vector<int32_t> container_index = current_robots.index;
    // the index of robots found in gazebo
    std::vector<int32_t> gazebo_index;
    gazebo_index.clear();
    for (int i=0; i<model_quantity; i++) {
        // check if it is a two wheel robot
        // there is a underscore between the name and the index
        std::size_t found = current_model_states.name[i].find("two_wheel_robot");
        if (found != std::string::npos) {
            // a two_wheel_robot has been found
            // get the robot index
            // 16 = 15 + 1, 15 is the length of "two_wheel_robot"
            std::string index_str = current_model_states.name[i].substr(16);
            int index_found = std::atoi(index_str.c_str());
            // search in the container
            int container_size = container_index.size();
            for (int j=0; j<container_size; j++) {
                if (index_found == container_index[j])
            }
        }
    }


    // reset robot_position_updated flag
    robot_position_updated = true;
}

// callback for service to change robot models in gazebo
bool twoWheelRobotUpdateCallback(swarm_robot_srv::swarm_robot_updateRequest& request
    , swarm_robot_srv::swarm_robot_updateResponse& response
    , ros::ServiceClient add_model_client
    , ros::ServiceClient delete_model_client) {

}



int main(int argc, char **argv) {
    ros::init(argc, argv, "/swarm_sim/two_wheel_robot_manager");
    ros::NodeHandle nh;

    // handshake with robot name in parameter server, and get model urdf
    std::string robot_name;
    std::string two_wheel_robot_urdf;
    bool get_name, get_urdf;
    get_name = nh.getParam("/swarm_sim/robot_name", robot_name);
    get_urdf = nh.getParam("/swarm_sim/two_wheel_robot_urdf", two_wheel_robot_urdf);
    if (!(get_name && get_urdf)) {
        ROS_ERROR("parameter server is not set");
        return 0;  // return when parameter server is not good
    }
    if (robot_name != "two_wheel_robot") {
        ROS_ERROR("wrong robot according to parameter server");
        return 0;  // return when wrong robot manager is called
    }

    // check if gazebo is up and running by check service "/gazebo/set_physics_properties"
    // this service seems like the last service hosted by gazebo
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

    // initialize THE container
    current_robots.index.clear();
    current_robots.x.clear();
    current_robots.y.clear();
    current_robots.orientation.clear();
    current_robots.left_wheel_vel.clear();
    current_robots.right_wheel_vel.clear();

    // instantiate a publisher for the managed information of two wheel robot
    ros::Publisher two_wheel_robot_publisher
        = nh.advertise<swarm_robot_msgs::two_wheel_robot>("/swarm_sim/two_wheel_robot", 1);

    // instantiate a subscriber for "/gazebo/model_states"
    ros::Subscriber model_states_subscriber
        = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);
    // this topic publish at 1000hz rate

    // instantiate a service client to get wheel velocities
    ros::ServiceClient joint_properties_client = nh.serviceClient<gazebo_msgs::GetJointProperties>(
        "/gazebo/get_joint_properties");
    gazebo_msgs::GetJointProperties joint_properties_srv_msg;

    // instantiate a service server to modify the robots in gazebo
    // add or delete robot models in gazebo
    ros::ServiceServer two_wheel_robot_service
        = nh.advertiseService("/swarm_sim/two_wheel_robot_update", twoWheelRobotUpdateCallback);

    // instantiate a service client for "/gazebo/spawn_urdf_model"
    ros::ServiceClient add_model_client
        = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    gazebo_msgs::SpawnModel add_model_srv_msg;  // service message
    geometry_msgs::Pose model_pose;  // pose message for service message

    // instantiate a service client for "/gazebo/delete_model"
    ros::ServiceClient delete_model_client
        = nh.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");
    gazebo_msgs::DeleteModel delete_model_srv_msg;  // service message






}







