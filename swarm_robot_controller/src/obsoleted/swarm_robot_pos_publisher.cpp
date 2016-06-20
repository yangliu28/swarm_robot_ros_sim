// this node will publish the topic "swarm_robot_poses"
// which are the current positions and orientations of all robots

// by subscribe to the topic: /gazebo/model_state (publishing at 1000Hz)
// the purpose is to simplify message from above topic and publish them

#include <ros/ros.h>
#include <string>
#include <vector>
#include <math.h>
#include <gazebo_msgs/ModelStates.h>
// be careful, there is another message call gazebo_msgs/ModelState
#include <swarm_robot_msgs/swarm_robot_poses.h>

// constant
const double g_publish_frequency = 50.0;  // publish swarm_robot_poses at this frequency

// other global variables
std::vector<double> g_x;  // containers for swarm_robot_pos message
std::vector<double> g_y;
std::vector<double> g_angle;
bool g_callback_started = false;  // first time callback is invoked
std::string g_robot_model_name;  // from parameter server
int g_robot_quantity;  // from parameter server

void modelStatesCallback(const gazebo_msgs::ModelStates& current_model_states) {
    // indicate other processes that modelStatesCallback has been invoked
    if (!g_callback_started) {
        g_callback_started = true;
    }
    // be aware that there will be other robot models in gazebo like the ground_plane
    int model_quantity = current_model_states.name.size();
    for (int i=0; i<model_quantity; i++) {
        // check the match with parameter server
        std::size_t found = current_model_states.name[i].find(g_robot_model_name);
        if (found != std::string::npos) {
            // find a name that is an robot model name
            // get the robot index from the name
            // there is an underscore between the robot_model_name and index
            std::string s_index = current_model_states.name[i].substr(g_robot_model_name.size() + 1);
            int i_index = std::atoi(s_index.c_str());  // why stoi not working here?
            // refresh data
            g_x[i_index] = current_model_states.pose[i].position.x;
            g_y[i_index] = current_model_states.pose[i].position.y;
            double quaternion_w = current_model_states.pose[i].orientation.w;
            double quaternion_z = current_model_states.pose[i].orientation.z;
            g_angle[i_index] = 2 * atan(quaternion_z/quaternion_w);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "swarm_robot_pos_publisher");
    ros::NodeHandle nh;

    // get initialization information of robot swarm from parameter server
    bool get_name, get_quantity;
    get_name = nh.getParam("/robot_model_name", g_robot_model_name);
    get_quantity = nh.getParam("/robot_quantity", g_robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // initialize the message holders
    g_x.resize(g_robot_quantity);
    g_y.resize(g_robot_quantity);
    g_angle.resize(g_robot_quantity);

    // initialize a subscriber for "/gazebo/model_states"
    // buffer size: 1, discard any data that is not real time
    // this topic is being published by gazebo at 1000Hz
    ros::Subscriber model_state_subscriber = nh.subscribe("/gazebo/model_states", 1, modelStatesCallback);

    // initialize a publisher with topic "/swarm_robot_poses", buffer size is 1
    ros::Publisher swarm_robot_poses_publisher = 
        nh.advertise<swarm_robot_msgs::swarm_robot_poses>("swarm_robot_poses", 1);

    // check if modelStatesCallback has been invoked the first time
    while (!g_callback_started) {
        ros::spinOnce();  // allow data update from callback
    }

    // if here, modelStatesCallback has started to be invoked
    // prepare messages to be published
    swarm_robot_msgs::swarm_robot_poses current_poses;
    current_poses.x.resize(g_robot_quantity);
    current_poses.y.resize(g_robot_quantity);
    current_poses.angle.resize(g_robot_quantity);

    // publishing loop
    ros::Rate naptime(g_publish_frequency);  // publishing frequency control
    while (ros::ok()) {
        current_poses.x = g_x;
        current_poses.y = g_y;
        current_poses.angle = g_angle;
        swarm_robot_poses_publisher.publish(current_poses);
        ros::spinOnce();  // have to do this
        naptime.sleep();  // frequency control
    }

    return 0;
}
