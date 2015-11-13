// the trajectory action server for two wheel robot

// considering the wheel trajectory of moving to a target position
    // one seemingly simple way is first rotating to right angle
    // then moving in straight line to the target
// the strategy using here is, the center of the robot will always moving in a circular path
// so that the wheel trajectory could be linearly interpolated
// the center of the circular path is on the common axis of the two wheel
// and the distances to both current robot position and target robot position are the same

// there is a situation that target robot position is on the common axis
// the strategy is compatible with this situation

// communication includes
    // subscribe to topic "swarm_robot_poses" with message swarm_robot_poses.msg
    // subscribe to topic "two_wheel_poses" with message two_wheel_poses.msg
    // publish to topic "two_wheel_poses_cmd" with message two_wheel_poses.msg
    // host a action server "two_wheel_traj_action" with message swarm_robot_traj.action

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>
#include <swarm_robot_msgs/two_wheel_poses.h>

class TwoWheelTrajActionServer {
public:
    TwoWheelTrajActionServer(ros::NodeHandle* nodehandle);

private:
    void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder);
    void twoWheelPosesCb(const swarm_robot_msgs::two_wheel_poses& message_holder);
    void executeCb(const 
        actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction>::GoalConstPtr& goal);

    ros::NodeHandle nh_;  // a node handle is needed

    // define a subscribers to topic "swarm_robot_poses" and "two_wheel_poses"
    ros::Subscriber swarm_robot_poses_subscriber;
    ros::Subscriber two_wheel_poses_subscriber;
    // define a publisher to topic "two_wheel_poses_cmd"
    ros::Publisher two_wheel_poses_cmd_publisher;
    // define an action server
    actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction> as_;
    
    // messages
    swarm_robot_msgs::swarm_robot_poses robot_poses_msg;  // current robot poses
    swarm_robot_msgs::two_wheel_poses wheel_poses_msg;  // current wheel poses
    swarm_robot_msgs::two_wheel_poses wheel_poses_cmd_msg;  // wheel poses command
    // action messages
    swarm_robot_action::swarm_robot_trajActionGoal goal_;
    swarm_robot_action::swarm_robot_trajActionResult result_;
    swarm_robot_action::swarm_robot_trajActionFeedback feedback_;

    // other variables
    std::string robot_model_name;  // from parameter server
    int robot_quantity;  // from parameter server
    bool b_robot_poses_cb_started;
    bool b_wheel_poses_cb_started;
};

// most initialization work will be done here
TwoWheelTrajActionServer::TwoWheelTrajActionServer(ros::NodeHandle* nodehandle):
    nh_(*nodehandle),  // dereference the pointer and pass the value
    as_(nh_, "two_wheel_traj_action", boost::bind(&TwoWheelTrajActionServer::executeCb, this, _1), false)
{
    ROS_INFO("in constructor of TwoWheelTrajActionServer...");

    // initialize the subscribers
    b_robot_poses_cb_started = false;
    swarm_robot_poses_subscriber = nh_.subscribe("swarm_robot_poses", 1, swarmRobotPosesCb);
    b_poses_callback_started = false;
    two_wheel_poses_subscriber = nh_.subscribe("two_wheel_poses", 1, twoWheelPosesCb);
    // initialize the publisher
    two_wheel_poses_cmd_publisher = nh_.advertise<swarm_robot_msgs::two_wheel_poses>("two_wheel_poses_cmd", 1);

    // get initialization message of robot swarm from parameter server
    bool get_name, get_quantity;
    get_name = nh_.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh_.getParam("/robot_quantity", robot_quantity);
    if (!(get_name && get_quantity))
        return 0;  // return if fail to get parameter

    // resize messages




    as_.start();  // start the "two_wheel_traj_action"
}

// callback for message from topic "swarm_robot_poses"
void TwoWheelTrajActionServer::swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder) {
    if (!b_robot_poses_cb_started)  // first time to be invoked
        b_robot_poses_cb_started = true;
    robot_poses_msg = message_holder;
}

// callback for message from topic "two_wheel_poses"
void TwoWheelTrajActionServer::twoWheelPosesCb(const swarm_robot_msgs::two_wheel_poses& message_holder) {
    if (!b_wheel_poses_cb_started)  // first time to be invoked
        b_wheel_poses_cb_started = true;
    wheel_poses_msg = message_holder;
}

// most practical work will be done here
void TwoWheelTrajActionServer::executeCb(const actionlib::
    SimpleActionServer<swarm_robot_action::swarm_robot_trajAction>::GoalConstPtr& goal)
{
    ROS_INFO("in executeCb...");

}


int main(int argc, char **argv) {
    ros::init(argc, argv, "two_wheel_traj_interpolator_as");
    ros::NodeHandle nh;

    ROS_INFO("instantiating an object of class TwoWheelTrajActionServer...");
    TwoWheelTrajActionServer as_object(&nh);

    ROS_INFO("going into spin...");

    while (ros::ok()) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
    }
    return 0;
}

