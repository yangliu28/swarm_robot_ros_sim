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
    void executeCb(const 
        actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction>::GoalConstPtr& goal);

    ros::NodeHandle nh_;  // a node handle is needed

    // define a subscriber to topic "two_wheel_poses"
    ros::Subscriber two_wheel_poses_subscriber;
    // define a publisher to topic "two_wheel_poses_cmd"
    ros::Publisher two_wheel_poses_cmd_publisher;
    // define an action server
    actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction> as_;
    
    // messages
    swarm_robot_msgs::two_wheel_poses cmd_msg;
    swarm_robot_action::swarm_robot_trajActionGoal goal_;
    swarm_robot_action::swarm_robot_trajActionResult result_;
    swarm_robot_action::swarm_robot_trajActionFeedback feedback_;

};

// most initialization work will be done here
TwoWheelTrajActionServer::TwoWheelTrajActionServer(ros::NodeHandle* nodehandle):
    nh_(*nodehandle),  // dereference the pointer and pass the value
    as_(nh_, "two_wheel_traj_action", boost::bind(&TwoWheelTrajActionServer::executeCb, this, _1), false)
{
    ROS_INFO("in constructor of TwoWheelTrajActionServer...");

    // initialize the subscriber
    two_wheel_poses_subscriber = nh.subscribe("two_wheel_poses", 1, twoWheelPosesCallback);
    // initialize the publisher
    two_wheel_poses_cmd_publisher = nh.advertise<swarm_robot_msgs::two_wheel_poses>("two_wheel_poses_cmd", 1);








    as_.start();  // start the "two_wheel_traj_action"
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

