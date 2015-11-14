// the trajectory action server for two wheel robot

// communication includes
    // subscribe to topic "swarm_robot_poses" with message swarm_robot_poses.msg
    // subscribe to topic "two_wheel_poses" with message two_wheel_poses.msg
    // publish to topic "two_wheel_poses_cmd" with message two_wheel_poses.msg
    // host a action server "two_wheel_traj_action" with message swarm_robot_traj.action

// there are different wheel trajectories of moving to a target position
// Method 1:
    // first rotating the robot to the direction of the target position
    // then moving the robot in straight line to the target position
    // pros:
        // simple and straightforward wheel position increment calculation
        // easy to debug the potential problems
    // cons:
        // ugly and inefficient to take two movements to moving to the target
// Method 2:
    // the path between start position and target position is circular
    // the center of the circular path is on the common axis of the two wheels
    // pros:
        // taking only one movements to move to the target, much more elegant
        // two wheel position are linearly interpolated because of this circular path
    // cons:
        // a bit difficult to calculate the increment of the wheel positions
        // difficult to debug the algorithm for the wheel positions

// for the convenience of wheel position calculation and debug, method 1 is implemented here

// one significant problem of the controller of two_wheel_robot is that
    // when one wheel is commanded rotating and the other is remained current position
    // the static wheel will naturally rotating at the other direction because of inertia
    // how to solve or avoid this problem? and make position control more precise

#include <ros/ros.h>
#include <math.h>
#include <actionlib/server/simple_action_server.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>
#include <swarm_robot_msgs/two_wheel_poses.h>

// interpolation parameters, change setup here
const double dt = 0.01;  // interpolating resolution
// minimal distance with goal position to interpolate
// distance small than ds_min will be neglected, cube size of two_wheel_robot is 0.0254
// this value should be set very low
const double ds_min = 0.001;
// half distance of two wheels
// not a good way to get this value here
const double half_wheel_dist = 0.0177;

class TwoWheelTrajActionServer {
public:
    TwoWheelTrajActionServer(ros::NodeHandle* nodehandle);

private:
    void swarmRobotPosesCb(const swarm_robot_msgs::swarm_robot_poses& message_holder);
    void twoWheelPosesCb(const swarm_robot_msgs::two_wheel_poses& message_holder);
    void executeCb(const 
        actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction>::GoalConstPtr& goal);

    ros::NodeHandle nh_;  // a node handle is needed

    // declare a subscribers to topic "swarm_robot_poses" and "two_wheel_poses"
    ros::Subscriber swarm_robot_poses_subscriber;
    ros::Subscriber two_wheel_poses_subscriber;
    // declare a publisher to topic "two_wheel_poses_cmd"
    ros::Publisher two_wheel_poses_cmd_publisher;
    // declare an action server
    actionlib::SimpleActionServer<swarm_robot_action::swarm_robot_trajAction> as_;
    
    // messages
    swarm_robot_msgs::swarm_robot_poses robot_poses_msg;  // current robot poses
    swarm_robot_msgs::two_wheel_poses wheel_poses_msg;  // current wheel poses
    swarm_robot_msgs::two_wheel_poses wheel_poses_cmd_msg;  // wheel poses command
    // for trajectory interpolation
    swarm_robot_msgs::two_wheel_poses wheel_poses_self_rotating;
    swarm_robot_msgs::two_wheel_poses wheel_poses_line_moving;
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

    // make sure topics "swarm_robot_poses" and "two_wheel_poses" are active
    while (!(b_robot_poses_cb_started && b_wheel_poses_cb_started)) {
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    std::cout << "topic message from swarm_robot_poses and two_wheel_poses is ready" << std::endl;

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

    // copy the goal message, avoid using "->" too much
    double goal_x[] = goal -> x;
    double goal_y[] = goal -> y;
    double goal_time = goal -> time;



// messages
swarm_robot_msgs::swarm_robot_poses robot_poses_msg;  // current robot poses
swarm_robot_msgs::two_wheel_poses wheel_poses_msg;  // current wheel poses
swarm_robot_msgs::two_wheel_poses wheel_poses_cmd_msg;  // wheel poses command

swarm_robot_msgs::two_wheel_poses wheel_poses_self_rotating;
swarm_robot_msgs::two_wheel_poses wheel_poses_line_moving;



    // the time for this two movements are linearly distributed according to the wheel path

    double x_start;
    double y_start;
    double x_end;
    double y_end;
    double distance;
    double angle_start;
    double angle_end;
    double angle_rotate;
    // calculate the cmd message for each robot
    for (int i=0; i<robot_quantity; i++) {
        x_start = robot_poses_msg.x[i];
        y_start = robot_poses_msg.y[i];
        x_end = goal_x[i];
        y_end = goal_y[i];
        distance = sqrt(pow((x_end - x_start), 2) + pow((y_end - y_start), 2));
        // check if target position is too close
        if (distance < ds_min) {
            // copy the current wheel position
            wheel_poses_self_rotating.left_wheel_pos[i] = wheel_poses_msg.left_wheel_pos[i];
            wheel_poses_line_moving.left_wheel_pos[i] = wheel_poses_msg.left_wheel_pos[i];
            wheel_poses_self_rotating.right_wheel_pos[i] = wheel_poses_msg.right_wheel_pos[i];
            wheel_poses_line_moving.right_wheel_pos[i] = wheel_poses_msg.right_wheel_pos[i];
        }
        else {
            // two stage movement calculation
            // stage 1, self rotating calculation
            angle_start = robot_poses_msg.angle[i];
            angle_end = atan2(y_end - y_start, x_end - x_start);
            angle_rotate = angle_end - angle_start;  // rotate from angle_start to angle_end
            // angel_end and angle_start both belong to range of (-M_PI, M_PI)
            
        }



    }

// if the goal position is too close, neglect it
// rotation angle is between -M_PI and M_PI

// remember to ros::spinOnce at place that is necessary


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

