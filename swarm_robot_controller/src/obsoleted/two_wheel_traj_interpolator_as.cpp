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
#include <vector>
#include <swarm_robot_msgs/swarm_robot_poses.h>
#include <swarm_robot_msgs/two_wheel_poses.h>
#include <actionlib/server/simple_action_server.h>
#include <swarm_robot_action/swarm_robot_trajAction.h>

// interpolation parameters, change setup here
const double dt = 0.01;  // interpolating resolution of time
// minimal distance with goal position to interpolate
// distance small than ds_min will be neglected, cube size of two_wheel_robot is 0.0254
// this value should be set very low
const double ds_min = 0.001;
// half distance of two wheels
// not a good way to get this value here
const double half_wheel_dist = 0.0177;
const double wheel_radius = 0.015;

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
    // action messages, not used
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
    swarm_robot_poses_subscriber = nh_.subscribe("swarm_robot_poses", 1,
        &TwoWheelTrajActionServer::swarmRobotPosesCb, this);
    // why compiler give error on the use of boost::bind here
    // swarm_robot_poses_subscriber = nh_.subscribe("swarm_robot_poses", 1,
    //     boost::bind(&TwoWheelTrajActionServer::swarmRobotPosesCb, this, _1));
    b_wheel_poses_cb_started = false;
    two_wheel_poses_subscriber = nh_.subscribe("two_wheel_poses", 1,
        &TwoWheelTrajActionServer::twoWheelPosesCb, this);    
    // two_wheel_poses_subscriber = nh_.subscribe("two_wheel_poses", 1,
    //     boost::bind(&TwoWheelTrajActionServer::twoWheelPosesCb, this, _1));
    // initialize the publisher
    two_wheel_poses_cmd_publisher = nh_.advertise<swarm_robot_msgs::two_wheel_poses>("two_wheel_poses_cmd", 1);

    // get initialization message of robot swarm from parameter server
    bool get_name, get_quantity;
    get_name = nh_.getParam("/robot_model_name", robot_model_name);
    get_quantity = nh_.getParam("/robot_quantity", robot_quantity);
    // no return is allowed in constructor
    // if (!(get_name && get_quantity))
    //     return 0;  // return if fail to get parameter

    // resize swarm_robot_msgs that will be instantiate individually by element
    // IMPORTANT HERE!!!
    // robot_poses_msg and wheel_poses_msg are exempted from this
    // because they will be initialize as a whole, in the callback
    // the same with the robot_poses_msg_ and wheel_poses_msg_ in executeCb
    wheel_poses_cmd_msg.left_wheel_pos.resize(robot_quantity);
    wheel_poses_cmd_msg.right_wheel_pos.resize(robot_quantity);
    wheel_poses_self_rotating.left_wheel_pos.resize(robot_quantity);
    wheel_poses_self_rotating.right_wheel_pos.resize(robot_quantity);
    wheel_poses_line_moving.left_wheel_pos.resize(robot_quantity);
    wheel_poses_line_moving.right_wheel_pos.resize(robot_quantity);

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
    std::vector<double> goal_x = goal -> x;
    std::vector<double> goal_y = goal -> y;
    // why the following does not compile
    // double goal_x[robot_quantity];
    // goal_x = goal -> x;
    // double goal_y[robot_quantity];
    // goal_y = goal -> y;
    double goal_time_cost = goal -> time_cost;

    // get current robot poses and wheel poses
    ros::spinOnce();  // update messages in the class
    // create local variables to store current robot states
    swarm_robot_msgs::swarm_robot_poses robot_poses_msg_ = robot_poses_msg;
    swarm_robot_msgs::two_wheel_poses wheel_poses_msg_ = wheel_poses_msg;

    // variables needed in wheel trajectory calculation
    double x_start;
    double y_start;
    double x_end;
    double y_end;
    double distance;
    double angle_start;  // current angle of the robot
    double angle_end;  // target angle of the robot
    double angle_rotate;  // rotated angle of the robot
    double wheel_incre_self_rotating;  // the wheel postion increment in self rotating stage
    double wheel_incre_line_moving;  // the wheel position increment in line moving stage
        // both increment are signed after been given values
    // calculate the cmd message for each robot
    for (int i=0; i<robot_quantity; i++) {
        x_start = robot_poses_msg_.x[i];
        y_start = robot_poses_msg_.y[i];
        x_end = goal_x[i];
        y_end = goal_y[i];
        distance = sqrt(pow((x_end - x_start), 2) + pow((y_end - y_start), 2));
        // check if target position is too close
        if (distance < ds_min) {
            // copy the current wheel position
            wheel_poses_self_rotating.left_wheel_pos[i] = wheel_poses_msg_.left_wheel_pos[i];
            wheel_poses_self_rotating.right_wheel_pos[i] = wheel_poses_msg_.right_wheel_pos[i];
            wheel_poses_line_moving.left_wheel_pos[i] = wheel_poses_msg_.left_wheel_pos[i];
            wheel_poses_line_moving.right_wheel_pos[i] = wheel_poses_msg_.right_wheel_pos[i];
            // for the time cost calculation, randomly give value to these two variables
            wheel_incre_self_rotating = 1.0;
            wheel_incre_line_moving = 1.0;
        }
        else {
            // two stage movement calculation
            
            // stage 1, self rotating calculation
            angle_start = robot_poses_msg_.angle[i];
            angle_end = atan2(y_end - y_start, x_end - x_start);
            angle_rotate = angle_end - angle_start;  // rotate from angle_start to angle_end
            // angel_end and angle_start both belong to range of (-M_PI, M_PI)
            // change angle_rotate into the same range (-M_PI, M_PI)
            if (angle_rotate > M_PI)
                angle_rotate = angle_rotate - 2 * M_PI;
            if (angle_rotate < -M_PI)
                angle_rotate = angle_rotate + 2 * M_PI;
            bool robot_heading = true;  // whether the robot is heading or backing to the target
            // it's not necessary to rotate the heading direction to the target every time
            // because the two wheel robot can also moving backward
            // change angle_rotate into the range (-M_PI/2, M_PI/2), use robot_heading to mark it
            if (angle_rotate > M_PI/2) {
                robot_heading = false;
                angle_rotate = angle_rotate - M_PI;
            }
            if (angle_rotate < -M_PI/2) {
                robot_heading = false;
                angle_rotate = angle_rotate + M_PI;
            }
            // prepare wheel poses message base on current wheel positions
            wheel_incre_self_rotating = angle_rotate * half_wheel_dist / wheel_radius;
            // left wheel move backward when rotating angle is positive
            wheel_poses_self_rotating.left_wheel_pos[i] =
                wheel_poses_msg_.left_wheel_pos[i] - wheel_incre_self_rotating;
            // right wheel move forward when rotating angle is positive
            wheel_poses_self_rotating.right_wheel_pos[i] =
                wheel_poses_msg_.right_wheel_pos[i] + wheel_incre_self_rotating;

            // stage 2, line moving calculation
            // prepare wheel poses message bese on self rotated wheel positions
            if (robot_heading)
                wheel_incre_line_moving = distance / wheel_radius;  // moving forward
            else
                wheel_incre_line_moving = -distance / wheel_radius;  // moving backward
            // both left and right wheel rotating at same direction
            wheel_poses_line_moving.left_wheel_pos[i] = 
                wheel_poses_self_rotating.left_wheel_pos[i] + wheel_incre_line_moving;
            wheel_poses_line_moving.right_wheel_pos[i] = 
                wheel_poses_self_rotating.right_wheel_pos[i] + wheel_incre_line_moving;
        }
    }

    // trajectory interpolation
    // time for the two movements are linearly distributed wrt the wheel rotation
    double time_self_rotating;
    double time_line_moving;
    time_self_rotating = goal_time_cost * (abs(wheel_incre_self_rotating)) /
        (abs(wheel_incre_self_rotating) + abs(wheel_incre_line_moving));
    time_line_moving = goal_time_cost - time_self_rotating;
    // time control
    ros::Rate rate_timer(1/dt);

    double t_stream;
    double fraction_of_range;
    double wheel_rotate_range;
    // first interpolating between wheel_poses_msg_ and wheel_poses_self_rotating
    t_stream = 0.0;  // start with publish the start wheel poses
    while (t_stream < time_self_rotating) {
        fraction_of_range = t_stream / time_self_rotating;
        // prepare wheel poses command message
        for (int i=0; i<robot_quantity; i++) {
            wheel_rotate_range = wheel_poses_self_rotating.left_wheel_pos[i] -
                wheel_poses_msg_.left_wheel_pos[i];
            wheel_poses_cmd_msg.left_wheel_pos[i] = wheel_poses_msg_.left_wheel_pos[i] +
                fraction_of_range * wheel_rotate_range;
            wheel_rotate_range = wheel_poses_self_rotating.right_wheel_pos[i] -
                wheel_poses_msg_.right_wheel_pos[i];
            wheel_poses_cmd_msg.right_wheel_pos[i] = wheel_poses_msg_.right_wheel_pos[i] +
                fraction_of_range * wheel_rotate_range;
        }
        // publish this wheel poses command message
        two_wheel_poses_cmd_publisher.publish(wheel_poses_cmd_msg);
        t_stream = t_stream + dt;  // increase time by dt
        rate_timer.sleep();
    }
    // end with publish the wheel_poses_self_rotating
    two_wheel_poses_cmd_publisher.publish(wheel_poses_self_rotating);
    rate_timer.sleep();

    // second interpolating between wheel_poses_self_rotating and wheel_poses_line_moving
    t_stream = 0.0;
    while (t_stream < time_line_moving) {
        fraction_of_range = t_stream / time_line_moving;
        // prepare wheel poses command message
        for (int i=0; i<robot_quantity; i++) {
            wheel_rotate_range = wheel_poses_line_moving.left_wheel_pos[i] -
                wheel_poses_self_rotating.left_wheel_pos[i];
            wheel_poses_cmd_msg.left_wheel_pos[i] = wheel_poses_self_rotating.left_wheel_pos[i] +
                fraction_of_range * wheel_rotate_range;
            wheel_rotate_range = wheel_poses_line_moving.right_wheel_pos[i] -
                wheel_poses_self_rotating.right_wheel_pos[i];
            wheel_poses_cmd_msg.right_wheel_pos[i] = wheel_poses_self_rotating.right_wheel_pos[i] +
                fraction_of_range * wheel_rotate_range;
        }
        // publish this wheel poses command message
        two_wheel_poses_cmd_publisher.publish(wheel_poses_cmd_msg);
        t_stream = t_stream + dt;
        rate_timer.sleep();
    }
    // end with publish the wheel_poses_line_moving
    two_wheel_poses_cmd_publisher.publish(wheel_poses_line_moving);
    rate_timer.sleep();

    as_.setSucceeded();  // succeed on this action
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

