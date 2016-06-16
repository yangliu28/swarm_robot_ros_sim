// this node manage the number and index of present two wheel robot

// publish topic on two wheel robot information:
    // robot index, 2D position and speed of wheels
// use gazebo service to spawn and delete model in gazebo
// (compatible when a robot model is deleted directly from gazebo gui)
// accept service request the command of adding or deleting robot model
// the adding or deleting action is reflected directly from the topic msg

// this node is not compatible with different robot models
// because only two wheel robot has wheel speed, may write another node for other robot models

// both low and high level control subscribe to same robot information topic
// although it's better to divide the message into two, and publish at different frequency
// this one topic method can be reused when it comes to other robot models




// collision is checked when adding robots




#include <ros/ros.h>
#include 








