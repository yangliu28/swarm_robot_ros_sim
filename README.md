# swarm_robot_ros_sim
A swarm robot simulation environment for ROS to be used with Gazebo.

(Please also check out my new swarm robot simulations featuring consensus and formation algorithms.)

[https://github.com/yangliu28/swarm_formation_sim](https://github.com/yangliu28/swarm_formation_sim)

## Motivation
This project starts after investigating several swarm robot platforms like Kilobot, E-puck, Jasmine, Alice, etc, and working on a novel swarm microrobot called [Inchbot](http://www.case.edu/mae/robotics/#modular) for a while. It would be interesting to see what these swarm microrobots can do as a collective under simulation environment like ROS. This simulation aims to be a general swarm robots simulation platform.

Current research topic is decentrized formation control.

Other interests are Braitenberg vehicle, collective move, dealing with obstacles and closed environment, etc.

## What's inside
*swarm_robot_ros_sim* contains packages each of which features a set of functionalities for this simulation platform:

*swarm_robot_description* package describes the robot models and simulation environment initialization files.

*swarm_robot_msgs* folder contains packages for customized messages, services and actions that will be used in other packages.

*swarm_robot_controller* package contains the low level controllers for the swarm robots.

*swarm_robot_simulation* package contains the swarm algorithms.

## Setup
Have ROS Indigo installed and workspace setup. Clone this package in ~/ros_ws/src and build with catkin_make.
Make sure you clone https://github.com/catkin/catkin_simple.git in ~/ros_ws/src before build with catkin_make.

## Demo
Two wheel robot dispersion simulation:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`roslaunch swarm_robot_description two_wheel_robot.launch robot_quantity:=20 half_range:=0.5`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`rosrun swarm_robot_simulation two_wheel_robot_dispersion _spring_length:=0.7 _sensing_range:=2.0`

Two wheel robot aggregation simulation:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`roslaunch swarm_robot_description two_wheel_robot.launch robot_quantity:=30 half_range:=2.0`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`rosrun swarm_robot_simulation two_wheel_robot_aggregation _sensing_range:=1.5`

Two wheel robot line formation simulation:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`roslaunch swarm_robot_description two_wheel_robot.launch robot_quantity:=25 half_range:=0.7`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`rosrun swarm_robot_simulation two_wheel_robot_line_formation _spring_length:=0.7 _sensing_range:=2.0`

Optionally, in all previous tests, robots can be added or deleted through:

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`rosrun swarm_robot_description two_wheel_robot_batch_add _robot_quantity:=10 _half_range:=1.0`

&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`rosservice call /swarm_sim/two_wheel_robot_update (press tab)...`

[https://youtu.be/BKXuokwg3Xg](https://youtu.be/BKXuokwg3Xg)

## Why not ROS Stage
Some swarm robot research model the robot as a dot, like ROS Stage. In my idea, there is distinct difference between omnidirectional move of a dot robot and a complete robot model. Here we use full model (simplify the collision) of the real robots to bridge the gap between simulation and implementation. Although it has drawback that the simulation speed goes low when more than 100 of such complex robots are in Gazebo, we will try to overcome that.

## Contribution

## License
See the [LICENSE](LICENSE.md) file for license rights and limitations (MIT).

