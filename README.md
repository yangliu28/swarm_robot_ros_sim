# swarm_robot_ros_sim
A swarm robot simulation environment for ROS Indigo to be used with Gazebo.

## What's inside
*swarm_robot_ros_sim* contains packages each of which works on a set of functionalities of this simulation.

"swarm_robot_description" package will describe the robot models, one simple model could be a two-wheel cubic robot, and also the gazebo environment setting files and launch files.

"swarm_robot_msgs" folder will contain packages for costomized messages, services and actions that will be used in other packages.

"swarm_robot_simulator" package will contain simulators for physical properties, including actuation, communication and sensing capabilities.

"swarm_robot_controller" package will be the place in which different control algorithms reside.

## Motivation
This project starts after investigating several swarm robot platforms like Kilobot, Jasmine, E-puck, Alice, etc, and work on a novel swarm microrobot called [Inchbot](http://www.case.edu/mae/robotics/#modular) for a while, it would be interesting to see what these swarm microrobots can do as a collective under simulation environment like ROS. This project is also for the purpose of learning various tools and programming in ROS.

## Setup
Have ROS installed and workspace setup.(missing a lot of details)

Clone this package into ~/ros_ws/src and build using catkin_make.

## Demo tests
In different terminals:

```
roslaunch swarm_robot_description initialize.launch robot_quantity:=10
rosrun swarm_robot_description swarm_spawner_gazebo_client
roslaunch swarm_robot_controller pos_publisher.launch
rosrun swarm_robot_controller two_wheel_pos_minimal_controller
```

## Contributing

