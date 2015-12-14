# swarm_robot_ros_sim
A swarm robot simulation environment for ROS Indigo to be used with Gazebo.

## What's inside
*swarm_robot_ros_sim* contains packages each of which works on a set of functionalities of this simulation.

"swarm_robot_description" package will describe the robot models, one simple model could be a two-wheel cubic robot, and also the gazebo environment setting files and launch files.

"swarm_robot_msgs" folder will contain packages for customized messages, services and actions that will be used in other packages.

"swarm_robot_simulator" package will contain simulators for physical properties, including actuation, communication and sensing capabilities.

"swarm_robot_controller" package will be the place in which different control algorithms reside.

## Motivation
This project starts after investigating several swarm robot platforms like Kilobot, Jasmine, E-puck, Alice, etc, and work on a novel swarm microrobot called [Inchbot](http://www.case.edu/mae/robotics/#modular) for a while, it would be interesting to see what these swarm microrobots can do as a collective under simulation environment like ROS. This project is also for the purpose of learning various tools and programming in ROS.

## Setup
Have ROS installed and workspace setup.(missing a lot of details)

Clone this package into ~/ros_ws/src and build using catkin_make.

## Demo tests
In different terminals:

launch the gazebo environment setting (parameter values are default):
```
roslaunch swarm_robot_description initialize.launch robot_quantity:=10 half_range:=1.0
```
launch the controllers for two wheel robot:
```
roslaunch swarm_robot_controller two_wheel_robot_controller.launch
```
test 1: dispersion (parameter values are default):
```
rosrun swarm_robot_simulation two_wheel_dispersion _spring_length:=0.7 _wheel_speed:=2.0
```
test 2: line formation (parameter values are default)
```
rosrun swarm_robot_simulation two_wheel_line_formation _sensing_range:=3.0 _spring_length:=0.7 _wheel_speed:=2.0
```

## Contributing

## Problems and to-do list
Currently there is no impact avoiding mechanism.

Parameter server for swarm robot rviz visualization.

Sensor simulator to be added, distance calculation.

Load other robot model directly from 3D model file.

Mesh for fancier visual effect of robot model in gazebo.

There is no 100 node for 100 robot, remap to improve.

Check turtle bot for dynamics.

Circle formation: fitting neighbors into constant radius circle, calculate displacement into the circle, calculate displacement parallel the circle. (use this method to fit any other line shape)

There is no difference between my swarm research and galtech or kilobot, each robot will get all the location information of all other robots. (sounds like boring...)

## ~~Project proposal for EECS 600~~ (obsolete)
Aside from being a lab research, this project is also a graduate project for EECS 600 by Yang Liu.

This project aims to build a swarm robot simulation environment in ROS. That is, using the distributed mechanism provided, like package, node, message, service, action etc, with Gazebo being the physical simulator, to construct the minimal packages that will be necessary in the physical movement simulation of swarm robot. The purpose of the simulation is to explore how the action of a single robot can bring to the behavior of swarm robots as a collective, like we have seen in the ants, bees, fish swarms.

As time is limited for graduate project, all parts of this project will be simpilified to the minimal required version, with the main purpose of just examining the possibility of such a simulation in ROS. For example, when building a robot model that could moving around on the ground, a cube with two wheels will be the minimal requirement.

The goals by the time of graduate project evaluation: (*inappropriate goals will be changed here*)

1. A package contains robot model files and launch files for the initialization of the environment. (already done)

2. A package contains customized messages, services and actions that will be used in other packages. (partly done)

3. A package contains basic low-level control of swarm robot, like the position/speed control of a single wheel, the position/orientation of a single robot. (partly done)

4. A package contains high-level control of swarm robot, like making movement action based on the sensing messages from other robot.

5. For live demonstration, robot swarm will be able to move around on the ground. When a simple action strategy is employed, robot swarm should be able to demonstrate a sensible behavior, like simply avoiding hitting each other.
