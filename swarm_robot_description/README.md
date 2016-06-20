# swarm_robot_description

This package describes the robot models and environment setup of the simulaiton.


consider multiple nodes and remap, only need to write one node

load command lines in terminal so that a robot can be easily added or deleted

simulation result comparison with collision and without collision

least amount of computation to do everything







a light weight program to detect potential collision, arrange the priority of robots when collision is going to happen

collision free also involves how the simulation command the position of the robot, when command the robot, the positions should not overlap

robot goes in curves

wheels can always rotate at constant speed or the robot move in a zigzag way








a controller node subscribe to robot information, and apply torque to control the wheel speed, subscribe to wheel speed command node

the controller node is also compatible with adding or deleting operation






