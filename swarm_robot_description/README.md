# swarm_robot_description

This package describes the robot models and environment setup of the simulaiton.


load command lines in terminal so that a robot can be easily added or deleted


least amount of computation to do everything

a light weight program to detect potential collision, arrange the priority of robots when collision is going to happen

collision free also involves how the simulation command the position of the robot

robot goes in curves

wheels can always rotate at constant speed or the robot move in a zigzag way






one node to manage the number and index of the robots through service, adding and deleting robot

publish robot 2D position, publish wheel velocity (no position info)

a controller node subscribe to robot information, and apply torque to control the wheel speed, subscribe to wheel speed command node

a controller node 




setup initial position without collision



simulation result comparison with collision and without collision


