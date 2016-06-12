# swarm_robot_description

This package is for the swarm robot models.

Collada files for the wheels with stripe pattern are under urdf/mesh/. They are not used because to make a circular wheel rotate smoothly on the ground, at least hundreds of line segments are needed to approximate the circle (the only way to make circle in SketchUp), therefore producing a lot of vertices and edges, which lay a heavy burden on physics solver in Gazebo.

How to generate collada file for smooth circular wheel?



Is the cylinder in link tag also meshed from line segments? (very likely)

change the support ball radius.

