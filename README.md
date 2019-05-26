# alphabot2pi
This is the code used to run the simulation of the AlphaBot2pi using ROS and Gazebo

ROS and Gazebo must be installed before completing the following steps

## Intallation
Create catkin workspace and install repository inside "src" file

Run catkin_make to create "build" and "devel"

Run source devel/setup.bash

## Run the code
roslaunch alphabot2_world spawn_world.launch

roslaunch alphabot2_world spawn_robot.launch

roslaunch sim_control computer.launch

## Instructions
Create a ball object inside Gazebo with a Hue value of 220 (dark blue).
Select the last terminal window. More instructions on how to move the robot under alphabot2_simulation/sim_control/src/remote_node.py
