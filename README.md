# alphabot2pi
This is the code used to run the simulation of the AlphaBot2pi using ROS and Gazebo

ROS and Gazebo must be installed before completing the following steps

## Intallation
Create catkin workspace and install repository
git clone https://github.com/nestoregon/alphabot2pi_simulation/

mv alphabot2pi_simulation src (change the name to src)

catkin_make (to create "build" and "devel")

source devel/setup.bash

cd src/alphabot2_simulation/sim_control/src (directory with ROS code)

chmod  +x control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py

Type "ls" on the terminal to confirm that all the code is green! This means that is now executable and we can run the code.

## Run the code

You have to open multiple terminals to run the code. Keep in mind that you will have to source the workspace AGAIN for every new terminal window you open (source devel/setup.bash inside the worskpace). Now you're ready to run the code!

roslaunch alphabot2_world spawn_world.launch

roslaunch alphabot2_world spawn_robot.launch

roslaunch sim_control computer.launch

## Instructions
Create a ball object inside Gazebo with a Hue value of 220 (dark blue).
Select the last terminal window. More instructions on how to move the robot under alphabot2_simulation/sim_control/src/remote_node.py
