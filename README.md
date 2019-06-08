# Alphabot2pi ROS simulation
This is the code used to run the simulation of the AlphaBot2pi using ROS and Gazebo

ROS and Gazebo must be installed before completing the following steps

## Installation
Create catkin workspace. Make sure you are in a path where you want your worskpace to be and run the following commands on the terminal.
```
mkdir alphabot2pi_simulation_ws
cd alphabot2pi_simulation_ws
git clone https://github.com/nestoregon/alphabot2pi_simulation/
```
Change the name to src
```
mv alphabot2pi_simulation src 
```
Create build and devel files
```
catkin_make
```
Source the workspace
```
source devel/setup.bash
```
Go to the directory with control nodes
```
cd src/alphabot2_simulation/sim_control/src
```
Make the code executable
```
chmod  +x control_robot_node.py find_ball_blue_node.py servo_node.py drive_node.py remote_node.py
```
Type "ls" on the terminal to confirm that all the code is green! This means that we can run the code.

## Configure /.bashrc files. ROS_MASTER and ROS_URI
Write the following at the bottom of the /.bashrc file. This is done to source the workspace (**NOTE: change the path for your own workspace path!**)
```
# Identify the workspace every time the terminal is opened
source /home/nestoregon/ROS/alphabot2pi_real_ws/devel/setup.bash
```
Unlike the real robot control, in the simulation there is no need to set up a ROS_MASTER and a ROS_URI, because you will be running the simulation locally. If you are running the simulation after the real robot control **comment** the commands. NOTE: **uncomment** the commands if you are going to run the real robot control.
These lines need to be commented **IF** they are written:
```
# office
# export ROS_MASTER_URI=http://ipAddressRaspberry:11311
# export ROS_IP=ipAddressComputer
```

## Run the code

You have to open multiple terminals to run the code. Keep in mind that you will have to source the workspace AGAIN for every new terminal window you open (source devel/setup.bash inside the worskpace). Now you're ready to run the code!
```
roslaunch alphabot2_world spawn_world.launch

roslaunch alphabot2_world spawn_robot.launch

roslaunch sim_control computer.launch
```

## Instructions
Create a ball object inside Gazebo with a Hue value of 220 (dark blue).
Select the last terminal window. 
* **Task 1: Manual Mode** Use the keys to manually control the robot
* **Task 2: Ball Following Drive** The robot uses the wheels to follow a blue ball
More instructions on how to move the robot under alphabot2_simulation/sim_control/src/remote_node.py

## Future work
You can add as many input and output nodes as you like.
* Modify the control_robot_node.py code to set new algorithms. Brain of the robot. Set speeds. Set frequency. 
* Modify the remote_node.py node to set new keys and publish new topics
* Modify the find_ball_blue_node.py to set new values for different colors to find

## Authors

* **Nestor Morales** - *Control Code and Gazebo Simulation* - [nestoregon](https://github.com/nestoregon)
* **Manuel Serrano** - *Control Code and Gazebo Simulation*
