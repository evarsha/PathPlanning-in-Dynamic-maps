# Knowledge sharing for Optimal path planning of Multi Robot systems in Dynamic maps

Using ROS Gazebo and Python

The coding is done in python3

To execute these codes, the following python packages needs to be imported

import numpy
import matplotlib

To run the simulation we need a system with ubuntu 16.04 with ROS kinetic and Gazebo7 installed into it.

follow the below steps

1. first create the catkin work space using following command
mkdir catkin_ws

2. inside the catkin work space create the source folder using following command
mkdir src

3. inside src create a package using below command
catkin_create_pkg planning

4. now copy the files in the 'planning' folder that is submitted, into this newly created package

5. change to the catkin_ws directory and run the below command to build the package
catkin_make

6. run the following comand

source devel/setup.bash

7. change to the catkin_ws/src/planning/scripts folder and run the following command to generate the executable file as below

chmod +x publisher.py


8. open a new terminal and start the roscore by following command
roscore 

9. open a new terimal now run the launch file from the catkin_ws directory as
source devel/setup.bash
roslaunch planning multi.launch

this will open the gazebo simulator

10. now open a new terminal, change to catkin_ws and give the following commands

source devel/setup.bash

rosrun planning publisher.py

This will obtain the paths for five turtlebots and give to the publisher. publisher will publish the velocities to the turtulebots that are spawned and the output can be seen in the gazebo simulator
