# ENGRAIS
This repository contains all the packages to simulate and control a robot through an unknown field, using LIDAR sensors.

# How to use ?
This file will resume how to use all the components 

1) Clone this repository inside your <catkin_workspace>/src
2) After this, run open a terminal in you catkin workspace and run "catkin_make"
3) Open a terminal and run "roscore"
4) To run the gazebo model simulation, run "roslaunch engrais_gazebo engrais_easy.launch"
5) To run the control, run "rosrun engrais_control robot_control"
6) Finally, to visualize what the robot is seeing and the lines the control is finding, run "roslaunch engrais_gazebo rviz.launch"