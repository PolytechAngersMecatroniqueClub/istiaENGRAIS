# ENGRAIS
This repository contains all the packages to simulate and control a robot through an unknown field, using LIDAR sensors.

# Organization
This project contains 3 parts, where 2 of them are related to simulation, and 1 for control.

- Robot model and environment: robot_model is the folder containing all the robot's parts, physics as well as obstacles (in cylinder form).
- Robot plugin: robot_plugin is the folder containing the files to establish a communication betwen ROS and Gazebo. Without it, it's impossible to control robot's wheels or get sensor's data outside gazebo.
- Robot control: robot_control is the folder that contains the ROS nodes that get the sensor's data, calculate lines, send the models to rviz and (eventualy) send control commands to the robot's wheels.

# How to use ?
start the gazebo easy simulation : 
    roslaunch engrais_gazebo engrais_easy.launch
start the gazebo complicated simulation : 
    roslaunch engrais_gazebo engrais_complicated.launch
start rviz : 
    roslaunch engrais_gazebo rviz.launch
start the keyboard controller node
    rosrun engrais_control robot_keyboard_control
start the pearl associated node
    rosrun engrais_control robot_control

