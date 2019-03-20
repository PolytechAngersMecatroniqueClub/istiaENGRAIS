# ENGRAIS
This repository contains all the packages to simulate and control a robot through an unknown field, using LIDAR sensors.

This project contains 3 packages.

# engrais_control

This package allows to deal with the controllers to interract from ROS to gazebo

## organisation

engrais_control - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* config
* * engrais_control.yaml : configuration file for the controllers
* launch
* * engrais_control.launch : to start the controllers

## how to use

You should not have to deal with this package, the launch file is started from the main launch file (cf the package engrais_gazebo)

## work to do

The physical behavior of the robot is not as expected, some parameters of the controllers (PID values for instance) should be optimized.

