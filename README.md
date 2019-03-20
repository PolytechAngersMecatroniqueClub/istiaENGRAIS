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
  * engrais_control.yaml : configuration file for the controllers
* launch
  * engrais_control.launch : to start the controllers

## how to use

You should not have to deal with this package, the launch file is started from the main launch file (cf the package engrais_gazebo)

## work to do

The physical behavior of the robot is not as expected, some parameters of the controllers (PID values for instance) should be optimized.

# engrais_description

This package contains the description of the robot and the sensors.

## organisation

engrais_description - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch
  * rviz.launch : to start rviz with a good configuration
* rviz
  * engrais.rviz : configuration file for rviz tool
* urdf
  * engrais.gazebo : xml file that contains:
    * the gazebo colors to use for robot parst (caster wheels and body)
    * the gazebo configuration of the caster wheels
    * the gazebo configuration of the LiDAR (all the sensor parameters and plugins)
  * engrais.xacro : xml file that discribes the robot (links and joints) and define the robot constants
  * macros.xacro : xml file that defines macros to create wheels and to evaluate classical inertia matrices, those macros are used in the engrais.xacro file
  * materials.xacro : defines colors

## how to use

The only thing to do from this package it is to start rviz
```
roslaunch engrais_description rviz.launch
```

## Work to do

The behavior of the robot is not quite as expected, some parameters may be changed...

# engrais_gazebo

This package contains the launch files to start the simulation, and the world configuration

## organisation

engrais_gazebo - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch


