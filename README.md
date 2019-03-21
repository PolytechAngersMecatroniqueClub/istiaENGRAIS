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
  * engrais_world.launch : launch file to start the simulation, cf "how to use -> start the simulation" to check how to use it
* models
  * plantgreen : folder that contains the plantgreen model
    * model.config : configuration file for the plantgreen model, it includes the sdf file
    * plantgreen.sdf : description of the model, it is generated autonomatically, you must not modify it
    * plantgreen.xacro : description of the model, to generate the sdf file from the xacro you can use the Makefile (cf "how to use -> modify the plant models")
  * plantred : folder that contains the plantred model
    * model.config : configuration file for the plantred model, it includes the sdf file
    * plantred.sdf : description of the model, it is generated autonomatically, you must not modify it
    * plantred.xacro : description of the model, to generate the sdf file from the xacro you can use the Makefile (cf "how to use -> modify the plant models")
  * Makefile : to easilly generate the sdf files from the xacro files
* worlds : cf "how to use -> modify the worlds"
  * engrais.world : file generated automatically, should not be modified
  * engrais2.world : file generated automatically, should not be modified
  * engrais3.world : file generated automatically, should not be modified
  * engrais.world.xacro : xacro file that uses population tags to generate the default world
  * engrais2.world.py : python3 code that uses population tags to generate a world
  * engrais3.world.py : python3 code that fully generates a customable random world

## how to use

### start the simulation

To use the launch file starting the default world:
```
roslaunch engrais_gazebo engrais_world.launch
```

To start the simulation with another world configuration (worldname beeing the name of the file worldname.world that you want to use)
```
roslaunch engrais_gazebo engrais_world.launch world:=worldname
```

### modify the plant models

In order to ease the creation of the plant models, it was decided to use first an xacro file and then to convert this file into the needed sdf file. That is, if you want to modify the model, you should modify the corresponding xacro file, and then use the Makefile to generate the sdf file. The Makefile uses two toos: rosrun xacro, and sed bash tool.

To update both models:
```
make all
```

To update the plantgreen model
```
make green
```

To update the plantred model
```
make red
```

To clean the sdf files (also done with make all):
```
make clean
```
By cleaning the sdf files, we mean removing the xmlns:xacro tag, needed to convert xacro file into sdf file, but that causes a warning (because the tag is unkown) when importing the model into the world.

### modify the worlds

Two ways of generatic a world are considered: the use of xacro file or the use of a python3 script. To easily convert the xacro/python3 file into a world file, you can use the Makefile

To generate the world engrais.world (simple, world by default)
```
make engrais
```

To generate the world engrais2.world (work in progress, should not be used for now, may be removed)
```
make engrais2
```

To generate the world engrais3.world (random plants and weeds)
```
make engrais3
```

### control the robot

There are two ros topics (one for each wheel):
* /engrais/leftWheel_controller/command
* /engrais/rightWheel_controller/command

To publish a command using command line:
```
rostopic pub /engrais/leftWheel_controller/command std_msgs/Float64 "data: 10.0"
```

## work to do

* optimize the plant model by maybe removing the joint
* create new worlds

# general usefull commands

## xacro

To check an xacro file when generating urdf file:
```
xacro inputfile.xacro | check_urdfs
```
## git

To check wich branch you are working on
```
git branch
```

To change branch
```
git checkout branchname
```

To remove files or folders
```
git rm filepath
git rm -rf folderpath
```

## to be installed to use those packages

```
sudo apt install ros-melodic-gazebo-ros
sudo apt install ros-melodic-controller-manager
sudo apt install ros-melodic-velocity-controllers
sudo apt install ros-melodic-controller-interface
sudo apt install ros-melodic-joint-state-controller
sudo apt install ros-melodic-effort-controllers
```

