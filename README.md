



# ENGRAIS Master
This repository contains all the packages to simulate and control a robot through an unknown field, using LIDAR sensors.


# ROS Setup

To use all ROS nodes and liberaries created, we have a few settings to make. 

    1) Install ROS using the tutorial in wiki.ros.org/melodic/Installation/Ubuntu (Currently the last version is ROS Melodic)

    2) Create a catkin workspace
        ~$ mkdir catkin_ws
        ~$ cd catkin_ws/
        ~/catkin_ws$ mkdir src 
        ~/catkin_ws$ catkin_make

    3) Install Git using the commands 
        ~$ sudo apt-get update
        ~$ sudo apt-get install git

    4) Clone the git repository inside the workspace and change to the "raspberry_pi" branch
        ~$ cd catkin_ws/src/
        ~/catkin_ws/src$ git clone https://github.com/PolytechAngersMecatroniqueClub/istiaENGRAIS.git
        ~/catkin_ws/src$ git checkout raspberry_pi

    5) In directory ~/catkin_ws/src/istiaENGRAIS/engrais_control/src/.fuzzylite-6.0 copy fuzzylite-6.0-linux64.zip to ~/Downloads and extract it

    6) Build the library
        ~$ cd Downloads/fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/
        ~/Downloads/fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite$ chmod +x build.sh 
        ~/Downloads/fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite$ ./build.sh

    7) Copy the binaries in folder "~/Downloads/fuzzylite-6.0-linux64/fuzzylite-6.0/fuzzylite/release/bin/" to "~/catkin_ws/src/istiaENGRAIS/engrais_control/src/.fuzzylite-6.0/bin/"

    8) Build everything
        ~$ cd catkin_ws/
        ~/catkin_ws$ catkin_make 

    9) Using ~$ gedit .bashrc add the following lines to the end of it :
        source /opt/ros/melodic/setup.bash
        source ~/catkin_ws/devel/setup.bash 

# Gazebo Setup

The next step is to install the simulation environment gazebo and its packages

    1) Follow the alternative method in : http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install the last version is the 9.0
      *) If "gazebo: symbol lookup error: /usr/lib/x86_64-linux-gnu/libgazebo_common.so.9: undefined symbol" error appears, try running "sudo apt upgrade libignition-math2"
    
    2) Install these packages :

      sudo apt install ros-melodic-gazebo-ros
      sudo apt install ros-melodic-controller-manager
      sudo apt install ros-melodic-velocity-controllers
      sudo apt install ros-melodic-controller-interface
      sudo apt install ros-melodic-joint-state-controller
      sudo apt install ros-melodic-effort-controllers

Now all the silulation parts are ready 

# Scripts

This folder is not a packege, but it contains usefull scripts to our simulation. To run the simulation, you only have to install
  sudo apt-get install tmux

and then run it with ./simulation.sh All the results will be saved in <catkin_ws_name>/src/Results. Output will be the raw data that the algorithm prints, and Simulation Results a csv with time(s) | x | y | z | roll | pitch | yall, organized by environment and algorithm used.

To get the graph analysis you will have to install:
  1) Miniconda using the installer found in https://docs.conda.io/en/latest/miniconda.html
        \***WARNING**\* Say "no" when the installer asks "Do you wish the installer to initialize Miniconda3 by running conda init?"
        
  2) Plotly : conda install -c plotly plotly=4.2.1

  3) Orca : conda install -c plotly plotly-orca psutil requests
  
  4) Pip : sudo apt-get install python-pip

  5) These libraries:
```
sudo apt -y install libgconf2-4
sudo apt install libcanberra-gtk-module libcanberra-gtk3-module
pip install plotly
pip install cufflinks
```

Then, reopen terminal, run **export PATH="$HOME/miniconda3/bin:$PATH"** or wherever you installed miniconda, then run ./analysis.sh. All the graphs will be saved in <catkin_ws_name>/src/Results. The folder Plants Position has the plants positions for engrais3 and 4, and the folder Analysis results will have all the graphs

\***WARNING**\*

Conda PATH will interfere with ROS, thus they CANNOT run at the same time. Every time you want to run the analysis script, you will have to run **export PATH="$HOME/miniconda3/bin:$PATH"**

# engrais_control

This package implements the algorithm to receive the cloud of points, calculates the better lines, and use this information to calculate the control signals.

## organisation

engrais_control - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* config
  * engrais_control.yaml : configuration file for the controllers
* launch
  * findlines.launch : Launches node that receives the point cloud and sends the better lines
  * control.launch : Launches node that receives lines and calculates control signal

## how to use

One can launch each node with default parameters:
```
rosrun engrais_control robot_findlines
rosrun engrais_control robot_control
```
One can change these parameters in command line : 
```
rosrun engrais_control robot_findlines _parameter:="p" (you can find parameters names in the .cpp). 
```

roslaunch will launch the node with custom parameters. You can change in the launch file all values to be sent, or in the command line :
```
roslaunch engrais_control control.launch parameter:="p"
```

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
  * engrais4.world : file generated automatically, should not be modified
  * engrais.world.xacro : xacro file that uses population tags to generate the default world
  * engrais.world.py : python3 code that uses population tags to generate a world
  * engrais2.world.py : python3 code that uses population tags to generate a world
  * engrais3.world.py : python3 code that fully generates a customable random world
  * engrais4.world.py : python3 code that fully generates a customable random world

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
# engrais
This package is to unite everything to make launching multiple nodes easier.

## organisation

engrais - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch
  * system.launch : Launches all findlines node and control node to control the gazebo's robot
  * simulation.launch : Launches gazebo simulation, findlines and control node.
  
## how to use

One can load a custom environment and control the simulation with
```
roslaunch engrais system.launch
```
Or launch gazebo with our environments using
```
roslaunch engrais simulation.launch
```

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
