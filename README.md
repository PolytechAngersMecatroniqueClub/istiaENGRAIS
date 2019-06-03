# ENGRAIS Raspberry
This repository contains all the packages to install into our 4 raspberry pi

This project contains 3 packages.

# Network

Since we use ROS TCP/IP architecture, we have to define an IP for each object conected to the network

* Raspberry 1 : 192.168.10.101
* Raspberry 2 : 192.168.10.102
* Raspberry 3 : 192.168.10.103
* Raspberry 4 : 192.168.10.104

* Sick Tim LIDAR 1 : 192.168.10.111
* Sick Tim LIDAR 2 : 192.168.10.112

* ROS core master : 192.168.10.200

To change the default ROS core master, just run: 

-export ROS_MASTER_URI=http://<RosMasterMachineIP>:11311

-export ROS_IP=<YourMachineIP>

# engrais_control

This package has the code to find the lines into the set of points received from sick_tim. The lines found are sent using visualization_marker, making it possible to visualize it in rviz.

## organisation

engrais_control - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch
  * central.launch : to start the central of processing that will receive all lines found, select 2 models from them, and calculate the control to the wheels
  * rasp1.launch : to start rasp 1's code, receiving the control and sending the reference to the right wheel
  * rasp2.launch : to start rasp 2's code, receiving the control and sending the reference to the left wheel
  * rasp3.launch : to start rasp 3 and sick tim's code, getting all the information sent by the first sick Tim and finding the lines that are in the set of points 
  * rasp4.launch : to start rasp 4 and sick tim's code, getting all the information sent by the second sick Tim and finding the lines that are in the set of points

## how to use

None of the launch files contains arguments, meaning that manual modifications are needed if one wants to change the way the nodes work. 

# engrais_motors

This package comunicates with the EZ Wheels and send velocity reference

## organisation

engrais_motors - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch
  * motor.launch : 

## how to use

run motor.launch

# sick_tim

This package is made by the sick Tim development team and it receives the data from the sensor, sending it through ROS using LaserScan message type 


## organisation

Since it is an extern package, this readme will inclue our modifications, for more information go to https://github.com/uos/sick_tim

sick_tim - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch
  * laser1.launch : receives information from the sick tim 1
  * laser2.launch : receives information from the sick tim 2

## how to use

None of the launch files contains arguments, meaning that manual modifications are needed if one wants to change the way the nodes work. 

# Launching Everyting

First, run raspX.launch for each raspberry, where X is the number described in the network topic. Then, in a computer or central of processing, run central.launch. This will call every launch file needed to run the robot and everything should work if done correctly 