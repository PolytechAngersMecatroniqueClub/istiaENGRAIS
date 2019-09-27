# ENGRAIS Raspberry
This repository contains all the packages to install into our 4 raspberry pi

This project contains 3 packages.

# Network

Since we use ROS TCP/IP architecture, we have to define an IP for each object conected to the network

* Raspberry 1 : 192.168.10.101
* Raspberry 2 : 192.168.10.102
* Raspberry 3 : 192.168.10.103
* Raspberry 4 : 192.168.10.104
* Jetson TX2 :  192.168.10.105

* Sick Tim LIDAR 1 : 192.168.10.111
* Sick Tim LIDAR 2 : 192.168.10.112

* ROS core master : 192.168.10.105

To change the default ROS core master, just run: 

-export ROS_MASTER_URI=http://'RosMasterMachineIP':11311

-export ROS_IP='YourMachineIP'

Or change the ~/.bashrc

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

        IP=`ifconfig wlo1 | grep "inet " | cut -d' ' -f10`
        
        export ROS_MASTER_URI=http://192.168.10.105:11311
        export ROS_MASTER_IP=192.168.10.105
        export ROS_IP=$IP

Now your pc should be able to launch ENGRAIS' ROS nodes

# Network Setup
To be able to launch ROS nodes remotely, some configurations must be made to the ssh.

In your pc, run

    ~$ ssh -oHostKeyAlgorithms='ssh-rsa' username@IP

for each one of the connected devices. In our case it had to be done to all 4 Raspberry Pi and the Jetson TX2.

This proceadure must be repeated in every other device if you want to launch ROS nodes from everywhere.


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
