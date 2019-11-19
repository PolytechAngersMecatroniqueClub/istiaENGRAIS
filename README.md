# ENGRAIS Raspberry
This repository contains all the packages to install into our 4 raspberry pi and the Nvidia Jetson TX2

This project contains 4 packages.

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
        ~/catkin_ws/src$ cd istiaENGRAIS
        ~/catkin_ws/src/istiaENGRAIS$ git checkout raspberry_pi

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

# Jetson TX2 Ubuntu Installation
To install Jetson's OS, you will need to connect your turned off jetson to another PC using the Micro USB port, and connect it to a monitor using the HDMI port. 

    1) In your PC, download Nvidia SDK manager at developer.nvidia.com/nvidia-sdk-manager (you will have to create a developer account, unfortunately).
    
    2) Install the .deb file and run 'sdkmanager' at the terminal.
    
    3) Log in into your account and change Target Hardware to your product (in our case Jetson TX2 (P3310)) and click continue

    4) Accept the terms, click continue and put the sudo password

    5) When the "SDK Manager is about to flash your Jetson TX2" screen pops up, change to Manual Setup. Click the power up button (4th one from left to right) two leds should show up. Then, hold the 3rd button, click and release the 1st button, then release the 3rd button. Finally, click the Flash button on your PC

    6) After the flash is done, your Jetson Should boot up ubuntu and your PC will ask to Install SDK components on your Jetson TX2. Don't touch your PC and finish Ubuntu setup on your Jetson

    7) After completing Ubuntu's setup, put jetson's Username and Password in your PC.

    8) After installing everything, restart your jetson and it will be done!

# engrais_control

This package has the node to find the lines into the set of points received from sick_tim and the node responsible to get this information to calculate the control signal. The lines found and those selected are sent using visualization_marker, making it possible to visualize it in rviz.

## Organisation

engrais_control - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch

	  findlines.launch : launch the findlines and the sick_tim node, the sick_tim node gets the points found by the sensor, and send those points through ROS. Then, the findlines node uses this information to detect the 6 most probable lines in the field and sends those lines through ROS. In the launch file, one can change the Lidar IP, node names and ROS topic names

	  control.launch : launch the control node, that receives all lines detected, selects the lines that are the most frequent as the 'real' ones, and calculates the control signal to both wheels. In the launch file, one can change the node name, robot's max speed, robot's dimensions, control signal period and ROS Topic names


## How to use

None of the launch files contains arguments, meaning that manual modifications are needed if one wants to change the way the nodes work. 


# engrais_motors

This package comunicates with the EZ Wheels and periodically send velocity reference using serial communication to keep the wheels on.

## Organisation

engrais_motors - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch

	  motor.launch : launch the motor node, it connects via USB to 2 wheels and sends a velocity reference every 2ms. In launch file, one can change the node's name, ROS topic names, USB addresses and serial communication configurations. 

## How to use

run motor.launch

# sick_tim

This package is made by the sick Tim development team and it receives the data from the sensor, sending it through ROS using LaserScan message type 


## Organisation

Since it is an extern package, this readme will inclue our modifications, for more information go to https://github.com/uos/sick_tim

sick_tim - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch

	  laser_back.launch : receives information from the back sick tim
      
	  laser_front.launch : receives information from the front sick tim

## how to use

None of the launch files contains arguments, meaning that manual modifications are needed if one wants to change the way the nodes work. 


# engrais

This package aims to make it easier to launch everything. It contais 2 nodes, one that controls the robot using the keyboard's arrows and the other one that controls the robot using the PS4 controller

engrais - package created with catkin_create_pkg
* package.xml
* CMakeLists.txt
* launch

	1 ) Single Component Launch
    
	  rasp1.launch : Connects using SSH to rasp1, and launches motor.launch to the right side

      rasp2.launch : Connects using SSH to rasp2, and launches motor.launch to the left side

      rasp3.launch : Connects using SSH to rasp3, and launches findlines.launch to the back sick tim

      rasp4.launch : Connects using SSH to rasp4, and launches findlines.launch to the front sick tim

      central.launch : Connects using SSH to cental ROS, and launches control.launch

	2 ) Fragmented Component Launch
    
      control.launch : Connects using SSH to rasp3, rasp4 and central ROS, and launches findlines.launch on rasp3 for the back sensor, findlines.launch on rasp4 for the front sensor, and control.launch on central.

	  findlines.launch : Connects using SSH to rasp3 and rasp4, and launches findlines.launch on rasp3 for the back sensor and findlines.launch on rasp4 for the front sensor.
	
	  motors.launch : Connects using SSH to rasp1 and rasp2, and launches motor.launch on rasp1 for the right side and findlines.launch on rasp2 for the left side.


	* keyboard_move.launch : Connects using SSH to rasp1 and rasp2, and launches motor.launch on rasp1 for the right side, findlines.launch on rasp2 for the left side and keyboard_move.launch. After this, one can control the robot with the keyboard's arrows. In the launch file one can change the node name, ROS topics name, and robot's velocity
  
  * controller_move.launch : Connects using SSH to rasp1 and rasp2, and launches motor.launch on rasp1 for the right side, findlines.launch on rasp2 for the left side and controller node. After this, one can control the robot with the PS4 controller. In the launch file one can change the node name, controller's bluetooth and name, ROS topics name, and robot's velocity

  * system.launch : Launches everything. Connects using SSH to rasp1, rasp2, rasp3, rasp4 and central. Launches motor.launch on rasp1 for the right side and findlines.launch on rasp2 for the left side. Launches findlines.launch on rasp3 for the back sensor, findlines.launch on rasp4 for the front sensor and control.launch and controller node on central. The default mode is "manual", meaning that the user can control the robot with the PS4 controller's analog. The "X" button when PRESSED, will change the mode to "automatic" (meaning that the central node will send its control signal to the wheels instead of the controller) and back to "manual" when released. The "Triangle", "Circle" and "Square" buttons will always set the mode to "manual". The "Central" button will change the current state of the system when pressed. Any of the L1, L2, L3, R1, R2, R3 will trigger emergency and shut everything down.
