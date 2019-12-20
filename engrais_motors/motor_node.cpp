#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <thread>
#include <mutex>
#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

#include "src/ezwheel_serial.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

string side; //Robot side

mutex critSec; //Critical section

string node_name, emergecy_topic, back_port_name, front_port_name;

ros::NodeHandle* node;


EzWheelSerial* back_wheel; //Declare serial communication
EzWheelSerial* front_wheel;

struct Message{
	double data;
	bool isClockwise;
	ros::Time timeStamp;
};

Message message;
int counter = 0;

//--------------------------------------------------------------------------------------------------------
ros::Time lastMsg; //Store last time
bool comReady = false; //Flag to say the communication is ready
bool emergencyCalled = false; //Flag to signal exit

//--------------------------------------------------------------------------------------------------------
void OnEmergencyBrake(const std_msgs::Bool & msg){ //Emergency message 
    lastMsg = ros::Time::now(); //Store last time
    
    comReady = true; //Sets flag

    if(msg.data == true){ //If message is true, exit
        ROS_ERROR_STREAM(node_name << ": Emergency Shutdown Called");
        emergencyCalled = true;
        ros::shutdown();
    }
}
//--------------------------------------------------------------------------------------------------------
void emergencyThread(){ 
    lastMsg = ros::Time::now();

    ros::Subscriber emergencySub = node->subscribe(emergecy_topic, 10, OnEmergencyBrake); //Subscribe to topics
    ros::Publisher emergencyPub = node->advertise<std_msgs::Bool>(emergecy_topic, 10); //Topic to publish if automatic
    
    while(ros::ok() && !comReady) //If mode is manual, wait for comunication to be set
        ros::Duration(0.01).sleep();

    while(ros::ok() && !emergencyCalled){ //Until emergency is called
        ros::Time now = ros::Time::now(); //get time
        
        ros::Duration delta_t = now - lastMsg;

        if(!emergencyCalled && delta_t.toSec() > 0.2){ //If last emergency message was received more than 200ms ago, shutdown
            ROS_ERROR_STREAM(node_name << ": Emergency Timeout Shutdown");
            ros::shutdown();
        }

        ros::Duration(0.05).sleep();
    }

    emergencySub.shutdown(); //Shutdown everything
    emergencyPub.shutdown();
}


//--------------------------------------------------------------------------------------------------------
void sendSpeed(){ //Send wheel target speed 
    ros::Rate loop_rate(50);

    for(int i = 0; i <= 5; i++){ //Try 5 times to wake the wheels
        if(i == 5){ //If unable to wake them, exit code
            ROS_ERROR("UNABLE TO WAKE THE WHEELS");
            exit(-2);
        }
        
        if((back_port_name == "none" || back_wheel->getStateOfCharge() != -1) && (front_port_name == "none" || front_wheel->getStateOfCharge() != -1)) //Sends message to both wheels in order to wake them
            break;

        ros::Duration(0.05).sleep(); //Sleep 50ms
    }

	while(ros::ok()){ //While ROS is running
        critSec.lock(); //Lock critical section

        double data = message.data; //Get last message's data
        bool isClockwise = message.isClockwise; //Get last message's wheel sense
        ros::Duration messageLifeTime = ros::Time::now() - message.timeStamp; //Get last message's time stamp

        critSec.unlock(); //Unlock mutex

        if(messageLifeTime.toSec() > 0.5){ //If last message is from lass than 500ms, then it is valid
            data = 0.0;
            isClockwise = false;
        }

        if(back_port_name != "none" && !back_wheel->setWheelSpeed(data, isClockwise)) //Send message to back wheel
            ROS_ERROR("ERROR SENDING COMMAND TO BACK WHEEL");

        if(front_port_name != "none" && !front_wheel->setWheelSpeed(data, isClockwise))
            ROS_ERROR("ERROR SENDING COMMAND TO FRONT WHEEL"); //Send message to front wheel*/

        loop_rate.sleep(); //Wait 20ms
	}
}
//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const std_msgs::Float64 & msg){ //Target velocity 
    critSec.lock(); //Locks critical section

	message.data = fabs(msg.data); //Get absolute target speed

	message.isClockwise = (msg.data < 0 && side == "left") || (msg.data >= 0 && side == "right") ? false : true; //Decides if wheels have to turn clockwise or anti-clockwise

	message.timeStamp = ros::Time::now(); //Sets timestamp

    critSec.unlock(); //Unlock critical section
}

//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "motor_node"); //Initialize node
    node = new ros::NodeHandle();

	int baud, timeout, bytesize, parity, flowctrl, stop_bit;

    string sub_topic;

    node_name = ros::this_node::getName();


    node->param<string>(node_name + "/back_port", back_port_name, "/dev/ttyUSB0"); //Get parameters and sets default values
    node->param<string>(node_name + "/front_port", front_port_name, "/dev/ttyUSB1");

    node->param<string>(node_name + "/side", side, "right");

    node->param<int>(node_name + "/baud", baud, 57600);
    node->param<int>(node_name + "/timeout", timeout, 50);
    node->param<int>(node_name + "/data_bits", bytesize, 8);
    node->param<int>(node_name + "/parity", parity, 0);
    node->param<int>(node_name + "/hdw_flow_ctrl", flowctrl, 0);
    node->param<int>(node_name + "/stop_bit", stop_bit, 1);

    node->param<string>(node_name + "/sub_topic", sub_topic, "default/wheelCommand");
    node->param<string>(node_name + "/emergency_topic", emergecy_topic, "none");

    if(back_port_name != "none")
   		back_wheel = new EzWheelSerial(back_port_name, baud, timeout, bytesize, parity, stop_bit, flowctrl); //Sets serial communication for back wheel
    
	if(front_port_name != "none")
    	front_wheel = new EzWheelSerial(front_port_name, baud, timeout, bytesize, parity, stop_bit, flowctrl); //Sets serial communication for front wheel

    ros::Subscriber sub = node->subscribe(sub_topic, 10, OnRosMsg); //Subscribe to topic

    thread wheelThread(sendSpeed); //Launches thread to send velocity 

    thread* emergency_t; //Declare emergency thread
    if(emergecy_topic != "none")
        emergency_t = new thread(emergencyThread); //Launches thread if argument is different than none

    ROS_INFO("Code Running, press Control+C to end");
    ros::spin(); //Spin to receive message
    ROS_INFO("Shutting down...");

    wheelThread.join(); //Wait thread to finish

    if(emergecy_topic != "none"){ //Wait thread 
        emergency_t->join();
        delete emergency_t;
    }

    if(back_port_name != "none")
    	delete back_wheel; //Closes communication

	if(front_port_name != "none")
	    delete front_wheel;

    ROS_INFO("Code ended without errors");

    sub.shutdown(); //Ends ROS
    ros::shutdown();

    delete node;
    
    return 0;
}