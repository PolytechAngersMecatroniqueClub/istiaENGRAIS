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

#define SLEEP_TIME 500

using namespace std;

string side;

mutex critSec;



string node_name, emergecy_topic;

ros::NodeHandle* node;


serial::Serial* back_wheel;
serial::Serial* front_wheel;

struct Message{
	double data;
	bool isClockwise;
	ros::Time timeStamp;
};

Message message;


ros::Time lastMsg;
//--------------------------------------------------------------------------------------------------------
void OnEmergencyBrake(const std_msgs::Bool & msg){
    lastMsg = ros::Time::now();

    if(msg.data == true){
        ROS_ERROR("Emergency Shutdown Called");
        ros::shutdown();
    }
}
//--------------------------------------------------------------------------------------------------------
void emergencyThread(){
    lastMsg = ros::Time::now();
    ros::Subscriber emergencySub = node->subscribe(emergecy_topic, 10, OnEmergencyBrake);
    
    ros::Duration(0.5).sleep();

    while(ros::ok()){
        ros::Duration(0.05).sleep();

        ros::Time now = ros::Time::now();
        
        ros::Duration delta_t = now - lastMsg;

        if(ros::ok() && delta_t.toSec() > 0.2){
            ROS_ERROR("Emergency Timeout Shutdown");
            ros::shutdown();
        }
    }

    emergencySub.shutdown();
}

void sendSpeed(){ 
    ros::Rate loop_rate(50);

	while(ros::ok()){
        critSec.lock();

        double data = message.data;
        bool isClockwise = message.isClockwise;
        ros::Duration messageLifeTime = ros::Time::now() - message.timeStamp;

        critSec.unlock();

        if(messageLifeTime.toSec() < 0.5){
            if(setWheelSpeed(*back_wheel, data, isClockwise) != 1)
                ROS_ERROR("ERROR SENDING COMMAND TO BACK WHEEL");

            if(setWheelSpeed(*front_wheel, data, isClockwise) != 1)
                ROS_ERROR("ERROR SENDING COMMAND TO FRONT WHEEL");
        }
        else{
            if(setWheelSpeed(*back_wheel, 0, isClockwise) != 1)
                ROS_ERROR("ERROR SENDING COMMAND TO BACK WHEEL");

            if(setWheelSpeed(*front_wheel, 0, isClockwise) != 1)
                ROS_ERROR("ERROR SENDING COMMAND TO FRONT WHEEL");
        }

        loop_rate.sleep();
	}
}

void OnRosMsg(const std_msgs::Float64 & msg){ //Front node message received 
    critSec.lock(); 

	message.data = fabs(msg.data);

	message.isClockwise = (msg.data < 0 && side == "left") || (msg.data >= 0 && side == "right") ? false : true;

	message.timeStamp = ros::Time::now();

    critSec.unlock(); 
}

void initializeSerialPorts(string back_port_name, string front_port_name, int baud, int timeout, int bytesize, int parity, int stop_bit, int flowctrl){
	back_wheel = new serial::Serial(back_port_name,
									baud,
									serial::Timeout::simpleTimeout(timeout),
									serial::bytesize_t(bytesize),
									serial::parity_t(parity),
									serial::stopbits_t(stop_bit),
									serial::flowcontrol_t(flowctrl));

    front_wheel = new serial::Serial(front_port_name,
									 baud,
									 serial::Timeout::simpleTimeout(timeout),
									 serial::bytesize_t(bytesize),
									 serial::parity_t(parity),
									 serial::stopbits_t(stop_bit),
									 serial::flowcontrol_t(flowctrl));

    if(back_wheel->isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", back_port_name.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", back_port_name.c_str());
        exit(1);
    }
    if(front_wheel->isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", front_port_name.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", front_port_name.c_str());
        exit(1);

    }
}

void closeConexion(){
    back_wheel->close();
    front_wheel->close();
    
    delete back_wheel;
    delete front_wheel;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "motor_node");
    node = new ros::NodeHandle();

	int baud, timeout, bytesize, parity, flowctrl, stop_bit;

    string sub_topic, back_port_name, front_port_name;

    node_name = ros::this_node::getName();

    if(!node->getParam(node_name + "/back_port", back_port_name) || !node->getParam(node_name + "/front_port", front_port_name) ||
       !node->getParam(node_name + "/side", side) || !node->getParam(node_name + "/baud", baud) || !node->getParam(node_name + "/timeout", timeout) || 
       !node->getParam(node_name + "/data_bits", bytesize) || !node->getParam(node_name + "/parity", parity) || !node->getParam(node_name + "/timeout", timeout) ||
       !node->getParam(node_name + "/hdw_flow_ctrl", flowctrl) || !node->getParam(node_name + "/stop_bit", stop_bit) || !node->getParam(node_name + "/sub_topic", sub_topic)){ //Get mandatory parameters

	    ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected sub_topic, 'back_port_name', 'front_port_name', " <<
	             "'side', 'baud', 'timeout', 'data_bits', 'parity', 'timeout', 'hdw_flow_ctrl' and 'stop_bit'.\n\n");

        return -1;
    }

    node->param<string>(node_name + "/emergency_topic", emergecy_topic, "none");

    thread* emergency_t;
    if(emergecy_topic != "none")
        emergency_t = new thread(emergencyThread);

    initializeSerialPorts(back_port_name, front_port_name, baud, timeout, bytesize, parity, stop_bit, flowctrl);

    ros::Subscriber sub = node->subscribe(sub_topic, 10, OnRosMsg);

    thread wheelThread(sendSpeed);

    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shutting down...");

    wheelThread.join();

    closeConexion();

    if(emergecy_topic != "none"){
        emergency_t->join();
        delete emergency_t;
    }

    ROS_INFO("Code ended without errors");

    ros::shutdown();
    
    return 0;
}

