//********************************************************************************************************
#include <thread> 
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <termios.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define MESSAGE_FREQUENCY 30.0

using namespace std;

//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_keyboard_control_node");

    ros::NodeHandle node;

    ros::Publisher pub = node.advertise<std_msgs::Float64>("a", 10);

    std_msgs::Float64 msg;
    msg.data = 10;

    ros::Rate loop_rate(2);

    while(ros::ok()){
    	pub.publish(msg);

    	loop_rate.sleep();
    }

    ros::shutdown();

    return 0;
}

//********************************************************************************************************