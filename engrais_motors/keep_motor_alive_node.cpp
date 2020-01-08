#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "src/ezwheel_serial.h"

#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv){
    ros::init(argc, argv, "keep_motor_alive_node");
    ros::NodeHandle node;

    std::string port_name = "/dev/ttyUSB0";

    string node_name = ros::this_node::getName();
    int address = 0;

    node.param<int>(node_name + "/addr", address, 8);

    int baud = 57600;
    //node.getParam("baud", baud);
    int timeout = 50;
    //node.getParam("timeout", timeout);
    int bytesize = 8;
    //node.getParam("data_bits", bytesize);
    int parity = 0;
    //node.getParam("parity", parity);
    int flowctrl = 0;
    //node.getParam("hdw_flow_ctrl", flowctrl);
    int stop_bit = 1;
    //node.getParam("stop_bit", stop_bit);

    uint8_t addr = (uint8_t)address;

    EzWheelSerial wheel(port_name, baud, timeout, bytesize, parity, stop_bit, flowctrl); //Sets serial communication for back wheel

    double vel = 0;
    while(ros::ok()){
        auto r = wheel.getWheelStructure();
        auto r2 = wheel.setWheelSpeed(1,1);
        cout << "Return : " << r << endl << endl<< endl;//", r2: " << r2 << ", vel: " << vel << endl << endl << endl;
        ros::Duration(0.05).sleep();

        if(vel < 26)
            vel += 0.1;
    }
    return 0;
}

