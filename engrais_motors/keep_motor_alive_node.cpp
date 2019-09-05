#include "ros/ros.h"
#include "std_msgs/String.h"

#include "src/ezwheel_serial.h"

#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv){
    ros::init(argc, argv, "keep_motor_alive_node");
    ros::NodeHandle node;

    // ********************************************************************
    // Initialization of the node parameters

    std::string port_name = "/dev/ttyUSBX";
    node.getParam("port_name", port_name);
    int baud = 57600;
    node.getParam("baud", baud);
    int timeout = 10;
    node.getParam("timeout", timeout);
    int bytesize = 8;
    node.getParam("data_bits", bytesize);
    int parity = 0;
    node.getParam("parity", parity);
    int flowctrl = 0;
    node.getParam("hdw_flow_ctrl", flowctrl);
    int stop_bit = 1;
    node.getParam("stop_bit", stop_bit);

    serial::Serial my_serial(port_name,
                             baud,
                             serial::Timeout::simpleTimeout(timeout),
                             serial::bytesize_t(bytesize),
                             serial::parity_t(parity),
                             serial::stopbits_t(stop_bit),
                             serial::flowcontrol_t(flowctrl));

    // end of the initialization
    // ********************************************************************

    if(my_serial.isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", port_name.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", port_name.c_str());
        return 1;
    }

    double charge;

    while(ros::ok()){
        if(setWheelSpeed(my_serial, 1.5, 0) == 1){
            ROS_INFO("Message sent");
            ros::Duration(0.05).sleep();
        }
    }

    ros::shutdown();

    ROS_INFO("Code ended without errors");

    return 0;
}

