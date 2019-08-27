#include "ros/ros.h"
#include "std_msgs/String.h"

#include "src/ezwheel_serial.h"

#include <sstream>
#include <string>
#include <vector>

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle node;

    // ********************************************************************
    // Initialization of the node parameters

    std::string port_name = "/dev/ttyUSBX";
    node.getParam("port_name", port_name);
    int baud = 57600;
    node.getParam("baud", baud);
    int timeout = 1000;
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
    /*uint8_t charge;
    while(getStateOfCharge(my_serial, charge) != 1);
    ROS_INFO("State of charge (percent): %d", charge);*/

    /*double temp;
    while(getEngineTemperature(my_serial, temp)!= 1);
    ROS_INFO("Temperature  (deg C): %2.2f", temp);

    WheelStatus wheelstatus;


    for(uint8_t i = 0; i < 0x60; i++){
        if(getWheelStatus(my_serial, wheelstatus)==1){
            display_status(wheelstatus);
        }
    }

    

    int cpt_tmp = 0;*/

    /*for(uint8_t i = 0; i < 0x30; i++){
        ROS_INFO("session id: %x", i);
            setWheelSpeed(my_serial, 0, 0, i);
    }

    if(getWheelStatus(my_serial, wheelstatus)==1){
        display_status(wheelstatus);
    }*/
    size_t resp_size = 200; // 7 + 2 + 1
    size_t received_cpt;
    uint8_t response[resp_size];

    while(ros::ok()){
        received_cpt = my_serial.read(response, resp_size);
        print_frame(response, received_cpt);
    }


    /*size_t req_size = 17;
    uint8_t request[req_size];

    request[0]  = 0xAA; // synchronization
    request[1]  = 0x0F; // frame size (from the following byte)
    request[2]  = 0x08; // Quick session frame type

    // In every session, the value is incremented
    // the 4 MSB must be 0x1 (USB application)
    // the 4 LSB should count from 0x0 to 0xF and loop back
    // In every session, the value is incremented. After 0xF is 0x0
    request[3]  = 0x12; // Session identification - modify

    //The value is incremented in every exchange.
    request[4]  = 0x00; // Frame identification - modify

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type
    request[10] = 0x05; // Reserved
    request[11] = 0x00; // Data address - modify
    request[12] = 0x00; // Data address - modify
    request[13] = 0x04; // Data address - modify
    request[14] = 0x53; // Data address - modify
    request[15] = 0x02; // Data Size - modify
    request[16] = 0x00; // CRC

    ROS_INFO("computed CRC: %d", addCRC(request, req_size));

    ROS_INFO("nb written bytes: %d", my_serial.write(request, req_size));*/

    
    /*std::string test_str = "test";
    size_t bytes_wrote = my_serial.write(test_str);
    ROS_INFO("nb written bytes: %d", bytes_wrote);*/

    /*int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
    */

    return 0;
}

