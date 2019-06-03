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
    if(node.getParam("port_name", port_name)){
            ROS_INFO("motor_node::port_name parameter: %s", port_name.c_str());
    }else{
            ROS_WARN("motor_node::Could not get the port_name parameter, default value: %s", port_name.c_str());
    }

    int baud = 57600;
    if(node.getParam("baud", baud)){
            ROS_INFO("motor_node::baud parameter: %d", (int)baud);
    }else{
            ROS_WARN("motor_node::Could not get the baud parameter, default value: %d", (int)baud);
    }

    int timeout = 1000;
    if(node.getParam("timeout", timeout)){
            ROS_INFO("motor_node::timeout parameter: %d", (int)timeout);
    }else{
            ROS_WARN("motor_node::Could not get the timeout parameter, default value: %d", (int)timeout);
    }

    int bytesize = 8;
    if(node.getParam("data_bits", bytesize)){
            ROS_INFO("motor_node::data_bits parameter: %d", (int)bytesize);
    }else{
            ROS_WARN("motor_node::Could not get the data_bits parameter, default value: %d", (int)bytesize);
    }

    int parity = 0;
    if(node.getParam("parity", parity)){
            ROS_INFO("motor_node::parity parameter: %d", (int)parity);
    }else{
            ROS_WARN("motor_node::Could not get the parity parameter, default value: %d", (int)parity);
    }

    int flowctrl = 0;
    if(node.getParam("hdw_flow_ctrl", flowctrl)){
            ROS_INFO("motor_node::hdw_flow_ctrl parameter: %d", (int)flowctrl);
    }else{
            ROS_WARN("motor_node::Could not get the hdw_flow_ctrl parameter, default value: %d", (int)flowctrl);
    }

    int stop_bit = 1;
    if(node.getParam("stop_bit", stop_bit)){
            ROS_INFO("motor_node::stop_bit parameter: %d", (int)stop_bit);
    }else{
            ROS_WARN("motor_node::Could not get the stop_bit parameter, default value: %d", (int)stop_bit);
    }

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
    uint8_t charge;
    while(getStateOfCharge(my_serial, charge) != 1);
    ROS_INFO("State of charge (percent): %d", charge);

    double temp;
    if(getEngineTemperature(my_serial, temp)== 1){
        ROS_INFO("Temperature  (deg C): %2.2f", temp);
    }

    WheelStatus wheelstatus;
    if(getWheelStatus(my_serial, wheelstatus)==1){
        display_status(wheelstatus);
    }

    int cpt_tmp = 0;
    while(setWheelSpeed(my_serial, 20, 1)==1 && cpt_tmp < 50){
        cpt_tmp ++;
        ros::Duration(0.1).sleep();
    }

    if(getWheelStatus(my_serial, wheelstatus)==1){
        display_status(wheelstatus);
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

