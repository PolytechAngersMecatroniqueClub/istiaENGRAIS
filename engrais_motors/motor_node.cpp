#include "ros/ros.h"
#include "std_msgs/String.h"

#include "serial/serial.h"

#include <sstream>
#include <string>
#include <vector>

uint8_t cpt=0;

typedef struct{
    uint8_t direction;
    uint8_t control_mode;
    uint8_t brake_mode;
    uint16_t tork_estimate;
    uint16_t speed;
    uint8_t error;
    uint8_t charge_status;
    uint8_t power_status;
}WheelStatus;


uint8_t addCRC(uint8_t* frame, size_t size){
    uint8_t crc = 0;
    for(size_t i=2; i<size-1; i++){ // skip the header (first 2 bytes) and the crc (last byte)
        // the crc is just the LSB of the sum of the data
        crc += frame[i];
    }
    frame[size-1] = crc;
    return crc;
}

void initRequest(uint8_t* request){
    request[0]  = 0xAA; // synchronization
    request[1]  = 0x0F; // frame size (from the following byte)
    request[2]  = 0x08; // Quick session frame type

    // In every session, the value is incremented
    // the 4 MSB must be 0x1 (USB application)
    // the 4 LSB should count from 0x0 to 0xF and loop back
    // In every session, the value is incremented. After 0xF is 0x0
    request[3]  = 0x10; // Session identification - modify

    //The value is incremented in every exchange.
    request[4]  = 0x00 + cpt; // Frame identification - modify
    cpt++;

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type
    request[10] = 0x05; // Reserved
}

int getStateOfCharge(serial::Serial& my_serial, uint8_t& charge){
    size_t req_size = 17;
    uint8_t request[req_size];

    initRequest(request);

    request[11] = 0x01; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x06; // Data address
    request[14] = 0x06; // Data address

    request[15] = 0x01; // Data Size

    addCRC(request, req_size);

    my_serial.write(request, req_size);

    size_t resp_size = 10; // 7 + 2 + 1
    size_t received_cpt;
    uint8_t response[resp_size];

    received_cpt = my_serial.read(response, resp_size);
    if(received_cpt != resp_size){
        ROS_ERROR("getStateOfCharge::Expected %d bytes but got %d", resp_size, received_cpt);
        return -1;
    }
    if(response[6] == 0xFF){
        ROS_ERROR("getStateOfCharge::This is a Request Error frame!");
        return -2;
    }
    charge = response[8];
    return 1;
}


int getEngineTemperature(serial::Serial& my_serial, int16_t& temp){
    size_t req_size = 17;
    uint8_t request[req_size];

    initRequest(request);

    request[11] = 0x00; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x01; // Data address
    request[14] = 0x05; // Data address

    request[15] = 0x02; // Data Size

    addCRC(request, req_size);

    my_serial.write(request, req_size);

    size_t resp_size = 11; // 7 + 2 + 2
    size_t received_cpt;
    uint8_t response[resp_size];

    received_cpt = my_serial.read(response, resp_size);
    if(received_cpt != resp_size){
        ROS_ERROR("getEngineTemperature::Expected %d bytes but got %d", resp_size, received_cpt);
        return -1;
    }
    if(response[6] == 0xFF){
        ROS_ERROR("getEngineTemperature::This is a Request Error frame!");
        return -2;
    }

    memcpy(&temp, &response[8], sizeof(int16_t));

    return 1;
}

int getWheelStatus(serial::Serial& my_serial, WheelStatus& wheelstatus){
    size_t req_size = 17;
    uint8_t request[req_size];

    initRequest(request);

    request[11] = 0x00; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x04; // Data address
    request[14] = 0x07; // Data address

    request[15] = 0x04; // Data Size

    addCRC(request, req_size);

    my_serial.write(request, req_size);

    size_t resp_size = 13; // 7 + 2 + 4
    size_t received_cpt;
    uint8_t response[resp_size];

    received_cpt = my_serial.read(response, resp_size);
    if(received_cpt != resp_size){
        ROS_ERROR("getWheelStatus::Expected %d bytes but got %d", resp_size, received_cpt);
        return -1;
    }
    if(response[6] == 0xFF){
        ROS_ERROR("getWheelStatus::This is a Request Error frame!");
        return -2;
    }

    wheelstatus.direction     = (response[8] & 0xC0) >> 6;
    wheelstatus.control_mode  = (response[8] & 0x20) >> 5;
    wheelstatus.brake_mode    = (response[8] & 0x08) >> 3;
    wheelstatus.tork_estimate =  response[9] + (((uint16_t)response[8] & 0x0003) << 8);
    wheelstatus.speed         = ((uint16_t)response[10] << 2) + ((response[11] & 0xC0) >> 6);
    wheelstatus.error         = (response[11] & 0x20) >> 5;
    wheelstatus.charge_status = (response[11] & 0x18) >> 3;
    wheelstatus.power_status  = (response[11] & 0x06) >> 1;
    
    return 1;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "motor_node");
    ros::NodeHandle node;

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

    if(my_serial.isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", port_name.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", port_name.c_str());
        return 1;
    }
    uint8_t charge;
    if(getStateOfCharge(my_serial, charge)== 1){
        ROS_INFO("State of charge (percent): %d", charge);
    }

    int16_t temp;
    if(getEngineTemperature(my_serial, temp)== 1){
        ROS_INFO("Temperature  (0.1 deg C): %d", temp);
    }

    WheelStatus wheelstatus;
    if(getWheelStatus(my_serial, wheelstatus)==1){
        ROS_INFO("WheelStatus direction:        %d", wheelstatus.direction);
        ROS_INFO("WheelStatus control_mode:     %d", wheelstatus.control_mode);
        ROS_INFO("WheelStatus brake_mode:       %d", wheelstatus.brake_mode);
        ROS_INFO("WheelStatus tork_estimate:    %d", wheelstatus.tork_estimate);
        ROS_INFO("WheelStatus speed:            %d", wheelstatus.speed);
        ROS_INFO("WheelStatus error:            %d", wheelstatus.error);
        ROS_INFO("WheelStatus charge_status:    %d", wheelstatus.charge_status);
        ROS_INFO("WheelStatus power_status:     %d", wheelstatus.power_status);
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

