#include "ezwheel_serial.h"

#include "ros/ros.h"

uint8_t addCRC(uint8_t* frame, size_t size){
    uint8_t crc = 0;
    for(size_t i=2; i<size-1; i++){ // skip the header (first 2 bytes) and the crc (last byte)
        // the crc is just the LSB of the sum of the data
        crc += frame[i];
    }
    frame[size-1] = crc;
    return crc;
}

void initGetRequest(uint8_t* request){
    static int cpt=0;
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

void initSetRequest(uint8_t* request, uint8_t data_size){
    static int cpt=0;
    request[0]  = 0xAA; // synchronization
    request[1]  = 0x0F + data_size; // frame size (from the following byte)
    request[2]  = 0x08; // Quick session frame type

    // In every session, the value is incremented
    // the 4 MSB must be 0x1 (USB application)
    // the 4 LSB should count from 0x0 to 0xF and loop back
    // In every session, the value is incremented. After 0xF is 0x0
    request[3]  = 0x11; // Session identification - modify

    //The value is incremented in every exchange.
    request[4]  = 0x00 + cpt; // Frame identification - modify
    cpt++;

    request[5]  = 0x0A + data_size; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x02; // Set request type
    request[10] = 0x05; // Reserved
}


int getStateOfCharge(serial::Serial& my_serial, uint8_t& charge){
    size_t req_size = 17;
    uint8_t request[req_size];

    initGetRequest(request);

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


int getEngineTemperature(serial::Serial& my_serial, double& temp){
    size_t req_size = 17;
    uint8_t request[req_size];
    int16_t rcv_temp;

    initGetRequest(request);

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

    memcpy(&rcv_temp, &response[8], sizeof(int16_t));
    temp = rcv_temp / 100.0;

    /*ROS_WARN("RESPONSE TEMP");
    for(int i =0; i< resp_size; i++){
        ROS_WARN("[%d] %x", i, response[i]);
    }*/


    return 1;
}

int getWheelStatus(serial::Serial& my_serial, WheelStatus& wheelstatus){
    size_t req_size = 17;
    uint8_t request[req_size];

    initGetRequest(request);

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

int setWheelSpeed(serial::Serial& my_serial, uint16_t speed, uint8_t direction){
    size_t req_size = 21; // 17 + 4
    uint8_t request[req_size];

    initSetRequest(request, 4); // 4 bytes are needed to set the speed

    request[11] = 0x01; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x09; // Data address
    request[14] = 0x10; // Data address

    request[15] = 0x04; // Data Size

    // set direction
    request[16] = direction << 6;
    // set speed mode
    request[16] |= 0x20;
    // set break mode
    request[16] |= 0x08;
    // set not used bits
    request[16] &= 0xEB;
    
    // set Tork to 0, not used in speed mode...
    request[16] &= 0xFC;
    request[17] = 0x00;
    
    // set speed
    request[18] = 0x00FF&(speed >> 2); // Data Size
    request[19] = 0x00FF&(speed << 6); // Data Size
    // set not used bits
    request[19] &= 0xC0; // Data Size

    addCRC(request, req_size);

    /*ROS_WARN("REQUEST");
    for(int i =0; i< req_size; i++){
        ROS_WARN("[%d] %x", i, request[i]);
    }*/

    my_serial.write(request, req_size);

    size_t resp_size = 9; // 7 + 2 + 4
    size_t received_cpt;
    uint8_t response[resp_size];

    received_cpt = my_serial.read(response, resp_size);

    /*ROS_WARN("RESPONSE");
    for(int i =0; i< resp_size; i++){
        ROS_WARN("[%d] %x", i, response[i]);
    }*/

    if(received_cpt != resp_size){
        ROS_ERROR("setWheelSpeed::Expected %d bytes but got %d", resp_size, received_cpt);
        return -1;
    }
    if(response[6] == 0xFF){
        ROS_ERROR("setWheelSpeed::This is a Request Error frame!");
        return -2;
    }

    return 1;
}

void display_status(const WheelStatus& wheelstatus){
    // function to display a status
    ROS_INFO("WheelStatus direction:        %d", wheelstatus.direction);
    ROS_INFO("WheelStatus control_mode:     %d", wheelstatus.control_mode);
    ROS_INFO("WheelStatus brake_mode:       %d", wheelstatus.brake_mode);
    ROS_INFO("WheelStatus tork_estimate:    %d", wheelstatus.tork_estimate);
    ROS_INFO("WheelStatus speed:            %d", wheelstatus.speed);
    ROS_INFO("WheelStatus error:            %d", wheelstatus.error);
    ROS_INFO("WheelStatus charge_status:    %d", wheelstatus.charge_status);
    ROS_INFO("WheelStatus power_status:     %d", wheelstatus.power_status);
}


