#include "ezwheel_serial.h"

#include "ros/ros.h"

#include <string>
#include <iostream>

using namespace std;

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
    // static int cpt=0;
    request[0]  = 0xAA; // synchronization
    request[1]  = 0x0F; // frame size (from the following byte)
    request[2]  = 0x08; // Quick session frame type

    // In every session, the value is incremented
    // the 4 MSB must be 0x1 (USB application)
    // the 4 LSB should count from 0x0 to 0xF and loop back
    // In every session, the value is incremented. After 0xF is 0x0
    request[3]  = 0x1F; // Session identification - modify

    //The value is incremented in every exchange.
    request[4]  = 0x00; // + cpt; // Frame identification - modify
    // cpt++;

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type
    request[10] = 0x05; // Reserved
}

void initSetRequest(uint8_t* request, uint8_t data_size){
    // static int cpt=0;
    request[0]  = 0xAA; // synchronization
    request[1]  = 0x0F + data_size; // frame size (from the following byte)
    request[2]  = 0x08; // Quick session frame type

    // In every session, the value is incremented
    // the 4 MSB must be 0x1 (USB application)
    // the 4 LSB should count from 0x0 to 0xF and loop back
    // In every session, the value is incremented. After 0xF is 0x0
    request[3]  = 0x1F; // Session identification - modify

    //The value is incremented in every exchange.
    request[4]  = 0x00; // + cpt; // Frame identification - modify
    // cpt++;

    request[5]  = 0x0A + data_size; // Reserved

    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved

    request[9]  = 0x03; // Set request type
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

    if(response[6] == 0xFF){
        ROS_ERROR("getStateOfCharge::This is a Request Error frame!");
        return -2;
    }
    else if(received_cpt != resp_size){
        ROS_ERROR("getStateOfCharge::Expected %zu bytes but got %zu", resp_size, received_cpt);
        return -1;
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
 
    if(response[6] == 0xFF){
        ROS_ERROR("getEngineTemperature::This is a Request Error frame!");
        return -2;
    }
    else if(received_cpt != resp_size){
        ROS_ERROR("getEngineTemperature::Expected %zu bytes but got %zu", resp_size, received_cpt);
        return -1;
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
    /*size_t req_size = 17;
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
    wheelstatus.power_status  = (response[11] & 0x06) >> 1;*/
    
    return 1;
}

int setWheelSpeed(serial::Serial& my_serial, double speed, bool isClockwise){
    unsigned int speedBytes = speed * 10;

    if(speedBytes > 1023){
        //ROS_ERROR("Set speed error: Speed limit is 102.3 km/h, received %lf\n", speed);
        return -1;
    }

    size_t req_size = 22;
    uint8_t request[req_size];

    initSetRequest(request, 5); // 5 bytes are needed to set the speed

    request[11] = 0x00; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x04; // Data address
    request[14] = 0x07; // Data address

    request[15] = 0x04; // Data Size

    request[16] = 0x05 + !isClockwise; // 0b 0000 0101;
    request[17] = 0x00;
    request[18] = speedBytes;
    request[19] = speedBytes >> 8;
    request[20] = 0x00;

    addCRC(request, req_size);
    
    my_serial.write(request, req_size);

    return 1;
}

void display_status(const WheelStatus& wheelstatus){
    // function to display a status
    ROS_INFO("WheelStatus direction:        %u", wheelstatus.direction);
    ROS_INFO("WheelStatus control_mode:     %u", wheelstatus.control_mode);
    ROS_INFO("WheelStatus brake_mode:       %u", wheelstatus.brake_mode);
    ROS_INFO("WheelStatus tork_estimate:    %u", wheelstatus.tork_estimate);
    ROS_INFO("WheelStatus speed:            %u", wheelstatus.speed);
    ROS_INFO("WheelStatus error:            %u", wheelstatus.error);
    ROS_INFO("WheelStatus charge_status:    %u", wheelstatus.charge_status);
    ROS_INFO("WheelStatus power_status:     %u\n\n", wheelstatus.power_status);
}


void print_frame(const uint8_t * frame, uint8_t size){
    std::cout.fill('0');
    for(int i=0; i<size; i++){
        //printf("frame[%d] = %2.2x\n", i, frame[i]);
        std::cout << "frame[" << std::setw(2) << i << "] ; 0x" << std::hex << std::uppercase << std::setw(2) << (int)frame[i] << std::dec << std::endl;
        //cout << "\t[" << setw(2) << i << "]: 0x" << hex << uppercase << setw(2) << (int)msg[i] << dec << endl;
    }
    std::cout << std::endl << std::endl;
}