#include "ezwheel_serial.h"

#include "ros/ros.h"

#include <string>
#include <iostream>

using namespace std;

uint8_t frame_count = 0x00;

uint8_t addCRC(uint8_t* frame, size_t size){ 
    uint8_t crc = 0;
    for(size_t i = 2; i < size-1; i++){ // skip the header (first 2 bytes) and the crc (last byte)
        // the crc is just the LSB of the sum of the data
        crc += frame[i];
    }
    frame[size-1] = crc;
    return crc;
}

void initGetRequest(uint8_t request[]){ 
    request[0]  = 0xAA; // Reserved
    request[1]  = 0x0F; // Reserved
    request[2]  = 0x08;

    request[3]  = 0x10 + frame_count; // Session identification - modify

    request[4]  = 0x01;

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type

    request[10] = 0x05; // Reserved
}
int getWheelSpeed(serial::Serial & my_serial, double & speed){
    size_t request_size = 17;
    uint8_t request[request_size];

    for(int i = 0; i < request_size; i++)
    	request[i] = 0x00;

    initGetRequest(request);

    print_frame(request, request_size);

    return 0;
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
    request[3]  = 0x00 + frame_count++; // Session identification - modify

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

int setWheelSpeed(serial::Serial& my_serial, double speed, bool isClockwise){
    unsigned int speedBytes = speed * 10;

    if(speedBytes > 1023){
        ROS_ERROR("Set speed error: Speed limit is 102.3 km/h, received %lf\n", speed);
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

    size_t question_size = 17;
    uint8_t question[question_size];

    initGetRequest(question);

    question[11] = 0x01;
    question[12] = 0x00;
    question[13] = 0x09;
    question[14] = 0x0A;
    question[15] = 0x04;
    
    addCRC(question, question_size);

    my_serial.write(question, question_size);

    return 1;
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

/*int getStateOfCharge(serial::Serial& my_serial, uint8_t& charge){
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
