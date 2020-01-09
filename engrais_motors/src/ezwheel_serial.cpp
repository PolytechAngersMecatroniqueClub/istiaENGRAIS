#include "ezwheel_serial.h"

using namespace std;
//########################################################################################################

//--------------------------------------------------------------------------------------------------------
EzWheelSerial::EzWheelSerial(const std::string port_name, const int baud, const int timeout, const int bytesize, const int parity, const int stop_bit, const int flowctrl){ //Creates communication 
    this->my_serial = new serial::Serial (port_name,
                                          baud,
                                          serial::Timeout::simpleTimeout(timeout),
                                          serial::bytesize_t(bytesize),
                                          serial::parity_t(parity),
                                          serial::stopbits_t(stop_bit),
                                          serial::flowcontrol_t(flowctrl));

    if(this->my_serial->isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", port_name.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", port_name.c_str());
        exit(-1);
    }
}
//--------------------------------------------------------------------------------------------------------
EzWheelSerial::~EzWheelSerial(){ //Destructor to destroy pointer 
    if(this->my_serial != NULL){
        this->my_serial->close();
        delete this->my_serial; 
    }
}

//########################################################################################################

int EzWheelSerial::listenResponse(uint8_t* frame){ //Listens and builds response frame, -1 if it fails 
    uint8_t synch = 0; //Synchronization Byte

    while(synch != 0xAA){ //Wait for 0xAA
        if(!this->my_serial->read(&synch, 1)){ //Read 1 byte
            return -1; //If it didn't receive data, return -1 
        }

        frame[0] = synch; //Store 0xAA to frame[0]
    }

    if(!this->my_serial->read(&synch, 1)){ //Read 1 byte
        return -1; //If it didn't receive data, return -1 
    }

    frame[1] = synch; //Store frame size

    uint8_t response[synch]; //Declare message body
    if(this->my_serial->read(response, synch) != synch){ //Read X bytes
        return -1;
    }

    for(int i = 0; i < synch; i++) //Writes all bytes into response frame
        frame[i + 2] = response[i];

    return synch + 2; //Return size
}
//--------------------------------------------------------------------------------------------------------
uint8_t EzWheelSerial::calculateCRC(uint8_t frame[], const size_t size){ //Calculates Frame CRC 
    uint8_t crc = 0; //Initialize Sum
    for(size_t i = 2; size > 0 && i < size-1; i++){ //Skip the header (first 2 bytes) and the crc (last byte)
        crc += frame[i];
    }
    return crc;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::initSetRequest(uint8_t request[], const uint8_t data_size){ //Initialize common bytes for set requests 
    request[0]  = 0xAA; //Synchronization
    request[1]  = 0x0F + data_size; //Frame size
    request[2]  = 0x08; //Reserved

    request[3]  = 0x10 + this->frame_count; //Frame identification

    request[4]  = 0x00; //Reserved

    request[5]  = 0x0A + data_size; // Reserved

    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved

    request[9]  = 0x02; // Set request type
    request[10] = 0x05; // Reserved
}
//--------------------------------------------------------------------------------------------------------
bool EzWheelSerial::setWheelSpeed(const double speed, const bool isClockwise){ //Sends Velocity Target 
    unsigned int speedBytes = speed * 10; //Translate double value into integer

    if(speedBytes > 1023){ //Return fail if more than max vel
        ROS_ERROR("Set speed error: Speed limit is 102.3 km/h, received %lf\n", speed);
        return -1;
    }

    size_t req_size = 21;
    uint8_t request[req_size];

    this->initSetRequest(request, 4); //Initialize common bytes

    request[11] = 0x00; //Data address
    request[12] = 0x00; //Data address
    request[13] = 0x04; //Data address
    request[14] = 0x07; //Data address

    request[15] = 0x04; //Return data size

    request[16] = 0x15 + !isClockwise; //Data Byte 1 (0x15 = clockwise, 0x16 = anti-clockwise)
    request[17] = 0x00; //Don't change
    request[18] = speedBytes & 0xFF; // 8 Least significant bits for velocity
    request[19] = (speedBytes >> 8) & 0x3; // 2 most significant bits for velocity
    request[req_size - 1] = this->calculateCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends signal
    //cout << "Sent: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 9; // 9 + data size
    uint8_t response[resp_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    int recv = this->listenResponse(response);
    /*cout << "Received: " << recv << endl;

    if(recv != -1){
        print_frame(response, recv);
    }*/

    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3] && this->calculateCRC(response, recv) == response[recv - 1]){
        //If response size has the correct size, session ID, Return Type and CRC are correct, then this is a valid response
        if(++this->frame_count >= 0x10)
            this->frame_count = 0x00;

        return true; //If communication was OK, return absolute speed
    }

    return false; //If error frame, returns 0*/
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::initGetRequest(uint8_t request[]){ //Initialize common bytes for get requests 
    request[0]  = 0xAA; // Synchronization
    request[1]  = 0x0F; // Frame Size
    request[2]  = 0x08;

    request[3]  = 0x10 + this->frame_count; //Frame identification

    request[4]  = 0x01;

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type

    request[10] = 0x05; // Reserved
}
//--------------------------------------------------------------------------------------------------------
double EzWheelSerial::getWheelSpeed(){ //Get absolute wheel speed, -1 if communication failed 
    size_t req_size = 17; //Declare frame
    uint8_t request[req_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    this->initGetRequest(request); //Initialize common byts

    request[11] = 0x00; //Data address
    request[12] = 0x00; //Data address
    request[13] = 0x04; //Data address
    request[14] = 0x08; //Data address

    request[15] = 0x02; //Data Size
    request[req_size - 1] = this->calculateCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends frame
    //cout << "Sent: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 11; // 9 + data size
    uint8_t response[resp_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    int recv = this->listenResponse(response);
    /*cout << "Received: " << recv << endl;

    if(recv != -1){
        print_frame(response, recv);
    }*/

    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3] && this->calculateCRC(response, recv) == response[recv - 1]){
        //If response size has the correct size, session ID, Return Type and CRC are correct, then this is a valid response
        if(++this->frame_count >= 0x10)
            this->frame_count = 0x00;

        return (response[9] << 8 | response[8])/10.0; //If communication was OK, return absolute speed
    }

    return -1;
}
//--------------------------------------------------------------------------------------------------------
int EzWheelSerial::getStateOfCharge(){ //Get wheel's battery level 
    size_t req_size = 17; //Declare frame
    uint8_t request[req_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    this->initGetRequest(request); //Initialize common byts

    request[11] = 0x01; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x06; // Data address
    request[14] = 0x06; // Data address

    request[15] = 0x01; // Data Size
    request[req_size - 1] = this->calculateCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends frame
    //cout << "Sent: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 10; // 9 + data size
    uint8_t response[resp_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    int recv = this->listenResponse(response);
    /*cout << "Received: " << recv << endl;

    if(recv != -1){
        print_frame(response, recv);
    }*/
    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3] && this->calculateCRC(response, recv) == response[recv - 1]){
        //If response size has the correct size, session ID, Return Type and CRC are correct, then this is a valid response
        if(++this->frame_count >= 0x10)
            this->frame_count = 0x00;

        return (int)response[8]; //If communication was OK, return battery level in %
    }

    return -1;
}
//--------------------------------------------------------------------------------------------------------
WheelStatusStructure EzWheelSerial::getWheelStructure(){ //Get wheel's status structure
    WheelStatusStructure r;

    size_t req_size = 17; //Declare frame
    uint8_t request[req_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    this->initGetRequest(request); //Initialize common byts

    request[11] = 0x00; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x04; // Data address
    request[14] = 0x06; //Data address

    request[15] = 0x04; // Data Size
    request[req_size - 1] = this->calculateCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends frame
    //cout << "Sent: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 13; // 9 + data size
    uint8_t response[resp_size] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

    int recv = this->listenResponse(response);
    /*cout << "Received: " << recv << endl;

    if(recv != -1){
        print_frame(response, recv);
    }*/
    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3] && this->calculateCRC(response, recv) == response[recv - 1]){
        //If response size has the correct size, session ID, Return Type and CRC are correct, then this is a valid response
        if(++this->frame_count >= 0x10)
            this->frame_count = 0x00;

        r.direction = response[8] & 0b0011;
        r.type = (response[8] & 0b0100) >> 2;
        r.brakeMode = (response[8] & 0b10000) >> 4;
        r.torque = (response[9] & 0xFF) << 2 | (response[8] & 0xC0) >> 6;
        r.speed = (response[10] & 0xFF) | (response[11] & 0b11) << 2;
        r.error = (response[11] & 0b100) >> 2;
        r.chargeStatus = (response[11] & 0b11000) >> 3;
        r.batteryStatus = (response[11] & 0b1100000) >> 5;
    }

    return r;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::print_frame(const uint8_t * frame, const uint8_t size){ //Print frame for debugging 
    std::cout.fill('0');

    for(int i = 0; i < size; i++)
        std::cout << "frame[" << std::setw(2) << i << "] : 0x" << std::hex << std::uppercase << std::setw(2) << (int)frame[i] << std::dec << std::endl;
    
    std::cout << std::endl << std::endl;
}
