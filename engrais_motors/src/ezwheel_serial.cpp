#include "ezwheel_serial.h"


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

//--------------------------------------------------------------------------------------------------------
uint8_t EzWheelSerial::addCRC(uint8_t frame[], const size_t size){ //Calculates Frame CRC 
    uint8_t crc = 0; //Initialize Sum
    for(size_t i = 2; i < size-1; i++){ //Skip the header (first 2 bytes) and the crc (last byte)
        crc += frame[i];
    }
    frame[size-1] = crc; //Assign to last value

    return crc;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::initSetRequest(uint8_t request[], const uint8_t data_size){ //Initialize common bytes for set requests 
    request[0]  = 0xAA; //Reserved
    request[1]  = 0x0F + data_size;
    request[2]  = 0x08; //Reserved

    request[3]  = 0x10 + this->frame_count; //Frame identification

    request[4]  = 0x00; //Reserved

    request[5]  = 0x0A + data_size; // Reserved

    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved

    request[9]  = 0x02; // Set request type
    request[10] = 0x05; // Reserved

    if(++this->frame_count >= 0x10) //Reset counter if more than 16
        this->frame_count = 0x00;
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
    this->addCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends signal
    //cout << "Send: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 9;
    uint8_t response[resp_size];

    int recv = this->my_serial->read(response, resp_size); //Waits for response
    //cout << "Received: " << recv << endl;
    //print_frame(response, recv);  

    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3]){ //If response frame was ok, then communication was a success
        return true;
    }

    return false; //If error frame, returns 0
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::initGetRequest(uint8_t request[]){ //Initialize common bytes for get requests 
    request[0]  = 0xAA; // Reserved
    request[1]  = 0x0F; // Reserved
    request[2]  = 0x08;

    request[3]  = 0x10 + this->frame_count; //Frame identification

    request[4]  = 0x01;

    request[5]  = 0x0A; // Reserved
    request[6]  = 0x00; // Reserved
    request[7]  = 0x00; // Reserved
    request[8]  = 0x00; // Reserved
    request[9]  = 0x01; // Get request type

    request[10] = 0x05; // Reserved

    if(++this->frame_count >= 0x10)
        this->frame_count = 0x00;
}
//--------------------------------------------------------------------------------------------------------
double EzWheelSerial::getWheelSpeed(){ //Get absolute wheel speed, -1 if communication failed 
    size_t req_size = 17; //Declare frame
    uint8_t request[req_size];

    this->initGetRequest(request); //Initialize common bytes

    request[11] = 0x00; //Data address
    request[12] = 0x00; //Data address
    request[13] = 0x04; //Data address
    request[14] = 0x08; //Data address

    request[15] = 0x02; //Data Size
    this->addCRC(request, req_size); //Calculates CRC
    
    this->my_serial->write(request, req_size); //Send frame
    //cout << "Send: " << endl;
    //print_frame(request, req_size);

    size_t resp_size = 11; //9 + data_size;
    uint8_t response[resp_size];

    int recv = this->my_serial->read(response, resp_size); //Wait for response
    //cout << "Received: " << recv << endl;
    //print_frame(response, recv);  

    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3]) //If communication was OK, return speed
        return (response[9] << 8 | response[8])/10.0;

    return -1; //If not, returns -1
}
//--------------------------------------------------------------------------------------------------------
int EzWheelSerial::getStateOfCharge(){ //Get wheel's battery level 
    size_t req_size = 17; //Declare frame
    uint8_t request[req_size];

    this->initGetRequest(request); //Initialize common byts

    request[11] = 0x01; // Data address
    request[12] = 0x00; // Data address
    request[13] = 0x06; // Data address
    request[14] = 0x06; // Data address

    request[15] = 0x01; // Data Size
    this->addCRC(request, req_size); //Calculates CRC

    this->my_serial->write(request, req_size); //Sends frame
    //cout << "Send: " << endl;
    //print_frame(request, req_size);


    size_t resp_size = 10; //9 + data_size;
    uint8_t response[resp_size];


    int recv = this->my_serial->read(response, resp_size); //Wait for response
    //cout << "Received: " << recv << endl;
    //print_frame(response, recv);  

    if(recv == resp_size && response[6] != 0xFF && request[3] == response[3]) //If communication was OK, return battery level in %
        return (int)response[8];
    
    return -1; //If not, return -1
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void EzWheelSerial::print_frame(const uint8_t * frame, const uint8_t size){ //Print frame for debugging 
    std::cout.fill('0');

    for(int i = 0; i < size; i++)
        std::cout << "frame[" << std::setw(2) << i << "] : 0x" << std::hex << std::uppercase << std::setw(2) << (int)frame[i] << std::dec << std::endl;
    
    std::cout << std::endl << std::endl;
}