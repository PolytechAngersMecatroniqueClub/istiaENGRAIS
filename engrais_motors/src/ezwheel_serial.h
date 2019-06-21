#ifndef EZWHEEL_SERIAL
#define EZWHEEL_SERIAL

#include "serial/serial.h"

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

/* To display the value of a wheelstatus
 - wheelstatus: the wheel status structure we want to display
*/
void display_status(const WheelStatus& wheelstatus);

/* compute the CRC and add it at the end of the frame
 - frame: the array you want to add the CRC at the end
 - size: the size of the array
 - return: the value of the CRC that has been added
*/
uint8_t addCRC(uint8_t* frame, size_t size);

/* Initialize the frame as a get request
 - request: the frame (array) you want to initialize
*/
void initGetRequest(uint8_t* request);

/* Initialize the frame as a set request
 - request: the frame (array) you want to initialize
 - data_size: the size of the data for the set request
*/
void initSetRequest(uint8_t* request, uint8_t data_size);

/* To get the state of charge (percent 0-100) of the wheel
 - serial: the variable to access the serial medium
 - charge: variable to store the value of the charge (percent, integer values)
 - return : 1 if everything went well
           -1 if the received data do not have the right size
           -2 if the received data are an error frame
*/
int getStateOfCharge(serial::Serial& my_serial, uint8_t& charge);

/* To get the temperature (°C) of the wheel
 - serial: the variable to access the serial medium
 - temp: variable to store the value of the temperature (°C)
 - return : 1 if everything went well
           -1 if the received data do not have the right size
           -2 if the received data are an error frame
*/
int getEngineTemperature(serial::Serial& my_serial, double& temp);

/* To get the status of the wheel - cf the status structure
 - serial: the variable to access the serial medium
 - wheelstatus: variable to store the status of the wheel (cf previous structure)
 - return : 1 if everything went well
           -1 if the received data do not have the right size
           -2 if the received data are an error frame
*/
int getWheelStatus(serial::Serial& my_serial, WheelStatus& wheelstatus);

/* To set the wheel speed and direction
 - serial: the variable to access the serial medium
 - speed: The value of the speed we want to wheel to be (0.1 km/h)
 - direction: The direction of the wheel (forward, backward, free)
 - return : 1 if everything went well
           -1 if the received data do not have the right size
           -2 if the received data are an error frame
*/
int setWheelSpeed(serial::Serial& my_serial, uint16_t speed, uint8_t direction, uint8_t sess_id = 0);


void print_frame(const uint8_t * frame, uint8_t size);

#endif
