//********************************************************************************************************
#ifndef EZWHEEL_SERIAL
#define EZWHEEL_SERIAL

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <stdexcept>
#include "serial/serial.h"

class WheelStatusStructure{ 
	public:
		int direction = -1; //0 = No movement, 1 = Clockwise, 2 = Counter Clock-wise
		int type = -1; //1 = Speed Mode
		int brakeMode = -1; //1 = Motor brake active, 0 = Motor brake not active
		double torque = -1; //Torque estimation (not reliable)
		double speed = -1; //Absolute Speed
		int error = -1; //Unused
		int chargeStatus = -1; //Unused
		int batteryStatus = -1; //Unused

		WheelStatusStructure();

        friend std::ostream & operator << (std::ostream & out, const WheelStatusStructure & w); //Print Point
};

class EzWheelSerial{
	private:
		uint8_t frame_count = 0x00; //Frame count

		serial::Serial* my_serial = NULL; //Declare communication pointer

	public:
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		EzWheelSerial(const std::string port_name, const int baud, const int timeout, const int bytesize, const int parity, const int flowctrl, const int stop_bit); //Creates communication
		//------------------------------------------------------------------------------------------------
		~EzWheelSerial(); //Destructor to destroy pointer

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		bool setWheelSpeed(const double speed, const bool isClockwise); //Sends Velocity Target 

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double getWheelSpeed(); //Get absolute wheel speed, -1 if communication failed
		//------------------------------------------------------------------------------------------------
		int getStateOfCharge(); //Get wheel's battery level
		//------------------------------------------------------------------------------------------------
		WheelStatusStructure getWheelStructure(); //Gets wheel status structure

		//################################################################################################

	private:

		//################################################################################################
		int listenResponse(uint8_t* frame); //Listens and builds response frame, -1 if it fails
		//------------------------------------------------------------------------------------------------
		uint8_t calculateCRC(uint8_t* frame, const size_t size); //Calculates Frame CRC 
		//------------------------------------------------------------------------------------------------
		void print_frame(const uint8_t * frame, uint8_t size); //Print frame for debugging

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		void initGetRequest(uint8_t* request); //Initialize common bytes for get requests
		//------------------------------------------------------------------------------------------------
		void initSetRequest(uint8_t* request, const uint8_t data_size); //Initialize common bytes for set requests

		//################################################################################################
};

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
inline WheelStatusStructure::WheelStatusStructure() {}
//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const WheelStatusStructure &w) { return (out << "WheelStatusStructure: [ direction: " << w.direction << ", type: " << w.type << ", brakeMode: " << w.brakeMode << ", torque: " << w.torque << ", speed: " << w.speed << ", error: " <<  w.error << ", chargeStatus: " << w.chargeStatus << ", batteryStatus: " << w.batteryStatus << " ]"); } //Print Point

//########################################################################################################

#endif