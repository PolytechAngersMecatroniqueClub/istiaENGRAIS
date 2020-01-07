//********************************************************************************************************
#ifndef EZWHEEL_SERIAL
#define EZWHEEL_SERIAL

#include <string>
#include <iostream>
#include <ros/ros.h>
#include <stdexcept>
#include "serial/serial.h"

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

#endif
