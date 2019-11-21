#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

#include "src/ezwheel_serial.h"

#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char **argv){
    /*ros::init(argc, argv, "keep_motor_alive_node");
    ros::NodeHandle node;

    // ********************************************************************
    // Initialization of the node parameters

    std::string port_name_0 = "/dev/ttyUSB0";
    std::string port_name_1 = "/dev/ttyUSB1";
    int baud = 57600;
    //node.getParam("baud", baud);
    int timeout = 50;
    //node.getParam("timeout", timeout);
    int bytesize = 8;
    //node.getParam("data_bits", bytesize);
    int parity = 0;
    //node.getParam("parity", parity);
    int flowctrl = 0;
    //node.getParam("hdw_flow_ctrl", flowctrl);
    int stop_bit = 1;
    //node.getParam("stop_bit", stop_bit);

    ofstream com("communication.txt");
    ofstream arq("sent.txt");

    serial::Serial my_serial_0(port_name_0,
                             baud,
                             serial::Timeout::simpleTimeout(timeout),
                             serial::bytesize_t(bytesize),
                             serial::parity_t(parity),
                             serial::stopbits_t(stop_bit),
                             serial::flowcontrol_t(flowctrl));

    serial::Serial my_serial_1(port_name_1,
                             baud,
                             serial::Timeout::simpleTimeout(timeout),
                             serial::bytesize_t(bytesize),
                             serial::parity_t(parity),
                             serial::stopbits_t(stop_bit),
                             serial::flowcontrol_t(flowctrl));

    // end of the initialization
    // ********************************************************************

    if(my_serial_0.isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", port_name_0.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", port_name_0.c_str());
        return 1;
    }

    if(my_serial_1.isOpen()){
        ROS_INFO("\n ******** \n \tSerial port named %s is oppened \n ********", port_name_1.c_str());
    }
    else{
        ROS_ERROR("Serial port named %s can not be oppened", port_name_1.c_str());
        return 1;
    }

    double charge;

    uint8_t cont = 0x00;

    size_t tramme_1_size = 17;
    uint8_t tramme_1[tramme_1_size];

    tramme_1[ 0] = 0xAA;
    tramme_1[ 1] = 0x0F;
    tramme_1[ 2] = 0x08;
    tramme_1[ 3] = 0x10 + cont; 
    tramme_1[ 4] = 0x01;
    tramme_1[ 5] = 0x0A;
    tramme_1[ 6] = 0x00;
    tramme_1[ 7] = 0x00;
    tramme_1[ 8] = 0x00;
    tramme_1[ 9] = 0x01;
    tramme_1[10] = 0x05;

    tramme_1[11] = 0x00;
    tramme_1[12] = 0x00;
    tramme_1[13] = 0x20;
    tramme_1[14] = 0x02;

    tramme_1[15] = 0x04;
    addCRC(tramme_1, tramme_1_size);


    size_t tramme_2_size = 21;
    uint8_t tramme_2[tramme_1_size];

    tramme_2[ 0] = 0xAA;
    tramme_2[ 1] = 0x13;
    tramme_2[ 2] = 0x08;
    tramme_2[ 3] = 0x10 + cont;
    tramme_2[ 4] = 0x00;
    tramme_2[ 5] = 0x0E;
    tramme_2[ 6] = 0x00;
    tramme_2[ 7] = 0x00;
    tramme_2[ 8] = 0x00;
    tramme_2[ 9] = 0x02;
    tramme_2[10] = 0x05;
    tramme_2[11] = 0x00;
    tramme_2[12] = 0x00;
    tramme_2[13] = 0x04;
    tramme_2[14] = 0x07;
    tramme_2[15] = 0x04;

    tramme_2[16] = 0x16;
    tramme_2[17] = 0x00;
    tramme_2[18] = 0x0A * 2;
    tramme_2[19] = 0x00;

    addCRC(tramme_2, tramme_2_size);

    size_t sniff_size = 13;
    uint8_t sniff[sniff_size];
    int response = 0;

    cout.fill('0');
    com.fill('0');
    arq.fill('0');

    for(int i = 0; i < 40; i++){
	    my_serial_1.write(tramme_1, tramme_1_size);
	    ros::Duration(0.05).sleep();
	}

    int vel = 0;

    tramme_2[18] = 0x00;
    addCRC(tramme_2, tramme_2_size);

    int requestCont = 1;

    while(ros::ok()){

	    my_serial_1.write(tramme_1, tramme_1_size);

	    //ros::Duration(0.05).sleep();

	    cout << "Send Question " << requestCont << ": " << endl;
	    com << "Send Question " << requestCont << ": " << endl;

	    for(int i = 0; i < tramme_1_size; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_1[i]  << dec << endl;
	    	arq << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_1[i]  << dec << endl;
	    	com << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_1[i]  << dec << endl;
	    }

	    cout << endl << endl;
	    com << endl << endl;

	    
	    for(int i = 0; i < sniff_size; i++){
	    	sniff[i] = 0;
	    }

        response = 0;

        response = my_serial_1.read(sniff, sniff_size);

        cout << "Question Reponse " << requestCont << ": " << response << endl;
        com << "Question Reponse " << requestCont << ": " << response << endl;

	    for(int i = 0; i < response; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)sniff[i] << dec << endl;
	    	com << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)sniff[i] << dec << endl;
	    }

	    cout << endl << endl;
	    com << endl << endl;

	    ros::Duration(0.01).sleep();

        /*if(++cont == 0x10){
            cont = 0;
        }
	    tramme_2[3] = 0x10 + cont;
        addCRC(tramme_2, tramme_2_size);


	    my_serial_1.write(tramme_2, tramme_2_size);

	    cout << "Send Control Signal " << requestCont << ": " << endl;
	    com << "Send Control Signal " << requestCont << ": " << endl;

	    for(int i = 0; i < tramme_2_size; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_2[i]  << dec << endl;
	    	arq << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_2[i]  << dec << endl;
	    	com << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)tramme_2[i]  << dec << endl;
	    }

	    cout << endl << endl;
	    com << endl << endl;

	    for(int i = 0; i < sniff_size; i++){
	    	sniff[i] = 0;
	    }

        response = 0;

        response = my_serial_1.read(sniff, sniff_size);

        cout << "Control Signal Reponse " << requestCont << ": " << response << endl;
        com << "Control Signal Reponse " << requestCont << ": " << response << endl;

	    for(int i = 0; i < response; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)sniff[i] << dec << endl;
	    	com << "\t[" << setw(2) << i << "]:\t" << "0x" << uppercase << hex << setw(2) << (int)sniff[i] << dec << endl;
	    }

	    cout << endl << endl;
	    com << endl << endl;*/
        
        /*if(++cont == 0x10){
            cont = 0;
        }

        tramme_1[3] = 0x10 + cont;
        addCRC(tramme_1, tramme_1_size);*/

        /*tramme_1[3] = 0x10 + cont;
        addCRC(tramme_1, tramme_1_size);*/


        /*requestCont++;*/
        /*ros::Duration(0.01).sleep();

    }

    for(int i = 0; i < 40; i++){
	    my_serial_1.write(tramme_1, tramme_1_size);
	    ros::Duration(0.05).sleep();
	}

    /*while(ros::ok()){

	    for(int i = 0; i < response; i++){
	    	sniff[i] = 0;
	    }

	    response = my_serial_0.read(sniff, sniff_size);

	    
	   	cerr << "Sizew: " << response << endl;
	    for(int i = 0; i < response; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t0x" << uppercase << hex << setw(2) << (int)sniff[i]  << dec << endl;
	    }

	    cout << endl << endl;

	    /*ros::Duration(0.01).sleep();

	    tramme_2[3] = ++cont;
        addCRC(tramme_2, tramme_2_size);

	    my_serial_0.write(tramme_2, tramme_2_size);

	    for(int i = 0; i < tramme_2_size; i++){
	    	cout << "\t[" << setw(2) << i << "]:\t0x" << uppercase << hex << setw(2) << (int)tramme_2[i]  << dec << endl;
	    }

	    cout << endl << endl;

	    ros::Duration(0.01).sleep();

	    tramme_1[3] = ++cont;
        addCRC(tramme_1, tramme_1_size);

    }*/

    /*int size = 2;
    uint8_t a[size] ;
    a[0] = 0xAA;
    a[1] = 0x51;

    while(ros::ok()){
	    my_serial_0.write(a, size);
	    cerr << "enviado" << endl;
	    ros::Duration(0.01).sleep();
	}*/
    return 0;
}

