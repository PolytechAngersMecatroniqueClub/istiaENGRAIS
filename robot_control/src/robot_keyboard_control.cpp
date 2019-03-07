#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <signal.h>
#include <termios.h>
#include <thread> 

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

using namespace std;

bool sai = false;

void RestoreKeyboardBlocking(struct termios *initial_settings){
    tcsetattr(0, TCSANOW, initial_settings);
    return;
}

void SetKeyboardNonBlock(struct termios *initial_settings){
    struct termios new_settings;
    tcgetattr(0,initial_settings);
    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
}


int main(int argc, char **argv)
{
	char c;

	ros::init(argc, argv, "robot_keyboard_control_node");

    ros::NodeHandle node;
	
	ros::Rate loop_rate(30); // frequency of the ros loop (in Hz)

	struct termios term_settings;
	SetKeyboardNonBlock(&term_settings);

	
	ros::Publisher pub = node.advertise<std_msgs::Float32MultiArray>("/robot/wheels/instant_vel_cmd", 100);

	std_msgs::Float32MultiArray cmd;
	cmd.data.clear();

	cmd.data.push_back(0);
	cmd.data.push_back(0);

	ROS_INFO("Controlling Robot with keyboard, please use the arrows to control it, and Ctrl+C to stop");

	while(true){
		cmd.data[0] = cmd.data[1] = 0;
        if((c = getchar()) != -1 && c == '\033'){
    		if((c = getchar()) != -1 && c == '['){
    			c = getchar();
    			if(c == 'A'){
    				//cout << "up" << endl;
    				cmd.data[0] = 2;
    				cmd.data[1] = 2;
    			}
    			else if(c == 'B'){
    				//cout << "down" << endl;
    				cmd.data[0] = -2;
    				cmd.data[1] = -2;
    			}
    			else if(c == 'C'){
    				//cout << "right" << endl;
    				cmd.data[0] = 2;
    				cmd.data[1] = -2;
    			}
    			else if(c == 'D'){
    				//cout << "left" << endl;
    				cmd.data[0] = -2;
    				cmd.data[1] = 2;
    			}
    		}
        }

        pub.publish(cmd);

        loop_rate.sleep(); 

		if(int(c) == 3)
			break;	
	}

	ROS_INFO("Exiting program...");

	pub.shutdown();
	ros::shutdown();

	RestoreKeyboardBlocking(&term_settings);
	//keyboard_t.join();
	return 0;
}