//********************************************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

#include <signal.h>
#include <termios.h>
#include <thread> 

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"

#define MESSAGE_FREQUENCY 30.0

using namespace std;

bool sai = false;


//--------------------------------------------------------------------------------------------------------
void RestoreKeyboardBlocking(struct termios *initial_settings){
    tcsetattr(0, TCSANOW, initial_settings);
    return;
}
//--------------------------------------------------------------------------------------------------------
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
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
	char c;

	ros::init(argc, argv, "robot_keyboard_control_node");

    ros::NodeHandle node;
	
	struct termios term_settings;
	SetKeyboardNonBlock(&term_settings);

	ros::Publisher pub = node.advertise<std_msgs::Float32MultiArray>("/robot_engrais/wheels/instant_vel_cmd", 100);

	std_msgs::Float32MultiArray cmd;
    
	cmd.data.clear();

	cmd.data.push_back(0);
	cmd.data.push_back(0);

    ros::Rate loop_rate(30);

	ROS_INFO("Controlling Robot with keyboard, please use the arrows to control it, and Ctrl+C to stop");

	while(true){
		cmd.data[0] = cmd.data[1] = 0;
        if((c = getchar()) != -1 && c == '\033'){
            if(int(c) == 3)
                break;

    		if((c = getchar()) != -1 && c == '['){
                if(int(c) == 3)
                    break;

    			c = getchar();

                if(int(c) == 3)
                    break;
    			else if(c == 'A'){
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

        if(int(c) == 3)
            break;

        pub.publish(cmd);

        loop_rate.sleep();  
	}

	ROS_INFO("Exiting program...");

	pub.shutdown();
	ros::shutdown();

	RestoreKeyboardBlocking(&term_settings);

	return 0;
}

//********************************************************************************************************