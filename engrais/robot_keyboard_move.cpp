//********************************************************************************************************
#include <thread> 
#include <stdio.h>
#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <signal.h>
#include <termios.h>

#include <std_msgs/Bool.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

using namespace std;

string node_name, emergecy_topic;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubEmergency;


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
void sendSpeedCommand(){
    char c;

    std_msgs::Float64 rightCmd, leftCmd, lastRightCmd, lastLeftCmd;
    std_msgs::Bool emerg;

    emerg.data = false;
    
    bool wasZero = true;
    int notZeroCount = 0;

    ros::Rate loop_rate(30);

    while(true){
        rightCmd.data = leftCmd.data = 0;
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
                    leftCmd.data = 1;
                    rightCmd.data = 1;
                }
                else if(c == 'B'){
                    //cout << "down" << endl;
                    leftCmd.data = -1;
                    rightCmd.data = -1;
                }
                else if(c == 'C'){
                    //cout << "right" << endl;
                    leftCmd.data = 1;
                    rightCmd.data = -1;
                }
                else if(c == 'D'){
                    //cout << "left" << endl;
                    leftCmd.data = -1;
                    rightCmd.data = 1;
                }
            }
        }
        //cout << "Left: " << leftCmd.data << ", Right: " << rightCmd.data << endl;
        if(int(c) == 3)
            break;
        
        
        if(leftCmd.data != 0 && rightCmd.data != 0 && wasZero == true){
            wasZero = false;

            lastLeftCmd.data = leftCmd.data;
            lastRightCmd.data = rightCmd.data;
        }

        else if(leftCmd.data == 0 && rightCmd.data == 0 && wasZero == false && notZeroCount < 14){          
            leftCmd.data = lastLeftCmd.data;
            rightCmd.data = lastRightCmd.data;

            notZeroCount++;
        }

        else if(leftCmd.data == 0 && rightCmd.data == 0 && wasZero == false &&  notZeroCount >= 14){          
            lastLeftCmd.data = leftCmd.data;
            lastRightCmd.data = rightCmd.data;

            wasZero = true;
            notZeroCount = 0;
        }

        else if(leftCmd.data != lastLeftCmd.data || rightCmd.data != lastRightCmd.data){
            lastLeftCmd.data = leftCmd.data;
            lastRightCmd.data = rightCmd.data;

            notZeroCount = 0;
            wasZero == false;
        }

        leftCmd.data *= 1.5;
        rightCmd.data *= 1.5;

        pubLeftControl.publish(leftCmd);
        pubRightControl.publish(rightCmd);

        if (emergecy_topic != "none")
            pubEmergency.publish(emerg);

        cout << "a" << endl;

        loop_rate.sleep();  
    }
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_keyboard_control_node");

    ros::NodeHandle node;

    string pub_topic_right, pub_topic_left;
    
    node_name = ros::this_node::getName();

    if(!node.getParam(node_name + "/pub_topic_right", pub_topic_right) || !node.getParam(node_name + "/pub_topic_left", pub_topic_left)){ //Get mandatory parameters
        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected pub_topic_right, 'pub_topic_left'.\n\n");
        return -1;
    }

    node.param<string>(node_name + "/emergency_topic", emergecy_topic, "none");
    
    struct termios term_settings;
    SetKeyboardNonBlock(&term_settings);

    pubLeftControl = node.advertise<std_msgs::Float64>(pub_topic_left, 10);
    pubRightControl = node.advertise<std_msgs::Float64>(pub_topic_right, 10);
    
    if (emergecy_topic != "none")
        pubEmergency = node.advertise<std_msgs::Bool>(emergecy_topic, 10);


    ROS_INFO("Controlling Robot with keyboard, please use the arrows to control it, and Ctrl+C to stop");

    sendSpeedCommand();

    ROS_INFO("Shutting down...");

    RestoreKeyboardBlocking(&term_settings);

    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    if (emergecy_topic != "none")
        pubEmergency.shutdown();

    ros::shutdown();

    ROS_INFO("Code ended without errors");

    return 0;
}

//********************************************************************************************************
