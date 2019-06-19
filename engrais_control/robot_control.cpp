//********************************************************************************************************
#include <chrono>
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

#include <Point.h>
#include <Model.h>
#include <Utility.h>

#include <WeightedModel.h>
#include <RobotControl.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>


#define BODY_SIZE 2.0
#define SLEEP_TIME 500
#define PI 3.1415926535
#define TO_MILLISECOND 1000
#define DISTANCE_REFERENCE 1.5

using namespace std;

mutex critSec; //Critical section mutex

string mapName; //rviz name
bool endProgram = false; //End node

ros::Publisher pubLeftControl; //Publish left wheel command
ros::Publisher pubRightControl; //Publish right wheel command
ros::Publisher pubSelectedLines; //Publish selected lines

RobotControl control; //Declare control

//--------------------------------------------------------------------------------------------------------
void sendLine(const pair<Model, Model> & models){ //Send model's first and last point using the visualization marker 
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;

    line_list.header.frame_id = mapName; //Assign map name
    line_list.header.stamp = ros::Time::now(); //Assign time
    line_list.ns = "points_and_lines"; //Message type 
    line_list.action = visualization_msgs::Marker::ADD; //Add points
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0; //Set id
    line_list.type = visualization_msgs::Marker::LINE_LIST; //Message line list flag

    line_list.scale.x = 0.08; //Line list scale

    line_list.color.g = 1.0; //Line list color
    line_list.color.a = 0.4;

    if(models.first.isPopulated()){ //If left model is populated
        pair<Point, Point> points = models.first.getFirstAndLastPoint(); //Get first and last point

        p.x = points.first.getX(); //Get first point X coordinate
        p.y = models.first.getSlope()*p.x + models.first.getIntercept(); //Calculate Y using model

        line_list.points.push_back(p); //Push point

        p.x = points.second.getX(); //Get last point X coordinate
        p.y = models.first.getSlope()*p.x + models.first.getIntercept(); //Calculate Y using model

        line_list.points.push_back(p); //Push point
    }

    if(models.second.isPopulated()){ //If right model is populated
        pair<Point, Point> points = models.second.getFirstAndLastPoint(); //Get first and last point

        p.x = points.first.getX(); //Get first point X coordinate
        p.y = models.second.getSlope()*p.x + models.second.getIntercept(); //Calculate Y using model

        line_list.points.push_back(p); //Push point

        p.x = points.second.getX(); //Get last point X coordinate
        p.y = models.second.getSlope()*p.x + models.second.getIntercept(); //Calculate Y using model

        line_list.points.push_back(p); //Push point
    }
    
    pubSelectedLines.publish(line_list); //Push lines list
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ //Control Thread 
    while(!endProgram){ //While the program runs

        critSec.lock(); //Lock critical section
        control.clearModels(); //Clear all models
        critSec.unlock(); //Unlock critical section

        usleep(SLEEP_TIME * TO_MILLISECOND); //Sleep for a time

        critSec.lock();  //Lock critical section

        pair<Model, Model> selectedModels = control.selectModels(); //Select models
        sendLine(selectedModels); //Send selected lines
        pair<std_msgs::Float64, std_msgs::Float64> wheels = control.getWheelsCommand(selectedModels); //Calculates wheels' commands

        critSec.unlock(); //Unlock

        pubLeftControl.publish(wheels.first); //Send left wheel command
        pubRightControl.publish(wheels.second); //Send right wheel command

        cout << "Left: " << wheels.first.data <<  ", Right: " << wheels.second.data << endl << endl << endl << endl << endl; //Print commands calculated

    }
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ //Back node message received 
    if(msg.type != 5) //Check flag to see if it's line list
        return;

    critSec.lock(); //Lock critical section
    control.backMessage(msg); //Feed back message to control
    critSec.unlock(); //Unlock critical section
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ //Front node message received 
    if(msg.type != 5) //Check flag to see if it's line list
        return;
    
    critSec.lock(); //Lock critical section
    control.frontMessage(msg); //Feed front message to control
    critSec.unlock(); //Unlock critical section
}


//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main function 

    ros::init(argc, argv, "robot_move_node"); //Initialize ROS
    ros::NodeHandle node;

    string subTopicFront, subTopicBack, pubTopicLeft, pubTopicRight, pubTopicSelected, node_name = ros::this_node::getName();

    if(!node.getParam(node_name + "/subscribe_topic_front", subTopicFront) || !node.getParam(node_name + "/subscribe_topic_back", subTopicBack) || 
       !node.getParam(node_name + "/publish_topic_left", pubTopicLeft) || !node.getParam(node_name + "/publish_topic_right", pubTopicRight)){ //Get mandatory parameters

        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic_front', 'subscribe_topic_back', " <<
                         "'publish_topic_left', 'publish_topic_right' and [optional: 'rviz_topic' and 'rviz_frame']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/rviz_frame", mapName, "world"); //Get optional parameters
    node.param<string>(node_name + "/rviz_topic", pubTopicSelected, "/engrais/robot_move/selected_lines");

    ros::Subscriber subFront = node.subscribe(subTopicFront, 10, FrontLinesMsg); //Subscribe to topic
    ros::Subscriber subBack = node.subscribe(subTopicBack, 10, BackLinesMsg);

    pubLeftControl = node.advertise<std_msgs::Float64>(pubTopicLeft, 10); //Publish topics
    pubRightControl = node.advertise<std_msgs::Float64>(pubTopicRight, 10);


    pubSelectedLines = node.advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread); //Declare and launch new thread

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin(); //SPin to receive messages
    Utility::printInColor("Shitting down...", CYAN);

    endProgram = true; //Set to end program
    control_t.join(); //Wait thread to finish

    subFront.shutdown(); //Shutdown ROS
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();

    Utility::printInColor("Code ended without errors", BLUE);
}

//********************************************************************************************************