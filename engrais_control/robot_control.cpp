//********************************************************************************************************
#define BODY_SIZE 1.1
#define SLEEP_TIME 250
#define PI 3.1415926535
#define TO_MILLISECOND 1000
#define DISTANCE_REFERENCE 1.5
#define MAX_VEL 0.7

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
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

using namespace std;

mutex critSec; //Critical section mutex

string mapName; //rviz name

ros::Publisher pubLeftControl; //Publish left wheel command
ros::Publisher pubRightControl; //Publish right wheel command
ros::Publisher pubSelectedLines; //Publish selected lines

RobotControl control; //Declare control

std::vector<int> contMsgs(2,0);

//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models){ //Send model's first and last point using the visualization marker 
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

    for(int i = 0; i < models.size(); i++){ //For every model found
        if(models[i].isPopulated() && models[i].getPointsSize() >= 2){ //If model is populated
            pair<Point, Point> points = models[i].getFirstAndLastPoint(); //Finds negative and positive-most points (x-axis)

            p.x = points.first.getX(); //Get first point X coordinate
            p.y = models[i].getSlope()*p.x + models[i].getIntercept(); //Calculate Y using the model's information

            line_list.points.push_back(p); //Push point

            p.x = points.second.getX(); //Last point X coordinate
            p.y = models[i].getSlope()*p.x + models[i].getIntercept(); //Calculate using model's information

            line_list.points.push_back(p);
        }
    }
    
    pubSelectedLines.publish(line_list); //Push lines list
}


//--------------------------------------------------------------------------------------------------------
void exitApp(const std_msgs::Bool & msg){ //ROS message received 
    if(msg.data)
        ros::shutdown();
}
//--------------------------------------------------------------------------------------------------------
void controlThread(){ //Control Thread 
    while(ros::ok()){ //While the program runs

        critSec.lock(); //Lock critical section

        contMsgs = {0,0};

        control.clearModels(); //Clear all models

        critSec.unlock(); //Unlock critical section



        usleep(SLEEP_TIME * TO_MILLISECOND); //Sleep for a time



        critSec.lock();  //Lock critical section

        //std::cout << "ContFront: " << contMsgs[1] << ", contBack: " << contMsgs[0] << std::endl << std::endl;

        std::cout << "Before change: " << control << std::endl << std::endl << std::endl << std::endl;


        vector<Model> selectedModels = control.selectModels(contMsgs); //Select models

        //std::cout << "After change: " << control << std::endl << std::endl << std::endl << std::endl;

        std::cout << "Selected Models:: " << endl << endl;

        Utility::printVector(selectedModels);


        sendLine(selectedModels); //Send selected lines

        //pair<std_msgs::Float64, std_msgs::Float64> wheels = control.getWheelsCommand(selectedModels); //Calculates wheels' commands

        //cout << "\n---------------------------------------------------------------\n\n";
        
        critSec.unlock(); //Unlock*/

        //pubLeftControl.publish(wheels.first); //Send left wheel command
        //pubRightControl.publish(wheels.second); //Send right wheel command

    }
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ //Back node message received 
    critSec.lock(); //Lock critical section

    if(msg.type == visualization_msgs::Marker::LINE_LIST){ //Check flag to see if it's line list
        control.backLinesMessage(msg); //Feed back message to control
        contMsgs[0]++;
    }
    
    else if(msg.type == visualization_msgs::Marker::POINTS)
        control.backPointsMessage(msg); //Feed back message to control

    critSec.unlock(); //Unlock critical section
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ //Front node message received 
    critSec.lock(); //Lock critical section
    
    if(msg.type == visualization_msgs::Marker::LINE_LIST){ //Check flag to see if it's line list    
        control.frontLinesMessage(msg); //Feed front message to control
        contMsgs[1]++;
    }

    else if(msg.type == visualization_msgs::Marker::POINTS)
        control.frontPointsMessage(msg); //Feed front message to control
    
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
    ros::Subscriber exitSub = node.subscribe("exit_app_topic", 10, exitApp); // /engrais/laser_front/scan or /engrais/laser_back/scan

    pubLeftControl = node.advertise<std_msgs::Float64>(pubTopicLeft, 10); //Publish topics
    pubRightControl = node.advertise<std_msgs::Float64>(pubTopicRight, 10);


    pubSelectedLines = node.advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread); //Declare and launch new thread

    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    ros::spin(); //Spin to receive messages
    Utility::printInColor(node_name + ": Shutting down...", CYAN);

    control_t.join(); //Wait thread to finish

    subFront.shutdown(); //Shutdown ROS
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();

    Utility::printInColor(node_name + ": Code ended without errors", BLUE);
}

//********************************************************************************************************