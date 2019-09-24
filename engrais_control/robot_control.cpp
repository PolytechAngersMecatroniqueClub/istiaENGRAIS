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

#define PI 3.1415926535
#define TO_MILLISECOND 1000

using namespace std;

mutex critSec;

string mapName;
bool endProgram = false;

int SLEEP_TIME;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubSelectedLines;

RobotControl* control;


//--------------------------------------------------------------------------------------------------------
void sendLine(const pair<Model, Model> & models){ 
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;

    line_list.header.frame_id = mapName;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.08;

    line_list.color.g = 1.0;
    line_list.color.a = 0.4;

    if(models.first.isPopulated()){
        pair<Point, Point> points = models.first.getFirstAndLastPoint();

        p.x = points.first.getX();
        p.y = models.first.getSlope()*p.x + models.first.getIntercept();

        line_list.points.push_back(p);

        p.x = points.second.getX();
        p.y = models.first.getSlope()*p.x + models.first.getIntercept();

        line_list.points.push_back(p);
    }

    if(models.second.isPopulated()){
        pair<Point, Point> points = models.second.getFirstAndLastPoint();

        p.x = points.first.getX();
        p.y = models.second.getSlope()*p.x + models.second.getIntercept();

        line_list.points.push_back(p);

        p.x = points.second.getX();
        p.y = models.second.getSlope()*p.x + models.second.getIntercept();

        line_list.points.push_back(p);
    }
    
    pubSelectedLines.publish(line_list);
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ 
    while(!endProgram){

        critSec.lock();
        control->clearModels();
        critSec.unlock();

        usleep(SLEEP_TIME * TO_MILLISECOND);

        critSec.lock();  

        pair<Model, Model> selectedModels = control->selectModels();
        sendLine(selectedModels);
        pair<std_msgs::Float64, std_msgs::Float64> wheels = control->getWheelsCommand(selectedModels);

        //cout << control << endl;
        critSec.unlock();

        pubLeftControl.publish(wheels.first);
        pubRightControl.publish(wheels.second);

        //cout << "Left: " << wheels.first.data <<  ", Right: " << wheels.second.data << endl << endl << endl << endl << endl;

    }
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;

    critSec.lock();
    control->backMessage(msg);
    critSec.unlock();
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;
    
    critSec.lock();
    control->frontMessage(msg);
    critSec.unlock();
}


//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    ros::init(argc, argv, "robot_move_node");
    ros::NodeHandle node;

    double MAX_VEL, BODY_SIZE, DISTANCE_REFERENCE;

    string subTopicFront, subTopicBack, pubTopicLeft, pubTopicRight, pubTopicSelected, node_name = ros::this_node::getName();

    if(!node.getParam(node_name + "/subscribe_topic_front", subTopicFront) || !node.getParam(node_name + "/subscribe_topic_back", subTopicBack) || 
       !node.getParam(node_name + "/publish_topic_left", pubTopicLeft) || !node.getParam(node_name + "/publish_topic_right", pubTopicRight) ||
       !node.getParam(node_name + "/max_velocity", MAX_VEL) || !node.getParam(node_name + "/body_size", BODY_SIZE) || 
       !node.getParam(node_name + "/distance_reference", DISTANCE_REFERENCE) || !node.getParam(node_name + "/sleep_time_ms", SLEEP_TIME)) {

        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic_front', 'subscribe_topic_back', " <<
                         "'publish_topic_left', 'publish_topic_right', 'robot_max_velocity', 'robot_body_size', 'robot_distance_reference' and [optional: 'rviz_topic' and 'rviz_frame']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/rviz_frame", mapName, "world");
    node.param<string>(node_name + "/rviz_topic", pubTopicSelected, "/engrais/robot_move/selected_lines");

    control = new RobotControl(MAX_VEL, BODY_SIZE, DISTANCE_REFERENCE);

    ros::Subscriber subFront = node.subscribe(subTopicFront, 10, FrontLinesMsg);
    ros::Subscriber subBack = node.subscribe(subTopicBack, 10, BackLinesMsg);

    pubLeftControl = node.advertise<std_msgs::Float64>(pubTopicLeft, 10);
    pubRightControl = node.advertise<std_msgs::Float64>(pubTopicRight, 10);


    pubSelectedLines = node.advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread);

    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shutting down...");

    endProgram = true;
    control_t.join();

    delete control;

    subFront.shutdown();
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();

    ROS_INFO("Code ended without errors");

    return 0;
}

//********************************************************************************************************