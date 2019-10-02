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
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define PI 3.1415926535
#define TO_MILLISECOND 1000

using namespace std;

mutex critSec;

string mapName, node_name, emergecy_topic, mode, changeModeTopic;

int SLEEP_TIME;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubSelectedLines;

RobotControl* control;

ros::NodeHandle* node;

ros::MultiThreadedSpinner spinner(2);

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
void ModeChangeMsg(const std_msgs::String & msg){ 
    if(msg.data != "automatic"){
        mode = "manual";
        
    }
    else{
        mode = "automatic";
    }

    Utility::printInColor(node_name + ": Mode changed to " + mode + "\n", CYAN);
    //ROS_WARN_STREAM(node_name << ": Mode changed to " << mode);
}
//--------------------------------------------------------------------------------------------------------
void changeModeThread(){ 
    ros::Subscriber subMode = node->subscribe(changeModeTopic, 10, ModeChangeMsg);

    spinner.spin();

    subMode.shutdown();
}


ros::Time lastMsg;
bool comReady = false;
bool emergencyCalled = false;

//--------------------------------------------------------------------------------------------------------
void OnEmergencyBrake(const std_msgs::Bool & msg){
    lastMsg = ros::Time::now();
    
    comReady = true;

    if(msg.data == true){
        Utility::printInColor(node_name + ": Emergency Shutdown Called", RED);
        emergencyCalled = true;
        ros::shutdown();
    }
}
//--------------------------------------------------------------------------------------------------------
void emergencyThread(){
    lastMsg = ros::Time::now();

    ros::Subscriber emergencySub = node->subscribe(emergecy_topic, 10, OnEmergencyBrake);
    ros::Publisher emergencyPub = node->advertise<std_msgs::Bool>(emergecy_topic, 10);

    std_msgs::Bool msg;
    msg.data = false;
    
    while(ros::ok() && !comReady && mode != "automatic")
        ros::Duration(0.01).sleep();
    
    while(ros::ok() && !emergencyCalled){
        if(mode != "automatic" && !emergencyCalled){
            ros::Time now = ros::Time::now();
            
            ros::Duration delta_t = now - lastMsg;

            if(!emergencyCalled && delta_t.toSec() > 0.2){
                Utility::printInColor(node_name + ": Emergency Timeout Shutdown", RED);
                ros::shutdown();
            }

        }
        else{
            emergencyPub.publish(msg);
        }

        ros::Duration(0.05).sleep();
    }

    emergencySub.shutdown();
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ 
    while(ros::ok()){

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
        
        if(mode == "automatic"){
            pubLeftControl.publish(wheels.first);
            pubRightControl.publish(wheels.second);
        }

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
    node = new ros::NodeHandle();

    double MAX_VEL, BODY_SIZE, DISTANCE_REFERENCE;

    string subTopicFront, subTopicBack, pubTopicLeft, pubTopicRight, pubTopicSelected;
   
    node_name = ros::this_node::getName();

    if(!node->getParam(node_name + "/subscribe_topic_front", subTopicFront) || !node->getParam(node_name + "/subscribe_topic_back", subTopicBack) || 
       !node->getParam(node_name + "/publish_topic_left", pubTopicLeft) || !node->getParam(node_name + "/publish_topic_right", pubTopicRight) ||
       !node->getParam(node_name + "/max_velocity", MAX_VEL) || !node->getParam(node_name + "/body_size", BODY_SIZE) || 
       !node->getParam(node_name + "/distance_reference", DISTANCE_REFERENCE) || !node->getParam(node_name + "/sleep_time_ms", SLEEP_TIME) || !node->getParam(node_name + "/mode", mode) || !node->getParam(node_name + "/change_mode_topic", changeModeTopic)) {

        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic_front', 'subscribe_topic_back', " <<
                         "'publish_topic_left', 'publish_topic_right', 'robot_max_velocity', 'robot_body_size', 'robot_distance_reference', 'mode', 'change_mode_topic' and [optional: 'rviz_topic' and 'rviz_frame']\n\n");
        return -1;
    }

    node->param<string>(node_name + "/rviz_frame", mapName, "world");
    node->param<string>(node_name + "/rviz_topic", pubTopicSelected, "/engrais/robot_move/selected_lines");
    node->param<string>(node_name + "/emergency_topic", emergecy_topic, "none");

    control = new RobotControl(MAX_VEL, BODY_SIZE, DISTANCE_REFERENCE);

    ros::Subscriber subFront = node->subscribe(subTopicFront, 10, FrontLinesMsg);
    ros::Subscriber subBack = node->subscribe(subTopicBack, 10, BackLinesMsg);

    pubLeftControl = node->advertise<std_msgs::Float64>(pubTopicLeft, 10);
    pubRightControl = node->advertise<std_msgs::Float64>(pubTopicRight, 10);

    pubSelectedLines = node->advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread);


    thread* emergency_t;
    if(emergecy_topic != "none")
        emergency_t = new thread(emergencyThread);

    thread changeMode_t(changeModeThread);


    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    spinner.spin();
    Utility::printInColor(node_name + ": Shutting down...", CYAN);

    if(emergecy_topic != "none"){
        emergency_t->join();
        delete emergency_t;
    }

    control_t.join();
    changeMode_t.join();

    delete control;

    subFront.shutdown();
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();
    
    delete node;    

    Utility::printInColor(node_name + ": Code ended without errors", BLUE);

    return 0;
}

//********************************************************************************************************
