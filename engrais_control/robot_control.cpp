#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <fstream> 

#include <cmath>
#include <time.h>  
#include <algorithm>
#include <signal.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <chrono>
#include <thread>

#include "src/Point.h"
#include "src/Utility.h"
#include "src/Model.h"
#include "src/Pearl.h"

using namespace std;


Pearl pearl;

ros::Subscriber sub;
ros::Publisher pubLineNode;


void prepareLineList(visualization_msgs::Marker & line_list, const double botX, const double topX, const double botY, const double topY) { 
        line_list.header.frame_id = "/map";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;


        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.1;


        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;
        geometry_msgs::Point p;

        p.x = botX;
        p.y = topY;
        line_list.points.push_back(p);

        p.x = botX;
        p.y = botY;
        line_list.points.push_back(p);

        p.x = topX;
        p.y = topY;
        line_list.points.push_back(p);

        p.x = topX;
        p.y = botY;
        line_list.points.push_back(p);



        p.x = botX;
        p.y = topY;
        line_list.points.push_back(p);

        p.x = topX;
        p.y = topY;
        line_list.points.push_back(p);

        p.x = botX;
        p.y = botY;
        line_list.points.push_back(p);

        p.x = topX;
        p.y = botY;
        line_list.points.push_back(p);
}
//--------------------------------------------------------------------------------------------------------
void sendLine(const pair<Model, Model> & models) { 
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;

    prepareLineList(line_list, 0.6, -1.2, 0.8, -0.8);
    
    

    p.x = 0;
    p.y = models.first.getSlope()*0 + models.first.getIntercept();
    p.z = 0;

    line_list.points.push_back(p);

    p.x = 20;
    p.y = models.first.getSlope()*p.x + models.first.getIntercept();

    line_list.points.push_back(p);


    p.x = 0;
    p.y = models.second.getSlope()*0 + models.second.getIntercept();
    p.z = 0;

    line_list.points.push_back(p);

    p.x = 20;
    p.y = models.second.getSlope()*p.x + models.second.getIntercept();

    line_list.points.push_back(p);

    
    pubLineNode.publish(line_list);
}

//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    //Utility::printVector(pearl.outliers);
    pearl.populateOutliers(msg);

    pair <Model, Model> lines = pearl.findLines();
    sendLine(lines);
}

// -------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){  

    Utility::printInColor("Initializing Robot Control Ros Node", CYAN);

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/robot_engrais/lidar_engrais/data", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    Utility::printInColor("Code Running, press Control+C to end", BLUE);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);


    sub.shutdown();
    pubLineNode.shutdown();
    ros::shutdown();


    Utility::printInColor("Code ended without errors", BLUE);

    return 0;
}

// *******************************************************************************************************
