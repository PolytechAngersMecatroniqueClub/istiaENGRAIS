#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include <cmath>
#include <chrono>
#include <time.h>  
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "src/Point.h"
#include "src/WeightedPoint.h"
#include "src/Utility.h"
#include "src/Model.h"
#include "src/Pearl.h"

#include "src/Ruby_Versions/1_RubyPure.h"
#include "src/Ruby_Versions/2_RubyGenetic.h"
#include "src/Ruby_Versions/3_RubyGeneticOnePoint.h"

using namespace std;
	
Pearl pearl;

RubyPure rubyPure;
RubyGenetic rubyGen;
RubyGeneticOnePoint rubyGenOP;

ros::Subscriber sub;
ros::Publisher pubLineNode;

double totalExecutionTime = 0;
int timesExecuted = 0;

//--------------------------------------------------------------------------------------------------------
void preparePointsAndLines(visualization_msgs::Marker & line_list, visualization_msgs::Marker & points, const double botX, const double topX, const double botY, const double topY) { 
        points.header.frame_id = line_list.header.frame_id = "map";
        points.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_list.ns = "points_and_lines";
        points.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list.pose.orientation.w = 1.0;


        line_list.id = 0;
        points.id = 1;


        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

		// POINTS markers use x and y scale for width/height respectively
		points.scale.x = 0.05;
		points.scale.y = 0.05;

		// Points are blue
		points.color.r = 1.0;
		points.color.a = 1.0;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.1;


        // Line list is red
        line_list.color.b = 1.0;
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
    visualization_msgs::Marker line_list, points;
    geometry_msgs::Point p;

    preparePointsAndLines(line_list, points, 0.6, -1.2, 0.8, -0.8);

    for(Point point : rubyGenOP.getInitialField()){
		p.x = point.getX();
		p.y = point.getY();
		p.z = 0.1;

		points.points.push_back(p);
    }

    if(models.first.getSlope() != MAX_DBL && models.first.getIntercept() != MAX_DBL){
	    p.x = 0;
	    p.y = models.first.getSlope()*0 + models.first.getIntercept();
	    p.z = 0;

	    line_list.points.push_back(p);

	    p.x = 20;
	    p.y = models.first.getSlope()*p.x + models.first.getIntercept();

	    line_list.points.push_back(p);
	}

	if(models.second.getSlope() != MAX_DBL && models.second.getIntercept() != MAX_DBL){
	    p.x = 0;
	    p.y = models.second.getSlope()*0 + models.second.getIntercept();
	    p.z = 0;

	    line_list.points.push_back(p);

	    p.x = 20;
	    p.y = models.second.getSlope()*p.x + models.second.getIntercept();

	    line_list.points.push_back(p);
   	}

    pubLineNode.publish(points);
    pubLineNode.publish(line_list);

    /*if(fabs(models.first.getIntercept()) > 3)
    	exit(1);

    if(fabs(models.second.getIntercept()) > 3)
    	exit(1);*/
}


//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    auto start = std::chrono::system_clock::now();

    rubyGen.populateOutliers(msg);

    pair <Model, Model> lines = rubyGen.findLines();

    sendLine(lines);

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    totalExecutionTime += elapsed_seconds.count();
    timesExecuted++;

    std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
// -------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    ROS_INFO("Initializing Robot Control Ros Node");

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/robot_engrais/lidar_engrais/data", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shitting down...");


    sub.shutdown();
    pubLineNode.shutdown();
    ros::shutdown();


    ROS_INFO("Code ended without errors");

    Utility::printInColor("Total Execution Calculations Time: " + to_string(totalExecutionTime) + "s, running " + to_string(timesExecuted) + " times.\nMean Calculation time: " + to_string(totalExecutionTime/(double)timesExecuted), BLUE);
    
    return 0;
}

// *******************************************************************************************************