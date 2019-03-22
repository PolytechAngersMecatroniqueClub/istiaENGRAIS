//********************************************************************************************************

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

#include "src/robot_findlines_include/1_Point/Point.h"
#include "src/robot_findlines_include/2_WeightedPoint/WeightedPoint.h"
#include "src/robot_findlines_include/3_Utility/Utility.h"
#include "src/robot_findlines_include/4_Model/Model.h"

#include "src/robot_findlines_include/5_Pearl/Pearl.h"
#include "src/robot_findlines_include/6_Ruby_Versions/1_RubyPure.h"
#include "src/robot_findlines_include/6_Ruby_Versions/2_RubyGenetic.h"
#include "src/robot_findlines_include/6_Ruby_Versions/3_RubyGeneticOnePoint.h"
#include "src/robot_findlines_include/6_Ruby_Versions/4_RubyGeneticOnePointPosNeg.h"
#include "src/robot_findlines_include/6_Ruby_Versions/5_RubyGeneticOnePointPosNegInfinite.h"


using namespace std;
	
Pearl pearl;
RubyPure rubyPure;
RubyGenetic rubyGen;
RubyGeneticOnePoint rubyGenOP;
RubyGeneticOnePointPosNeg rubyGenOPPN;
RubyGeneticOnePointPosNegInfinite rubyGenOPPNInf;

ros::Subscriber sub;
ros::Publisher pubLineNode;

double totalExecutionTime = 0;
int timesExecuted = 0;



//--------------------------------------------------------------------------------------------------------
void preparePointsAndLines(visualization_msgs::Marker & line_list, visualization_msgs::Marker & points) { 
        points.header.frame_id = line_list.header.frame_id = "sick_front_link";
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

		// Points are red
		points.color.r = 1.0;
		points.color.a = 1.0;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.03;


        // Line list is blue
        line_list.color.b = 1.0;
        line_list.color.a = 1.0;
}
//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models, Pearl & pearl) { 
    visualization_msgs::Marker line_list, points;
    geometry_msgs::Point p;

    preparePointsAndLines(line_list, points);

    for(Point point : pearl.getInitialField()){
		p.x = point.getX();
		p.y = point.getY();
		p.z = 0.1;

		points.points.push_back(p);
    }
    p.z = 0;

    for(int i = 0; i < models.size(); i++){
        if(models[i].getSlope() != MAX_DBL && models[i].getIntercept() != MAX_DBL){
    	    p.x = 0;
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

    	    line_list.points.push_back(p);

    	    p.x = 20;
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

    	    line_list.points.push_back(p);
    	}
    }

    pubLineNode.publish(points);
    pubLineNode.publish(line_list);
}


//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    auto start = std::chrono::system_clock::now();

    rubyGenOPPNInf.populateOutliers(msg);

    vector <Model> lines = rubyGenOPPNInf.findLines();

    sendLine(lines, rubyGenOPPNInf);

    //cout << rubyGenOPPNInf << endl;

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    totalExecutionTime += elapsed_seconds.count();
    timesExecuted++;
//exit(1);
    std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    ROS_INFO("Initializing Robot Control Ros Node");

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/engrais/laser_front/scan", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<visualization_msgs::Marker>("/robot_engrais/all_lines_found", 10);

    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shitting down...");


    sub.shutdown();
    pubLineNode.shutdown();
    ros::shutdown();


    ROS_INFO("Code ended without errors");

    Utility::printInColor("Total Calculations Time: " + to_string(totalExecutionTime) + "s, code ran " + to_string(timesExecuted) + " times.\nMean Calculation time: " + to_string(totalExecutionTime/(double)timesExecuted) + "s", BLUE);
    

    return 0;
}

//********************************************************************************************************