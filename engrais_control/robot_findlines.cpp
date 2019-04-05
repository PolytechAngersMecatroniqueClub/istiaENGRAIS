//********************************************************************************************************
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include <cmath>
#include <chrono>
#include <time.h>  
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

#include <engrais_control/Model.h>
#include <engrais_control/Results.h>

#include "src/include/1_Point/Point.h"
#include "src/include/2_WeightedPoint/WeightedPoint.h"
#include "src/include/3_Utility/Utility.h"
#include "src/include/4_Model/Model.h"

#include "src/robot_findlines_include/1_Pearl/Pearl.h"
#include "src/robot_findlines_include/2_Ruby_Versions/1_RubyPure.h"
#include "src/robot_findlines_include/2_Ruby_Versions/2_RubyGenetic.h"
#include "src/robot_findlines_include/2_Ruby_Versions/3_RubyGeneticOnePoint.h"
#include "src/robot_findlines_include/2_Ruby_Versions/4_RubyGeneticOnePointPosNeg.h"
#include "src/robot_findlines_include/2_Ruby_Versions/5_RubyGeneticOnePointPosNegInfinite.h"

using namespace std;
	
//Pearl pearl;
//RubyPure rubyPure;
//RubyGenetic rubyGen;
//RubyGeneticOnePoint rubyGenOP;
RubyGeneticOnePointPosNeg rubyGenOPPN;
//RubyGeneticOnePointPosNegInfinite rubyGenOPPNInf;

string mapName;

ros::Publisher pubLineNode;
ros::Publisher resultsPubNode;


std::chrono::duration<double> elapsed_seconds;
std::time_t end_time;


double totalExecutionTime = 0;
int timesExecuted = 0;


//--------------------------------------------------------------------------------------------------------
void preparePointsAndLines(visualization_msgs::Marker & line_list, visualization_msgs::Marker & points){ 
        points.header.frame_id = line_list.header.frame_id = mapName;
        points.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_list.ns = "points_and_lines";
        points.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_list.pose.orientation.w = 1.0;


        line_list.id = 0;
        points.id = 1;


        points.type = visualization_msgs::Marker::POINTS;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

		points.scale.x = 0.05;
		points.scale.y = 0.05;

		points.color.r = 1.0;
		points.color.a = 1.0;

        line_list.scale.x = 0.03;


        line_list.color.b = 1.0;
        line_list.color.a = 1.0;
}
//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models, const Pearl & pearl){ 
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
            pair<Point, Point> points = models[i].getFirstAndLastPoint();
    	    p.x = points.first.getX();
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

    	    line_list.points.push_back(p);

    	    p.x = points.second.getX();
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

    	    line_list.points.push_back(p);
    	}
    }

    pubLineNode.publish(points);
    pubLineNode.publish(line_list);
}
//--------------------------------------------------------------------------------------------------------
void sendExecutionResults(const sensor_msgs::LaserScan & msg, Pearl & method, const string methodType){
    auto start = std::chrono::system_clock::now();

    method.populateOutliers(msg);

    vector <Model> lines = method.findLines();

    sendLine(lines, method);

    auto end = std::chrono::system_clock::now();
    elapsed_seconds = end - start;

    end_time = std::chrono::system_clock::to_time_t(end);


    engrais_control::Results results;
    results.foundModels.clear();

    for(int i = 0; i < lines.size(); i++){
        engrais_control::Model model;
        if(lines[i].getSlope() != MAX_DBL && lines[i].getIntercept() != MAX_DBL){
            model.slope = lines[i].getSlope();
            model.intercept = lines[i].getIntercept();

            results.foundModels.push_back(model);
        }
    }
    
    results.runTime = elapsed_seconds.count();
    results.method = methodType;

    totalExecutionTime += elapsed_seconds.count();

    //resultsPubNode.publish(results);
}


//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    //sendExecutionResults(msg, pearl, Utility::getClassName(pearl));
    //sendExecutionResults(msg, rubyPure, Utility::getClassName(rubyPure));
    //sendExecutionResults(msg, rubyGen, Utility::getClassName(rubyGen));
    //sendExecutionResults(msg, rubyGenOP, Utility::getClassName(rubyGenOP));
    sendExecutionResults(msg, rubyGenOPPN, Utility::getClassName(rubyGenOPPN));
    //sendExecutionResults(msg, rubyGenOPPNInf, Utility::getClassName(rubyGenOPPNInf));

    //cout << pearl << endl << endl;
    //cout << rubyPure << endl << endl;
    //cout << rubyGen << endl << endl;
    //cout << rubyGenOP << endl << endl;
    //cout << rubyGenOPPN << endl << endl;
    //cout << rubyGenOPPNInf << endl << endl;
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    srand (time(NULL));
    ros::init(argc, argv, "engrais_findlines");
    ros::NodeHandle node;

    string sub_topic, pub_topic, node_name = ros::this_node::getName();

    Utility::printInColor("Initializing Robot Control Ros Node", CYAN);

    if(!node.getParam(node_name + "/subscribe_topic", sub_topic) || !node.getParam(node_name + "/publish_topic", pub_topic)){
        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic', 'publish_topic' and [optional: 'rviz map name']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/rviz_topic", mapName, "world");

    ros::Subscriber sub = node.subscribe(sub_topic, 10, OnRosMsg); // /engrais/laser_front/scan or /engrais/laser_back/scan
    pubLineNode = node.advertise<visualization_msgs::Marker>(pub_topic, 10);// /engrais/laser_front/lines or /engrais/laser_back/lines

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);

    sub.shutdown();
    pubLineNode.shutdown();
    resultsPubNode.shutdown();
    ros::shutdown();

    return 0;
}

//********************************************************************************************************