//********************************************************************************************************
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include <cmath>
#include <chrono>
#include <time.h>  
#include <algorithm>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"

#include <engrais_control/Model.h>
#include <engrais_control/Results.h>

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


ros::Publisher resultsPubNode;


std::chrono::duration<double> elapsed_seconds;
std::time_t end_time;


double totalExecutionTime = 0;
int timesExecuted = 0;


//--------------------------------------------------------------------------------------------------------
void printResults(const engrais_control::Results & results){ 

    cout << "Method: "<< results.method << ", Run Time: " << results.runTime << endl;

    cout << "Models: Vector {" << endl;
    for(int i = 0; i < results.foundModels.size(); i++){
        cout << "\t[" << i << "]: Slope: " << results.foundModels[i].slope << ", Intercept: " << results.foundModels[i].intercept << endl;
    }
    cout << "}" << endl << endl;
}
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
void sendLine(const vector<Model> & models, const Pearl & pearl) { 
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

    //printResults(results);

    resultsPubNode.publish(results);
}
//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    /*sendExecutionResults(msg, pearl, Utility::getClassName(pearl));
    sendExecutionResults(msg, rubyPure, Utility::getClassName(rubyPure));
    sendExecutionResults(msg, rubyGen, Utility::getClassName(rubyGen));
    sendExecutionResults(msg, rubyGenOP, Utility::getClassName(rubyGenOP));*/
    sendExecutionResults(msg, rubyGenOPPN, Utility::getClassName(rubyGenOPPN));
    //sendExecutionResults(msg, rubyGenOPPNInf, Utility::getClassName(rubyGenOPPNInf));

    /*cout << pearl << endl;
    cout << rubyPure << endl;
    cout << rubyGen << endl;
    cout << rubyGenOP << endl;
    cout << rubyGenOPPN << endl;
    cout << rubyGenOPPNInf << endl;

    exit(1);*/

    timesExecuted++;

    std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    
    Utility::printInColor("Initializing Robot Control Ros Node", CYAN);

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/engrais/laser_front/scan", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<visualization_msgs::Marker>("/robot_engrais/all_lines_found", 10);

    resultsPubNode = node.advertise<engrais_control::Results>("/testing/robot_engrais/results", 10);

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);


    sub.shutdown();
    pubLineNode.shutdown();


    resultsPubNode.shutdown();
    ros::shutdown();


    Utility::printInColor("Code ended without errors", CYAN);

    Utility::printInColor("Total Calculations Time: " + to_string(totalExecutionTime) + "s, code ran " + to_string(timesExecuted) + " times.\nMean Calculation time: " + to_string(totalExecutionTime/(double)timesExecuted) + "s", BLUE);
    

    /*ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    resultsPubNode = node.advertise<engrais_control::Results>("/testing/robot_engrais/results", 10);

    sleep(1);

    vector<Model> lines;

    lines.push_back(Model(1,3));
    lines.push_back(Model(5,2));
    lines.push_back(Model(1,-2));
    lines.push_back(Model(-1,-6));
    lines.push_back(Model(0.03,-1.002));

    while(ros::ok()){

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
        
        results.runTime = 1.2;
        results.method = "Pearl";

        printResults(results);

        resultsPubNode.publish(results);

        results.runTime = 1.3;
        results.method = "Ruby";

        printResults(results);

        resultsPubNode.publish(results);
        
        sleep(1);
    }*/

    return 0;
}

//********************************************************************************************************