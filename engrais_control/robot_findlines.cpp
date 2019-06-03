//********************************************************************************************************
#include <cmath>
#include <chrono>
#include <time.h>  
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <Point.h>
#include <WeightedPoint.h>
#include <Model.h>
#include <Utility.h>

#include <Pearl.h>
#include <1_RubyPure.h>
#include <2_RubyGenetic.h>
#include <3_RubyGeneticOnePoint.h>
#include <4_RubyGeneticOnePointPosNeg.h>
#include <5_RubyGeneticOnePointPosNegInfinite.h>

using namespace std;
	
RubyGeneticOnePointPosNeg rubyGenOPPN;

string mapName;

ros::Publisher pubLineNode;
ros::Publisher resultsPubNode;


//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models, const Pearl & pearl){ 
    visualization_msgs::Marker line_list, points;
    geometry_msgs::Point p;

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


    for(Point point : pearl.getInitialField()){
		p.x = point.getX();
		p.y = point.getY();
		p.z = 0.05;

		points.points.push_back(p);
    }
    p.z = 0;

    for(int i = 0; i < models.size(); i++){
        if(models[i].isPopulated() && models[i].getPointsSize() >= 2){
            pair<Point, Point> points = models[i].getFirstAndLastPoint();

    	    p.x = points.first.getX();
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

            //cout << "Line [" << i << "] x_1: " << p.x << ", y_1: " << p.y;

    	    line_list.points.push_back(p);

    	    p.x = points.second.getX();
    	    p.y = models[i].getSlope()*p.x + models[i].getIntercept();

            //cout << ", x_2: " << p.x << ", y_2: " << p.y << endl;
    	    line_list.points.push_back(p);
    	}

        //cout << endl << endl;
    }

    pubLineNode.publish(points);
    pubLineNode.publish(line_list);
}


//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    static int msgCont = 0;
    const static auto firstMsgTime = std::chrono::system_clock::now();

    auto start = std::chrono::system_clock::now();

    std::chrono::duration<double> total_time = start - firstMsgTime;
    std::chrono::duration<double> elapsed_seconds = std::chrono::duration<double>::zero();

    const double msgPeriod = msgCont == 0 ? 0 : total_time.count() / msgCont;

    while(ros::ok() && elapsed_seconds.count() <= msgPeriod * 0.8){

        rubyGenOPPN.populateOutliers(msg);

        vector <Model> lines = rubyGenOPPN.findLines();
        //cout << rubyGenOPPN << endl << endl;

        sendLine(lines, rubyGenOPPN);

        auto end = std::chrono::system_clock::now();

        elapsed_seconds = end - start;
    }
    
    msgCont++;
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    srand (time(NULL));
    ros::init(argc, argv, "engrais_findlines");
    ros::NodeHandle node;

    string sub_topic, pub_topic, node_name = ros::this_node::getName();

    Utility::printInColor("Initializing Robot Control Ros Node", CYAN);

    if(!node.getParam(node_name + "/subscribe_topic", sub_topic) || !node.getParam(node_name + "/publish_topic", pub_topic)){
        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic', 'publish_topic' and [optional: 'frame_name']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/frame_name", mapName, "world");

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