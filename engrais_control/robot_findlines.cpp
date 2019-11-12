//********************************************************************************************************
#include <cmath>
#include <chrono>
#include <time.h>  
#include <stdio.h> 
#include <stdlib.h>
#include <fstream>   
#include <iostream>
#include <algorithm>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
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

Pearl* usedAlgo;

Pearl pearl;
RubyPure rubyPure; //Ruby object
RubyGenetic rubyGenetic; //Ruby objectPearl pearl; //Ruby object
RubyGeneticOnePoint rubyGeneticOP; //Ruby objectPearl pearl; //Ruby object
RubyGeneticOnePointPosNeg rubyGeneticOPPN; //Ruby objectPearl pearl; //Ruby object
RubyGeneticOnePointPosNegInfinite rubyGeneticOPPNInf; //Ruby objectPearl pearl; //Ruby object

ofstream arq;

string mapName, node_name, arq_name; //rviz map name

ros::Publisher pubLineNode; //Lines found publisher


//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models, const Pearl & pearl){ //Send model's first and last point using the visualization marker 
    visualization_msgs::Marker line_list, points;
    geometry_msgs::Point p;

    points.header.frame_id = line_list.header.frame_id = mapName; //Assign map name
    points.header.stamp = line_list.header.stamp = ros::Time::now(); //Assign time
    points.ns = line_list.ns = "points_and_lines"; //Message type
    points.action = line_list.action = visualization_msgs::Marker::ADD; //Add points
    points.pose.orientation.w = line_list.pose.orientation.w = 1.0;

    line_list.id = 0; //Set id
    points.id = 1;

    points.type = visualization_msgs::Marker::POINTS; //Message points flag
    line_list.type = visualization_msgs::Marker::LINE_LIST; //Message line list flag

    points.scale.x = 0.05; //Points size
    points.scale.y = 0.05;

    points.color.r = 1.0; //Points color
    points.color.a = 1.0;

    line_list.scale.x = 0.03; //Line list scale

    line_list.color.b = 1.0; //Line list color
    line_list.color.a = 1.0;


    for(Point point : pearl.getInitialField()){ //Get initial field to compare with the sensor's data
		p.x = point.getX(); //Get X coordinate
		p.y = point.getY(); //Get Y coordinate
		p.z = 0.05; //Set a bit higher to help visualize

		points.points.push_back(p); //Add points
    }
    p.z = 0; //Reset hight

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

    pubLineNode.publish(points); //Publish field points
    pubLineNode.publish(line_list); //Publish line list
}


//--------------------------------------------------------------------------------------------------------
void exitApp(const std_msgs::Bool & msg){ //ROS message received 
    if(msg.data)
        ros::shutdown();
}
//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){ //ROS message received 
    static auto last = std::chrono::system_clock::now();

    auto start = std::chrono::system_clock::now(); //Get time now

    std::chrono::duration<double> diff = start - last; //Get elapsed time from first message
    std::chrono::duration<double> elapsed_seconds = std::chrono::duration<double>::zero(); //Set 0

    while(ros::ok() && elapsed_seconds.count() <= diff.count() * 0.7){ //Continue calculating until hit 80% of message period
        usedAlgo->populateOutliers(msg); //Populate outliers

        auto exec_start = std::chrono::system_clock::now(); //Get time now

        vector <Model> lines = usedAlgo->findLines(); //Find models in cloud

        auto exec_end = std::chrono::system_clock::now(); //Get time now

        std::chrono::duration<double> elapsed_exec = exec_end - exec_start;

        if(arq_name != "none"){
            arq << usedAlgo->getInitialField().size() << ";" << elapsed_exec.count() * 1000.0 << endl;
        }

        sendLine(lines, *usedAlgo); //Send found models via ROS

        auto end = std::chrono::system_clock::now();

        elapsed_seconds = end - start; //Update elapsed time
    }

    last = start;
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main function 
    srand (time(NULL)); //Set RNG seed
    ros::init(argc, argv, "engrais_findlines"); //Initialize ROS
    ros::NodeHandle node;

    string algorithm, sub_topic, pub_topic;

    node_name = ros::this_node::getName();

    Utility::printInColor(node_name + ": Initializing Robot Control Ros Node", CYAN);

    if(!node.getParam(node_name + "/subscribe_topic", sub_topic) || !node.getParam(node_name + "/publish_topic", pub_topic)){ //Get mandatory parameters
        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic', 'publish_topic' and [optional: 'rviz_frame']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/rviz_frame", mapName, "world"); //Get optional parameters
    node.param<string>(node_name + "/algorithm", algorithm, "Pearl"); //Get optional parameters

    node.param<string>(node_name + "/arq_name", arq_name, "none");

    if(arq_name != "none"){
        arq.open(arq_name, std::ofstream::out | std::ofstream::trunc);

        arq << "nPoints;execution_time" << endl;
    }

    if(algorithm == "Pearl")
        usedAlgo = &pearl;

    else if(algorithm == "RubyPure")
        usedAlgo = &rubyPure;

    else if(algorithm == "RubyGenetic")
        usedAlgo = &rubyGenetic;

    else if(algorithm == "RubyGeneticOnePoint")
        usedAlgo = &rubyGeneticOP;

    else if(algorithm == "RubyGeneticOnePointPosNeg")
        usedAlgo = &rubyGeneticOPPN;

    else if(algorithm == "RubyGeneticOnePointPosNegInfinite")
        usedAlgo = &rubyGeneticOPPNInf;

    ros::Subscriber sub = node.subscribe(sub_topic, 10, OnRosMsg); // /engrais/laser_front/scan or /engrais/laser_back/scan
    ros::Subscriber exitSub = node.subscribe("/exit_app_topic", 10, exitApp); // /engrais/laser_front/scan or /engrais/laser_back/scan

    pubLineNode = node.advertise<visualization_msgs::Marker>(pub_topic, 10);// /engrais/laser_front/lines or /engrais/laser_back/lines

    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    ros::spin(); //Spin to receive messages
    Utility::printInColor(node_name + ": Shutting down...", CYAN);

    sub.shutdown(); //Shutdown ROS
    pubLineNode.shutdown();
    ros::shutdown();

    if(arq_name != "none"){
        arq.close();
    }
    return 0;
}

//********************************************************************************************************