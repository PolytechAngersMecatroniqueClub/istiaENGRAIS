//********************************************************************************************************
#include <cmath>
#include <chrono>
#include <time.h>  
#include <thread>
#include <stdio.h> 
#include <stdlib.h>   
#include <fstream>
#include <iostream>
#include <algorithm>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
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

Pearl* usedAlgo; //Used algorithm

ofstream arq; //Output file

string mapName, node_name, emergecy_topic, arq_name;

ros::NodeHandle* node;

ros::Publisher pubLineNode; //Found lines publisher

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
ros::Time lastMsg; //Store last time
bool comReady = false; //Flag to say the communication is ready
bool emergencyCalled = false; //Flag to signal exit

//--------------------------------------------------------------------------------------------------------
void OnEmergencyBrake(const std_msgs::Bool & msg){ //Emergency message
    lastMsg = ros::Time::now(); //Store last time
    
    comReady = true; //Sets flag

    if(msg.data == true){ //If message is true, exit
        Utility::printInColor(node_name + ": Emergency Shutdown Called", RED);
        emergencyCalled = true;
        ros::shutdown();
    }
}
//--------------------------------------------------------------------------------------------------------
void emergencyThread(){
    lastMsg = ros::Time::now();

    ros::Subscriber emergencySub = node->subscribe(emergecy_topic, 10, OnEmergencyBrake); //Subscribe to topics
    ros::Publisher emergencyPub = node->advertise<std_msgs::Bool>(emergecy_topic, 10); //Topic to publish if automatic

    std_msgs::Bool msg; //Sets message
    msg.data = false;
    
    while(ros::ok() && !comReady) //If mode is manual, wait for comunication to be set
        ros::Duration(0.01).sleep();

    while(ros::ok() && !emergencyCalled){ //Until emergency is called
        ros::Time now = ros::Time::now(); //get time
        
        ros::Duration delta_t = now - lastMsg;

        if(!emergencyCalled && delta_t.toSec() > 0.2){ //If last emergency message was received more than 200ms ago, shutdown
            Utility::printInColor(node_name + ": Emergency Timeout Shutdown", RED);
            ros::shutdown();
        }

        ros::Duration(0.05).sleep();
    }

    emergencySub.shutdown(); //Shutdown everything
    emergencyPub.shutdown();
}

//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){ //ROS message received 
    static std::chrono::time_point<std::chrono::system_clock> last = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::system_clock> exec_start, exec_end;

    std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now(); //Get time now

    std::chrono::duration<double> diff = start - last; //Get elapsed time from first message
    std::chrono::duration<double> elapsed_seconds = std::chrono::duration<double>::zero(); //Set 0

    while(ros::ok() && elapsed_seconds.count() <= diff.count() * 0.7){ //Continue calculating until hit 70% of message period
        usedAlgo->populateOutliers(msg); //Populate outliers

        if(arq_name != "none")
            exec_start = std::chrono::system_clock::now(); //Get time now

        vector <Model> lines = usedAlgo->findLines(); //Find models in cloud

        if(arq_name != "none"){
            exec_end = std::chrono::system_clock::now(); //Get time now

            std::chrono::duration<double> elapsed_exec = exec_end - exec_start;

            arq << usedAlgo->getInitialField().size() << ";" << elapsed_exec.count() * 1000.0 << endl;
        }

        sendLine(lines, *usedAlgo); //Send found models via ROS

        std::chrono::time_point<std::chrono::system_clock> end = std::chrono::system_clock::now();

        elapsed_seconds = end - start; //Update elapsed time
    }

    last = start;
}


//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main function 
    srand (time(NULL));
    ros::init(argc, argv, "engrais_findlines");
    
    node = new ros::NodeHandle();

    string algorithm, sub_topic, pub_topic;

    node_name = ros::this_node::getName();

    node->param<string>(node_name + "/subscribe_topic", sub_topic, "default/lidarMsg"); //Get parameters or set default values
    node->param<string>(node_name + "/publish_topic", pub_topic, "default/foundLines");
    
    node->param<string>(node_name + "/rviz_frame", mapName, "world");
    node->param<string>(node_name + "/algorithm", algorithm, "RubyGeneticOnePointPosNeg");

    node->param<string>(node_name + "/emergency_topic", emergecy_topic, "none");
    node->param<string>(node_name + "/arq_name", arq_name, "none");

    if(arq_name != "none"){ //Open file if necessary
        arq.open(arq_name, std::ofstream::out | std::ofstream::trunc);
        arq << "nPoints;execution_time" << endl;
    }

    if(algorithm == "Pearl") //Declare used findlines algorithm
        usedAlgo = new Pearl();

    else if(algorithm == "RubyPure")
        usedAlgo = new RubyPure();

    else if(algorithm == "RubyGenetic")
        usedAlgo = new RubyGenetic();

    else if(algorithm == "RubyGeneticOnePoint")
        usedAlgo = new RubyGeneticOnePoint();

    else if(algorithm == "RubyGeneticOnePointPosNeg")
        usedAlgo = new RubyGeneticOnePointPosNeg();

    else if(algorithm == "RubyGeneticOnePointPosNegInfinite")
        usedAlgo = new RubyGeneticOnePointPosNegInfinite();


    ros::Subscriber sub = node->subscribe(sub_topic, 10, OnRosMsg); // /engrais/laser_front/scan or /engrais/laser_back/scan

    pubLineNode = node->advertise<visualization_msgs::Marker>(pub_topic, 10);// /engrais/laser_front/lines or /engrais/laser_back/lines

    thread* emergency_t; //Run thread to exit
    if(emergecy_topic != "none")
        emergency_t = new thread(emergencyThread);


    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor(node_name + ": Shutting down...", CYAN);
    
    if(emergecy_topic != "none"){
        emergency_t->join();
        delete emergency_t;
    }

    sub.shutdown(); //Shutdown everything
    pubLineNode.shutdown();

    ros::shutdown();
       
    delete node;

    if(arq_name != "none"){ //Close file
        arq.close();
    }

    Utility::printInColor(node_name + ": Code ended without errors", BLUE);

    return 0;
}