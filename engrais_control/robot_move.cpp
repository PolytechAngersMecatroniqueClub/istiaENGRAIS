#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>


#include "src/Point.h"
#include "src/Utility.h"
#include "src/Model.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32MultiArray.h>


#define MIN_DBL -1e+20
#define MAX_DBL 1e+20

#define MIN_INT -10000000
#define MAX_INT 10000000

#define KP 1
#define DISTANCE_REFERENCE 2.0

#define LEFT 0
#define RIGHT 1

using namespace std;

ros::Subscriber sub;
ros::Publisher pubLineNode;


//--------------------------------------------------------------------------------------------------------
vector<Model> getFoundLines(const visualization_msgs::Marker & msg, Model & ClosestLeftModel, Model & ClosestRightModel, int & numberOfPositiveModels){
    vector<Model> foundLines;

    numberOfPositiveModels = 0;

    for(int i = 8; i < msg.points.size() - 1; i += 2){
        double dx = msg.points[i].x - msg.points[i + 1].x;
        double dy = msg.points[i].y - msg.points[i + 1].y;

        double a = MAX_DBL;
        
        if(dx != 0)
            a = dy / dx;

        double b = msg.points[i].y - a * msg.points[i].x;

        if(b >= 0){
            if(fabs(ClosestLeftModel.getIntercept()) > fabs(b))
                ClosestLeftModel = Model(a, b);

            foundLines.insert(foundLines.begin(), 1, Model(a,b));
            numberOfPositiveModels++;
        }
        else{
            if(fabs(ClosestRightModel.getIntercept()) > fabs(b))
                ClosestRightModel = Model(a, b);

            foundLines.insert(foundLines.end(), 1, Model(a,b));
        }
        
    }
    return foundLines;
}
//--------------------------------------------------------------------------------------------------------
double getControlSignal(Model & ClosestLeftModel, Model & ClosestRightModel){
    double aErr = 0, bErr = 0;

    if(ClosestRightModel.isPopulated() && !ClosestLeftModel.isPopulated()){
        aErr = ClosestRightModel.getSlope();
        bErr = DISTANCE_REFERENCE - fabs(ClosestRightModel.getIntercept());
    }
    else if(ClosestLeftModel.isPopulated() && !ClosestRightModel.isPopulated()){
        aErr = ClosestLeftModel.getSlope();
        bErr = fabs(ClosestLeftModel.getIntercept()) - DISTANCE_REFERENCE;
    }
    else {
        aErr = (ClosestLeftModel.getSlope() + ClosestRightModel.getSlope()) / 2.0;
        bErr = ClosestLeftModel.getIntercept() + ClosestRightModel.getIntercept();
    }

    return KP * (aErr + bErr);
}


//--------------------------------------------------------------------------------------------------------
void OnLineMsg(const visualization_msgs::Marker & msg){
    if(msg.type != 5)
        return;

    int numberOfPositiveModels;
    Model ClosestLeftModel;
    Model ClosestRightModel;

    vector<Model> foundLines = getFoundLines(msg, ClosestLeftModel, ClosestRightModel, numberOfPositiveModels);

    //cout << numberOfPositiveModels << endl;

    //Utility::printVector(foundLines);

    //cout << endl << ClosestLeftModel << endl << endl << ClosestRightModel << endl << endl;

    double controlSig = getControlSignal(ClosestLeftModel, ClosestRightModel);

    //cout << controlSig << endl << endl<< endl << endl<< endl << endl;

    /*std_msgs::Float32MultiArray wheel_cmd;

    pubLineNode.publish(wheel_cmd);*/
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_move_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/robot_engrais/lines", 10, OnLineMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<std_msgs::Float32MultiArray>("/robot_engrais/wheels/vel_cmd", 10);

    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shitting down...");

    sub.shutdown();
    pubLineNode.shutdown();
    ros::shutdown();

    ROS_INFO("Code ended without errors");
}


