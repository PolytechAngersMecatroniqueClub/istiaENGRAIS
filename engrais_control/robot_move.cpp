//********************************************************************************************************

#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include "src/robot_findlines_include/1_Point/Point.h"
#include "src/robot_findlines_include/3_Utility/Utility.h"
#include "src/robot_findlines_include/4_Model/Model.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>


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
ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;

ros::Publisher pubFoundLines;

//--------------------------------------------------------------------------------------------------------
void preparePointsAndLines(visualization_msgs::Marker & line_list) { 
        line_list.header.frame_id = "sick_front_link";
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;

        line_list.id = 0;
        line_list.type = visualization_msgs::Marker::LINE_LIST;


        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_list.scale.x = 0.08;


        // Line list is green
        line_list.color.g = 1.0;
        line_list.color.a = 0.4;
}
//--------------------------------------------------------------------------------------------------------
vector<Model> getFoundLines(const visualization_msgs::Marker & msg, Model & ClosestLeftModel, Model & ClosestRightModel, int & numberOfPositiveModels){
    vector<Model> foundLines;

    visualization_msgs::Marker line_list;

    numberOfPositiveModels = 0;

    geometry_msgs::Point leftModelPoint1, leftModelPoint2;
    geometry_msgs::Point rightModelPoint1, rightModelPoint2;

    for(int i = 0; i < msg.points.size() - 1; i += 2){
        double dx = msg.points[i].x - msg.points[i + 1].x;
        double dy = msg.points[i].y - msg.points[i + 1].y;

        double a = MAX_DBL;
        
        if(dx != 0)
            a = dy / dx;

        double b = msg.points[i].y - a * msg.points[i].x;

        if(b >= 0){

            if(fabs(ClosestLeftModel.getIntercept()) > fabs(b)){
                ClosestLeftModel = Model(a, b);

                leftModelPoint1.x = msg.points[i].x;
                leftModelPoint1.y = msg.points[i].y;

                leftModelPoint2.x = msg.points[i+1].x;
                leftModelPoint2.y = msg.points[i+1].y; 
            }

            foundLines.insert(foundLines.begin(), 1, Model(a,b));
            numberOfPositiveModels++;
        }
        else{

            if(fabs(ClosestRightModel.getIntercept()) > fabs(b)){
                ClosestRightModel = Model(a, b);

                rightModelPoint1.x = msg.points[i].x;
                rightModelPoint1.y = msg.points[i].y;

                rightModelPoint2.x = msg.points[i+1].x;
                rightModelPoint2.y = msg.points[i+1].y; 
            }

            foundLines.insert(foundLines.end(), 1, Model(a,b));
        }
        
    }

    preparePointsAndLines(line_list);

    leftModelPoint1.z = leftModelPoint2.z = rightModelPoint1.z = rightModelPoint2.z = 0.002;

    line_list.points.push_back(leftModelPoint1);
    line_list.points.push_back(leftModelPoint2);
    line_list.points.push_back(rightModelPoint1);
    line_list.points.push_back(rightModelPoint2);

    pubFoundLines.publish(line_list);

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

    double controlSig = getControlSignal(ClosestLeftModel, ClosestRightModel);

    std_msgs::Float64 left_wheel_cmd;
    std_msgs::Float64 right_wheel_cmd;

    if(controlSig > 0){
        controlSig /= 10;
        controlSig = min(controlSig, 1.0);

        left_wheel_cmd.data = 1.0 * (1 - controlSig);
        right_wheel_cmd.data = 1.0;
    }
    else if (controlSig < 0){
        controlSig /= -10;
        controlSig = min(controlSig, 1.0);

        left_wheel_cmd.data = 1.0;
        right_wheel_cmd.data = 1.0 * (1 - controlSig);
    }
    else{
        left_wheel_cmd.data = 1.0;
        right_wheel_cmd.data = 1.0;
    }

    pubLeftControl.publish(left_wheel_cmd);
    pubRightControl.publish(right_wheel_cmd);
}
void stop(ros::Rate & loop_rate){
    while(ros::ok()){
        std_msgs::Float64 stop;
        stop.data = 0;
        pubLeftControl.publish(stop);
        pubRightControl.publish(stop);
        loop_rate.sleep();
    }
}
void left(ros::Rate & loop_rate){
    while(ros::ok()){
        std_msgs::Float64 left, right;
        left.data = 2;
        right.data = -2;
        pubLeftControl.publish(left);
        pubRightControl.publish(right);
        loop_rate.sleep();
    }
}
void right(ros::Rate & loop_rate){
    while(ros::ok()){
        std_msgs::Float64 left, right;
        left.data = -2;
        right.data = 2;
        pubLeftControl.publish(left);
        pubRightControl.publish(right);
        loop_rate.sleep();
    }
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_move_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/robot_engrais/all_lines_found", 10, OnLineMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLeftControl = node.advertise<std_msgs::Float64>("/engrais/leftWheel_controller/command", 10);
    pubRightControl = node.advertise<std_msgs::Float64>("/engrais/rightWheel_controller/command", 10);

    pubFoundLines = node.advertise<visualization_msgs::Marker>("/robot_engrais/selected_lines", 10);

    ros::Rate loop_rate(5);

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);

    //stop(loop_rate);
    //left(loop_rate);
    //right(loop_rate);

    sub.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubFoundLines.shutdown();
    ros::shutdown();

    Utility::printInColor("Code ended without errors", BLUE);
}

//********************************************************************************************************