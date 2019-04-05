//********************************************************************************************************
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <thread>
#include <mutex>

#include "src/include/1_Point/Point.h"
#include "src/include/3_Utility/Utility.h"
#include "src/include/4_Model/Model.h"

//#include "src/robot_move_include/1_Control/Control.h"

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

#define KP 1
#define DISTANCE_REFERENCE 1.5
#define TO_MILLISECOND 1000

using namespace std;

bool endProgram = false;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubSelectedLines;


class Control{
    public:
        double sameSlopeThreshold = 0.1;
        double sameInterceptThreshold = 0.3;


    public:
        std::vector<Model> backModels;
        std::vector<Model> frontModels;


    public:
        Control(){}

        void frontMessage(const visualization_msgs::Marker & msg){
            if(msg.type != 5)
                return;

            this->frontModels = this->getFoundLines(msg);
        }

        void backMessage(const visualization_msgs::Marker & msg){
            if(msg.type != 5)
                return;

            this->backModels = this->getFoundLines(msg);
        }

        pair<std_msgs::Float64, std_msgs::Float64> getControlSignal(){

            pair<std_msgs::Float64, std_msgs::Float64> cmd;

            pair<Model, Model> selectedModels;

            if(frontModels.size() >= backModels.size())
                selectedModels = selectModels(frontModels, backModels);
            else
                selectedModels = selectModels(backModels, frontModels);


            double controlSig = calculateControlSig(selectedModels.first, selectedModels.second);

            if(controlSig >= 0){
                controlSig /= 10;
                controlSig = min(controlSig, 1.0);

                cmd.first.data = 1.0 * (1 - controlSig);
                cmd.second.data = 1.0;
            }
            else if (controlSig < 0){
                controlSig /= -10;
                controlSig = min(controlSig, 1.0);

                cmd.first.data = 1.0;
                cmd.second.data = 1.0 * (1 - controlSig);
            }

            return cmd;
        }        

        pair<std_msgs::Float64, std_msgs::Float64> getControlSignal(ros::Publisher & publisher){

            pair<std_msgs::Float64, std_msgs::Float64> cmd;

            pair<Model, Model> selectedModels;

            if(frontModels.size() >= backModels.size())
                selectedModels = selectModels(frontModels, backModels);
            else
                selectedModels = selectModels(backModels, frontModels);


            visualization_msgs::Marker line_list;
            geometry_msgs::Point leftModelPoint1, leftModelPoint2;
            geometry_msgs::Point rightModelPoint1, rightModelPoint2;

            preparePointsAndLines(line_list);

            leftModelPoint1.z = leftModelPoint2.z = rightModelPoint1.z = rightModelPoint2.z = 0.002;

            leftModelPoint1.x = -20;
            leftModelPoint1.y = selectedModels.first.getSlope() * leftModelPoint1.x + selectedModels.first.getIntercept();

            leftModelPoint2.x = 20;
            leftModelPoint2.y = selectedModels.first.getSlope() * leftModelPoint2.x + selectedModels.first.getIntercept();


            rightModelPoint1.x = -20;
            rightModelPoint1.y = selectedModels.first.getSlope() * rightModelPoint1.x + selectedModels.first.getIntercept();

            rightModelPoint2.x = 20;
            rightModelPoint2.y = selectedModels.first.getSlope() * rightModelPoint2.x + selectedModels.first.getIntercept();

            line_list.points.push_back(leftModelPoint1);
            line_list.points.push_back(leftModelPoint2);
            line_list.points.push_back(rightModelPoint1);
            line_list.points.push_back(rightModelPoint2);

            pubSelectedLines.publish(line_list);

            double controlSig = calculateControlSig(selectedModels.first, selectedModels.second);

            if(controlSig >= 0){
                controlSig /= 10;
                controlSig = min(controlSig, 1.0);

                cmd.first.data = 1.0 * (1 - controlSig);
                cmd.second.data = 1.0;
            }
            else if (controlSig < 0){
                controlSig /= -10;
                controlSig = min(controlSig, 1.0);

                cmd.first.data = 1.0;
                cmd.second.data = 1.0 * (1 - controlSig);
            }

            return cmd;
        }


    public:
        vector<Model> getFoundLines(const visualization_msgs::Marker & msg){ 
            vector<Model> foundLines;

            for(int i = 0; i < msg.points.size() - 1; i += 2){
                double dx = msg.points[i].x - msg.points[i + 1].x;
                double dy = msg.points[i].y - msg.points[i + 1].y;

                double a = MAX_DBL;
                
                if(dx != 0)
                    a = dy / dx;

                double b = msg.points[i].y - a * msg.points[i].x;
                
                foundLines.push_back(Model(a,b));                
            }

            return foundLines;
        }

        pair<Model, Model> selectModels(const vector<Model> & largerModelVec, const vector<Model> & smallerModelVec){
            pair<Model, Model> selectedModels;

            for(int i = 0; i < largerModelVec.size(); i++){

                if(largerModelVec[i].getSlope() >= 0){

                    if(!selectedModels.first.isPopulated()){
                        selectedModels.first = largerModelVec[i];
                    }

                    else{

                        for(int j = 0; j < smallerModelVec.size(); j++){
                            if(fabs(largerModelVec[i].getSlope() - smallerModelVec[j].getSlope()) < this->sameSlopeThreshold){
                                selectedModels.first = largerModelVec[i];
                                break;
                            }
                        }
                    }
                }

                else{

                    if(!selectedModels.second.isPopulated()){
                        selectedModels.second = largerModelVec[i];
                    }

                    else{

                        for(int j = 0; j < smallerModelVec.size(); j++){
                            if(fabs(largerModelVec[i].getSlope() - smallerModelVec[j].getSlope()) < this->sameSlopeThreshold){
                                selectedModels.second = largerModelVec[i];
                                break;
                            }
                        }
                    }
                }
            }

            return selectedModels;
        }

        double calculateControlSig(const Model & selectedLeftModel, const Model & selectedRightModel){
            double aErr = 0, bErr = 0;

            if(selectedRightModel.isPopulated() && !selectedLeftModel.isPopulated()){
                aErr = selectedRightModel.getSlope();
                bErr = DISTANCE_REFERENCE - fabs(selectedRightModel.getIntercept());
            }

            else if(selectedLeftModel.isPopulated() && !selectedRightModel.isPopulated()){
                aErr = selectedLeftModel.getSlope();
                bErr = fabs(selectedLeftModel.getIntercept()) - DISTANCE_REFERENCE;
            }

            else{
                aErr = (selectedLeftModel.getSlope() + selectedRightModel.getSlope()) / 2.0;
                bErr = selectedLeftModel.getIntercept() + selectedRightModel.getIntercept();
            }

            return KP * (aErr + bErr);
        }

        void preparePointsAndLines(visualization_msgs::Marker & line_list){ 
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
};

Control control;


//--------------------------------------------------------------------------------------------------------
/*vector<Model> getFoundLines(const visualization_msgs::Marker & msg, Model & ClosestLeftModel, Model & ClosestRightModel, int & numberOfPositiveModels){ 
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

            foundLines.push_back(Model(a,b));
        }
        
    }

    preparePointsAndLines(line_list);

    leftModelPoint1.z = leftModelPoint2.z = rightModelPoint1.z = rightModelPoint2.z = 0.002;

    line_list.points.push_back(leftModelPoint1);
    line_list.points.push_back(leftModelPoint2);
    line_list.points.push_back(rightModelPoint1);
    line_list.points.push_back(rightModelPoint2);

    pubSelectedLines.publish(line_list);

    return foundLines;
}
//--------------------------------------------------------------------------------------------------------
double getControlSignal(const Model & ClosestLeftModel, const Model & ClosestRightModel){ 
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
void OnRosMsg(const visualization_msgs::Marker & msg){ 

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
}*/


//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){
    control.frontMessage(msg);
    //
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){
    control.backMessage(msg);
    //
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){
    while(!endProgram){
        usleep(100 * TO_MILLISECOND);

        control.getControlSignal(pubSelectedLines);
    }
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    ros::init(argc, argv, "robot_move_node");

    ros::NodeHandle node;

    ros::Subscriber subFront = node.subscribe("/engrais/laser_front/lines", 10, FrontLinesMsg);
    ros::Subscriber subBack = node.subscribe("/engrais/laser_back/lines", 10, BackLinesMsg);


    pubLeftControl = node.advertise<std_msgs::Float64>("/engrais/leftWheel_controller/command", 10);
    pubRightControl = node.advertise<std_msgs::Float64>("/engrais/rightWheel_controller/command", 10);


    pubSelectedLines = node.advertise<visualization_msgs::Marker>("/engrais/robot_move/selected_lines", 10);

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);

    subFront.shutdown();
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();

    Utility::printInColor("Code ended without errors", BLUE);

    return 0;
}

//********************************************************************************************************