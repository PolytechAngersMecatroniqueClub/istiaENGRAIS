//********************************************************************************************************
#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <iostream>
#include <vector>

#include <Point.h>
#include <Model.h>
#include <Utility.h>
#include <WeightedModel.h>

#include <Pearl.h>
#include <1_RubyPure.h>
#include <2_RubyGenetic.h>
#include <3_RubyGeneticOnePoint.h>
#include <4_RubyGeneticOnePointPosNeg.h>
#include <5_RubyGeneticOnePointPosNegInfinite.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

#include <FuzzyController.h>
#include <StateMachine.h>


class RobotControl{ 
    private:
        StateMachine robotFSM;
        std::vector<WeightedModel> models;

    public:
        //------------------------------------------------------------------------------------------------
        RobotControl(); //Default constructor
        //------------------------------------------------------------------------------------------------
        void clearModels(); //Clear all weighted models
        //------------------------------------------------------------------------------------------------
        void frontMessage(const visualization_msgs::Marker & msg); //Receive message from front lines (found by front LIDAR)
        //------------------------------------------------------------------------------------------------
        void backMessage(const visualization_msgs::Marker & msg); //Receive message from back lines (found by back LIDAR)   
        //------------------------------------------------------------------------------------------------
        std::pair<Model, Model> selectModels() const; //Select left and right models
        //------------------------------------------------------------------------------------------------
        std::pair<std_msgs::Float64, std_msgs::Float64> getWheelsCommand(const std::pair<Model, Model> & selectedModels); //Get wheel command from finite state machine

    private:
        //------------------------------------------------------------------------------------------------
        void addMsgModels(const std::vector<Model> & modelsInMsg); //Add models found to weighted models
        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY) const; //Translate points from origin
        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker rotateAxis(const visualization_msgs::Marker & msg, const double angleRot) const; //Rotate points from origin
        //------------------------------------------------------------------------------------------------
        std::vector<Model> getFoundLines(const visualization_msgs::Marker & msg) const; //Extract models from message

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const RobotControl & rc); //Print Object
};

//--------------------------------------------------------------------------------------------------------
inline RobotControl::RobotControl(){} //Default constructor
//--------------------------------------------------------------------------------------------------------
inline void RobotControl::clearModels(){ models.clear(); } //Clear all weighted models

#endif
//********************************************************************************************************
