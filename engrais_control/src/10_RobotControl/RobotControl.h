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

        const double MAX_VEL;
        const double BODY_SIZE;
        const double DISTANCE_REFERENCE;

        StateMachine robotFSM;
        std::vector<WeightedModel> models;

    public:
        //------------------------------------------------------------------------------------------------
        RobotControl(double MVel, double BSize, double DReference);
        //------------------------------------------------------------------------------------------------
        void clearModels();
        //------------------------------------------------------------------------------------------------
        void frontMessage(const visualization_msgs::Marker & msg);
        //------------------------------------------------------------------------------------------------
        void backMessage(const visualization_msgs::Marker & msg);   
        //------------------------------------------------------------------------------------------------
        std::pair<Model, Model> selectModels();
        //------------------------------------------------------------------------------------------------
        std::pair<std_msgs::Float64, std_msgs::Float64> getWheelsCommand(const std::pair<Model, Model> & selectedModels);

    private:
        //------------------------------------------------------------------------------------------------
        void addMsgModels(const std::vector<Model> & modelsInMsg);
        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY);
        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker rotateAxis(const visualization_msgs::Marker & msg, const double angleRot);
        //------------------------------------------------------------------------------------------------
        std::vector<Model> getFoundLines(const visualization_msgs::Marker & msg);

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const RobotControl & rc);
};

//--------------------------------------------------------------------------------------------------------
inline RobotControl::RobotControl(double MVel, double BSize, double DReference) : MAX_VEL(MVel), BODY_SIZE(BSize), DISTANCE_REFERENCE(DReference), robotFSM(MVel, BSize, DReference) {}
//----------------------------------------------------------------------------- ---------------------------
inline void RobotControl::clearModels(){ models.clear(); }

#endif
//********************************************************************************************************
