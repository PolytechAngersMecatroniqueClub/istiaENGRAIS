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

class RobotControl{ //Robot Control Class 
    private:

        const int NUM_OF_LINES; //Number of lines present in selected models

        const double BODY_SIZE;

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        class SavedInfos{ //Information that will be necessary to navigation 
            public:

                double a; //Field's slope
                double dist; //Mean distance between two adjacent models
                int cont; //Counter

            public:
                //----------------------------------------------------------------------------------------
                SavedInfos(); //Constructor
                //----------------------------------------------------------------------------------------
                void initializeValues(const std::vector<Model> & models, int numLines);        
        }; 

        //################################################################################################
        
        SavedInfos si;

        StateMachine robotFSM;

        std::vector<Model> selected;

        std::vector<WeightedModel> models;

        std::vector<Point> frontPoints;
        std::vector<Point> backPoints;

    public:
        //################################################################################################

        //------------------------------------------------------------------------------------------------
        RobotControl(const int NLines, const int NTimesTurn, const double MVel, const double BSize); //Default constructor

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void clearModels(); //Clear all weighted models

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void frontPointsMessage(const visualization_msgs::Marker & msg); //Populate Field's Points with front LIDAR's point
        //------------------------------------------------------------------------------------------------
        void backPointsMessage(const visualization_msgs::Marker & msg); //Populate Field's Points with back LIDAR's point

        //------------------------------------------------------------------------------------------------
        void frontLinesMessage(const visualization_msgs::Marker & msg); //Receive message from front lines (found by front LIDAR)
        //------------------------------------------------------------------------------------------------
        void backLinesMessage(const visualization_msgs::Marker & msg); //Receive message from back lines (found by back LIDAR)   

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        std::pair<std::vector<Model>, std::vector<bool>> selectModels(); //Return best models and if it was estimated or found inside findLines' messages
        //------------------------------------------------------------------------------------------------
        std::pair<std_msgs::Float64, std_msgs::Float64> getWheelsCommand(); //Get wheel command from finite state machine

        //################################################################################################

    private:

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        std::vector<Model> initializeRobot() const;

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        std::pair<std::vector<Model>, std::vector<bool>> findBestModels(); //Search models received to find models that are more coherent and calculates models that are not found
        //------------------------------------------------------------------------------------------------
        void addPointsToModels(std::pair<std::vector<Model>, std::vector<bool>> & models) const; //Adds points to the models

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void addMsgModels(const std::vector<Model> & modelsInMsg, bool isFrontMsg); //Add models in msg found to weighted models

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        std::vector<Point> translateAxis(const std::vector<Point> & points, const double newOX, const double newOY) const; //Translate points from origin
        //------------------------------------------------------------------------------------------------
        std::vector<Point> rotateAxis(const std::vector<Point> & points, const double angleRot) const; //Rotate points from origin

        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY) const; //Translate points from origin
        //------------------------------------------------------------------------------------------------
        visualization_msgs::Marker rotateAxis(const visualization_msgs::Marker & msg, const double angleRot) const; //Rotate points from origin

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        std::vector<Model> getFoundLines(const visualization_msgs::Marker & msg) const; //Extract models from message

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const RobotControl & rc); //Print Object
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const SavedInfos & si); //Print Object
};

//--------------------------------------------------------------------------------------------------------
inline RobotControl::SavedInfos::SavedInfos() { a = dist = cont = 0; }
//--------------------------------------------------------------------------------------------------------
inline RobotControl::RobotControl(const int NLines, const int NTimesTurn, const double MVel, const double BSize)
 : robotFSM(NLines, NTimesTurn, MVel, BSize), NUM_OF_LINES(NLines), BODY_SIZE(BSize), selected(NLines) {} //Default constructor

//--------------------------------------------------------------------------------------------------------
inline void RobotControl::clearModels(){ models.clear(); } //Clear all weighted models

#endif
//********************************************************************************************************
