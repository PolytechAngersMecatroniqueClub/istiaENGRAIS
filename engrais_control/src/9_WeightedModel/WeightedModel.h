//********************************************************************************************************
#ifndef WEIGHTED_MODEL_H
#define WEIGHTED_MODEL_H

#include <iostream>
#include <vector>

#include <Point.h>
#include <Model.h>

#include <Pearl.h>
#include <1_RubyPure.h>
#include <2_RubyGenetic.h>
#include <3_RubyGeneticOnePoint.h>
#include <4_RubyGeneticOnePointPosNeg.h>
#include <5_RubyGeneticOnePointPosNegInfinite.h>


class WeightedModel{ //Class to combine models into a "Center Of Mass" model 
    private:
        std::vector<int> cont; //Counter to calculate the final model

        double a = MAX_DBL;  //Final slope
        double b = MAX_DBL; //Final intercept

        std::pair<Point, Point> positivePoints; //Initial and final point to 
        std::pair<Point, Point> negativePoints; //Initial and final point to 

    public:
        //################################################################################################

        //------------------------------------------------------------------------------------------------
        WeightedModel(); //Default Constructor
        //------------------------------------------------------------------------------------------------
        WeightedModel(const Model & m, bool isFrontMsg = true); //Constructor using a model
        //------------------------------------------------------------------------------------------------
        WeightedModel(const double aa, const double bb); //Constructor to assign slope and intercept

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        double getSlope() const; //Get slope
        //------------------------------------------------------------------------------------------------
        double getIntercept() const; //Get intercept
        //------------------------------------------------------------------------------------------------
        int getTotalCounter() const { return (cont[0] + cont[1]);}

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void assignPoints(const Model & m, bool isFrontMsg = true); //Assign points to weighted model
        //------------------------------------------------------------------------------------------------
        bool checkIfSameModel(const Model & m) const; //Check if the two models are approximately the same
        //------------------------------------------------------------------------------------------------
        void fuseModels(const Model & m, bool isFrontMsg = true); //Fuse two models
        //------------------------------------------------------------------------------------------------
        Model toModel() const; //Converts to regular model

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const WeightedModel & wm); //Print object
};

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel() : cont(2,0) {} //Default Constructor
//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel(const Model & m, bool isFrontMsg) : a(m.getSlope()), b(m.getIntercept()), cont(2,0) { this->assignPoints(m, isFrontMsg); this->cont[isFrontMsg] = 1; } //Constructor using a model
//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel(const double aa, const double bb) : a(aa), b(bb), cont(2,0) {} //Constructor to assign slope and intercept

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
inline double WeightedModel::getSlope() const { return a; } //Get slope
//--------------------------------------------------------------------------------------------------------
inline double WeightedModel::getIntercept() const { return b; } //Get intercept

//########################################################################################################

#endif
//********************************************************************************************************