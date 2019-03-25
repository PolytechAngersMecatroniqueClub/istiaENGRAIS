//********************************************************************************************************
#ifndef PEARL_H
#define PEARL_H

#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>

#include "../1_Point/Point.h"
#include "../3_Utility/Utility.h"
#include "../4_Model/Model.h"

#define INITIAL_NUMBER_OF_POINTS 3

class Pearl{ 
    public: 
        std::vector<Model> models;
        std::vector<Point> outliers;   
        std::vector<Point> initialField;
        
    public:
        int maxNumberOfIterations = 10; 
        int divideFactor = 2*INITIAL_NUMBER_OF_POINTS;     

        double outlierPenalty = 3;
        double distanceForOutlier = 0.5;

        double additionalEnergyLambda = 4;
        double additionalEnergyCsi = 3;

        double sameSlopeThreshold = 0.2;
        double sameInterceptThreshold = 0.5;

        double worstEnergySizeRatioAllowed = 0.7;

    public:   
        //------------------------------------------------------------------------------------------------
        Pearl(); //Checked
        //------------------------------------------------------------------------------------------------
        virtual void populateOutliers(const sensor_msgs::LaserScan & ); //Checked
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Model> findLines(); //Checked
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Point> getInitialField() const; //Checked
            
    public:
    	//------------------------------------------------------------------------------------------------
        virtual void removeModel(const int ); //Checked
        //------------------------------------------------------------------------------------------------
        virtual void removePointsInModels(); //Checked

		//################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual std::vector<Point> randomPointsInField(const int ); //Checked
        //------------------------------------------------------------------------------------------------
        virtual void searchModels(const int ); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual double redistributePoints(); //Checked
        //------------------------------------------------------------------------------------------------
        virtual double removeTinyModels(const int = INITIAL_NUMBER_OF_POINTS); //Checked
        //------------------------------------------------------------------------------------------------
        virtual double calculateAdditionalEnergy() const; //Checked
        //------------------------------------------------------------------------------------------------
        virtual double expansionEnergy(); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual void reEstimation(); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual void eraseBadModels(const double ); //Checked

        //################################################################################################

		//------------------------------------------------------------------------------------------------
        virtual void fuseEqualModels(); //Checked
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream &, const Pearl &); //Checked

        //################################################################################################
};


//--------------------------------------------------------------------------------------------------------
inline Pearl::Pearl(){} //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> Pearl::getInitialField() const { return initialField; } //Checked


#endif
//********************************************************************************************************