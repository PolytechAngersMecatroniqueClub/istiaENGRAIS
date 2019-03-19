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
    protected: 
        std::vector<Model> models;
        std::vector<Point> outliers;    

        
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
        void populateOutliers(const sensor_msgs::LaserScan & ); //Checked
        //------------------------------------------------------------------------------------------------
        std::pair<Model, Model> findLines(); //Checked
            
    protected:
    	//------------------------------------------------------------------------------------------------
        void removeModel(const int ); //Checked

		//################################################################################################

        //------------------------------------------------------------------------------------------------
        std::vector<Point> randomPointsInField(const int ); //Checked
        //------------------------------------------------------------------------------------------------
        void searchModels(const int ); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        double redistributePoints(); //Checked
        //------------------------------------------------------------------------------------------------
        double removeTinyModels(const int = INITIAL_NUMBER_OF_POINTS); //Checked 
        //------------------------------------------------------------------------------------------------
        double calculateAdditionalEnergy() const; //Checked
        //------------------------------------------------------------------------------------------------
        double expansionEnergy(); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void reEstimation(); //Checked

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        void eraseBadModels(const double ); //Checked

        //################################################################################################

		//------------------------------------------------------------------------------------------------
        void fuseEqualModels(); //Checked
        //------------------------------------------------------------------------------------------------
        std::pair<Model, Model> findBestModels() const; //Checked
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream &, const Pearl &); //Checked

        //################################################################################################
};


//--------------------------------------------------------------------------------------------------------
inline Pearl::Pearl(){} //Checked

#endif
//********************************************************************************************************