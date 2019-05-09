//********************************************************************************************************
#ifndef PEARL_H
#define PEARL_H

#include <iostream>
#include <vector>
#include <sensor_msgs/LaserScan.h>

#include <Point.h>
#include <Model.h>
#include <Utility.h>

#ifndef INITIAL_NUMBER_OF_POINTS
#define INITIAL_NUMBER_OF_POINTS 3
#endif


class Pearl{ 
    protected: 
        std::vector<Model> models;
        std::vector<Point> outliers;   
        std::vector<Point> initialField;
        
    protected:
        int maxNumberOfIterations = 10; 
        int divideFactor = 2*INITIAL_NUMBER_OF_POINTS;     

        double outlierPenalty = 3;
        double distanceForOutlier = 0.5;

        double additionalEnergyLambda = 4;
        double additionalEnergyCsi = 3;

        double sameSlopeThreshold = 0.1;
        double sameInterceptThreshold = 0.3;

        double worstEnergySizeRatioAllowed = 0.7;

    public:   
        //------------------------------------------------------------------------------------------------
        Pearl(); //Checked
        //------------------------------------------------------------------------------------------------
        virtual void populateOutliers(const sensor_msgs::LaserScan &); //Checked
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Model> findLines(); //Checked
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Point> getInitialField() const; //Checked
            
            
    protected:
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
inline std::vector<Point> Pearl::getInitialField() const { return this->initialField; } //Checked


#endif
//********************************************************************************************************