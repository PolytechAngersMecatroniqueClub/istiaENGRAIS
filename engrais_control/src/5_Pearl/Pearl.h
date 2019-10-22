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
        std::vector<Model> models; //Store models
        std::vector<Point> outliers; //Outliers vector
        std::vector<Point> initialField; //Stores field's points 
        
    public:
        static const int maxNumberOfIterations = 10; //Maximum number of iterations
        static const int divideFactor = 2 * INITIAL_NUMBER_OF_POINTS; //To calculate the minumum amount of points in the field

        static constexpr double outlierPenalty = 3; //Additional energy for each outlier point
        static constexpr double distanceForOutlier = 0.2; //Everything farther than 50cm of every model will be considered outlier

        static constexpr double additionalEnergyLambda = 4; //Lambda to calculate additional energy
        static constexpr double additionalEnergyCsi = 3; //Csi to calculate additional energy

        static constexpr double sameSlopeThreshold = 0.1; //Threshold to determine if 2 models has the same slope
        static constexpr double sameInterceptThreshold = 0.2; //Threshold to determine if 2 models has the same intercept

        static constexpr double worstEnergySizeRatioAllowed = 0.7; //Quantity to know if model has to be deleted

    public:   
        //------------------------------------------------------------------------------------------------
        Pearl(); //Default Constructor
        //------------------------------------------------------------------------------------------------
        virtual void populateOutliers(const sensor_msgs::LaserScan & msg); //Receive LaserScan message and puts all the points into outlier vector
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Model> findLines(); //Find the best lines into the cloud of points
        //------------------------------------------------------------------------------------------------
        virtual std::vector<Point> getInitialField() const; //Get field points
            
            
    protected:
    	//------------------------------------------------------------------------------------------------
        virtual void removeModel(const int modelIndex); //Removes model from the vector
        //------------------------------------------------------------------------------------------------
        virtual void removePointsInModels(); //Removes all the points attached to all the models

		//################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual std::vector<Point> randomPointsInField(const int num); //Picks 'num' different points in the outlier vector
        //------------------------------------------------------------------------------------------------
        virtual void searchModels(const int nbOfModels); //Searches for 'nbOfModels' models that are possible

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual double redistributePoints(); //Reattach points to the closest model
        //------------------------------------------------------------------------------------------------
        virtual double removeTinyModels(const int points = INITIAL_NUMBER_OF_POINTS); //Remove models that have 'points' points or less
        //------------------------------------------------------------------------------------------------
        virtual double calculateAdditionalEnergy() const; //Calculates additional energy
        //------------------------------------------------------------------------------------------------
        virtual double expansionEnergy(); //Redistributes every point to the closest one, removes tiny models and calculates set's final energy

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual void reEstimation(); //Uses linearFit in every model to recalculete the best line for each set of points

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        virtual void eraseBadModels(const double threshRatio); //Erase all models that the ratio energy / number of points is lower than 'threshRatio'

        //################################################################################################

		//------------------------------------------------------------------------------------------------
        virtual void fuseEqualModels(); //Fuse models that are considered to be the same, if they have close slope and intercept 
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream &, const Pearl & p); //Print object

        //################################################################################################
};


//--------------------------------------------------------------------------------------------------------
inline Pearl::Pearl(){} //Default Constructor
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> Pearl::getInitialField() const { return this->initialField; } //Get field points


#endif
//********************************************************************************************************