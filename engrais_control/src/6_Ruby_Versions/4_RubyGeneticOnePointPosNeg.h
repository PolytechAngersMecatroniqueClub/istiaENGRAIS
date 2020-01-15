//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT_POS_NEG
#define RUBY_GENETIC_ONE_POINT_POS_NEG

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>


#include <Point.h>
#include <WeightedPoint.h>
#include <Utility.h>
#include <Model.h>
#include <Pearl.h>

class RubyGeneticOnePointPosNeg : public Pearl{ //Fourth ruby version. This class keeps the changes in Genetic and One Point, but implements a difference between points that have a positive and negative Y coordinate. The reason is to minimize the probability to search for crossing models (models that cross both real lines)
	private:
		int numberOfPositivePointsInOutliers = 0; //Count positive points in outliers

	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.15; //Anything closer than this distance will be
		static constexpr double numberOfModelsToSearch = 40; //Number of individuals each generation
		static constexpr double factorToDeletePoints = 0.8; //Percentage of the average points to not delete a model

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePointPosNeg(); //Default Constructor
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg); //Populate outliers vector with laser scan message
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines(); //Find the best lines into the cloud of points
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ; //Get initial field


	private:
		//------------------------------------------------------------------------------------------------
		void removePointsInModels(); //Removes all the points attached to all the models 
		//------------------------------------------------------------------------------------------------
		void removeModel(const int modelIndex); //Removes model from the vector

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int minNum, const int maxNum, const int num) const; //Selects 'num' points in outliers between index 'minNum' and 'maxNum'
		//------------------------------------------------------------------------------------------------
		void searchModels(const int nbOfModels); //Searches for 'nbOfModels' models that are possible

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double redistributePoints(); //Reattach points to the closest model
		//------------------------------------------------------------------------------------------------
		double calculateEnergy() const; //Calculate set's total energy
		//------------------------------------------------------------------------------------------------
		double meanNumOfPoints() const; //Calculate mean number of points in the models

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines(); //Calculates parallel count for each model
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(); //Erases models that are considered bad
		
		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNeg &r); //Print Object
};


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePointPosNeg::RubyGeneticOnePointPosNeg(){} //Default Constructor
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePointPosNeg::getInitialField() const {  return this->initialField; } //Get initial field


#endif
//********************************************************************************************************