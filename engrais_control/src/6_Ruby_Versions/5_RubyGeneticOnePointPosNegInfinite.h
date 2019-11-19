//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT_POS_NEG_INFINITE
#define RUBY_GENETIC_ONE_POINT_POS_NEG_INFINITE

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>


#include <Point.h>
#include <WeightedPoint.h>
#include <Utility.h>
#include <Model.h>
#include <Pearl.h>

class RubyGeneticOnePointPosNegInfinite : public Pearl{ //This class implements Ruby genetic one point positive/negative infinite, meaning that the points are no longer 'consumable'and can be attached to multiple model
	private:
		std::vector<Point> field; //Field of points
		
		int numberOfPositivePointsInField = 0; //Count positive points in outliers

	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.05; //Anything closer than this distance will be
		static constexpr double numberOfModelsToSearch = 40; //Number of individuals each generation
		static constexpr double factorToDeletePoints = 0.8; //Percentage of the average points to not delete a model
		

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePointPosNegInfinite(); //Default Constructor
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg); //Populate outliers vector with laser scan message
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines(); //Find the best lines into the cloud of points
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const; //Get initial field


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
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNegInfinite &r); //Print Object
};


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePointPosNegInfinite::RubyGeneticOnePointPosNegInfinite(){} //Default Constructor
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePointPosNegInfinite::getInitialField() const {  return this->field; }

#endif
//********************************************************************************************************