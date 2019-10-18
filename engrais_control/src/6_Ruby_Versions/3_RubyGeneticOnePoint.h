//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT
#define RUBY_GENETIC_ONE_POINT

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>

#include <Point.h>
#include <WeightedPoint.h>
#include <Utility.h>
#include <Model.h>
#include <Pearl.h>

class RubyGeneticOnePoint : public Pearl{ //Class to implement 3rd version of ruby. It keeps the Genetic approach, but now unites cluster of points into their center of mass, making the algorithm much faster and robust
	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.05; //Anything closer than this distance will be 
		static constexpr double numberOfModelsToSearch = 40; //Number of individuals each generation
		static constexpr double factorToDeletePoints = 0.8; //Percentage of the average points to not delete a model

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePoint(); //Default Constructor
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg); //Populate outliers vector with laser scan message
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines(); //Find the best lines into the cloud of points
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ; //Get initial field


	private:
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int num); //Picks 'num' different points in the outlier vector
		//------------------------------------------------------------------------------------------------
		void searchModels(const int nbOfModels); //Searches for 'nbOfModels' models that are possible

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double calculateEnergy(); //Calculate set's total energy
		//------------------------------------------------------------------------------------------------
		double meanNumOfPoints(); //Calculate mean number of points in the models

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines(); //Calculates parallel count for each model
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(); //Erases models that are considered bad

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePoint &r); //Print object
};


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePoint::RubyGeneticOnePoint(){} //Default Constructor
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePoint::getInitialField() const { return this->initialField; } //Get initial field


#endif
//********************************************************************************************************