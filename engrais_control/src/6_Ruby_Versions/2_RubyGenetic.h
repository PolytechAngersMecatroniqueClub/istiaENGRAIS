//********************************************************************************************************
#ifndef RUBY_GENETIC
#define RUBY_GENETIC

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>

#include <Point.h>
#include <Utility.h>
#include <Model.h>
#include <Pearl.h>


class RubyGenetic : public Pearl{ //Second version that implement a genetic approach to Pearl
	public:

		static constexpr double numberOfModelsToSearch = 40; //Number of individuals each generation
		static constexpr double factorToDeletePoints = 0.8; //Percentage of the average points to not delete a model

	public:
		//------------------------------------------------------------------------------------------------
		RubyGenetic(); //Default Constructor
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg); //Populate outliers vector with laser scan message
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines(); //Find the best lines into the cloud of points


	private:
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int num) const; //Picks 'num' different points in the outlier vector
		//------------------------------------------------------------------------------------------------
		void searchModels(const int nbOfModels); //Searches for 'nbOfModels' models that are possible

		//################################################################################################

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
		
		friend std::ostream & operator << (std::ostream &out, const RubyGenetic &r); //Print object
};


//--------------------------------------------------------------------------------------------------------
inline RubyGenetic::RubyGenetic(){} //Default Constructor

#endif
//********************************************************************************************************