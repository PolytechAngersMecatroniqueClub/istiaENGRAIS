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

class RubyGeneticOnePoint : public Pearl{
	private:
		std::vector<Point> initialField;

		
	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.1; 
		static constexpr double numberOfModelsToSearch = 40;
		static constexpr double factorToDeletePoints = 0.8;

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePoint();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ;


	private:
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int);
		//------------------------------------------------------------------------------------------------
		void searchModels(const int );

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double calculateEnergy();
		//------------------------------------------------------------------------------------------------
		double meanNumOfPoints();

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines();
		//------------------------------------------------------------------------------------------------
		void eraseBadModels();

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePoint &r);
};


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePoint::RubyGeneticOnePoint(){}
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePoint::getInitialField() const { return this->initialField; }


#endif
//********************************************************************************************************