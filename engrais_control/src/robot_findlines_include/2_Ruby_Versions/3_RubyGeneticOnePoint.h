//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT
#define RUBY_GENETIC_ONE_POINT

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>


#include "../../include/1_Point/Point.h"
#include "../../include/2_WeightedPoint/WeightedPoint.h"
#include "../../include/3_Utility/Utility.h"
#include "../../include/4_Model/Model.h"
#include "../1_Pearl/Pearl.h"

class RubyGeneticOnePoint : public Pearl{
	public:
		std::vector<Point> initialField;

		double distanceToBeConsideredSamePoint = 0.1; 
		double numberOfModelsToSearch = 40;
		double factorToDeletePoints = 0.8;

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePoint();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ;


	public:
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
inline std::vector<Point> RubyGeneticOnePoint::getInitialField() const { return initialField; }


#endif
//********************************************************************************************************