//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT
#define RUBY_GENETIC_ONE_POINT

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>


#include "../1_Point/Point.h"
#include "../2_WeightedPoint/WeightedPoint.h"
#include "../3_Utility/Utility.h"
#include "../4_Model/Model.h"
#include "../5_Pearl/Pearl.h"

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
		std::pair<Model, Model> findLines();
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
		double meanNumbOfPoints();

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		std::vector<int> countParallelLines();
		//------------------------------------------------------------------------------------------------
		std::pair<Model, Model> eraseBadModels();

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePoint &r);
};

inline RubyGeneticOnePoint::RubyGeneticOnePoint(){}

inline std::vector<Point> RubyGeneticOnePoint::getInitialField() const { return initialField; }

#endif
//********************************************************************************************************