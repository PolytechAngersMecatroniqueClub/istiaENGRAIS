//********************************************************************************************************
#ifndef RUBY_GENETIC_ONE_POINT_POS_NEG
#define RUBY_GENETIC_ONE_POINT_POS_NEG

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>


#include "../1_Point/Point.h"
#include "../2_WeightedPoint/WeightedPoint.h"
#include "../3_Utility/Utility.h"
#include "../4_Model/Model.h"
#include "../5_Pearl/Pearl.h"

class RubyGeneticOnePointPosNeg : public Pearl{
	public:
		std::vector<Point> initialField;

		std::vector<int> numberOfPositivePointsInModels;

		int numberOfPositivePointsInOutliers;

		double distanceToBeConsideredSamePoint = 0.1; 
		double numberOfModelsToSearch = 40;
		double factorToDeletePoints = 0.8;

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePointPosNeg();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ;

	public:

		void clearPointsInModels();
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int , const int , const int ) const;
		//------------------------------------------------------------------------------------------------
		void searchModels(const int );
		//------------------------------------------------------------------------------------------------
		void removeModel(const int );

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double redistributePoints();
		//------------------------------------------------------------------------------------------------
		double calculateEnergy() const;
		//------------------------------------------------------------------------------------------------
		double meanNumOfPoints() const;

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines();
		//------------------------------------------------------------------------------------------------
		void eraseBadModels();
		//------------------------------------------------------------------------------------------------
		void fuseEqualModels();

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNeg &r);
};

inline RubyGeneticOnePointPosNeg::RubyGeneticOnePointPosNeg(){}

inline std::vector<Point> RubyGeneticOnePointPosNeg::getInitialField() const { return initialField; }

#endif
//********************************************************************************************************