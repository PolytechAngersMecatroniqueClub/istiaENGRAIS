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

class RubyGeneticOnePointPosNeg : public Pearl{
	private:
		std::vector<Point> initialField;
		
		int numberOfPositivePointsInOutliers = 0;

	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.1; 
		static constexpr double numberOfModelsToSearch = 40;
		static constexpr double factorToDeletePoints = 0.8;

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePointPosNeg();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const ;


	private:

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


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePointPosNeg::RubyGeneticOnePointPosNeg(){}
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePointPosNeg::getInitialField() const {  return this->initialField; }


#endif
//********************************************************************************************************