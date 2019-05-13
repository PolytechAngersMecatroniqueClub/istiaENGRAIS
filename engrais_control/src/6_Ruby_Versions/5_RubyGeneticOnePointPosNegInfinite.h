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

class RubyGeneticOnePointPosNegInfinite : public Pearl{
	private:
		std::vector<Point> field;
		
		int numberOfPositivePointsInField = 0;

	public:
		static constexpr double distanceToBeConsideredSamePoint = 0.1; 
		static constexpr double numberOfModelsToSearch = 40;
		static constexpr double factorToDeletePoints = 0.8;
		

	public:
		//------------------------------------------------------------------------------------------------
		RubyGeneticOnePointPosNegInfinite();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();
		//------------------------------------------------------------------------------------------------
		std::vector<Point> getInitialField() const;


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

		//################################################################################################*/
		
		friend std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNegInfinite &r);
};


//--------------------------------------------------------------------------------------------------------
inline RubyGeneticOnePointPosNegInfinite::RubyGeneticOnePointPosNegInfinite(){}
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> RubyGeneticOnePointPosNegInfinite::getInitialField() const {  return this->field; }

#endif
//********************************************************************************************************