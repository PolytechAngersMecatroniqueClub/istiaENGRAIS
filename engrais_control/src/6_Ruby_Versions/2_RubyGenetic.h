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


class RubyGenetic : public Pearl{
	public:

		static constexpr double numberOfModelsToSearch = 40;
		static constexpr double factorToDeletePoints = 0.8;

	public:
		//------------------------------------------------------------------------------------------------
		RubyGenetic();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );
		//------------------------------------------------------------------------------------------------
		std::vector<Model> findLines();


	private:
		//################################################################################################

		//------------------------------------------------------------------------------------------------
		std::vector<Point> randomPointsInField(const int) const;
		//------------------------------------------------------------------------------------------------
		void searchModels(const int );

		//################################################################################################

		//------------------------------------------------------------------------------------------------
		double calculateEnergy() const;
		//------------------------------------------------------------------------------------------------
		double meanNumbOfPoints() const;

		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines();
		//------------------------------------------------------------------------------------------------
		void eraseBadModels();

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyGenetic &r);
};


//--------------------------------------------------------------------------------------------------------
inline RubyGenetic::RubyGenetic(){}

#endif
//********************************************************************************************************