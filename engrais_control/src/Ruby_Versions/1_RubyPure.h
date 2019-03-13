//********************************************************************************************************
#ifndef RUBY_PURE_H
#define RUBY_PURE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>

#include "../Point.h"
#include "../Utility.h"
#include "../Model.h"
#include "../Pearl.h"

class RubyPure : public Pearl{
	std::pair<Model,Model> savedModels;
	
	public:
		//------------------------------------------------------------------------------------------------
		RubyPure();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );

	public:
		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		std::vector<int> countParallelLines();
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(const double );
		//------------------------------------------------------------------------------------------------
		std::pair<Model, Model> findBestModels();

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyPure &r);
};

inline RubyPure::RubyPure(){}

#endif
//********************************************************************************************************