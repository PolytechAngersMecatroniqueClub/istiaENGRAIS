//********************************************************************************************************
#ifndef RUBY_PURE_H
#define RUBY_PURE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>

#include "../1_Point/Point.h"
#include "../3_Utility/Utility.h"
#include "../4_Model/Model.h"
#include "../5_Pearl/Pearl.h"

class RubyPure : public Pearl{
	public:
		//------------------------------------------------------------------------------------------------
		RubyPure();
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & );

	public:
		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines();
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(const double );
		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyPure &r);
};

inline RubyPure::RubyPure(){}

#endif
//********************************************************************************************************