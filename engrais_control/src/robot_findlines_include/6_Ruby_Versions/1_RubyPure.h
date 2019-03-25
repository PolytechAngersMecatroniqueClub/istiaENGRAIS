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
		RubyPure(); //Checked
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & ); //Checked

	public:
		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines(); //Checked
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(const double ); //Checked
		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyPure &r); //Checked
};

inline RubyPure::RubyPure(){} //Checked

#endif
//********************************************************************************************************