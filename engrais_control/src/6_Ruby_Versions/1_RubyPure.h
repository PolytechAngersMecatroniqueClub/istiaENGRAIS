//********************************************************************************************************
#ifndef RUBY_PURE_H
#define RUBY_PURE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <sensor_msgs/LaserScan.h>

#include <Point.h>
#include <Utility.h>
#include <Model.h>
#include <Pearl.h>

class RubyPure : public Pearl{ //Class to implement Ruby pure. This algorithm is a modified version of Pearl, boosting models that have parallel lines and keeping models that were found in the previous execution
	public:
		//------------------------------------------------------------------------------------------------
		RubyPure(); //Default constructor
		//------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg); //Populate outliers vector with laser scan message


	private:
		//################################################################################################
		
		//------------------------------------------------------------------------------------------------
		void countParallelLines(); //Calculates parallel count for each model
		//------------------------------------------------------------------------------------------------
		void eraseBadModels(const double threshRatio); //Erases models that are considered bad, using Energy / (nPoints * parallel count)

		//################################################################################################
		
		friend std::ostream & operator << (std::ostream &out, const RubyPure &r); //Print ruby
};

//--------------------------------------------------------------------------------------------------------
inline RubyPure::RubyPure(){} //Default constructor


#endif
//********************************************************************************************************