//********************************************************************************************************
#include "WeightedPoint.h"

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
bool WeightedPoint::fusePoint(const WeightedPoint & wp, const double limitDist){ //Calculates the center of mass between the 2 Points

	double dist = sqrt( pow(this->getX() - wp.getX(), 2) + pow(this->getY() - wp.getY(), 2) ); //Calculates the Euclidean between the 2 points 

	if(dist <= limitDist){ //If the distance is greater than the limit to calculate the average

		double avrgX = (this->getX()*this->pointWeight + wp.getX() * wp.getWeight()) / (double)(this->pointWeight + wp.getWeight()); //Calculates the weighted average for X coordinate
		double avrgY = (this->getY()*this->pointWeight + wp.getY() * wp.getWeight()) / (double)(this->pointWeight + wp.getWeight()); //Calculates the weighted average for Y coordinate

		(*this) = WeightedPoint(avrgX, avrgY, this->pointWeight + wp.getWeight()); //Changes the point to the calculated average

		return true; //return that the function succeeded to fuse the 2 points
	}
	
	return false; //Else, return that the function failed to fuse the 2 points
}

//########################################################################################################

//********************************************************************************************************