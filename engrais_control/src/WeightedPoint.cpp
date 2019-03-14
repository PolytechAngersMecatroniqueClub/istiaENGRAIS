//********************************************************************************************************
#include "WeightedPoint.h"


bool WeightedPoint::fusePoint(WeightedPoint p, double limitDist){
	double dist = sqrt( pow(this->getX() - p.getX(), 2) + pow(this->getY() - p.getY(), 2) );
	if(dist <= limitDist){
		double avrgX = (this->getX()*this->pointWeight + p.getX() * p.getWeight()) / (double)(this->pointWeight + p.getWeight());
		double avrgY = (this->getY()*this->pointWeight + p.getY() * p.getWeight()) / (double)(this->pointWeight + p.getWeight());

		(*this) = WeightedPoint(avrgX, avrgY, this->pointWeight + p.getWeight());
		return true;
	}
	
	return false;
}



//********************************************************************************************************