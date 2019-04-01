//********************************************************************************************************
#include "WeightedPoint.h"


bool WeightedPoint::fusePoint(const WeightedPoint & p, const double limitDist){ //Checked

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