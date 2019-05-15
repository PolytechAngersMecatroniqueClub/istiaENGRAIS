//********************************************************************************************************
#include "WeightedModel.h"


//--------------------------------------------------------------------------------------------------------
void WeightedModel::assignPoints(const Model & m){
    std::pair<Point, Point> points = m.getFirstAndLastPoint();

    if(points.first.getX() >= 0){
        this->positivePoints.first = points.first;
        this->positivePoints.second = points.second;
    }
    else{
        this->negativePoints.first = points.first;
        this->negativePoints.second = points.second;
    }
}
//--------------------------------------------------------------------------------------------------------
bool WeightedModel::checkIfSameModel(const Model & m){
    double slopeRatio = this->a / m.getSlope();
    double interceptRatio = this->b / m.getIntercept();

    double slopeDifference = fabs(this->a - m.getSlope());
    double interceptDifference = fabs(this->b - m.getIntercept());

    bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold);
    bool isInterceptTheSame = ((1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= 1 + Pearl::sameInterceptThreshold) || interceptDifference <= Pearl::sameInterceptThreshold);

    if(isSlopeTheSame && isInterceptTheSame){
        return true;
    }

    return false;
}
//--------------------------------------------------------------------------------------------------------
void WeightedModel::fuseModels(const Model & m){
    this->a = (this->getSlope() * cont + m.getSlope()) / (double)(cont + 1);
    this->b = (this->getIntercept() * cont + m.getIntercept()) / (double)(cont + 1);

    this->assignPoints(m);

    this->cont++;
}
//--------------------------------------------------------------------------------------------------------
Model WeightedModel::toModel(){
    Model ret(a,b);

    if(negativePoints.second.isAssigned())
        ret.pushPoint(negativePoints.second);
    else if(positivePoints.first.isAssigned())
        ret.pushPoint(positivePoints.first);
        

    if(positivePoints.second.isAssigned())
        ret.pushPoint(positivePoints.second);
    else if(negativePoints.first.isAssigned())
        ret.pushPoint(negativePoints.first);


    return ret;
}


//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const WeightedModel & wm){
    out << "WeightedModel: [ a: " << wm.a << ", b: " << wm.b << ", cont: " << wm.cont << std::endl;

    out << wm.positivePoints.first << "    " << wm.positivePoints.second << "]" << std::endl;
    out << wm.negativePoints.first << "    " << wm.negativePoints.second << "]" << std::endl;

    return out;
}

//********************************************************************************************************