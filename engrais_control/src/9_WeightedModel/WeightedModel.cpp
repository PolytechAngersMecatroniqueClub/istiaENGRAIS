//********************************************************************************************************
#include "WeightedModel.h"


//--------------------------------------------------------------------------------------------------------
void WeightedModel::assignPoints(const Model & m){ //Assign points to weighted model 
    std::pair<Point, Point> points = m.getFirstAndLastPoint(); //Get closest and farthest point

    if(points.first.getX() >= 0){ //If closest point is positive, assign it to closest points
        this->positivePoints.first = points.first;
        this->positivePoints.second = points.second;
    }
    else{ //If they are negative, assign it to negative points
        this->negativePoints.first = points.first;
        this->negativePoints.second = points.second;
    }
}
//--------------------------------------------------------------------------------------------------------
bool WeightedModel::checkIfSameModel(const Model & m) const { //Check if the two models are approximately the same 
    double slopeRatio = this->a / m.getSlope(); //Calculate slope ratio
    double interceptRatio = this->b / m.getIntercept(); //Calculate intercept ratio

    double slopeDifference = fabs(this->a - m.getSlope()); //Calculate slope difference
    double interceptDifference = fabs(this->b - m.getIntercept()); //Calculate intercept difference

    bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold); //Checks if slope is approximately the same
    bool isInterceptTheSame = ((1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= 1 + Pearl::sameInterceptThreshold) || interceptDifference <= Pearl::sameInterceptThreshold); //Checks if interpect is approximately the same

    if(isSlopeTheSame && isInterceptTheSame){ //Return true if both are the same
        return true;
    }

    return false; //False otherwise
}
//--------------------------------------------------------------------------------------------------------
void WeightedModel::fuseModels(const Model & m){ //Calculate the final model using center of mass formula 
    this->a = (this->getSlope() * cont + m.getSlope()) / (double)(cont + 1); //Calculate the final slope
    this->b = (this->getIntercept() * cont + m.getIntercept()) / (double)(cont + 1); //Calculate the final intercept

    this->assignPoints(m); //Reassign points to the new models 

    this->cont++; //Increment counter
}
//--------------------------------------------------------------------------------------------------------
Model WeightedModel::toModel() const { //Converts to regular model 
    Model ret(this->a, this->b); //Declare model with calculated slope and intercept

    if(negativePoints.second.isAssigned()) //Assign negative-most point
        ret.pushPoint(negativePoints.second);
    else if(positivePoints.first.isAssigned())
        ret.pushPoint(positivePoints.first);
        

    if(positivePoints.second.isAssigned()) //Assign positive-most point
        ret.pushPoint(positivePoints.second);
    else if(negativePoints.first.isAssigned())
        ret.pushPoint(negativePoints.first);


    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const WeightedModel & wm){ //Print Object 
    out << "WeightedModel: [ a: " << wm.a << ", b: " << wm.b << ", cont: " << wm.cont << std::endl;

    out << wm.positivePoints.first << "    " << wm.positivePoints.second << "]" << std::endl;
    out << wm.negativePoints.first << "    " << wm.negativePoints.second << "]" << std::endl;

    return out;
}

//********************************************************************************************************