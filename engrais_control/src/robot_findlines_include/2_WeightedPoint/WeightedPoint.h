//********************************************************************************************************
#ifndef WEIGHTED_POINT_H
#define WEIGHTED_POINT_H

#include <iostream>
#include <cmath>

#include "../1_Point/Point.h"

class WeightedPoint : public Point{ 

    private: 
        int pointWeight = 1;

    public:
        //------------------------------------------------------------------------------------------------
        WeightedPoint(); //Checked
        //------------------------------------------------------------------------------------------------
        WeightedPoint(const double , const double ); //Checked
        //------------------------------------------------------------------------------------------------
        WeightedPoint(const double , const double, const int ); //Checked
        

        //------------------------------------------------------------------------------------------------
        int getWeight(); //Checked
        //------------------------------------------------------------------------------------------------
        bool fusePoint(WeightedPoint p, double limitDist); //Checked
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const WeightedPoint & ); //Checked

};


//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint(){} //Checked
//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint(const double xx, const double yy) : Point(xx, yy) {} //Checked
//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint(const double xx, const double yy, const int w) : Point(xx, yy), pointWeight(w) {} //Checked


inline int WeightedPoint::getWeight() { return this->pointWeight; } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const WeightedPoint &p){ //Checked
    out << "WeightedPoint: [ x: " << p.getX() << ", y: " << p.getY() << ", Weight: " << p.pointWeight << " ]";  
    return out; 
}

#endif
//********************************************************************************************************
