//********************************************************************************************************
#ifndef WEIGHTED_POINT_H
#define WEIGHTED_POINT_H

#include <iostream>
#include <cmath>

#include "../1_Point/Point.h"

class WeightedPoint : public Point{ 

    private: 
        int pointWeight;

    public:
        //------------------------------------------------------------------------------------------------
        WeightedPoint(); //Checked
        //------------------------------------------------------------------------------------------------
        WeightedPoint(const double , const double, const int = 1); //Checked
        

        //------------------------------------------------------------------------------------------------
        int getWeight() const; //Checked
        //------------------------------------------------------------------------------------------------
        bool fusePoint(const WeightedPoint & , const double ); //Checked
        //------------------------------------------------------------------------------------------------
        bool operator == (const WeightedPoint & ) const; //Checked
        //------------------------------------------------------------------------------------------------
        bool operator != (const WeightedPoint & ) const; //Checked


        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const WeightedPoint & ); //Checked

};


//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint() : Point(), pointWeight(1) {} //Checked
//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint(const double xx, const double yy, const int w) : Point(xx, yy), pointWeight(w) {} //Checked


//--------------------------------------------------------------------------------------------------------
inline int WeightedPoint::getWeight() const { return this->pointWeight; } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool WeightedPoint::operator == (const WeightedPoint & wp) const { return (this->getX() == wp.getX() && this->getY() == wp.getY() && this->getWeight() == wp.getWeight()); } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool WeightedPoint::operator != (const WeightedPoint & wp) const { return !(*this == wp); } //Checked


//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const WeightedPoint &p){ return (out << "WeightedPoint: [ x: " << p.getX() << ", y: " << p.getY() << ", Weight: " << p.pointWeight << " ]"); }

#endif
//********************************************************************************************************
