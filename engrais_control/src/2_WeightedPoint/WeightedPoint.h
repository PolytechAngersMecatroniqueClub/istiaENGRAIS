//********************************************************************************************************
#ifndef WEIGHTED_POINT_H
#define WEIGHTED_POINT_H

#include <iostream>
#include <cmath>

#include <Point.h> //Include Point class

class WeightedPoint : public Point { //Class to combine point into a "Center Of Mass" Point 

    private: 
        int pointWeight = 1; //Counter to calculate the weighted average

    public:
        //------------------------------------------------------------------------------------------------
        WeightedPoint(); //Default constructor
        //------------------------------------------------------------------------------------------------
        WeightedPoint(const double , const double, const int = 1); //Constructor to assign values
        

        //------------------------------------------------------------------------------------------------
        int getWeight() const; //Get counter
        //------------------------------------------------------------------------------------------------
        bool fusePoint(const WeightedPoint & , const double ); //Calculates the center of mass between the 2 Points
        //------------------------------------------------------------------------------------------------
        bool operator == (const WeightedPoint & ) const; //Checks if equal
        //------------------------------------------------------------------------------------------------
        bool operator != (const WeightedPoint & ) const; //Checks if different


        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const WeightedPoint & ); //Print WeightedPoint

};


//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint() : Point() {} //Default constructor
//--------------------------------------------------------------------------------------------------------
inline WeightedPoint::WeightedPoint(const double xx, const double yy, const int w) : Point(xx, yy), pointWeight(w) {} //Constructor to assign values


//--------------------------------------------------------------------------------------------------------
inline int WeightedPoint::getWeight() const { return this->pointWeight; } //Get counter
//--------------------------------------------------------------------------------------------------------
inline bool WeightedPoint::operator == (const WeightedPoint & wp) const { return (this->getX() == wp.getX() && this->getY() == wp.getY() && this->getWeight() == wp.getWeight()); } //Checks if equal
//--------------------------------------------------------------------------------------------------------
inline bool WeightedPoint::operator != (const WeightedPoint & wp) const { return !(*this == wp); } //Checks if different


//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const WeightedPoint &p){ return (out << "WeightedPoint: [ x: " << p.getX() << ", y: " << p.getY() << ", Weight: " << p.pointWeight << " ]"); } //Print WeightedPoint

#endif
//********************************************************************************************************
