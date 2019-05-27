//********************************************************************************************************
#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <cmath>

#ifndef MIN_DBL //Minimum value for double
#define MIN_DBL -1e+20
#endif

#ifndef MAX_DBL //Maximum value for double
#define MAX_DBL 1e+20
#endif

#ifndef MIN_INT //Minimum value for integers
#define MIN_INT -10000000
#endif

#ifndef MAX_INT //Maximum value for integers
#define MAX_INT 10000000
#endif

class Point{ //Class to store (X,Y) coordinates

    private: 
        double x = MIN_DBL;
        double y = MIN_DBL;

    public:
        //------------------------------------------------------------------------------------------------
        Point(); //Default constructor 
        //------------------------------------------------------------------------------------------------
        Point(const double , const double ); //Constructor to assign values



        //------------------------------------------------------------------------------------------------
        double getX() const; //Get X coordinate
        //------------------------------------------------------------------------------------------------
        double getY() const; //Get Y coordinate
        //------------------------------------------------------------------------------------------------
        bool isAssigned() const; //Checks if X and Y are different than the default value
        //------------------------------------------------------------------------------------------------
        double distanceToOrigin() const; //Calculates the Euclidean distance from this point to (0,0)


        //------------------------------------------------------------------------------------------------
        bool operator == (const Point & ) const; //Operator to check if equal
        //------------------------------------------------------------------------------------------------
        bool operator != (const Point & ) const; //Operator to check if different


        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const Point & ); //Print Point

};


//--------------------------------------------------------------------------------------------------------
inline Point::Point() {} //Default constructor 
//--------------------------------------------------------------------------------------------------------
inline Point::Point(const double xx, const double yy) : x(xx), y(yy) {} //Constructor to assign values


//--------------------------------------------------------------------------------------------------------
inline double Point::getX() const { return this->x; } //Get X coordinate
//--------------------------------------------------------------------------------------------------------
inline double Point::getY() const { return this->y; } //Get Y coordinate
//--------------------------------------------------------------------------------------------------------
inline bool Point::isAssigned() const { return (this->x != MIN_DBL && this->y != MIN_DBL); } //Checks if X and Y are different than the default value
//--------------------------------------------------------------------------------------------------------
inline double Point::distanceToOrigin() const { return sqrt(pow(this->x, 2) + pow(this->y, 2)); } //Calculates the Euclidean distance from this point to (0,0)


//--------------------------------------------------------------------------------------------------------
inline bool Point::operator == (const Point & p) const { return (this->x == p.getX() && this->y == p.getY()); } //Operator to check if equal
//--------------------------------------------------------------------------------------------------------
inline bool Point::operator != (const Point & p) const { return !(*this == p); } //Operator to check if different


//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const Point &p) { return (out << "Point: [ x: " << p.x << ", y: " << p.y << " ]"); } //Print Point


#endif
//********************************************************************************************************
