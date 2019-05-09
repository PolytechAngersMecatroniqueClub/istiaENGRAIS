//********************************************************************************************************
#ifndef POINT_H
#define POINT_H

#include <iostream>
#include <cmath>

#ifndef MIN_DBL
#define MIN_DBL -1e+20
#endif

#ifndef MAX_DBL
#define MAX_DBL 1e+20
#endif

#ifndef MIN_INT
#define MIN_INT -10000000
#endif

#ifndef MAX_INT
#define MAX_INT 10000000
#endif

class Point{ 

    private: 
        double x = MIN_DBL;
        double y = MIN_DBL;

    public:
        //------------------------------------------------------------------------------------------------
        Point(); //Checked
        //------------------------------------------------------------------------------------------------
        Point(const double , const double ); //Checked



        //------------------------------------------------------------------------------------------------
        double getX() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getY() const; //Checked
        //------------------------------------------------------------------------------------------------
        bool isAssigned() const;
        //------------------------------------------------------------------------------------------------
        double distanceToOrigin() const;


        //------------------------------------------------------------------------------------------------
        bool operator == (const Point & ) const; //Checked
        //------------------------------------------------------------------------------------------------
        bool operator != (const Point & ) const; //Checked


        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const Point & ); //Checked

};


//--------------------------------------------------------------------------------------------------------
inline Point::Point() {} //Checked
//--------------------------------------------------------------------------------------------------------
inline Point::Point(const double xx, const double yy) : x(xx), y(yy) {} //Checked


//--------------------------------------------------------------------------------------------------------
inline double Point::getX() const { return this->x; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Point::getY() const { return this->y; } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Point::isAssigned() const { return (this->x != MIN_DBL && this->y != MIN_DBL); }
//--------------------------------------------------------------------------------------------------------
inline double Point::distanceToOrigin() const { return sqrt(pow(this->x, 2) + pow(this->y, 2)); }


//--------------------------------------------------------------------------------------------------------
inline bool Point::operator == (const Point & p) const { return (this->x == p.getX() && this->y == p.getY()); } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Point::operator != (const Point & p) const { return !(*this == p); } //Checked


//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const Point &p) { return (out << "Point: [ x: " << p.x << ", y: " << p.y << " ]"); }


#endif
//********************************************************************************************************