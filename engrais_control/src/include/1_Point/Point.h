
//********************************************************************************************************
#ifndef POINT_H
#define POINT_H

#include <iostream>

#define MIN_DBL -1e+20
#define MAX_DBL 1e+20

#define MIN_INT -10000000
#define MAX_INT 10000000

class Point{ 

    private: 
        double x;
        double y;

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
        bool operator == (const Point & ) const; //Checked
        //------------------------------------------------------------------------------------------------
        bool operator != (const Point & ) const; //Checked


        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const Point & ); //Checked

};



//--------------------------------------------------------------------------------------------------------
inline Point::Point() : x(MIN_DBL), y(MIN_DBL) {} //Checked
//--------------------------------------------------------------------------------------------------------
inline Point::Point(const double xx, const double yy) : x(xx), y(yy) {} //Checked


//--------------------------------------------------------------------------------------------------------
inline double Point::getX() const { return this->x; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Point::getY() const { return this->y; } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Point::operator == (const Point & p) const { return ((this->x == p.getX() && this->y == p.getY())); } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Point::operator != (const Point & p) const { return !(*this == p); } //Checked


//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const Point &p){ //Checked
    out << "Point: [ x: " << p.x << ", y: " << p.y << " ]";  
    return out; 
}

#endif
//********************************************************************************************************