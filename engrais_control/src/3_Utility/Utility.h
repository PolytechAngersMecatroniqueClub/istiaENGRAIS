//********************************************************************************************************
#ifndef UTILITY_H
#define UTILITY_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>

#include <Point.h>

#ifndef BLUE
#define BLUE 34 //Blue color to console
#endif

#ifndef RED
#define RED 31 //Red color to console
#endif

#ifndef CYAN
#define CYAN 36 //Cyan color to console
#endif

class Utility{
    public:
        //------------------------------------------------------------------------------------------------
        static int randomInt(const int min, const int max); //Generates a random integer between "min" and "max", boundaries included
        //------------------------------------------------------------------------------------------------
        static double distPoints(const Point & , const Point & ); //Calculates the Euclidean distance between 2 points
        //------------------------------------------------------------------------------------------------
        static double calcSumDist(const std::vector<Point> & , const double , const double ); //Calculates the sum of all points in the vector and a line 
        //------------------------------------------------------------------------------------------------
        static void printInColor(const std::string , const int ); //Print colored text to console
        //------------------------------------------------------------------------------------------------
        static std::vector<int> randomDiffVector(const int min, const int max, const int size); //Returns a vector, sized "size", of different random numbers ranging from "min" from "max", boundaries included
        //------------------------------------------------------------------------------------------------
        template < class T> static std::string getClassName(T classElement){ //Template function to get any class name 
            std::string type = typeid(classElement).name(); //Get Class' name  
            while('0' <= type[0] && type[0] <= '9') //Erase Numbers that appear in front
                type.erase(0, 1);
                
            return type;
        }
        //------------------------------------------------------------------------------------------------
        template < class T> static void printVector(const std::vector<T> & vec){ //Template function to print a vector of any class 
        	std::cout << std::endl << "Vector {" << std::endl; //Print Header
		    for(int i = 0; i < vec.size(); i++){
		        std::cout << "\t" << i << ": [" << vec[i] << "]" << std::endl; //Print each element and their position
		    }
		    std::cout << "}" << std::endl; //End
	    }
        //------------------------------------------------------------------------------------------------
        template < typename T> static int findIndex(const std::vector<T> & vec, const T element){ //Template function to find a certain element in a vector, returning its position or MIN_INT if it does not find 
            int index = MIN_INT; //Default value
            auto it = std::find(vec.begin(), vec.end(), element); //Search for element
            if (it != vec.end()) //If it found
                index = std::distance(vec.begin(), it); //Then store its position 

            return index;
        }
};


//--------------------------------------------------------------------------------------------------------
inline int Utility::randomInt(const int min, const int max){ return (rand() % (max - min + 1)) + min; } //Generates a random integer between "min" and "max", boundaries included
//--------------------------------------------------------------------------------------------------------
inline double Utility::distPoints(const Point & P1, const Point & P2) { return sqrt( pow(P1.getX() - P2.getX(), 2) + pow(P1.getY() - P2.getY(), 2) ); } //Calculates the Euclidean distance between 2 points

#endif
//********************************************************************************************************