//********************************************************************************************************
#ifndef UTILITY_H
#define UTILITY_H

#include "Point.h" 
#include <iostream>
#include <algorithm>
#include <vector>
#include <string>
#include <cmath>

#define BLUE 34
#define RED 31
#define CYAN 36

class Utility{
    public:
        //------------------------------------------------------------------------------------------------
        static double calcSumDist(const std::vector<Point> & , const double , const double ); //Checked
        //------------------------------------------------------------------------------------------------
        static int randomInt(const int min, const int max); //Checked
        //------------------------------------------------------------------------------------------------
        static void printInColor(const std::string , const int ); //Checked
        //------------------------------------------------------------------------------------------------
        static std::vector<int> randomDiffVector(const int min, const int max, const int size);
        //------------------------------------------------------------------------------------------------
        template < class T> static void printVector(const std::vector<T> & vec){ //Checked
        	std::cout << std::endl << "Vector {" << std::endl;
		    for(int i = 0; i < vec.size(); i++){
		        std::cout << "\t" << i << ": [" << vec[i] << "]" << std::endl; 
		    }
		    std::cout << "}" << std::endl;
	    }
        //------------------------------------------------------------------------------------------------
        template < typename T> static int findIndex(const std::vector<T> & vec, const T element){
            int index = MIN_INT;
            auto it = std::find(vec.begin(), vec.end(), element);
            if (it != vec.end())
                index = std::distance(vec.begin(), it);

            return index;
        }
};

//--------------------------------------------------------------------------------------------------------
inline int Utility::randomInt(const int min, const int max){ return (rand() % (max - min + 1)) + min; } //Checked

#endif
//********************************************************************************************************