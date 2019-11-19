//********************************************************************************************************
#include "Utility.h"

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Utility::printInColor(const std::string msg, const int color){ //Print colored text to console 
    std::string out;

    if(color == RED)
        out = "\n\033[1;" + std::to_string(color) + "m[ERR] " + msg + "\033[0m\n\n";

    else if(color == CYAN)
        out = "\033[1;" + std::to_string(color) + "m[OK] " + msg + "\033[0m\n";

    else if(color == BLUE)
        out = "\n\033[1;" + std::to_string(color) + "m[OK] " + msg + "\033[0m\n\n";

    else
        out = "\033[1;" + std::to_string(color) + "m[MSG] " + msg + "\033[0m\n";

    std::cout << out;
}
// -------------------------------------------------------------------------------------------------------
std::vector<int> Utility::randomDiffVector(const int min, const int max, const int size){ //Returns a vector, sized "size", of different random numbers ranging from "min" from "max", boundaries included 
    std::vector<int> r(size, MIN_INT); //Initializes vector
    int randNum;

    if((max-min+1) < size){ //If the number of available numbers is smaller than the amount passed, prints an error
        Utility::printInColor("Wrong usage of randomDiffVector, max - min + 1 should be greater than the size", RED);
        return std::vector<int>();
    }

    for(int i = 0; i < size; i++){ //Randomizes a number that is not inside the vector
        while(true){
            randNum = Utility::randomInt(min, max); //Guess a number
            if(findIndex(r, randNum) == MIN_INT) //If the element exists in the vector, try again
                break;
        }
        r[i] = randNum; //Put the number into vector
    }

    return r;
}
//--------------------------------------------------------------------------------------------------------
double Utility::calcSumDist(const std::vector<Point> & vec_p, const double a, const double b) { //Calculates the sum of all points in the vector and a line 
    double dist = 0; //Sum = 0
    for(Point p : vec_p){ //For each point
        dist +=  fabs(a * p.getX() - p.getY() + b) / sqrt(pow(a, 2) + 1.0); //Sum the distance from point to line
    }
    
    return dist;
}

//########################################################################################################

//********************************************************************************************************