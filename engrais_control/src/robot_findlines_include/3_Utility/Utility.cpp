//********************************************************************************************************
#include "Utility.h"


//--------------------------------------------------------------------------------------------------------
double Utility::calcSumDist(const std::vector<Point> & vec_p, const double a, const double b) { //Checked
    double dist = 0;
    for(Point p : vec_p){ 
        dist +=  fabs(a * p.getX() - p.getY() + b) / sqrt(pow(a, 2) + 1.0); 
    }
    
    return dist;
}
//--------------------------------------------------------------------------------------------------------
void Utility::printInColor(const std::string msg, const int color){ //Checked 
    std::string msgType = "[MSG]";
    if(color == RED)
        msgType = "[ERR]";
    else if(color == CYAN || color == BLUE)
        msgType = "[OK]";

    if(color == RED || color == BLUE)
        std::cout << std::endl;
    std::cout << std::endl << "\033[1;" << color << "m" << msgType << " " << msg << "\033[0m";

    if(color == RED || color == BLUE)
        std::cout << std::endl;
}
// -------------------------------------------------------------------------------------------------------
std::vector<int> Utility::randomDiffVector(const int min, const int max, const int size){ //Checked
    std::vector<int> r(size, MIN_INT);
    int randNum;

    if((max-min+1) < size){
        Utility::printInColor("Wrong usage of randomDiffVector, max - min + 1 should be greater than the size", RED);
        return std::vector<int>();
    }

    for(int i = 0; i < size; i++){
        while(true){
            randNum = Utility::randomInt(min, max);
            if(findIndex(r, randNum) == MIN_INT)
                break;
        }
        r[i] = randNum;
    }

    return r;
}

//********************************************************************************************************