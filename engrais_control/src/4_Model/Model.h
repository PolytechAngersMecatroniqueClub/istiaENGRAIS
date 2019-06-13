//********************************************************************************************************
#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <vector>

#include <Point.h>
#include <Utility.h>


class Model{ //Class to store models (lines)
    private: 
        double a; //Model's slope
        double b; //Model's intercept
        double energy = MAX_DBL; //Model's energy

        int parallelCount = 1; //Number of parallel models
        double fitness = MAX_DBL; //Model's fitness (how good it is)

        int positivePoints = 0; //Nuber of positive points in the model
        std::vector<Point> pointsInModel; //Ponts that are in the model

    public:
        //------------------------------------------------------------------------------------------------
        Model(const double aa = MAX_DBL , const double bb = MAX_DBL); //Model Constructor 


        //------------------------------------------------------------------------------------------------
        static Model linearFit(const std::vector<Point> & vec); //Uses gauss' linear regression to find best model
        //------------------------------------------------------------------------------------------------
        void findBestModel(); //Using the points attached, calculate the best line possible 
        //------------------------------------------------------------------------------------------------
        void findBestModel(const std::vector<Point> & vec); //Using the points passed, calculate the best line possible and store points inside model


        //------------------------------------------------------------------------------------------------
        double getSlope() const; //Get model's slope
        //------------------------------------------------------------------------------------------------
        double getIntercept() const; //Get model's intercept
        //------------------------------------------------------------------------------------------------
        double getEnergy() const; //Get model's energy
        //------------------------------------------------------------------------------------------------
        int getParallelCount() const; //Get model's parallel count
        //------------------------------------------------------------------------------------------------
        double getFitness() const; //Get model's fitness
        //------------------------------------------------------------------------------------------------
        int getPositivePointsNum() const; //Get number of positive points in model
        //------------------------------------------------------------------------------------------------
        int getPointsSize() const; //Get number of points in model
        //------------------------------------------------------------------------------------------------
        std::pair<Point, Point> getFirstAndLastPoint() const; //Get the negative-most point (first) and the positive-most point (second) using only x-coordinate
        //------------------------------------------------------------------------------------------------
        std::vector<Point> getPointsInModel() const; //Get points in model 
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecBegin() const; //Get iterator to first point
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecEnd() const; //Get iterator to last point
        //------------------------------------------------------------------------------------------------
        bool isPopulated() const; //Checks if model is populated (a != MAX_DBL and b != MAX_DBL)


        //------------------------------------------------------------------------------------------------
        void pushPoint(const Point & p); //Interts a point in the model
        //------------------------------------------------------------------------------------------------
        void fuseModel(const Model & m); //Fuse two models, combining point and calculating the new best model 
        //------------------------------------------------------------------------------------------------
        void incrementParallelCount(); //Increment parallel count
        //------------------------------------------------------------------------------------------------
        void resetParallelCount(); //Reset parallel count to 1
        //------------------------------------------------------------------------------------------------
        double calculateFitness(); //Calculate model's fitness
        //------------------------------------------------------------------------------------------------
        void clearPoints(); //Clear model's points


        //------------------------------------------------------------------------------------------------
        bool operator < (const Model & m) const; //Compare two models fitness
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const Model & m); //Print model 
};


//--------------------------------------------------------------------------------------------------------
inline Model::Model(const double aa, const double bb) : a(aa), b(bb) {} //Model Constructor 


//--------------------------------------------------------------------------------------------------------
inline double Model::getSlope() const { return this->a; } //Get model's slope
//--------------------------------------------------------------------------------------------------------
inline double Model::getIntercept() const { return this->b; } //Get model's intercept
//--------------------------------------------------------------------------------------------------------
inline double Model::getEnergy() const { return this->energy; } //Get model's energy
//--------------------------------------------------------------------------------------------------------
inline int Model::getParallelCount() const { return this->parallelCount; } //Get model's parallel count
//--------------------------------------------------------------------------------------------------------
inline double Model::getFitness() const { return this->fitness; } //Get model's fitness
//--------------------------------------------------------------------------------------------------------
inline int Model::getPositivePointsNum() const { return this->positivePoints; } //Get number of positive points in model
//--------------------------------------------------------------------------------------------------------
inline int Model::getPointsSize() const { return this->pointsInModel.size(); } //Get number of points in model
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> Model::getPointsInModel() const { return this->pointsInModel; } //Get points in model 
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecBegin() const { return this->pointsInModel.begin(); } //Get iterator to first point
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecEnd() const { return this->pointsInModel.end(); } //Get iterator to last point
//--------------------------------------------------------------------------------------------------------
inline bool Model::isPopulated() const { //Checks if model is populated (a != MAX_DBL and b != MAX_DBL)
    if(this->a == MAX_DBL && this->b == MAX_DBL)
        return false;
    return true;
}


//--------------------------------------------------------------------------------------------------------
inline void Model::incrementParallelCount(){ this->parallelCount++; } //Increment parallel count
//--------------------------------------------------------------------------------------------------------
inline void Model::resetParallelCount() { this->parallelCount = 1; } //Reset parallel count to 1
//--------------------------------------------------------------------------------------------------------
inline double Model::calculateFitness() { this->fitness = (this->getPointsSize() != 0 ? (pow(this->getIntercept(), 2) + this->getEnergy() * 10) / (double)(pow(this->getPointsSize(), 2) * this->parallelCount) : MAX_DBL); return this->fitness; } //Calculate model's fitness
//--------------------------------------------------------------------------------------------------------
inline void Model::clearPoints() { this->energy = MAX_DBL; this->positivePoints = 0; this->pointsInModel.clear(); } //Clear model's points


//--------------------------------------------------------------------------------------------------------
inline bool Model::operator < (const Model & m) const { return (this->getFitness() < m.getFitness()); } //Compare two models fitness

#endif
//********************************************************************************************************