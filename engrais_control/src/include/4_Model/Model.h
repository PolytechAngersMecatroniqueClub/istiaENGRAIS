//********************************************************************************************************
#ifndef MODEL_H
#define MODEL_H

#include <iostream>
#include <vector>

#include "../1_Point/Point.h"
#include "../3_Utility/Utility.h"


class Model{ 
    private: 
        double a;
        double b;
        double energy;

        int parallelCount;
        double fitness;

        int positivePoints;
        std::vector<Point> pointsInModel;

    public:
        //------------------------------------------------------------------------------------------------
        Model(const double = MAX_DBL , const double = MAX_DBL , const double = MAX_DBL); //Checked


        //------------------------------------------------------------------------------------------------
        static Model linearFit(const std::vector<Point> & ); //Checked
        //------------------------------------------------------------------------------------------------
        void findBestModel(); //Checked 
        //------------------------------------------------------------------------------------------------
        void findBestModel(const std::vector<Point> & ); //Checked 


        //------------------------------------------------------------------------------------------------
        double getSlope() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getIntercept() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getEnergy() const; //Checked
        //------------------------------------------------------------------------------------------------
        int getParallelCount() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getFitness() const; //Checked
        //------------------------------------------------------------------------------------------------
        int getPositivePointsNum() const; //Checked
        //------------------------------------------------------------------------------------------------
        int getPointsSize() const; //Checked
        //------------------------------------------------------------------------------------------------
        std::pair<Point, Point> getFirstAndLastPoint() const;
        //------------------------------------------------------------------------------------------------
        std::vector<Point> getPointsInModel() const; //Checked
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecBegin() const; //Checked
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecEnd() const; //Checked
        //------------------------------------------------------------------------------------------------
        bool isPopulated() const; //Checked


        //------------------------------------------------------------------------------------------------
        void pushPoint(const Point & ); //Checked
        //------------------------------------------------------------------------------------------------
        void fuseModel(const Model & ); //Checked 
        //------------------------------------------------------------------------------------------------
        void incrementParallelCount(); //Checked
        //------------------------------------------------------------------------------------------------
        void resetParallelCount(); //Checked
        //------------------------------------------------------------------------------------------------
        double calculateFitness(); //Checked
        //------------------------------------------------------------------------------------------------
        void clearPoints(); //Checked


        //------------------------------------------------------------------------------------------------
        bool operator < (const Model & ) const; //Checked 
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const Model & ); //Checked 
};


//--------------------------------------------------------------------------------------------------------
inline Model::Model(const double aa, const double bb, const double e) : a(aa), b(bb), energy(e), parallelCount(1), fitness(MAX_DBL), positivePoints(0) {} //Checked


//--------------------------------------------------------------------------------------------------------
inline double Model::getSlope() const { return this->a; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::getIntercept() const { return this->b; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::getEnergy() const { return this->energy; } //Checked
//--------------------------------------------------------------------------------------------------------
inline int Model::getParallelCount() const { return parallelCount; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::getFitness() const { return fitness; } //Checked
//--------------------------------------------------------------------------------------------------------
inline int Model::getPositivePointsNum() const { return this->positivePoints; } //Checked
//--------------------------------------------------------------------------------------------------------
inline int Model::getPointsSize() const { return pointsInModel.size(); } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> Model::getPointsInModel() const { return this->pointsInModel; } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecBegin() const { return pointsInModel.begin(); } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecEnd() const { return pointsInModel.end(); } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Model::isPopulated() const { //Checked 
    if(a == MAX_DBL && b == MAX_DBL)
        return false;
    return true;
}


//--------------------------------------------------------------------------------------------------------
inline void Model::incrementParallelCount(){ parallelCount++; } //Checked
//--------------------------------------------------------------------------------------------------------
inline void Model::resetParallelCount() { parallelCount = 1; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::calculateFitness() { this->fitness = (this->getPointsSize() != 0 ? (pow(this->getIntercept(), 2) + this->getEnergy() * 10) / (double)(pow(this->getPointsSize(), 2) * this->parallelCount) : MAX_DBL); return this->fitness; } //Checked
//--------------------------------------------------------------------------------------------------------
inline void Model::clearPoints() { this->energy = MAX_DBL; this->positivePoints = 0; this->pointsInModel.clear(); } //Checked


//--------------------------------------------------------------------------------------------------------
inline bool Model::operator < (const Model & m) const { return (this->getFitness() < m.getFitness()); } //Checked 

#endif
//********************************************************************************************************