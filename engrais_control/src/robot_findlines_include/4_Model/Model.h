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

        int parallelCount = 1;
        double fitness = MAX_DBL;

        std::vector<Point> pointsInModel;

    public:
        //------------------------------------------------------------------------------------------------
        Model(const double = MAX_DBL , const double = MAX_DBL , const double = MAX_DBL); //Checked
        //------------------------------------------------------------------------------------------------
        Model(const std::vector<Point> & ); //Checked


        //------------------------------------------------------------------------------------------------
        static Model linearFit(const std::vector<Point> & ); //Checked
        //------------------------------------------------------------------------------------------------
        void findBestModel();
        //------------------------------------------------------------------------------------------------
        void findBestModel(const std::vector<Point> & );


        //------------------------------------------------------------------------------------------------
        double getSlope() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getIntercept() const; //Checked
        //------------------------------------------------------------------------------------------------
        double getEnergy() const; //Checked
        //------------------------------------------------------------------------------------------------
        bool isPopulated() const;
        //------------------------------------------------------------------------------------------------
        double getFitness() const;
        //------------------------------------------------------------------------------------------------
        int getPointsSize() const; //Checked
        //------------------------------------------------------------------------------------------------
        int getParallelCount() const;
        //------------------------------------------------------------------------------------------------
        std::vector<Point> getPointsInModel() const; //Checked
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecBegin() const; //Checked
        //------------------------------------------------------------------------------------------------
        std::vector<Point>::const_iterator getPointsVecEnd() const; //Checked



        //------------------------------------------------------------------------------------------------
        void pushPoint(const Point & ); //Checked
        //------------------------------------------------------------------------------------------------
        void pushPointAtBeginning(const Point & );
        //------------------------------------------------------------------------------------------------
        void fuseModel(const Model & );
        //------------------------------------------------------------------------------------------------
        void setEnergy(const double ); //Checked
        //------------------------------------------------------------------------------------------------
        void addEnergy(const double ); //Checked
        //------------------------------------------------------------------------------------------------
        void setParallelCount(const int );
        //------------------------------------------------------------------------------------------------
        void incrementParallelCount();
        //------------------------------------------------------------------------------------------------
        double calculateFitness();
        //------------------------------------------------------------------------------------------------
        void clearPoints(); //Checked


        //------------------------------------------------------------------------------------------------
        bool operator < (const Model & ) const;
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const Model & ); //Checked
};


//--------------------------------------------------------------------------------------------------------
inline Model::Model(const double aa, const double bb, const double e) : a(aa), b(bb), energy(e) {} //Checked
//--------------------------------------------------------------------------------------------------------
inline Model::Model(const std::vector<Point> & vec) : a(MAX_DBL), b(MAX_DBL), energy(MAX_DBL) { findBestModel(vec); } //Checked


//--------------------------------------------------------------------------------------------------------
inline double Model::getSlope() const { return this->a; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::getIntercept() const { return this->b; } //Checked
//--------------------------------------------------------------------------------------------------------
inline double Model::getEnergy() const { return this->energy; } //Checked
//--------------------------------------------------------------------------------------------------------
inline bool Model::isPopulated() const { 
    if(a == MAX_DBL && b == MAX_DBL)
        return false;
    return true;
}
//--------------------------------------------------------------------------------------------------------
inline double Model::getFitness() const { return fitness; }
//--------------------------------------------------------------------------------------------------------
inline int Model::getPointsSize() const { return pointsInModel.size(); } //Checked
//--------------------------------------------------------------------------------------------------------
inline int Model::getParallelCount() const { return parallelCount; }
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point> Model::getPointsInModel() const { return this->pointsInModel; } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecBegin() const { return pointsInModel.begin(); } //Checked
//--------------------------------------------------------------------------------------------------------
inline std::vector<Point>::const_iterator Model::getPointsVecEnd() const { return pointsInModel.end(); } //Checked


//--------------------------------------------------------------------------------------------------------
inline void Model::pushPoint(const Point & p) { pointsInModel.push_back(p); } //Checked
//--------------------------------------------------------------------------------------------------------
inline void Model::pushPointAtBeginning(const Point & p) { pointsInModel.insert(pointsInModel.begin(), 1, p); }
//--------------------------------------------------------------------------------------------------------
inline void Model::setEnergy(const double e) { this->energy = e; } //Checked
//--------------------------------------------------------------------------------------------------------
inline void Model::addEnergy(const double e) { this->energy += e; } //Checked
//--------------------------------------------------------------------------------------------------------
inline void Model::setParallelCount(const int p) { parallelCount = p; }
//--------------------------------------------------------------------------------------------------------
inline void Model::incrementParallelCount() { parallelCount++; }
//--------------------------------------------------------------------------------------------------------
inline double Model::calculateFitness() { this->fitness = (this->getPointsSize() != 0 ? (pow(this->getIntercept(), 2) + this->getEnergy() * 10) / (double)(pow(this->getPointsSize(), 2) * this->parallelCount) : MAX_DBL); return this->fitness; }
//--------------------------------------------------------------------------------------------------------
inline void Model::clearPoints() { pointsInModel.clear(); } //Checked


//--------------------------------------------------------------------------------------------------------
inline bool Model::operator < (const Model & m) const { return (this->getFitness() < m.getFitness()); }

#endif
//********************************************************************************************************