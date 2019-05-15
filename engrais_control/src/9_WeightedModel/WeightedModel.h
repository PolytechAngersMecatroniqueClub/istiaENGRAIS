//********************************************************************************************************
#ifndef WEIGHTED_MODEL_H
#define WEIGHTED_MODEL_H

#include <iostream>
#include <vector>

#include <Point.h>
#include <Model.h>

#include <Pearl.h>
#include <1_RubyPure.h>
#include <2_RubyGenetic.h>
#include <3_RubyGeneticOnePoint.h>
#include <4_RubyGeneticOnePointPosNeg.h>
#include <5_RubyGeneticOnePointPosNegInfinite.h>


class WeightedModel{ 
    private:
        int cont = 1;

        double a = MAX_DBL;
        double b = MAX_DBL;

        std::pair<Point, Point> positivePoints;
        std::pair<Point, Point> negativePoints;

    public:
        //------------------------------------------------------------------------------------------------
        WeightedModel();
        //------------------------------------------------------------------------------------------------
        WeightedModel(const Model & m);
        //------------------------------------------------------------------------------------------------
        WeightedModel(const double aa, const double bb);
        //------------------------------------------------------------------------------------------------
        double getSlope();
        //------------------------------------------------------------------------------------------------
        double getIntercept();
        //------------------------------------------------------------------------------------------------
        int getCounter();
        //------------------------------------------------------------------------------------------------
        void assignPoints(const Model & m);
        //------------------------------------------------------------------------------------------------
        bool checkIfSameModel(const Model & m);
        //------------------------------------------------------------------------------------------------
        void fuseModels(const Model & m);
        //------------------------------------------------------------------------------------------------
        Model toModel();

        
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const WeightedModel & wm);
};

//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel() {}
//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel(const Model & m) : a(m.getSlope()), b(m.getIntercept()) { this->assignPoints(m); }
//--------------------------------------------------------------------------------------------------------
inline WeightedModel::WeightedModel(const double aa, const double bb) : a(aa), b(bb) {}


//--------------------------------------------------------------------------------------------------------
inline double WeightedModel::getSlope() { return a; }
//--------------------------------------------------------------------------------------------------------
inline double WeightedModel::getIntercept() { return b; }
//--------------------------------------------------------------------------------------------------------
inline int WeightedModel::getCounter() { return cont; }

#endif
//********************************************************************************************************