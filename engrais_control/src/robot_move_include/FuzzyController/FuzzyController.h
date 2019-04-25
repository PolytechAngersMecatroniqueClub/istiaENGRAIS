//********************************************************************************************************
#ifndef FUZZY_CONTROLLER
#define FUZZY_CONTROLLER

#include <iostream>
#include <fl/Headers.h>

class FuzzyController{
    private:
        fl::Engine* fuzzy;

        fl::InputVariable* angle;
        fl::InputVariable* distanceToCenter;
        
        fl::OutputVariable* leftWheel;
        fl::OutputVariable* rightWheel;
        

    public:
    	//------------------------------------------------------------------------------------------------
        FuzzyController(fl::TNorm* AndMethod = new fl::Minimum, fl::SNorm* OrMethod = new fl::Maximum, fl::TNorm* ImplicationMethod = new fl::Minimum, fl::SNorm* AggregationMethod = new fl::Maximum, fl::Defuzzifier* defuzzMethod = new fl::Centroid(100));   
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> getOutputValues(const double dist, const double ang);

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const FuzzyController & );
};

#endif
//********************************************************************************************************