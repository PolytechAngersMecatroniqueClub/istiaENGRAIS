//********************************************************************************************************
#ifndef FUZZY_CONTROLLER
#define FUZZY_CONTROLLER

#include <cmath>
#include <iostream>
#include <fl/Headers.h>

#ifndef PI 
#define PI 3.1415926535
#endif

class FuzzyController{
    private:
        fl::Engine* fuzzy;

        fl::InputVariable* angle;
        fl::InputVariable* ratio;
        
        fl::OutputVariable* leftWheel;
        fl::OutputVariable* rightWheel;
        

    public:
    	//------------------------------------------------------------------------------------------------
        FuzzyController(fl::TNorm* AndMethod = new fl::Minimum, fl::SNorm* OrMethod = new fl::Maximum, fl::TNorm* ImplicationMethod = new fl::Minimum, fl::SNorm* AggregationMethod = new fl::Maximum, fl::Defuzzifier* defuzzMethod = new fl::Centroid(100));   
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> getOutputValues(const double ratio, const double ang);

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const FuzzyController & );
};

#endif
//********************************************************************************************************