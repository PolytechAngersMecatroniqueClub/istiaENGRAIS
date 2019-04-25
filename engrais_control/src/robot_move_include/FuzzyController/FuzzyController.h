//********************************************************************************************************
#ifndef FUZZY_CONTROLLER
#define FUZZY_CONTROLLER

#include <iostream>
#include <fl/Headers.h>

class FuzzyController{
    private:
        fl::Engine fuzzy;

        fl::InputVariable angle;
        fl::InputVariable distanceToCenter;
        
        fl::OutputVariable leftWheel;
        fl::OutputVariable rightWheel;


    public:
    	//------------------------------------------------------------------------------------------------
        FuzzyController(fl::TNorm* = new fl::Minimum, fl::SNorm* = new fl::Maximum, fl::TNorm* = new fl::Minimum, fl::SNorm* = new fl::Maximum, fl::Defuzzifier* = new fl::Centroid(100));   
		//------------------------------------------------------------------------------------------------
        std::pair<double, double> getOutputValues(const double , const double );

        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & , const FuzzyController & );
};

#endif
//********************************************************************************************************