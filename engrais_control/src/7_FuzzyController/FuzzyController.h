//********************************************************************************************************

// This class implements the fuzzy controller for ENGRAIS robot, you can find its description in "fuzzy_controller.pdf"

// To learn how fuzzy controllers work, the presentation Fuzzy.pdf is available for consultation 

// One thing changed due to fuzzy logic limitations, being the position of the robot
// Since we cannot use absolute measures (1 meter from center can be a disaster or accetable depending on how the plants are arranged), we have to use ratios to define if the robot is in the middle or not

// First, we have 2 lines as information, containing its slope[a], intercept[b], negative-most point[model->getPointsInModel()[0]] and positive-most point [model->getPointsInModel()[1]]

// Second, using both intercepts (b_left and b_right) we can calculate the absolute distance to center as (||b_left| - |b_right||)/2, but this is not usefull to our problem

// As a solution we came with the ratio. Defining b_max = max( |b_left|, |b_right| ) and b_min = min( |b_left|, |b_right| ) using r = b_min/b_max, we know that if r = 1, then we are in the center

// As an additional information, if |b_left| < |b_right|, then the robot is to the left. By extention if |b_right| < |b_left|, then the robot is to the right.

// To define the final input, we have the metric ratio = (b_min / b_max - 1.0) and if |b_right| < |b_left|, we multiply ratio by -1.

// This way, if ratio = 0 => robot is at the center of rows; if ratio < 0 => robot is to the left of central line; if ratio > 0 => robot is to the right of central line

// Ratio, of cource, ranges from -1 to 1.


//The angle is a lot more straightforward, meaning the angle between the robot central line (x axis) and the lines found (normally the average between the two), ranging from -90 to 90º 

//********************************************************************************************************
#ifndef FUZZY_CONTROLLER
#define FUZZY_CONTROLLER

#include <cmath>
#include <iostream>
#include <fl/Headers.h>
#include <Model.h>

#ifndef PI 
#define PI 3.1415926535 //Define PI
#endif

class FuzzyController{ //Class to encapsulate the fuzzy controller 
    private:
        fl::Engine* fuzzy; //Declares Engine

        fl::InputVariable* angle; //Declares Input
        fl::InputVariable* ratio;
        
        fl::OutputVariable* leftWheel; //Declares Output
        fl::OutputVariable* rightWheel;
        

    public:
    	//------------------------------------------------------------------------------------------------
        FuzzyController(fl::TNorm* AndMethod = new fl::Minimum, 
        				fl::SNorm* OrMethod = new fl::Maximum, 
        				fl::TNorm* ImplicationMethod = new fl::AlgebraicProduct, 
                        fl::SNorm* AggregationMethod = new fl::Maximum, 
                        fl::Defuzzifier* defuzzMethod = new fl::Centroid(100)
		); //Fuzzy controller constructor to default operators
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> getOutputValues(const double ratio, const double ang); //Use the inputs passed as parameters to calculate the output
        //------------------------------------------------------------------------------------------------
        friend std::ostream & operator << (std::ostream & out, const FuzzyController & fz); //Print Fuzzy Controller
        
};

#endif
//********************************************************************************************************