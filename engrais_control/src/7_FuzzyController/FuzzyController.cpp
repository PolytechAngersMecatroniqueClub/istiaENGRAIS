//********************************************************************************************************
#include "FuzzyController.h"


//--------------------------------------------------------------------------------------------------------
FuzzyController::FuzzyController(fl::TNorm* AndMethod, fl::SNorm* OrMethod, fl::TNorm* ImplicationMethod, fl::SNorm* AggregationMethod, fl::Defuzzifier* defuzzMethod){ //Fuzzy controller constructor to default operators 

    this->fuzzy = new fl::Engine; //Declare engine
    this->fuzzy->setName("EngraisController"); //Name
    this->fuzzy->setDescription(""); //Description

    this->angle = new fl::InputVariable; //Declare angle input variable
    this->angle->setName("angle"); //Name
    this->angle->setDescription("Angle made with central line of robot and the rows found. Negative means at left side of center, positive means right side"); //Description
    this->angle->setEnabled(true); //Enable it
    this->angle->setRange(-PI / 4.0, PI / 4.0); //Set range
    this->angle->setLockValueInRange(false);
    this->angle->addTerm(new fl::Ramp("veryLeft", -PI / 12.0, -PI / 6.0)); //Adds a decrescent ramp membership function
    this->angle->addTerm(new fl::Triangle("left", -PI / 6.0, -PI / 12.0, 0.000)); //Adds a triangular membership function
    this->angle->addTerm(new fl::Triangle("center", -PI / 12.0, 0.000, PI / 12.0)); //Adds a triangular membership function
    this->angle->addTerm(new fl::Triangle("right", 0.000, PI / 12.0, PI / 6.0)); //Adds a triangular membership function
    this->angle->addTerm(new fl::Ramp("veryRight", PI / 12.0, PI / 6.0)); //Adds a crescent ramp membership function

    this->fuzzy->addInputVariable(angle); //Adds input variable

    this->ratio = new fl::InputVariable; //Declares distance input variable
    this->ratio->setName("ratio"); //Name
    this->ratio->setDescription("A ratio that tells how distant the robot is from the center. Negative means at left side of center, positive means right side, Read .h for more information"); //Description
    this->ratio->setEnabled(true); //Enable it
    this->ratio->setRange(-1.000, 1.000); //Set range
    this->ratio->setLockValueInRange(false);
    this->ratio->addTerm(new fl::Ramp("veryLeft", -0.500, -1.000)); //Adds a decrescent ramp membership function
    this->ratio->addTerm(new fl::Triangle("left", -1.000, -0.500, 0.000)); //Adds a triangular membership function
    this->ratio->addTerm(new fl::Triangle("center", -0.500, 0.000, 0.500)); //Adds a triangular membership function
    this->ratio->addTerm(new fl::Triangle("right", 0.000, 0.500, 1.000)); //Adds a triangular membership function
    this->ratio->addTerm(new fl::Ramp("veryRight", 0.500, 1.000)); //Adds a crescent ramp membership function

    this->fuzzy->addInputVariable(ratio); //Adds variable

    this->leftWheel = new fl::OutputVariable; //Declare output
    this->leftWheel->setName("leftWheel"); //Name
    this->leftWheel->setDescription(""); //Description
    this->leftWheel->setEnabled(true); //Enable it
    this->leftWheel->setRange(-1.000, 1.000); //Set range
    this->leftWheel->setLockValueInRange(false);
    this->leftWheel->setAggregation(AggregationMethod); //Define aggregation method
    this->leftWheel->setDefuzzifier(defuzzMethod); //Define defuzzifier
    this->leftWheel->setDefaultValue(fl::nan); //No default value
    this->leftWheel->setLockPreviousValue(false);
    this->leftWheel->addTerm(new fl::Ramp("normalBackward", -0.666, -1.000)); //Adds a decrescent ramp membership function
    this->leftWheel->addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333)); //Adds a triangular membership function
    this->leftWheel->addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000)); //Adds a triangular membership function
    this->leftWheel->addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333)); //Adds a triangular membership function
    this->leftWheel->addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666)); //Adds a triangular membership function
    this->leftWheel->addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000)); //Adds a triangular membership function
    this->leftWheel->addTerm(new fl::Ramp("normalForward", 0.666, 1.000)); //Adds a crescent ramp membership function

    this->fuzzy->addOutputVariable(leftWheel); //Add output

    this->rightWheel = new fl::OutputVariable; //Declare output
    this->rightWheel->setName("rightWheel"); //Name
    this->rightWheel->setDescription(""); //Description
    this->rightWheel->setEnabled(true); //Enable it
    this->rightWheel->setRange(-1.000, 1.000); //Set range
    this->rightWheel->setLockValueInRange(false);
    this->rightWheel->setAggregation(AggregationMethod); //Define aggregation method
    this->rightWheel->setDefuzzifier(defuzzMethod); //Define defuzzifier
    this->rightWheel->setDefaultValue(fl::nan); //No default value
    this->rightWheel->setLockPreviousValue(false);
    this->rightWheel->addTerm(new fl::Ramp("normalBackward", -0.666, -1.000)); //Adds a decrescent ramp membership function
    this->rightWheel->addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333)); //Adds a triangular membership function
    this->rightWheel->addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000)); //Adds a triangular membership function
    this->rightWheel->addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333)); //Adds a triangular membership function
    this->rightWheel->addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666)); //Adds a triangular membership function
    this->rightWheel->addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000)); //Adds a triangular membership function
    this->rightWheel->addTerm(new fl::Ramp("normalForward", 0.666, 1.000)); //Adds a crescent ramp membership function

    this->fuzzy->addOutputVariable(rightWheel); //Add output

    fl::RuleBlock* rules = new fl::RuleBlock; //Declare rule block
    rules->setName("rules"); //Name it 
    rules->setDescription(""); //Description
    rules->setEnabled(true); //Enable it
    rules->setConjunction(AndMethod); //Define And operator
    rules->setDisjunction(OrMethod); //Define Or operator
    rules->setImplication(ImplicationMethod); //Define implication operator
    rules->setActivation(new fl::General); //Activation method

    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is veryLeft then leftWheel is middleForward and rightWheel is middleBackward", fuzzy)); //Add rules
    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is left then leftWheel is slowForward and rightWheel is slowBackward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is center then leftWheel is normalForward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is right then leftWheel is middleForward and rightWheel is middleForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is veryRight then leftWheel is middleForward and rightWheel is normalForward", fuzzy));
 
    rules->addRule(fl::Rule::parse("if ratio is left and angle is veryLeft then leftWheel is middleForward and rightWheel is middleBackward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is left and angle is left then leftWheel is slowForward and rightWheel is slowBackward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is left and angle is center then leftWheel is normalForward and rightWheel is middleForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is left and angle is right then leftWheel is slowForward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is left and angle is veryRight then leftWheel is slowForward and rightWheel is normalForward", fuzzy));
    
    rules->addRule(fl::Rule::parse("if ratio is center and angle is veryLeft then leftWheel is slowForward and rightWheel is slowBackward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is center and angle is left then leftWheel is middleForward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is center and angle is center then leftWheel is normalForward and rightWheel is normalForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is center and angle is right then leftWheel is slowForward and rightWheel is middleForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is center and angle is veryRight then leftWheel is slowBackward and rightWheel is slowForward", fuzzy));
    
    rules->addRule(fl::Rule::parse("if ratio is right and angle is veryLeft then leftWheel is normalForward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is right and angle is left then leftWheel is slowForward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is right and angle is center then leftWheel is middleForward and rightWheel is normalForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is right and angle is right then leftWheel is slowBackward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is right and angle is veryRight then leftWheel is middleBackward and rightWheel is middleForward", fuzzy));

    rules->addRule(fl::Rule::parse("if ratio is veryRight and angle is veryLeft then leftWheel is normalForward and rightWheel is middleForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryRight and angle is left then leftWheel is middleForward and rightWheel is middleForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryRight and angle is center then leftWheel is slowForward and rightWheel is normalForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryRight and angle is right then leftWheel is slowBackward and rightWheel is slowForward", fuzzy));
    rules->addRule(fl::Rule::parse("if ratio is veryRight and angle is veryRight then leftWheel is middleBackward and rightWheel is middleForward", fuzzy));
    
    this->fuzzy->addRuleBlock(rules); //Add rule block
}
//--------------------------------------------------------------------------------------------------------
std::pair<double, double> FuzzyController::getOutputValues(const double ratio, const double ang){ //Use the inputs passed as parameters to calculate the output 
    this->angle->setValue(ang); //Set angle
    this->ratio->setValue(ratio); //Set ratio
    
    this->fuzzy->process(); //Calculates output

    return std::pair<double, double>(this->leftWheel->getValue(), this->rightWheel->getValue()); //Return output
}
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const FuzzyController & fz){ //Print Fuzzy Controller 
    out << "FuzzyController: [ Input Value Angle: " << fz.angle->getValue() << ", Input Value Ratio: " << fz.ratio->getValue();
    out << ", Output Value LeftWheel: " << fz.leftWheel->getValue() << ", Output Value RightWheel: " << fz.rightWheel->getValue() << "]";

    return out; 
}

//********************************************************************************************************