//********************************************************************************************************
#include "FuzzyController.h"


//--------------------------------------------------------------------------------------------------------
FuzzyController::FuzzyController(fl::TNorm* AndMethod, fl::SNorm* OrMethod, fl::TNorm* ImplicationMethod, fl::SNorm* AggregationMethod, fl::Defuzzifier* defuzzMethod){

    this->fuzzy = new fl::Engine;
    this->fuzzy->setName("EngraisController");
    this->fuzzy->setDescription("");

    this->angle = new fl::InputVariable;
    this->angle->setName("angle");
    this->angle->setDescription("Angle made with central line of robot and the rows found-> Negative means at left side of center, positive means right sides ");
    this->angle->setEnabled(true);
    this->angle->setRange(-PI / 4.0, -PI / 4.0);
    this->angle->setLockValueInRange(false);
    this->angle->addTerm(new fl::Ramp("veryLeft", -PI / 12.0, -PI / 6.0));
    this->angle->addTerm(new fl::Triangle("left", -PI / 6.0, -PI / 12.0, 0.000));
    this->angle->addTerm(new fl::Triangle("center", -PI / 12.0, 0.000, PI / 12.0));
    this->angle->addTerm(new fl::Triangle("right", 0.000, PI / 12.0, PI / 6.0));
    this->angle->addTerm(new fl::Ramp("veryRight", PI / 12.0, PI / 6.0));

    this->fuzzy->addInputVariable(angle);

    this->ratio = new fl::InputVariable;
    this->ratio->setName("ratio");
    this->ratio->setDescription("");
    this->ratio->setEnabled(true);
    this->ratio->setRange(-1.000, 1.000);
    this->ratio->setLockValueInRange(false);
    this->ratio->addTerm(new fl::Ramp("veryLeft", -0.500, -1.000));
    this->ratio->addTerm(new fl::Triangle("left", -1.000, -0.500, 0.000));
    this->ratio->addTerm(new fl::Triangle("center", -0.500, 0.000, 0.500));
    this->ratio->addTerm(new fl::Triangle("right", 0.000, 0.500, 1.000));
    this->ratio->addTerm(new fl::Ramp("veryRight", 0.500, 1.000));

    this->fuzzy->addInputVariable(ratio);

    this->leftWheel = new fl::OutputVariable;
    this->leftWheel->setName("leftWheel");
    this->leftWheel->setDescription("");
    this->leftWheel->setEnabled(true);
    this->leftWheel->setRange(-1.000, 1.000);
    this->leftWheel->setLockValueInRange(false);
    this->leftWheel->setAggregation(AggregationMethod);
    this->leftWheel->setDefuzzifier(defuzzMethod);
    this->leftWheel->setDefaultValue(fl::nan);
    this->leftWheel->setLockPreviousValue(false);
    this->leftWheel->addTerm(new fl::Ramp("normalBackward", -0.666, -1.000));
    this->leftWheel->addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333));
    this->leftWheel->addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000));
    this->leftWheel->addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333));
    this->leftWheel->addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666));
    this->leftWheel->addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000));
    this->leftWheel->addTerm(new fl::Ramp("normalForward", 0.666, 1.000));

    this->fuzzy->addOutputVariable(leftWheel);

    this->rightWheel = new fl::OutputVariable;
    this->rightWheel->setName("rightWheel");
    this->rightWheel->setDescription("");
    this->rightWheel->setEnabled(true);
    this->rightWheel->setRange(-1.000, 1.000);
    this->rightWheel->setLockValueInRange(false);
    this->rightWheel->setAggregation(AggregationMethod);
    this->rightWheel->setDefuzzifier(defuzzMethod);
    this->rightWheel->setDefaultValue(fl::nan);
    this->rightWheel->setLockPreviousValue(false);
    this->rightWheel->addTerm(new fl::Ramp("normalBackward", -0.666, -1.000));
    this->rightWheel->addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333));
    this->rightWheel->addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000));
    this->rightWheel->addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333));
    this->rightWheel->addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666));
    this->rightWheel->addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000));
    this->rightWheel->addTerm(new fl::Ramp("normalForward", 0.666, 1.000));

    this->fuzzy->addOutputVariable(rightWheel);

    fl::RuleBlock* rules = new fl::RuleBlock;
    rules->setName("rules");
    rules->setDescription("");
    rules->setEnabled(true);
    rules->setConjunction(AndMethod);
    rules->setDisjunction(OrMethod);
    rules->setImplication(ImplicationMethod);
    rules->setActivation(new fl::General);

    rules->addRule(fl::Rule::parse("if ratio is veryLeft and angle is veryLeft then leftWheel is middleForward and rightWheel is middleBackward", fuzzy));
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
    
    this->fuzzy->addRuleBlock(rules);
}
//--------------------------------------------------------------------------------------------------------
std::pair<double, double> FuzzyController::getOutputValues(const double ratio, const double ang){
    this->angle->setValue(ang);
    this->ratio->setValue(ratio);
    
    this->fuzzy->process();

    return std::pair<double, double>(this->leftWheel->getValue(), this->rightWheel->getValue());
}
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const FuzzyController & fz){
    out << "FuzzyController: [ Input Value Angle: " << fz.angle->getValue() << ", Input Value Ratio: " << fz.ratio->getValue();
    out << ", Output Value LeftWheel: " << fz.leftWheel->getValue() << ", Output Value RightWheel: " << fz.rightWheel->getValue() << "]";

    return out; 
}

//********************************************************************************************************