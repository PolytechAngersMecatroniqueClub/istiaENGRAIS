//********************************************************************************************************
#include "FuzzyController.h"


//--------------------------------------------------------------------------------------------------------
FuzzyController::FuzzyController(fl::TNorm* AndMethod, fl::SNorm* OrMethod, fl::TNorm* ImplicationMethod, fl::SNorm* AggregationMethod, fl::Defuzzifier* defuzzMethod){

    this->fuzzy.setName("EngraisController");
    this->fuzzy.setDescription("");

    this->angle.setName("angle");
    this->angle.setDescription("Angle made with central line of robot and the rows found. Negative means at left side of center, positive means right sides ");
    this->angle.setEnabled(true);
    this->angle.setRange(-45.000, 45.000);
    this->angle.setLockValueInRange(false);
    this->angle.addTerm(new fl::Ramp("veryLeft", -15.000, -30.000));
    this->angle.addTerm(new fl::Triangle("left", -30.000, -15.000, 0.000));
    this->angle.addTerm(new fl::Triangle("center", -15.000, 0.000, 15.000));
    this->angle.addTerm(new fl::Triangle("right", 0.000, 15.000, 30.000));
    this->angle.addTerm(new fl::Ramp("veryRight", 15.000, 30.000));

    this->fuzzy.addInputVariable(&angle);

    this->distanceToCenter.setName("distanceToCenter");
    this->distanceToCenter.setDescription("Distance to centrer of rows. Negative means at left side of center, positive means right side");
    this->distanceToCenter.setEnabled(true);
    this->distanceToCenter.setRange(-2.000, 2.000);
    this->distanceToCenter.setLockValueInRange(false);
    this->distanceToCenter.addTerm(new fl::Ramp("veryLeft", -0.750, -1.500));
    this->distanceToCenter.addTerm(new fl::Triangle("left", -1.500, -0.750, 0.000));
    this->distanceToCenter.addTerm(new fl::Triangle("center", -0.750, 0.000, 0.750));
    this->distanceToCenter.addTerm(new fl::Triangle("right", 0.000, 0.750, 1.500));
    this->distanceToCenter.addTerm(new fl::Ramp("veryRight", 0.750, 1.500));

    this->fuzzy.addInputVariable(&distanceToCenter);

    this->leftWheel.setName("leftWheel");
    this->leftWheel.setDescription("");
    this->leftWheel.setEnabled(true);
    this->leftWheel.setRange(-1.000, 1.000);
    this->leftWheel.setLockValueInRange(false);
    this->leftWheel.setAggregation(AggregationMethod);
    this->leftWheel.setDefuzzifier(defuzzMethod);
    this->leftWheel.setDefaultValue(fl::nan);
    this->leftWheel.setLockPreviousValue(false);
    this->leftWheel.addTerm(new fl::Ramp("normalBackward", -0.666, -1.000));
    this->leftWheel.addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333));
    this->leftWheel.addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000));
    this->leftWheel.addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333));
    this->leftWheel.addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666));
    this->leftWheel.addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000));
    this->leftWheel.addTerm(new fl::Ramp("normalForward", 0.666, 1.000));

    this->fuzzy.addOutputVariable(&leftWheel);

    this->rightWheel.setName("rightWheel");
    this->rightWheel.setDescription("");
    this->rightWheel.setEnabled(true);
    this->rightWheel.setRange(-1.000, 1.000);
    this->rightWheel.setLockValueInRange(false);
    this->rightWheel.setAggregation(AggregationMethod);
    this->rightWheel.setDefuzzifier(defuzzMethod);
    this->rightWheel.setDefaultValue(fl::nan);
    this->rightWheel.setLockPreviousValue(false);
    this->rightWheel.addTerm(new fl::Ramp("normalBackward", -0.666, -1.000));
    this->rightWheel.addTerm(new fl::Triangle("middleBackward", -1.000, -0.666, -0.333));
    this->rightWheel.addTerm(new fl::Triangle("slowBackward", -0.666, -0.333, 0.000));
    this->rightWheel.addTerm(new fl::Triangle("stop", -0.333, 0.000, 0.333));
    this->rightWheel.addTerm(new fl::Triangle("slowForward", 0.000, 0.333, 0.666));
    this->rightWheel.addTerm(new fl::Triangle("middleForward", 0.333, 0.666, 1.000));
    this->rightWheel.addTerm(new fl::Ramp("normalForward", 0.666, 1.000));

    this->fuzzy.addOutputVariable(&rightWheel);

    fl::RuleBlock rules;
    rules.setName("rules");
    rules.setDescription("");
    rules.setEnabled(true);
    rules.setConjunction(AndMethod);
    rules.setDisjunction(OrMethod);
    rules.setImplication(ImplicationMethod);
    rules.setActivation(new fl::General);

    rules.addRule(fl::Rule::parse("if distanceToCenter is veryLeft and angle is veryLeft then leftWheel is middleForward and rightWheel is middleBackward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryLeft and angle is left then leftWheel is slowForward and rightWheel is slowBackward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryLeft and angle is center then leftWheel is normalForward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryLeft and angle is right then leftWheel is middleForward and rightWheel is middleForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryLeft and angle is veryRight then leftWheel is middleForward and rightWheel is normalForward", &fuzzy));
    
    rules.addRule(fl::Rule::parse("if distanceToCenter is left and angle is veryLeft then leftWheel is middleForward and rightWheel is middleBackward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is left and angle is left then leftWheel is slowForward and rightWheel is slowBackward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is left and angle is center then leftWheel is normalForward and rightWheel is middleForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is left and angle is right then leftWheel is slowForward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is left and angle is veryRight then leftWheel is slowForward and rightWheel is normalForward", &fuzzy));
    
    rules.addRule(fl::Rule::parse("if distanceToCenter is center and angle is veryLeft then leftWheel is slowForward and rightWheel is slowBackward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is center and angle is left then leftWheel is middleForward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is center and angle is center then leftWheel is normalForward and rightWheel is normalForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is center and angle is right then leftWheel is slowForward and rightWheel is middleForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is center and angle is veryRight then leftWheel is slowBackward and rightWheel is slowForward", &fuzzy));
    
    rules.addRule(fl::Rule::parse("if distanceToCenter is right and angle is veryLeft then leftWheel is normalForward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is right and angle is left then leftWheel is slowForward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is right and angle is center then leftWheel is middleForward and rightWheel is normalForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is right and angle is right then leftWheel is slowBackward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is right and angle is veryRight then leftWheel is middleBackward and rightWheel is middleForward", &fuzzy));

    rules.addRule(fl::Rule::parse("if distanceToCenter is veryRight and angle is veryLeft then leftWheel is normalForward and rightWheel is middleForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryRight and angle is left then leftWheel is middleForward and rightWheel is middleForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryRight and angle is center then leftWheel is slowForward and rightWheel is normalForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryRight and angle is right then leftWheel is slowBackward and rightWheel is slowForward", &fuzzy));
    rules.addRule(fl::Rule::parse("if distanceToCenter is veryRight and angle is veryRight then leftWheel is middleBackward and rightWheel is middleForward", &fuzzy));
    
    this->fuzzy.addRuleBlock(&rules);
}
//--------------------------------------------------------------------------------------------------------
std::pair<double, double> FuzzyController::getOutputValues(const double d, const double a){
    this->angle.setValue(a);
    this->distanceToCenter.setValue(d);
    
    this->fuzzy.process();

    return std::pair<double, double>(leftWheel.getValue(), rightWheel.getValue());
}
//--------------------------------------------------------------------------------------------------------
/*friend std::ostream & operator << (std::ostream & out, const FuzzyController & f){
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy << ", parallelCount: " << m.parallelCount << ", fitness: " << m.fitness;
    out << "\n\t Positive Points: " << m.positivePoints << ", Points: Vector {";
    for(int i = 0; i < m.pointsInModel.size(); i++){
        out << "\n\t\t [" << i << "]: " << m.pointsInModel[i];
    }
    out << "\n\t }\n       ]";

    return out; 
}*/

//********************************************************************************************************