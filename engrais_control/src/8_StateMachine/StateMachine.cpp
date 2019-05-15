//********************************************************************************************************
#include "StateMachine.h"


//--------------------------------------------------------------------------------------------------------
StateMachine::Transition::Transition(const States & st, const std::pair<double, double> & out){
    nextState = st;
    output = out;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::pair<double, double> StateMachine::makeTransition(const std::pair<Model, Model> & models){ 
    Transition stateTransition(INITIAL, std::pair<double, double> (0,0));

    switch(currentState){
        case INITIAL:
            stateTransition = initialStateRoutine(models);
            break;

        case FORWARD:
            stateTransition = forwardStateRoutine(models);
            break;

        case BACKWARD:
            stateTransition = backwardStateRoutine(models);
            break;

        case LINEAR_STOP:
            stateTransition = linearStopStateRoutine(models);
            break;

        case ANGULAR_STOP:
            stateTransition = angularStopStateRoutine(models);
            break;

        case LEFT_TURN_BEGIN:
            stateTransition = leftTurnBeginStateRoutine(models);
            break;

        case LEFT_TURN_MID:
            stateTransition = leftTurnMidStateRoutine(models);
            break;

        case LEFT_TURN_REMERGE:
            stateTransition = leftTurnRemergeStateRoutine(models);
            break;

        default:
            stateTransition = impossibleStateRoutine(models);

    }

    currentState = stateTransition.nextState;
    return std::pair<double, double> (stateTransition.output.first * MAX_VEL, stateTransition.output.second * MAX_VEL);
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::initialStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "Initial state" << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){    
        std::vector<Point> lPoints = models.first.getPointsInModel();
        std::vector<Point> rPoints = models.second.getPointsInModel();

        if((lPoints.size() >= 2 && lPoints[1].getX() < 0) || (rPoints.size() >= 2 && rPoints[1].getX() < 0))
            return Transition(BACKWARD, std::pair<double, double> (0,0));
        
        else
            return Transition(FORWARD, std::pair<double, double> (0,0));
    }

    else{
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::forwardStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "forward state" << endl;

    lastMovement = FORWARD;

    if(models.first.isPopulated() || models.second.isPopulated()){
        std::vector<Point> lPoints = models.first.getPointsInModel();
        std::vector<Point> rPoints = models.second.getPointsInModel();

        if((lPoints.size() >= 2 && lPoints[1].getX() + BODY_SIZE/2.0 <= -0.5) || (rPoints.size() >= 2 && rPoints[1].getX() + BODY_SIZE/2.0 <= -0.5)){
            if(toTurn == LEFT)
                tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25 * FORWARD, 0.25 * FORWARD));

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));
        }
        else{
            std::pair<double, double> controls = fuzzy.getOutputValues(calculateRatio(models), calculateAngle(models));
            
            return Transition(FORWARD, std::pair<double, double> (controls.first, controls.second));
        }
    }
    else
        return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::backwardStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "backward state" << endl;

    lastMovement = BACKWARD;

    if(models.first.isPopulated() || models.second.isPopulated()){
        std::vector<Point> lPoints = models.first.getPointsInModel();
        std::vector<Point> rPoints = models.second.getPointsInModel();

        if((lPoints.size() >= 2 && lPoints[0].getX() - BODY_SIZE/2.0 >= 0.5) || (rPoints.size() >= 2 && rPoints[0].getX() - BODY_SIZE/2.0 >= 0.5)){
            if(toTurn == LEFT)
                tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25, 0.25));

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));
        }
        else{
            std::pair<double, double> controls = fuzzy.getOutputValues(calculateRatio(models), -calculateAngle(models));
            
            return Transition(BACKWARD, std::pair<double, double> (-controls.first, -controls.second));
        }
    }
    else
        return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::linearStopStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "linear stop state" << endl;

    static std::chrono::time_point<std::chrono::system_clock> oldTime = std::chrono::system_clock::now();
    static double oldDistance = 0;
    static bool first = true;

    if(models.first.isPopulated() || models.second.isPopulated()){
        std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
        double distance = calculateDistance(models);

        std::chrono::duration<double> elapsed_seconds = time - oldTime;
        double deltaDist = distance - oldDistance;

        double x_vel = deltaDist / elapsed_seconds.count();

        oldDistance = distance;
        oldTime = time;

        if(first){
            first = false;
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));
        }
        
        if(!(-0.1 <= x_vel && x_vel <= -0.001 || 0.001 <= x_vel && x_vel <= 0.1))
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));

        else{
            first = true;
            return tAfterStop;
        }
    }

    else{
        first = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::angularStopStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "angular stop state" << endl;

    static std::chrono::time_point<std::chrono::system_clock> oldTime;
    static double oldAngle;
    static bool first = true;

    if(models.first.isPopulated() || models.second.isPopulated()){
        std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now();
        double angle = calculateAngle(models);

        std::chrono::duration<double> elapsed_seconds = time - oldTime;
        double angDist = angle - oldAngle;

        double a_vel = angDist / elapsed_seconds.count();

        oldAngle = angle;
        oldTime = time;

        if(first){
            first = false;
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
        
        if(a_vel < -0.01 )
            return Transition(ANGULAR_STOP, std::pair<double, double> (0.25, -0.25));

        else if(a_vel > 0.01 )
            return Transition(ANGULAR_STOP, std::pair<double, double> (-0.25, 0.25));
        
        else if(-0.01 <= a_vel && a_vel <= -0.001 || 0.001 <= a_vel && a_vel <= 0.01)
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));

        else{
            first = true;
            return tAfterStop;
        }
    }

    else{
        first = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnBeginStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "Left turn begin state" << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){
        double angle = calculateAngle(models);

        if(angle > -PI / 3.5){
            return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25, 0.25));
        }

        else{
            tAfterStop = Transition(LEFT_TURN_MID, std::pair<double, double> (0.25, 0.25));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else
        return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMidStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "Left turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1;
    const bool condition = Sense == 1 ? models.second.isPopulated() : models.first.isPopulated();
    const double intercept = Sense == 1 ? models.second.getIntercept() : models.first.getIntercept();

    static bool firstAssing = true;
    static double savedIntercept;

    //cout << "Sentido: " << Sense << ", condition : " << condition << ", intercept : " << intercept << ", first intercept: " << savedIntercept << ", first assign: " << firstAssing << endl;

    if(firstAssing){
        //cout << "first" << endl;
        firstAssing = false;
        savedIntercept = intercept;

        return Transition(LEFT_TURN_MID, std::pair<double, double> (0.25 * Sense, 0.25 * Sense));
    }
    //cout << "not first" << endl;
    if(condition){
        if(0.5 <= fabs(intercept) && fabs(intercept) <= fabs(savedIntercept * 0.6)){
            firstAssing = true;

            tAfterStop = Transition(LEFT_TURN_REMERGE, std::pair<double, double> (0.25, -0.25));
            return Transition(LINEAR_STOP, std::pair<double, double> (0, 0));
        }
        else{
            if(fabs(savedIntercept) <= fabs(intercept * 0.9))
                savedIntercept = intercept;

            return Transition(LEFT_TURN_MID, std::pair<double, double> (0.25 * Sense, 0.25 * Sense));
        }
    }

    else{
        firstAssing = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnRemergeStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "Left turn remerge state" << endl;
    //cout << "angle: " << calculateAngle(models) << endl;
    if(models.first.isPopulated() || models.second.isPopulated()){
        double angle = calculateAngle(models);

        if(angle < PI / 7.0){
            return Transition(LEFT_TURN_REMERGE, std::pair<double, double> (0.25, -0.25));
        }

        else{
            if(lastMovement == BACKWARD)
                tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));


            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::impossibleStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "impossible state" << endl;

    return Transition(INITIAL, std::pair<double, double> (0,0));
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateRatio(const std::pair<Model, Model> & m){ 
    double lAbs = m.first.isPopulated() ? fabs(m.first.getIntercept()) : DISTANCE_REFERENCE;
    double rAbs = m.second.isPopulated() ? fabs(m.second.getIntercept()) : DISTANCE_REFERENCE;

    double ratio = lAbs < rAbs ? lAbs / rAbs - 1.0 : 1.0 - rAbs / lAbs;

    return ratio;
}
//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateDistance(const std::pair<Model, Model> & m){ 
    double distMean = 0;
    int cont = 0;

    std::vector<Point> lPoints = m.first.getPointsInModel();
    std::vector<Point> rPoints = m.second.getPointsInModel();

    if(lPoints.size() >= 2){
        distMean += (pow(lPoints[0].getX(), 2) + pow(lPoints[0].getY(), 2) < pow(lPoints[1].getX(), 2) + pow(lPoints[1].getY(), 2)) ? lPoints[0].getX() : lPoints[1].getX();
        cont++;
    }

    if(rPoints.size() >= 2){
        distMean += (pow(rPoints[0].getX(), 2) + pow(rPoints[0].getY(), 2) < pow(rPoints[1].getX(), 2) + pow(rPoints[1].getY(), 2)) ? rPoints[0].getX() : rPoints[1].getX();
        cont++;
    }
    
    return distMean / (double)cont;       
}
//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateAngle(const std::pair<Model, Model> & m){ 
    double slopeMean = 0;
    int cont = 0;

    
    if(m.first.isPopulated()){
        slopeMean += m.first.getSlope();
        cont++;
    }

    if(m.second.isPopulated()){
        slopeMean += m.second.getSlope();
        cont++;
    }
    
    return slopeMean == 0 ? 0 : atan(slopeMean / (double)cont);
}


//********************************************************************************************************