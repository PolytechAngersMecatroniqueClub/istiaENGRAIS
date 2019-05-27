//********************************************************************************************************
#include "StateMachine.h"

//using namespace std;
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition::Transition(const States & st, const std::pair<double, double> & out){ //Constructor 
    this->nextState = st; //Stores next state
    this->output = out; //Stores wheels output
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::pair<double, double> StateMachine::makeTransition(const std::pair<Model, Model> & models){ //Compute a transition from the State machine 
    Transition stateTransition(INITIAL, std::pair<double, double> (0,0)); //Initialize Transition

    switch(this->currentState){
        case INITIAL:
            stateTransition = initialStateRoutine(models); //Initial State
            break;

        case FORWARD:
            stateTransition = forwardStateRoutine(models); //Forward State
            break;

        case BACKWARD:
            stateTransition = backwardStateRoutine(models); //Backward State
            break;

        case LINEAR_STOP:
            stateTransition = linearStopStateRoutine(models); //Linear Stop State
            break;

        case ANGULAR_STOP:
            stateTransition = angularStopStateRoutine(models); //Angular Stop State
            break;

        case LEFT_TURN_BEGIN:
            stateTransition = leftTurnBeginStateRoutine(models); //Left Turn Begin State
            break;

        case LEFT_TURN_MID:
            stateTransition = leftTurnMidStateRoutine(models); //Left Turn Mid State
            break;

        case LEFT_TURN_MERGE:
            stateTransition = leftTurnMergeStateRoutine(models); //Left Turn Merge State
            break;

        default:
            stateTransition = impossibleStateRoutine(models); //Impossible state as default, if something goes wrong

    }

    this->currentState = stateTransition.nextState; //Goes to next state
    return std::pair<double, double> (stateTransition.output.first * MAX_VEL, stateTransition.output.second * MAX_VEL); //Returns wheels command
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::initialStateRoutine(const std::pair<Model, Model> & models){ //Initial State 
    //cout << "Initial state" << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){ //If found some model
        std::vector<Point> lPoints = models.first.getPointsInModel(); //Left model points
        std::vector<Point> rPoints = models.second.getPointsInModel(); //Left model points

        if((lPoints.size() >= 2 && lPoints[1].getX() < 0) || (rPoints.size() >= 2 && rPoints[1].getX() < 0)) //If positive-most point is negative, then to backwards
            return Transition(BACKWARD, std::pair<double, double> (0,0));
        
        else
            return Transition(FORWARD, std::pair<double, double> (0,0)); //otherwise, goes forward
    }

    else{
        return Transition(INITIAL, std::pair<double, double> (0,0)); //Nothing was found, stands still
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::forwardStateRoutine(const std::pair<Model, Model> & models){ //Forward State 
    //cout << "forward state" << endl;

    this->lastMovement = FORWARD; //Stores last movement

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found

        std::vector<Point> lPoints = models.first.getPointsInModel(); //Left model points
        std::vector<Point> rPoints = models.second.getPointsInModel(); //Left model points

        if((lPoints.size() >= 2 && lPoints[1].getX() + BODY_SIZE/2.0 <= -0.5) || (rPoints.size() >= 2 && rPoints[1].getX() + BODY_SIZE/2.0 <= -0.5)){ //If positive-most point is smaller than -50cm, robot is at the end of the rows and needs to turn
            
            if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25, 0.25)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(models), calculateAngle(models)); //Calculates fuzzy output
            
            return Transition(FORWARD, std::pair<double, double> (controls.first, controls.second));
        }
    }
    else
        return Transition(INITIAL, std::pair<double, double> (0,0)); //Nothing was found, goes to initial state
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::backwardStateRoutine(const std::pair<Model, Model> & models){ //Backward State 
    //cout << "backward state" << endl;

    this->lastMovement = BACKWARD; //Stores last movement

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found

        std::vector<Point> lPoints = models.first.getPointsInModel(); //Left model points
        std::vector<Point> rPoints = models.second.getPointsInModel(); //Left model points

        if((lPoints.size() >= 2 && lPoints[0].getX() - BODY_SIZE/2.0 >= 0.5) || (rPoints.size() >= 2 && rPoints[0].getX() - BODY_SIZE/2.0 >= 0.5)){ //If negative-most point is bigger than 50cm, robot is at the end of the rows and needs to turn
            
            if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25, 0.25)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(models), -calculateAngle(models)); //Calculates fuzzy output
            
            return Transition(BACKWARD, std::pair<double, double> (-controls.first, -controls.second));
        }
    }
    else
        return Transition(INITIAL, std::pair<double, double> (0,0)); //Nothing was found, goes to initial state
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::linearStopStateRoutine(const std::pair<Model, Model> & models){ //Linear Stop State
    //cout << "linear stop state" << endl;

    static std::chrono::time_point<std::chrono::system_clock> oldTime = std::chrono::system_clock::now(); //Initialize last iteration time
    static double oldDistance = 0; //Initialize last iteration distance
    static bool first = true; //Flag to first Iteration

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found

        std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now(); //Gets current time
        double distance = calculateDistance(models); //Calculates distance to models

        std::chrono::duration<double> elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double deltaDist = distance - oldDistance; //Variation in the distance during that time

        double x_vel = deltaDist / elapsed_seconds.count(); // Δx / Δt to calculate average velocity in x-coordinate

        oldDistance = distance; //Stores current time for next iteration
        oldTime = time; //Stores current distance for next iteration

        if(first){ //If it's the first iteration
            first = false;
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Does nothing and wait next iteration
        }
        
        if(!(-0.1 <= x_vel && x_vel <= -0.001 || 0.001 <= x_vel && x_vel <= 0.1)) //If velocity is too big, sends couter command to stop faster
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));

        else{
            first = true; //Sets flag to true again
            return this->tAfterStop;
        }
    }

    else{ //If nothing found
        first = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::angularStopStateRoutine(const std::pair<Model, Model> & models){ //Angular Stop State 
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
            return this->tAfterStop;
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
            this->tAfterStop = Transition(LEFT_TURN_MID, std::pair<double, double> (0.25, 0.25));

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

            this->tAfterStop = Transition(LEFT_TURN_MERGE, std::pair<double, double> (0.25, -0.25));
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
StateMachine::Transition StateMachine::leftTurnMergeStateRoutine(const std::pair<Model, Model> & models){ 
    //cout << "Left turn remerge state" << endl;
    //cout << "angle: " << calculateAngle(models) << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){
        double angle = calculateAngle(models);

        if(angle < PI / 6.0){
            return Transition(LEFT_TURN_MERGE, std::pair<double, double> (0.25, -0.25));
        }

        else{
            if(lastMovement == BACKWARD)
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));


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

//################################################################################################

//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateRatio(const std::pair<Model, Model> & m){ 
    double lAbs = m.first.isPopulated() ? fabs(m.first.getIntercept()) : DISTANCE_REFERENCE;
    double rAbs = m.second.isPopulated() ? fabs(m.second.getIntercept()) : DISTANCE_REFERENCE;

    double ratio = lAbs <= rAbs ? lAbs / rAbs - 1.0 : (rAbs / lAbs - 1.0) * (-1.0) ;

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