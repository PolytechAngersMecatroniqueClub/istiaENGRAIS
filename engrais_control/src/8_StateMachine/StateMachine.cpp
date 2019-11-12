//********************************************************************************************************
#include "StateMachine.h"

using namespace std;
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition::Transition(const States & st, const std::pair<double, double> & out){ //Constructor 
    this->nextState = st; //Stores next state
    this->output = out; //Stores wheels output
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::tuple<double, double, std::vector<Model>> StateMachine::makeTransition(std::vector<Model> & models, double dist){ //Compute a transition from the State machine 
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
            stateTransition = leftTurnMidStateRoutine(models, dist); //Left Turn Mid State
            break;

        case LEFT_TURN_MERGE:
            stateTransition = leftTurnMergeStateRoutine(models); //Left Turn Merge State
            break;

        case RIGHT_TURN_BEGIN:
            stateTransition = rightTurnBeginStateRoutine(models); //Left Turn Begin State
            break;

        case RIGHT_TURN_MID:
            stateTransition = rightTurnMidStateRoutine(models, dist); //Left Turn Mid State
            break;

        case RIGHT_TURN_MERGE:
            stateTransition = rightTurnMergeStateRoutine(models); //Left Turn Merge State
            break;

        case END:
            stateTransition = endStateRoutine(models); //Left Turn Merge State
            break;

        default:
            stateTransition = impossibleStateRoutine(models); //Impossible state as default, if something goes wrong

    }
    
    //std::cout << "Error count: " << errorCount <<", Next State: " << stateTransition.nextState << std::endl << std::endl;

    this->currentState = stateTransition.nextState; //Goes to next state
    return std::tuple<double, double, std::vector<Model>> (stateTransition.output.first * MAX_VEL, stateTransition.output.second * MAX_VEL, models); //Returns wheels command
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::initialStateRoutine(std::vector<Model> & models){ //Initial State 
    cout << "Initial state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //If found some model
        std::pair<Point, Point> lPoints = m.first.getFirstAndLastPoint(); //Left model points
        std::pair<Point, Point> rPoints = m.second.getFirstAndLastPoint(); //Left model points

        if((0 <= lPoints.second.getX() && lPoints.second.getX() < MAX_DBL) || (0 <= rPoints.second.getX() && rPoints.second.getX() < MAX_DBL)) //If positive-most point is negative, then to backwards
            return Transition(FORWARD, std::pair<double, double> (0,0));
        
        else
            return Transition(BACKWARD, std::pair<double, double> (0,0)); //otherwise, goes forward
    }

    else{
        return Transition(INITIAL, std::pair<double, double> (0,0)); //Nothing was found, stands still
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::forwardStateRoutine(std::vector<Model> & models){ //Forward State 
    static int distanceCounter = 0;

    cout << "Forward state" << endl;

    this->lastMovement = FORWARD; //Stores last movement

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;
        double maxDistance = -1000;

        for(int i = 0; i < models.size(); i++){
            std::pair<Point, Point> points = models[i].getFirstAndLastPoint(); //Left model points

            double distance = points.second.getX() != MAX_DBL ? points.second.getX() : -1000.0;

            maxDistance = std::max(maxDistance, distance);
        }

        if(maxDistance <= -(0.8 + BODY_SIZE/2.0) && ++distanceCounter >= 3){ //If positive-most point is smaller than -50cm, robot is at the end of the rows and needs to turn
            if(this->currentRowPos >= 5)
                this->tAfterStop = Transition(END, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == RIGHT)
                this->tAfterStop = Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            distanceCounter = 0;
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(m), calculateAngle(m)); //Calculates fuzzy output
            
            return Transition(FORWARD, std::pair<double, double> (controls.first, controls.second));
        }
    }
    else{
        distanceCounter = 0;
        if(++this->errorCount == 3)
            return Transition(END, std::pair<double, double> (0,0)); //Nothing was found, goes to initial state

        return Transition(FORWARD, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::backwardStateRoutine(std::vector<Model> & models){ //Backward State
    static int distanceCounter = 0;

    cout << "Backward state" << endl;

    this->lastMovement = BACKWARD; //Stores last movement

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;
        double minDistance = 1000.0;

        for(int i = 0; i < models.size(); i++){
            std::pair<Point, Point> points = models[i].getFirstAndLastPoint(); //Left model points

            double distance = points.first.getX() != MAX_DBL ? points.first.getX() : 1000.0;

            minDistance = std::min(minDistance, distance);
        }

        if(minDistance >= (0.8 + BODY_SIZE/2.0) && ++distanceCounter >= 3){ //If positive-most point is smaller than -50cm, robot is at the end of the rows and needs to turn
            if(this->currentRowPos >= 5)
                this->tAfterStop = Transition(END, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition
            
            else if(this->toTurn == RIGHT)
                this->tAfterStop = Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            distanceCounter = 0;
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(m), -calculateAngle(m)); //Calculates fuzzy output
            
            return Transition(BACKWARD, std::pair<double, double> (-controls.first, -controls.second));
        }
    }
    else{
        distanceCounter = 0;
        if(++this->errorCount == 3)
            return Transition(END, std::pair<double, double> (0,0)); //Nothing was found, goes to initial state

        return Transition(BACKWARD, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::linearStopStateRoutine(std::vector<Model> & models){ //Linear Stop State 
    cout << "Linear stop state" << endl;

    static std::chrono::time_point<std::chrono::system_clock> oldTime = std::chrono::system_clock::now(); //Initialize last iteration time
    static double oldDistance = 0; //Initialize last iteration distance
    static bool first = true; //Flag to first Iteration

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now(); //Gets current time
        double distance = calculateDistance(m); //Calculates distance to models

        std::chrono::duration<double> elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double deltaDist = distance - oldDistance; //Variation in the distance during that time

        double x_vel = deltaDist / elapsed_seconds.count(); // Δx / Δt to calculate average velocity in x-coordinate

        oldDistance = distance; //Stores current distance for next iteration
        oldTime = time; //Stores current time for next iteration

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
        if(++this->errorCount == 3){
            first = true;
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(LINEAR_STOP, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::angularStopStateRoutine(std::vector<Model> & models){ //Angular Stop State 
    cout << "Angular stop state" << endl;

    static std::chrono::time_point<std::chrono::system_clock> oldTime; //Initialize last iteration time
    static double oldAngle; //Initialize last iteration angle
    static bool first = true; //Flag to first Iteration

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        std::chrono::time_point<std::chrono::system_clock> time = std::chrono::system_clock::now(); //Gets current time
        double angle = calculateAngle(m); //Calculates angle

        std::chrono::duration<double> elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double angDist = angle - oldAngle; //Variation in the angle during that time

        double a_vel = angDist / elapsed_seconds.count(); // Δθ/ Δt to calculate average angular velocity

        oldAngle = angle; //Stores current angle for next iteration
        oldTime = time; //Stores current time for next iteration

        if(first){ //If it's the first iteration
            first = false;
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0)); //Does nothing and wait next iteration
        }
        
        if(a_vel < -0.05) //If velocity is too big, sends couter command to stop faster
            return Transition(ANGULAR_STOP, std::pair<double, double> (0.15, -0.15));

        else if(a_vel > 0.05)
            return Transition(ANGULAR_STOP, std::pair<double, double> (-0.15, 0.15));
        
        else if(-0.05 <= a_vel && a_vel <= -0.001 || 0.001 <= a_vel && a_vel <= 0.05)
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));

        else{
            first = true; //Sets flag to true again
            return this->tAfterStop;
        }
    }

    else{ //If nothing found
        if(++this->errorCount == 3){
            std::cout << "Error" << std::endl << std::endl;
            first = true; //Sets flag to true again
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnBeginStateRoutine(std::vector<Model> & models){ //Left Turn Begin State 
    cout << "Left turn begin state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        double angle = calculateAngle(m); //Calculates angle

        if(angle < PI / 4.5){ //While robot's angle is smaller than 50º
            return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0.20, -0.20)); //Keeps turning
        }

        else{ //Otherwise, stop and go forward
            this->tAfterStop = Transition(LEFT_TURN_MID, std::pair<double, double> (0, 0));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMidStateRoutine(std::vector<Model> & models, double dist){ //Left Turn Mid State 
    cout << "Left turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1; //Check which direction the robot has to go

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1 - Sense] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2 - Sense] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //If the model exists
        this->errorCount = 0;
        
        if(fabs(m.first.getIntercept() + m.second.getIntercept()) <= 0.6){
            if(Sense == 1){
                for(int i = models.size() - 1; i > 0; i--){
                    models[i] = models[i - 1];
                }

                models[0] = Model(models[1].getSlope(), models[1].getIntercept() + (dist * sqrt(pow(models[1].getSlope(), 2) + 1)));
            }
            else{
                for(int i = 0; i < models.size() - 1; i++){
                    models[i] = models[i + 1];
                }

                models[models.size() - 1] = Model(models[models.size() - 2].getSlope(), models[models.size() - 2].getIntercept() - (dist * sqrt(pow(models[models.size() - 2].getSlope(), 2) + 1)));
            }

            if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_MERGE, std::pair<double, double> (0, 0)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        return Transition(LEFT_TURN_MID, std::pair<double, double> (-Sense * 0.3, -Sense * 0.3));
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(LEFT_TURN_MID, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMergeStateRoutine(std::vector<Model> & models){ //Left Turn Merge State 
    cout << "Left turn remerge state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        double angle = calculateAngle(m); //Calculates angle

        if(angle > PI/10.0){ //While robot's angle is bigger than -30º, keeps turning
            return Transition(LEFT_TURN_MERGE, std::pair<double, double> (-0.20, 0.20));
        }

        else{ //Else, merge to lines using fuzzy controllers

            if(lastMovement == BACKWARD) //Goes the opposite direction than before
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));


            this->toTurn = RIGHT;
            this->currentRowPos++;

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(LEFT_TURN_MERGE, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnBeginStateRoutine(std::vector<Model> & models){ //Left Turn Begin State 
    cout << "Right turn begin state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        double angle = calculateAngle(m); //Calculates angle

        if(angle > -PI / 4.5){ //While robot's angle is smaller than 50º
            return Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (-0.20, 0.20)); //Keeps turning
        }

        else{ //Otherwise, stop and go forward
            this->tAfterStop = Transition(RIGHT_TURN_MID, std::pair<double, double> (0, 0));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnMidStateRoutine(std::vector<Model> & models, double dist){ //Left Turn Mid State 
    cout << "Right turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1; //Check which direction the robot has to go

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1 + Sense] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2 + Sense] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //If the model exists
        this->errorCount = 0;
        
        if(fabs(m.first.getIntercept() + m.second.getIntercept()) <= 0.6){
            if(Sense == -1){
                for(int i = models.size() - 1; i > 0; i--){
                    models[i] = models[i - 1];
                }

                models[0] = Model(models[1].getSlope(), models[1].getIntercept() + (dist * sqrt(pow(models[1].getSlope(), 2) + 1)));
            }
            else{
                for(int i = 0; i < models.size() - 1; i++){
                    models[i] = models[i + 1];
                }

                models[models.size() - 1] = Model(models[models.size() - 2].getSlope(), models[models.size() - 2].getIntercept() - (dist * sqrt(pow(models[models.size() - 2].getSlope(), 2) + 1)));
            }

            if(this->toTurn == RIGHT) //If it needs to go Left
                this->tAfterStop = Transition(RIGHT_TURN_MERGE, std::pair<double, double> (0, 0)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        return Transition(RIGHT_TURN_MID, std::pair<double, double> (-Sense * 0.3, -Sense * 0.3));
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(RIGHT_TURN_MID, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnMergeStateRoutine(std::vector<Model> & models){ //Left Turn Merge State 
    cout << "Right turn remerge state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == numOfLines ? models[numOfLines/2 - 1] : Model();
    m.second = models.size() == numOfLines ? models[numOfLines/2] : Model();

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0;

        double angle = calculateAngle(m); //Calculates angle

        if(angle < -PI/10.0){ //While robot's angle is bigger than -30º, keeps turning
            return Transition(RIGHT_TURN_MERGE, std::pair<double, double> (0.20, -0.20));
        }

        else{ //Else, merge to lines using fuzzy controllers

            if(lastMovement == BACKWARD) //Goes the opposite direction than before
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));

            this->toTurn = LEFT;
            this->currentRowPos++;

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 3){
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0));
        }

        return Transition(RIGHT_TURN_MERGE, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::endStateRoutine(std::vector<Model> & models){ //End State 
    cout << "End State" << endl;
    return Transition(END, std::pair<double, double> (0,0)); //End test
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::impossibleStateRoutine(std::vector<Model> & models){ //Impossible State 
    cout << "Impossible State" << endl;
    return Transition(IMPOSSIBLE, std::pair<double, double> (0,0)); 
}

//################################################################################################

//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateRatio(const std::pair<Model, Model> & m){ //Calculate fuzzy ratio input 
    double lAbs = m.first.isPopulated() ? fabs(m.first.getIntercept()) : DISTANCE_REFERENCE; //|b_left| or 1.5
    double rAbs = m.second.isPopulated() ? fabs(m.second.getIntercept()) : DISTANCE_REFERENCE; //|b_right| or 1.5

    double ratio = lAbs <= rAbs ? lAbs / rAbs - 1.0 : (rAbs / lAbs - 1.0) * (-1.0) ; //(|b_min| / |b_max| - 1) [* -1.0 if right from center]

    return ratio;
}
//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateDistance(const std::pair<Model, Model> & m){ //Calculate x-distance from robot to models 
    double distMean = 0; //Initialize sum
    int cont = 0;

    std::pair<Point, Point> lPoints = m.first.getFirstAndLastPoint(); //Left model points
    std::pair<Point, Point> rPoints = m.second.getFirstAndLastPoint(); //Left model points

    if(lPoints.first.isAssigned() && lPoints.second.isAssigned()){ //If left points is populated
        distMean += (pow(lPoints.first.getX(), 2) + pow(lPoints.first.getY(), 2) < pow(lPoints.second.getX(), 2) + pow(lPoints.second.getY(), 2)) ? lPoints.first.getX() : lPoints.second.getX(); //Gets x-distance to closest point
        cont++;
    }

    if(rPoints.first.isAssigned() && rPoints.second.isAssigned()){ //If right points is populated
        distMean += (pow(rPoints.first.getX(), 2) + pow(rPoints.first.getY(), 2) < pow(rPoints.second.getX(), 2) + pow(rPoints.second.getY(), 2)) ? rPoints.first.getX() : rPoints.second.getX(); //Gets x-distance to closest point
        cont++;
    }
    
    return distMean == 0 ? 0 : distMean / (double)cont; //Average distance
}
//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateAngle(const std::pair<Model, Model> & m){ //Calculate angle from models and robot x-axis 
    double slopeMean = 0; //Initialize slope mean
    int cont = 0;

    
    if(m.first.isPopulated()){ //If left model exists
        slopeMean += m.first.getSlope(); //Sum it to the mean
        cont++;
    }

    if(m.second.isPopulated()){ //If right model exists
        slopeMean += m.second.getSlope(); //Sum it to the mean
        cont++;
    }
    
    return slopeMean == 0 ? 0 : atan(slopeMean / (double)cont); //Return mean angle
}

//********************************************************************************************************