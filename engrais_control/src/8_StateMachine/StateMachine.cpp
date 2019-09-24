//********************************************************************************************************
#include "StateMachine.h"


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
    return std::pair<double, double> (stateTransition.output.first * this->MAX_VEL, stateTransition.output.second * this->MAX_VEL); //Returns wheels command
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

        if((lPoints.size() >= 2 && lPoints[1].getX() + this->BODY_SIZE/2.0 <= -0.5) || (rPoints.size() >= 2 && rPoints[1].getX() + this->BODY_SIZE/2.0 <= -0.5)){ //If positive-most point is smaller than -50cm, robot is at the end of the rows and needs to turn
            
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

        if((lPoints.size() >= 2 && lPoints[0].getX() - this->BODY_SIZE/2.0 >= 0.5) || (rPoints.size() >= 2 && rPoints[0].getX() - this->BODY_SIZE/2.0 >= 0.5)){ //If negative-most point is bigger than 50cm, robot is at the end of the rows and needs to turn
            
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

    static ros::Time oldTime; //Initialize last iteration time
    static double oldDistance = 0; //Initialize last iteration distance
    static bool first = true; //Flag to first Iteration

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found

        ros::Time time = ros::Time::now(); //Gets current time
        double distance = calculateDistance(models); //Calculates distance to models

        ros::Duration elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double deltaDist = distance - oldDistance; //Variation in the distance during that time

        double x_vel = deltaDist / elapsed_seconds.toSec(); // Δx / Δt to calculate average velocity in x-coordinate

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
        first = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::angularStopStateRoutine(const std::pair<Model, Model> & models){ //Angular Stop State 
    //cout << "angular stop state" << endl;

    static ros::Time oldTime; //Initialize last iteration time
    static double oldAngle = 0; //Initialize last iteration angle
    static bool first = true; //Flag to first Iteration

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found
        ros::Time time = ros::Time::now(); //Gets current time
        double angle = calculateAngle(models); //Calculates angle

        ros::Duration elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double angDist = angle - oldAngle; //Variation in the angle during that time

        double a_vel = angDist / elapsed_seconds.toSec(); // Δθ/ Δt to calculate average angular velocity

        oldAngle = angle; //Stores current angle for next iteration
        oldTime = time; //Stores current time for next iteration

        if(first){ //If it's the first iteration
            first = false;
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0)); //Does nothing and wait next iteration
        }
        
        if(a_vel < -0.01) //If velocity is too big, sends couter command to stop faster
            return Transition(ANGULAR_STOP, std::pair<double, double> (0.25, -0.25));

        else if(a_vel > 0.01)
            return Transition(ANGULAR_STOP, std::pair<double, double> (-0.25, 0.25));
        
        else if(-0.01 <= a_vel && a_vel <= -0.001 || 0.001 <= a_vel && a_vel <= 0.01)
            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));

        else{
            first = true; //Sets flag to true again
            return this->tAfterStop;
        }
    }

    else{ //If nothing found
        first = true; //Sets flag to true again
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnBeginStateRoutine(const std::pair<Model, Model> & models){ //Left Turn Begin State 
    //cout << "Left turn begin state" << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found
        double angle = calculateAngle(models); //Calculates angle

        if(angle > -PI / 3.5){ //While robot's angle is smaller than 50º
            return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (-0.25, 0.25)); //Keeps turning
        }

        else{ //Otherwise, stop and go forward
            this->tAfterStop = Transition(LEFT_TURN_MID, std::pair<double, double> (0.25, 0.25));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else //Nothing was found
        return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMidStateRoutine(const std::pair<Model, Model> & models){ //Left Turn Mid State 
    //cout << "Left turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1; //Check which direction the robot has to go
    const bool condition = Sense == 1 ? models.second.isPopulated() : models.first.isPopulated(); //Checks if the used model is populated
    const double intercept = Sense == 1 ? models.second.getIntercept() : models.first.getIntercept(); //Gets used model's Intercept

    static bool firstAssing = true; //Flag iteration
    static double savedIntercept;

    if(firstAssing){ //For first iteration
        firstAssing = false;
        savedIntercept = intercept; //Save intercept

        return Transition(LEFT_TURN_MID, std::pair<double, double> (0.25 * Sense, 0.25 * Sense)); //Keeps going forward
    }

    if(condition){ //If the model exists
        if(0.5 <= fabs(intercept) && fabs(intercept) <= fabs(savedIntercept * 0.6)){ //If intercept jumped from a high value to something greater than 0.5, this means that the robot changed lane
            firstAssing = true; //Reset flag

            this->tAfterStop = Transition(LEFT_TURN_MERGE, std::pair<double, double> (0.25, -0.25)); //Begin to turn to merge
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0));
        }
        else{
            if(fabs(savedIntercept) <= fabs(intercept * 0.9)) //If intercept grew more than 10%, save the new value
                savedIntercept = intercept;

            return Transition(LEFT_TURN_MID, std::pair<double, double> (0.25 * Sense, 0.25 * Sense)); //Goes forward or backward
        }
    }

    else{ //Nothing was found
        firstAssing = true;
        return Transition(INITIAL, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMergeStateRoutine(const std::pair<Model, Model> & models){ //Left Turn Merge State 
    //cout << "Left turn remerge state" << endl;
    //cout << "angle: " << calculateAngle(models) << endl;

    if(models.first.isPopulated() || models.second.isPopulated()){ //Something was found
        double angle = calculateAngle(models); //Calculates angle

        if(angle < PI / 6.0){ //While robot's angle is bigger than -30º, keeps turning
            return Transition(LEFT_TURN_MERGE, std::pair<double, double> (0.25, -0.25));
        }

        else{ //Else, merge to lines using fuzzy controllers

            if(lastMovement == BACKWARD) //Goes the opposite direction than before
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));


            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else //Nothing was found
        return Transition(INITIAL, std::pair<double, double> (0,0));
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::impossibleStateRoutine(const std::pair<Model, Model> & models){ //Impossible State 
    std::cout << "State Machine entered impossible state, please verify" << std::endl; //Error code to debug, something very wrong happened
    exit(-10);

    return Transition(IMPOSSIBLE, std::pair<double, double> (0,0)); 
}

//################################################################################################

//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateRatio(const std::pair<Model, Model> & m){ //Calculate fuzzy ratio input 
    double lAbs = m.first.isPopulated() ? fabs(m.first.getIntercept()) : this->DISTANCE_REFERENCE; //|b_left| or 1.5
    double rAbs = m.second.isPopulated() ? fabs(m.second.getIntercept()) : this->DISTANCE_REFERENCE; //|b_right| or 1.5

    double ratio = lAbs <= rAbs ? lAbs / rAbs - 1.0 : (rAbs / lAbs - 1.0) * (-1.0) ; //(|b_min| / |b_max| - 1) [* -1.0 if right from center]

    return ratio;
}
//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateDistance(const std::pair<Model, Model> & m){ //Calculate x-distance from robot to models 
    double distMean = 0; //Initialize sum
    int cont = 0;

    std::vector<Point> lPoints = m.first.getPointsInModel(); //Points in left model
    std::vector<Point> rPoints = m.second.getPointsInModel(); //Points in right model

    if(lPoints.size() >= 2){ //If left points is populated
        distMean += (pow(lPoints[0].getX(), 2) + pow(lPoints[0].getY(), 2) < pow(lPoints[1].getX(), 2) + pow(lPoints[1].getY(), 2)) ? lPoints[0].getX() : lPoints[1].getX(); //Gets x-distance to closest point
        cont++;
    }

    if(rPoints.size() >= 2){ //If right points is populated
        distMean += (pow(rPoints[0].getX(), 2) + pow(rPoints[0].getY(), 2) < pow(rPoints[1].getX(), 2) + pow(rPoints[1].getY(), 2)) ? rPoints[0].getX() : rPoints[1].getX(); //Gets x-distance to closest point
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
