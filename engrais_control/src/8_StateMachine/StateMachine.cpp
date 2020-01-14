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
std::pair<double, double> StateMachine::makeTransition(std::vector<Model> & models, const double dist){ //Compute a transition from the State machine 
    Transition stateTransition(INITIAL, std::pair<double, double> (0,0)); //Initialize Transition
    //Utility::printVector(models);
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
            stateTransition = rightTurnBeginStateRoutine(models); //Right Turn Begin State
            break;

        case RIGHT_TURN_MID:
            stateTransition = rightTurnMidStateRoutine(models, dist); //Right Turn Mid State
            break;

        case RIGHT_TURN_MERGE:
            stateTransition = rightTurnMergeStateRoutine(models); //Right Turn Merge State
            break;

        case END:
            stateTransition = endStateRoutine(models); //End State
            break;

        default:
            stateTransition = impossibleStateRoutine(models); //Impossible state as default, if something goes wrong

    }
    
    std::cout << "Error count: " << errorCount << ", Current State: " << currentState << ", Next State: " << stateTransition.nextState << std::endl << std::endl;

    this->currentState = stateTransition.nextState; //Goes to next state
    return stateTransition.output; //Returns wheels command
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::initialStateRoutine(std::vector<Model> & models){ //Initial State 
    //cout << "Initial state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Pick the model that is closest to the left
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Pick the model that is closest to the right

    if(m.first.isPopulated() || m.second.isPopulated()){ //If found some model
        std::pair<Point, Point> lPoints = m.first.getFirstAndLastPoint(); //Left model points
        std::pair<Point, Point> rPoints = m.second.getFirstAndLastPoint(); //Right model points

        if((0 <= lPoints.second.getX() && lPoints.second.getX() < MAX_DBL) || (0 <= rPoints.second.getX() && rPoints.second.getX() < MAX_DBL)) //If positive-most point is positive, then to forwards
            return Transition(FORWARD, std::pair<double, double> (0,0));
        
        else
            return Transition(BACKWARD, std::pair<double, double> (0,0)); //otherwise, goes backwards
    }

    else{
        return Transition(INITIAL, std::pair<double, double> (0,0)); //Nothing was found, stand still
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::forwardStateRoutine(std::vector<Model> & models){ //Forward State 
    //cout << "Forward state" << endl;

    static int distanceCounter = 0; //Counts number of times the robot is too far from last plant

    this->lastMovement = FORWARD; //Stores last movement

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset error count
        double maxDistance = -1000; //Default max distance

        for(int i = 0; i < models.size(); i++){ //Each model
            std::pair<Point, Point> points = models[i].getFirstAndLastPoint(); //Models points

            double distance = points.second.getX() != MAX_DBL ? points.second.getX() : -1000.0; //Get positive-most point X-coordinate

            maxDistance = std::max(maxDistance, distance); //Selects maximum X
        }

        if(maxDistance <= -(0.9 + this->BODY_SIZE/2.0) && ++distanceCounter >= 3){ //If positive-most point is smaller than -90 cm and this occurred 3 times in a row, robot is at the end of the rows and needs to turn.
        
            if(this->currentRowPos >= this->NUM_OF_TIMES_TURN) //If it already turned 
                this->tAfterStop = Transition(END, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == RIGHT) //If it needs to go right
                this->tAfterStop = Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            distanceCounter = 0; //Reset distance counter
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(m), calculateAngle(m)); //Calculates fuzzy output
            
            return Transition(FORWARD, std::pair<double, double> (MAX_VEL*controls.first, MAX_VEL*controls.second));
        }
    }

    else{ //If nothing was found
        distanceCounter = 0; //Reset distance counter
        if(++this->errorCount == 20) //Increment error counter
            return Transition(END, std::pair<double, double> (0,0)); //If 3 errors in a row, stop

        return Transition(FORWARD, std::pair<double, double> (0,0)); //Continue
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::backwardStateRoutine(std::vector<Model> & models){ //Backward State
    //cout << "Backward state" << endl;

    static int distanceCounter = 0; //Counts number of times the robot is too far from last plant

    this->lastMovement = BACKWARD; //Stores last movement

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset error count
        double minDistance = 1000.0; //Default min distance

        for(int i = 0; i < models.size(); i++){ //Each model
            std::pair<Point, Point> points = models[i].getFirstAndLastPoint(); //Model points

            double distance = points.first.getX() != MAX_DBL ? points.first.getX() : 1000.0; //Get negative-most point X-coordinate

            minDistance = std::min(minDistance, distance); //Selects minimum X
        }

        if(minDistance >= (0.9 + BODY_SIZE/2.0) && ++distanceCounter >= 3){ //If negative-most point is greater than 90 cm and this occurred 3 times in a row, robot is at the end of the rows and needs to turn.
            
            if(this->currentRowPos >= this->NUM_OF_TIMES_TURN) //If it already turned 
                this->tAfterStop = Transition(END, std::pair<double, double> (0, 0)); //Sets Stop State transition

            else if(this->toTurn == LEFT) //If it needs to go Left
                this->tAfterStop = Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition
            
            else if(this->toTurn == RIGHT) //If it needs to go right
                this->tAfterStop = Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (0, 0)); //Sets Stop State transition

            distanceCounter = 0; //Reset distance counter
            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        else{
            std::pair<double, double> controls = this->fuzzy.getOutputValues(calculateRatio(m), -calculateAngle(m)); //Calculates fuzzy output
            
            return Transition(BACKWARD, std::pair<double, double> (-MAX_VEL*controls.first, -MAX_VEL*controls.second));
        }
    }
    else{ //If nothing was found
        distanceCounter = 0; //Reset distance counter
        if(++this->errorCount == 20) //Increment error counter
            return Transition(END, std::pair<double, double> (0,0)); //If 3 errors in a row, stop

        return Transition(BACKWARD, std::pair<double, double> (0,0)); //Continue
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::linearStopStateRoutine(std::vector<Model> & models){ //Linear Stop State 
    //cout << "Linear stop state" << endl;

    static ros::Time oldTime = ros::Time::now(); //Initialize last iteration time
    static double oldDistance = 0; //Initialize last iteration distance
    static bool first = true; //Flag to first Iteration

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset Error count

        ros::Time time = ros::Time::now(); //Gets current time
        double distance = calculateDistance(m); //Calculates distance to models

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
        if(++this->errorCount == 20){ //Increment Err counter
            first = true;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Continue
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::angularStopStateRoutine(std::vector<Model> & models){ //Angular Stop State 
    //cout << "Angular stop state" << endl;

    static ros::Time oldTime = ros::Time::now(); //Initialize last iteration time
    static double oldAngle; //Initialize last iteration angle
    static bool first = true; //Flag to first Iteration

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset Err counter

        ros::Time time = ros::Time::now(); //Gets current time
        double angle = calculateAngle(m); //Calculates angle

        ros::Duration elapsed_seconds = time - oldTime; //Elapsed time between now and the last iteration
        double angDist = angle - oldAngle; //Variation in the angle during that time

        double a_vel = angDist / elapsed_seconds.toSec(); // Δθ/ Δt to calculate average angular velocity

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
        if(++this->errorCount == 20){ //Increment error
            std::cout << "Error" << std::endl << std::endl;
            first = true; //Sets flag to true again
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnBeginStateRoutine(std::vector<Model> & models){ //Left Turn Begin State 
    //cout << "Left turn begin state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset err counter

        double angle = calculateAngle(m); //Calculates angle

        if(angle < PI / 4.5){ //While robot's angle is smaller than 40º
            return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0.20, -0.20)); //Keeps turning
        }

        else{ //Otherwise, stop and go forward
            this->tAfterStop = Transition(LEFT_TURN_MID, std::pair<double, double> (0, 0));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMidStateRoutine(std::vector<Model> & models, const double dist){ //Left Turn Mid State 
    //cout << "Left turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1; //Check which direction the robot has to go

    std::pair<Model, Model> m; 

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1 - Sense] : Model(); //Gets second closest right or second closest left model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - Sense] : Model(); //Gests closest right or closest left model

    if(m.first.isPopulated() || m.second.isPopulated()){ //If the model exists
        this->errorCount = 0; //Increment err count
        
        if(fabs(m.first.getIntercept() + m.second.getIntercept()) <= 0.6){ //If |br + bl| <= 0.6, robot is approximately at the middle of the 2 rows

            if(Sense == 1){ //If it moved forward
                for(int i = models.size() - 1; i > 0; i--){ //Shift all models to the index below
                    models[i] = models[i - 1];
                }

                models[0] = Model(models[1].getSlope(), models[1].getIntercept() + (dist * sqrt(pow(models[1].getSlope(), 2) + 1))); //Calculate first model
            }
            else{ //If it moved backward
                for(int i = 0; i < models.size() - 1; i++){ //Shift all models to the index above
                    models[i] = models[i + 1];
                }

                models[models.size() - 1] = Model(models[models.size() - 2].getSlope(), models[models.size() - 2].getIntercept() - (dist * sqrt(pow(models[models.size() - 2].getSlope(), 2) + 1))); //Calculate last model
            }

            this->tAfterStop = Transition(LEFT_TURN_MERGE, std::pair<double, double> (0, 0)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }

        return Transition(LEFT_TURN_MID, std::pair<double, double> (-Sense * 0.3, -Sense * 0.3)); //Continue linear motion
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::leftTurnMergeStateRoutine(std::vector<Model> & models){ //Left Turn Merge State 
    //cout << "Left turn remerge state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Increment err count

        double angle = calculateAngle(m); //Calculates angle

        if(angle > PI/10.0){ //While robot's angle is bigger than -18º, keeps turning
            return Transition(LEFT_TURN_MERGE, std::pair<double, double> (-0.20, 0.20));
        }

        else{ //Else, merge to lines using fuzzy controllers

            if(lastMovement == BACKWARD) //Goes the opposite direction than before
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));


            this->toTurn = RIGHT; //Sets next turn
            this->currentRowPos++; //Increment current row

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnBeginStateRoutine(std::vector<Model> & models){ //Left Turn Begin State 
    //cout << "Right turn begin state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Reset err counter

        double angle = calculateAngle(m); //Calculates angle

        if(angle > -PI / 4.5){ //While robot's angle is bigger than -50º
            return Transition(RIGHT_TURN_BEGIN, std::pair<double, double> (-0.20, 0.20)); //Keeps turning
        }

        else{ //Otherwise, stop and go forward
            this->tAfterStop = Transition(RIGHT_TURN_MID, std::pair<double, double> (0, 0));

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnMidStateRoutine(std::vector<Model> & models, const double dist){ //Left Turn Mid State 
    //cout << "Right turn mid state" << endl;

    const int Sense = lastMovement == FORWARD ? 1 : -1; //Check which direction the robot has to go

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1 + Sense] : Model(); //Gets second closest right or second closest left model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 + Sense] : Model(); //Gests closest right or closest left model

    if(m.first.isPopulated() || m.second.isPopulated()){ //If the model exists
        this->errorCount = 0; //Increment err count
        
        if(fabs(m.first.getIntercept() + m.second.getIntercept()) <= 0.6){ //If |br + bl| <= 0.6, robot is approximately at the middle of the 2 rows
            
            if(Sense == -1){ //If it moved Backward
                for(int i = models.size() - 1; i > 0; i--){ //Shift all models to the index below
                    models[i] = models[i - 1];
                }

                models[0] = Model(models[1].getSlope(), models[1].getIntercept() + (dist * sqrt(pow(models[1].getSlope(), 2) + 1))); //Calculate first model
            }
            else{ //If it moved backward
                for(int i = 0; i < models.size() - 1; i++){ //Shift all models to the index above
                    models[i] = models[i + 1];
                }

                models[models.size() - 1] = Model(models[models.size() - 2].getSlope(), models[models.size() - 2].getIntercept() - (dist * sqrt(pow(models[models.size() - 2].getSlope(), 2) + 1))); //Calculate last model
            }

            this->tAfterStop = Transition(RIGHT_TURN_MERGE, std::pair<double, double> (0, 0)); //Sets Stop State transition

            return Transition(LINEAR_STOP, std::pair<double, double> (0,0)); //Stops robot
        }
        return Transition(RIGHT_TURN_MID, std::pair<double, double> (-Sense * 0.3, -Sense * 0.3)); //Continue linear motion
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::rightTurnMergeStateRoutine(std::vector<Model> & models){ //Left Turn Merge State 
    //cout << "Right turn remerge state" << endl;

    std::pair<Model, Model> m;

    m.first = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2 - 1] : Model(); //Left Model
    m.second = models.size() == this->NUM_OF_LINES ? models[this->NUM_OF_LINES/2] : Model(); //Right Model

    if(m.first.isPopulated() || m.second.isPopulated()){ //Something was found
        this->errorCount = 0; //Increment err count

        double angle = calculateAngle(m); //Calculates angle

        if(angle < -PI/10.0){ //While robot's angle is smaller than -18º, keeps turning
            return Transition(RIGHT_TURN_MERGE, std::pair<double, double> (0.20, -0.20));
        }

        else{ //Else, merge to lines using fuzzy controllers

            if(lastMovement == BACKWARD) //Goes the opposite direction than before
                this->tAfterStop = Transition(FORWARD, std::pair<double, double> (0,0));

            else if(lastMovement == FORWARD)
                this->tAfterStop = Transition(BACKWARD, std::pair<double, double> (0,0));

            this->toTurn = LEFT; //Sets next turn
            this->currentRowPos++; //Increment current row

            return Transition(ANGULAR_STOP, std::pair<double, double> (0,0));
        }
    }

    else{ //Nothing was found
        if(++this->errorCount == 20){ //Increment err counter
            std::cout << "Error" << std::endl << std::endl;
            return Transition(END, std::pair<double, double> (0,0)); //If error occurred more than 3 times, stop
        }

        return Transition(LEFT_TURN_BEGIN, std::pair<double, double> (0,0));
    }
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::endStateRoutine(std::vector<Model> & models){ //End State 
    //cout << "End State" << endl;
    return Transition(END, std::pair<double, double> (0,0)); //End test
}
//--------------------------------------------------------------------------------------------------------
StateMachine::Transition StateMachine::impossibleStateRoutine(std::vector<Model> & models){ //Impossible State 
    std::cout << "Impossible State" << std::endl;
    return Transition(IMPOSSIBLE, std::pair<double, double> (0,0)); 
}

//################################################################################################

//--------------------------------------------------------------------------------------------------------
double StateMachine::calculateRatio(const std::pair<Model, Model> & m){ //Calculate fuzzy ratio input 
    double lAbs = fabs(m.first.getIntercept());
    double rAbs = fabs(m.second.getIntercept());

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