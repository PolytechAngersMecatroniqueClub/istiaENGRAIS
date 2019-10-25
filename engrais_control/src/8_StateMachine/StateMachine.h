//********************************************************************************************************
//  For further explanations check StateMachine.pdf
//********************************************************************************************************
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <iostream>
#include <Point.h>
#include <Model.h>
#include <Utility.h>
#include <vector>
#include <chrono>

#include <FuzzyController.h>

#ifndef MAX_VEL
#define MAX_VEL 0.7 //Maximum robot velocity
#endif

#ifndef BODY_SIZE
#define BODY_SIZE 1.1 //Robot's body size
#endif

#ifndef DISTANCE_REFERENCE
#define DISTANCE_REFERENCE 1.5 //Distance to be from a line if just 1 is found
#endif

class StateMachine{ //Class to implement a state machine 
    private:
        //------------------------------------------------------------------------------------------------
        enum States { BACKWARD = -1, INITIAL = 0, FORWARD = 1, LINEAR_STOP = 2, ANGULAR_STOP = 3, LEFT_TURN_BEGIN = 4, LEFT_TURN_MID = 5, 
                      LEFT_TURN_MERGE = 6, END = 7, IMPOSSIBLE = 8 }; //All possible states
        //------------------------------------------------------------------------------------------------
        enum Turn { LEFT, RIGHT }; //Where to turn
        //------------------------------------------------------------------------------------------------
        class Transition{ //Class to make a state stransition 
            public:
                States nextState; //Next state to go
                std::pair<double, double> output; //Wheels command

                Transition(const States & st, const std::pair<double, double> & out); //Constructor
        };
        //------------------------------------------------------------------------------------------------

        Turn toTurn = LEFT; //For first turning, go Left

        States currentState = INITIAL; //Initialize current state
        States lastMovement = INITIAL; //Store robot's last movement 

        Transition tAfterStop; //Store the next state to go after stopping

        FuzzyController fuzzy; //Fuzzy controller to Backward / Forward motion

        int errorCount = 0;

        int numOfLines = 4;

    public:
        //------------------------------------------------------------------------------------------------
        StateMachine(); //Default Constructor
        //------------------------------------------------------------------------------------------------
        std::tuple<double, double, std::vector<Model>> makeTransition(std::vector<Model> & allModels, double dist); //Compute a transition from the State machine, imagine this as a way to make this machine without a clock, and the user calls this function as a way to customize the clock frequency 

    private:
        //------------------------------------------------------------------------------------------------
        Transition initialStateRoutine(std::vector<Model> & models); //Initial State
        //------------------------------------------------------------------------------------------------
        Transition forwardStateRoutine(std::vector<Model> & models); //Forward State
        //------------------------------------------------------------------------------------------------
        Transition backwardStateRoutine(std::vector<Model> & models); //Backward State
        //------------------------------------------------------------------------------------------------
        Transition linearStopStateRoutine(std::vector<Model> & models); //Linear Stop State
        //------------------------------------------------------------------------------------------------
        Transition angularStopStateRoutine(std::vector<Model> & models); //Angular Stop State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnBeginStateRoutine(std::vector<Model> & models); //Lert Turn Begin State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMidStateRoutine(std::vector<Model> & models, double dist); //Lert Turn Mid State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMergeStateRoutine(std::vector<Model> & models); //Lert Turn Merge State
        //------------------------------------------------------------------------------------------------
        Transition endStateRoutine(std::vector<Model> & models); //End State 
        //------------------------------------------------------------------------------------------------
        Transition impossibleStateRoutine(std::vector<Model> & models); //Impossible State, it shouldn't be here!

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        double calculateRatio(const std::pair<Model, Model> & m); //Calculate ratio input for fuzzy controller
        //------------------------------------------------------------------------------------------------
        double calculateDistance(const std::pair<Model, Model> & m); //Calculate distance to first or last point of models (whatever is closest)
        //------------------------------------------------------------------------------------------------
        double calculateAngle(const std::pair<Model, Model> & m); //Calculate angle input for fuzzy
};

//--------------------------------------------------------------------------------------------------------
inline StateMachine::StateMachine() : tAfterStop(INITIAL, std::pair<double, double> (0,0)) {} //Initialize class members

#endif
//********************************************************************************************************
