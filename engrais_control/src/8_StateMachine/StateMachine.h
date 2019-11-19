//********************************************************************************************************
//  For further explanations check StateMachine.pdf
//********************************************************************************************************
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <iostream>
#include <Point.h>
#include <Model.h>
#include <vector>
#include <chrono>

#include <ros/ros.h>

#include <FuzzyController.h>


class StateMachine{ //Class to implement a state machine 
    private:

        int errorCount = 0; //Error count to stop the robot's movement
        int currentRowPos = 1; //Current Row

        const int NUM_OF_LINES; //Number of lines present in selected models
        const int NUM_OF_TIMES_TURN; //Number of times the robot will turn

        const double MAX_VEL; //Robot's max velocity
        const double BODY_SIZE; //Robot's X length

        //------------------------------------------------------------------------------------------------
        enum States { BACKWARD = -1, INITIAL = 0, FORWARD = 1, LINEAR_STOP, ANGULAR_STOP, LEFT_TURN_BEGIN, LEFT_TURN_MID, 
                      LEFT_TURN_MERGE, RIGHT_TURN_BEGIN, RIGHT_TURN_MID, RIGHT_TURN_MERGE, IMPOSSIBLE, END}; //All possible states
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


    public:
        //------------------------------------------------------------------------------------------------
        StateMachine(const int NLines, const int NTimesTurn, const double MVel, const double BSize); //Default Constructor
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> makeTransition(std::vector<Model> & allModels, const double dist); //Compute a transition from the State machine, imagine this as a way to make this machine without a clock, and the user calls this function as a way to customize the clock frequency 

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
        Transition leftTurnMidStateRoutine(std::vector<Model> & models, const double dist); //Lert Turn Mid State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMergeStateRoutine(std::vector<Model> & models); //Lert Turn Merge State
        //------------------------------------------------------------------------------------------------
        Transition rightTurnBeginStateRoutine(std::vector<Model> & models); //Right Turn Begin State
        //------------------------------------------------------------------------------------------------
        Transition rightTurnMidStateRoutine(std::vector<Model> & models, const double dist); //Right Turn Mid State
        //------------------------------------------------------------------------------------------------
        Transition rightTurnMergeStateRoutine(std::vector<Model> & models); //Right Turn Merge State
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
inline StateMachine::StateMachine(const int NLines, const int NTimesTurn, const double MVel, const double BSize) : NUM_OF_TIMES_TURN(NTimesTurn), MAX_VEL(MVel), BODY_SIZE(BSize), NUM_OF_LINES(NLines), tAfterStop(INITIAL, std::pair<double, double> (0,0)) {} //Initialize class members

#endif
//********************************************************************************************************
