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

        const double MAX_VEL;
        const double BODY_SIZE;
        const double DISTANCE_REFERENCE;

        //------------------------------------------------------------------------------------------------
        enum States { BACKWARD = -1, INITIAL = 0, FORWARD = 1, LINEAR_STOP, ANGULAR_STOP, LEFT_TURN_BEGIN, LEFT_TURN_MID, LEFT_TURN_MERGE, IMPOSSIBLE }; //All possible states
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
        StateMachine(double MVel, double BSize, double DReference); //Default Constructor
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> makeTransition(const std::pair<Model, Model> & models); //Compute a transition from the State machine, imagine this as a way to make this machine without a clock, and the user calls this function as a way to customize the clock frequency 

    private:
        //------------------------------------------------------------------------------------------------
        Transition initialStateRoutine(const std::pair<Model, Model> & models); //Initial State
        //------------------------------------------------------------------------------------------------
        Transition forwardStateRoutine(const std::pair<Model, Model> & models); //Forward State
        //------------------------------------------------------------------------------------------------
        Transition backwardStateRoutine(const std::pair<Model, Model> & models); //Backward State
        //------------------------------------------------------------------------------------------------
        Transition linearStopStateRoutine(const std::pair<Model, Model> & models); //Linear Stop State
        //------------------------------------------------------------------------------------------------
        Transition angularStopStateRoutine(const std::pair<Model, Model> & models); //Angular Stop State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnBeginStateRoutine(const std::pair<Model, Model> & models); //Lert Turn Begin State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMidStateRoutine(const std::pair<Model, Model> & models); //Lert Turn Mid State
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMergeStateRoutine(const std::pair<Model, Model> & models); //Lert Turn Merge State
        //------------------------------------------------------------------------------------------------
        Transition impossibleStateRoutine(const std::pair<Model, Model> & models); //Impossible State, it shouldn't be here!

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        double calculateRatio(const std::pair<Model, Model> & m); //Calculate ratio input for fuzzy controller
        //------------------------------------------------------------------------------------------------
        double calculateDistance(const std::pair<Model, Model> & m); //Calculate distance to first or last point of models (whatever is closest)
        //------------------------------------------------------------------------------------------------
        double calculateAngle(const std::pair<Model, Model> & m); //Calculate angle input for fuzzy
};

//--------------------------------------------------------------------------------------------------------
inline StateMachine::StateMachine(double MVel, double BSize, double DReference) : tAfterStop(INITIAL, std::pair<double, double> (0,0)), MAX_VEL(MVel), BODY_SIZE(BSize), DISTANCE_REFERENCE(DReference) {} //Initialize class members

#endif
//********************************************************************************************************
