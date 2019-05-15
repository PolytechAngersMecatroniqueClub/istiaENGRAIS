//********************************************************************************************************
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <iostream>
#include <Point.h>
#include <Model.h>
#include <vector>
#include <chrono>

#include <FuzzyController.h>

#ifndef MAX_VEL
#define MAX_VEL 4.0
#endif

#ifndef BODY_SIZE
#define BODY_SIZE 2.0
#endif

#ifndef DISTANCE_REFERENCE
#define DISTANCE_REFERENCE 1.5
#endif

class StateMachine{ 
    private:
        //------------------------------------------------------------------------------------------------
        enum States { BACKWARD = -1, INITIAL = 0, FORWARD = 1, LINEAR_STOP, ANGULAR_STOP, LEFT_TURN_BEGIN, LEFT_TURN_MID, LEFT_TURN_REMERGE };
        //------------------------------------------------------------------------------------------------
        enum Turn { LEFT, RIGHT };
        //------------------------------------------------------------------------------------------------
        class Transition{ 
            public:
                States nextState;
                std::pair<double, double> output;

                Transition(const States & st, const std::pair<double, double> & out);
        };
        //------------------------------------------------------------------------------------------------

        Turn toTurn = LEFT;

        States currentState = INITIAL;
        States lastMovement = INITIAL;

        Transition tAfterStop;

        FuzzyController fuzzy;

    public:
        //------------------------------------------------------------------------------------------------
        StateMachine();
        //------------------------------------------------------------------------------------------------
        std::pair<double, double> makeTransition(const std::pair<Model, Model> & models);

    private:
        //------------------------------------------------------------------------------------------------
        Transition initialStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition forwardStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition backwardStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition linearStopStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition angularStopStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition leftTurnBeginStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition leftTurnMidStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition leftTurnRemergeStateRoutine(const std::pair<Model, Model> & models);
        //------------------------------------------------------------------------------------------------
        Transition impossibleStateRoutine(const std::pair<Model, Model> & models);

        //################################################################################################

        //------------------------------------------------------------------------------------------------
        double calculateRatio(const std::pair<Model, Model> & m);
        //------------------------------------------------------------------------------------------------
        double calculateDistance(const std::pair<Model, Model> & m);
        //------------------------------------------------------------------------------------------------
        double calculateAngle(const std::pair<Model, Model> & m);
};

//--------------------------------------------------------------------------------------------------------
inline StateMachine::StateMachine() : tAfterStop(INITIAL, std::pair<double, double> (0,0)) {}

#endif
//********************************************************************************************************
