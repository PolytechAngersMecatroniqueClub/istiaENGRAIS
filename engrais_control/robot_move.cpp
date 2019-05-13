//********************************************************************************************************
#include <chrono>
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <thread>
#include <mutex>

#include <Point.h>
#include <Model.h>
#include <Utility.h>

#include <FuzzyController.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>

#include <Pearl.h>
#include <1_RubyPure.h>
#include <2_RubyGenetic.h>
#include <3_RubyGeneticOnePoint.h>
#include <4_RubyGeneticOnePointPosNeg.h>
#include <5_RubyGeneticOnePointPosNegInfinite.h>


#define BODY_SIZE 2.0
#define SLEEP_TIME 500
#define PI 3.1415926535
#define TO_MILLISECOND 1000
#define DISTANCE_REFERENCE 1.5

using namespace std;

mutex critSec;

string mapName;
bool endProgram = false;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubSelectedLines;


class WeightedModel{ 
    public:
        int cont = 1;

        double a = MAX_DBL;
        double b = MAX_DBL;

        double sameSlopeThreshold = 0.1;
        double sameInterceptThreshold = 0.3;

        pair<Point, Point> positivePoints;
        pair<Point, Point> negativePoints;


    public:
        WeightedModel() {}

        WeightedModel(const Model & m) : a(m.getSlope()), b(m.getIntercept()) {
            assignPoints(m);
        }

        WeightedModel(const double aa, const double bb) : a(aa), b(bb) {}

        double getSlope() { return a; }

        double getIntercept() { return b; }

        int getCounter() { return cont; }

        void assignPoints(const Model & m){
            pair<Point, Point> points = m.getFirstAndLastPoint();

            if(points.first.getX() >= 0){
                this->positivePoints.first = points.first;
                this->positivePoints.second = points.second;
            }
            else{
                this->negativePoints.first = points.first;
                this->negativePoints.second = points.second;
            }
        }

        bool checkIfSameModel(const Model & m){
            if(fabs(this->getSlope() - m.getSlope()) < sameSlopeThreshold && fabs(this->getIntercept() - m.getIntercept()) < sameInterceptThreshold){
                return true;
            }

            return false;
        }

        void fuseModels(const Model & m){
            this->a = (this->getSlope() * cont + m.getSlope()) / (double)(cont + 1);
            this->b = (this->getIntercept() * cont + m.getIntercept()) / (double)(cont + 1);

            this->assignPoints(m);

            this->cont++;
        }

        Model toModel(){
            Model ret(a,b);

            if(negativePoints.second.isAssigned())
                ret.pushPoint(negativePoints.second);
            else if(positivePoints.first.isAssigned())
                ret.pushPoint(positivePoints.first);
                

            if(positivePoints.second.isAssigned())
                ret.pushPoint(positivePoints.second);
            else if(negativePoints.first.isAssigned())
                ret.pushPoint(negativePoints.first);


            return ret;
        }

        friend std::ostream & operator << (std::ostream & out, const WeightedModel & wm){
            out << "WeightedModel: [ a: " << wm.a << ", b: " << wm.b << ", cont: " << wm.cont << endl;

            out << wm.positivePoints.first << "    " << wm.positivePoints.second << "]" << endl;
            out << wm.negativePoints.first << "    " << wm.negativePoints.second << "]" << endl;

            return out;
        }
};

class StateMachine{
    public:
        enum States { BACKWARD = -1, INITIAL = 0, FORWARD = 1, LINEAR_STOP = 2, ANGULAR_STOP = 3, LEFT_TURN_BEGIN = 4, LEFT_TURN_MID = 5, LEFT_TURN_REMERGE = 6 };

        enum Turn { LEFT, RIGHT };

        class Transition{ 
            public:
                States nextState;
                pair<double, double> output;

                Transition(){}
                Transition(const States & st, const pair<double, double> & out){
                    nextState = st;
                    output = out;
                }
        };

        double max_vel = 4;

        States currentState = INITIAL;
        States lastMovement = INITIAL;

        Transition tAfterStop;

        Turn toTurn = LEFT;

        FuzzyController fuzzy;

        StateMachine(){}

        pair<double, double> makeTransition(const pair<Model, Model> & models){ 
            Transition stateTransition;

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
            return pair<double, double> (stateTransition.output.first * max_vel, stateTransition.output.second * max_vel);
        }


        Transition initialStateRoutine(const pair<Model, Model> & models){ 
            cout << "Initial state" << endl;

            if(models.first.isPopulated() || models.second.isPopulated()){    
                vector<Point> lPoints = models.first.getPointsInModel();
                vector<Point> rPoints = models.second.getPointsInModel();

                if((lPoints.size() >= 2 && lPoints[1].getX() < 0) || (rPoints.size() >= 2 && rPoints[1].getX() < 0))
                    return Transition(BACKWARD, pair<double, double> (0,0));
                
                else
                    return Transition(FORWARD, pair<double, double> (0,0));
            }

            else{
                return Transition(INITIAL, pair<double, double> (0,0));
            }
        }

        Transition forwardStateRoutine(const pair<Model, Model> & models){ 
            cout << "forward state" << endl;

            lastMovement = FORWARD;

            if(models.first.isPopulated() || models.second.isPopulated()){
                vector<Point> lPoints = models.first.getPointsInModel();
                vector<Point> rPoints = models.second.getPointsInModel();

                if((lPoints.size() >= 2 && lPoints[1].getX() + BODY_SIZE/2.0 <= -0.5) || (rPoints.size() >= 2 && rPoints[1].getX() + BODY_SIZE/2.0 <= -0.5)){
                    if(toTurn == LEFT)
                        tAfterStop = Transition(LEFT_TURN_BEGIN, pair<double, double> (-0.25 * FORWARD, 0.25 * FORWARD));

                    return Transition(LINEAR_STOP, pair<double, double> (0,0));
                }
                else{
                    pair<double, double> controls = fuzzy.getOutputValues(calculateRatio(models), calculateAngle(models));
                    
                    return Transition(FORWARD, pair<double, double> (controls.first, controls.second));
                }
            }
            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition backwardStateRoutine(const pair<Model, Model> & models){ 
            cout << "backward state" << endl;

            lastMovement = BACKWARD;

            if(models.first.isPopulated() || models.second.isPopulated()){
                vector<Point> lPoints = models.first.getPointsInModel();
                vector<Point> rPoints = models.second.getPointsInModel();

                if((lPoints.size() >= 2 && lPoints[0].getX() - BODY_SIZE/2.0 >= 0.5) || (rPoints.size() >= 2 && rPoints[0].getX() - BODY_SIZE/2.0 >= 0.5)){
                    if(toTurn == LEFT)
                        tAfterStop = Transition(LEFT_TURN_BEGIN, pair<double, double> (-0.25, 0.25));

                    return Transition(LINEAR_STOP, pair<double, double> (0,0));
                }
                else{
                    pair<double, double> controls = fuzzy.getOutputValues(calculateRatio(models), -calculateAngle(models));
                    
                    return Transition(BACKWARD, pair<double, double> (-controls.first, -controls.second));
                }
            }
            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition linearStopStateRoutine(const pair<Model, Model> & models){ 
            cout << "linear stop state" << endl;

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
                    return Transition(LINEAR_STOP, pair<double, double> (0,0));
                }
                
                if(!(-0.1 <= x_vel && x_vel <= -0.001 || 0.001 <= x_vel && x_vel <= 0.1))
                    return Transition(LINEAR_STOP, pair<double, double> (0,0));

                else{
                    first = true;
                    return tAfterStop;
                }
            }

            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition angularStopStateRoutine(const pair<Model, Model> & models){ 
            cout << "angular stop state" << endl;

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
                    return Transition(ANGULAR_STOP, pair<double, double> (0,0));
                }
                
                if(a_vel < -0.01 )
                    return Transition(ANGULAR_STOP, pair<double, double> (0.25, -0.25));

                else if(a_vel > 0.01 )
                    return Transition(ANGULAR_STOP, pair<double, double> (-0.25, 0.25));
                
                else if(-0.01 <= a_vel && a_vel <= -0.001 || 0.001 <= a_vel && a_vel <= 0.01)
                    return Transition(ANGULAR_STOP, pair<double, double> (0,0));

                else{
                    first = true;
                    return tAfterStop;
                }
            }

            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition leftTurnBeginStateRoutine(const pair<Model, Model> & models){ 
            cout << "Left turn begin state" << endl;

            if(models.first.isPopulated() || models.second.isPopulated()){
                double angle = calculateAngle(models);

                if(angle > -PI / 3.5){
                    return Transition(LEFT_TURN_BEGIN, pair<double, double> (-0.25, 0.25));
                }

                else{
                    tAfterStop = Transition(LEFT_TURN_MID, pair<double, double> (0.25, 0.25));

                    return Transition(ANGULAR_STOP, pair<double, double> (0,0));
                }
            }

            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition leftTurnMidStateRoutine(const pair<Model, Model> & models){ 
            cout << "Left turn mid state" << endl;

            const int Sense = lastMovement == FORWARD ? 1 : -1;
            const bool condition = Sense == 1 ? models.second.isPopulated() : models.first.isPopulated();
            const double intercept = Sense == 1 ? models.second.getIntercept() : models.first.getIntercept();

            cout << "Sentido: " << Sense << ", condition : " << condition << ", intercept : " << intercept << endl;

            if(condition){
                if(0.6 <= fabs(intercept) && fabs(intercept) <= 1.0){
                    tAfterStop = Transition(LEFT_TURN_REMERGE, pair<double, double> (0.25, -0.25));

                    return Transition(LINEAR_STOP, pair<double, double> (0, 0));
                }
                else{
                    return Transition(LEFT_TURN_MID, pair<double, double> (0.25 * Sense, 0.25 * Sense));
                }
            }

            else
                return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition leftTurnRemergeStateRoutine(const pair<Model, Model> & models){ 
            cout << "Left turn remerge state" << endl;

            if(models.first.isPopulated() || models.second.isPopulated()){
                double angle = calculateAngle(models);

                if(angle < PI / 8.0){
                    return Transition(LEFT_TURN_REMERGE, pair<double, double> (0.25, -0.25));
                }

                else{
                    if(lastMovement == BACKWARD)
                        tAfterStop = Transition(FORWARD, pair<double, double> (0,0));

                    else if(lastMovement == FORWARD)
                        tAfterStop = Transition(BACKWARD, pair<double, double> (0,0));


                    return Transition(ANGULAR_STOP, pair<double, double> (0,0));
                }
            }

            return Transition(INITIAL, pair<double, double> (0,0));
        }

        Transition impossibleStateRoutine(const pair<Model, Model> & models){ 
            cout << "impossible state" << endl;

            return Transition(INITIAL, pair<double, double> (0,0));
        }


        double calculateRatio(const pair<Model, Model> & m){ 
            double lAbs = m.first.isPopulated() ? fabs(m.first.getIntercept()) : DISTANCE_REFERENCE;
            double rAbs = m.second.isPopulated() ? fabs(m.second.getIntercept()) : DISTANCE_REFERENCE;

            double ratio = lAbs < rAbs ? lAbs / rAbs - 1.0 : 1.0 - rAbs / lAbs;

            return ratio;
        }

        double calculateDistance(const pair<Model, Model> & m){ 
            double distMean = 0;
            int cont = 0;

            vector<Point> lPoints = m.first.getPointsInModel();
            vector<Point> rPoints = m.second.getPointsInModel();

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

        double calculateAngle(const pair<Model, Model> & m){ 
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
};

class RobotControl{
    public:
        std::vector<WeightedModel> models;

        StateMachine robotFSM;

    public:
        RobotControl(){}

        void clearModels(){
            models.clear();
            //
        }

        void frontMessage(const visualization_msgs::Marker & msg){
            const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0);

            vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

            addMsgModels(modelsInMsg);
        }

        void backMessage(const visualization_msgs::Marker & msg){
            const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI);
            const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0);

            vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

            addMsgModels(modelsInMsg);
        }     

        pair<Model, Model> selectModels(){
            pair<Model, Model> ret;
            int bestCounterLeft = 0, bestCounterRight = 0;

            for(int i = 0; i < models.size(); i++){
                if(models[i].getIntercept() >= 0 && models[i].getCounter() > bestCounterLeft){
                    bestCounterLeft = models[i].getCounter();
                    ret.first = models[i].toModel();
                }

                if(models[i].getIntercept() < 0 && models[i].getCounter() > bestCounterRight){
                    bestCounterRight = models[i].getCounter();
                    ret.second = models[i].toModel();
                }
            }  

            return ret;
        }

        pair<std_msgs::Float64, std_msgs::Float64> getWheelsCommand(const pair<Model, Model> & selectedModels){
            pair<double, double> controls = robotFSM.makeTransition(selectedModels);

            pair<std_msgs::Float64, std_msgs::Float64> ret;

            ret.first.data = controls.first;
            ret.second.data = controls.second;

            return ret;
        }


    private:
        void addMsgModels(const vector<Model> & modelsInMsg){
            for(int i = 0; i < modelsInMsg.size(); i++){
                bool existsInModels = false;

                for(int j = 0; j < models.size(); j++){
                    if(models[j].checkIfSameModel(modelsInMsg[i])){
                        models[j].fuseModels(modelsInMsg[i]);
                        existsInModels = true;
                    }
                }

                if(!existsInModels){
                    models.push_back(WeightedModel(modelsInMsg[i]));
                }
            }
        }

        visualization_msgs::Marker translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY){
            visualization_msgs::Marker ret;
            geometry_msgs::Point p;

            for(int i = 0; i < msg.points.size(); i++){
                p.x = msg.points[i].x - newOX;
                p.y = msg.points[i].y - newOY;

                ret.points.push_back(p);
            }

            return ret;
        }

        visualization_msgs::Marker rotateAxis(const visualization_msgs::Marker & msg, const double angleRot){
            const double xMatrix[2][2] = { {cos(angleRot), -sin(angleRot)}, {sin(angleRot), cos(angleRot)} };

            visualization_msgs::Marker ret;
            geometry_msgs::Point p;

            for(int i = 0; i < msg.points.size(); i++){
                p.x = xMatrix[0][0] * msg.points[i].x + xMatrix[0][1] * msg.points[i].y;
                p.y = xMatrix[1][0] * msg.points[i].x + xMatrix[1][1] * msg.points[i].y;

                ret.points.push_back(p);
            }

            return ret;
        }

        vector<Model> getFoundLines(const visualization_msgs::Marker & msg){ 
            vector<Model> foundLines;

            for(int i = 0; i < msg.points.size(); i += 2){
                double dx = msg.points[i].x - msg.points[i + 1].x;
                double dy = msg.points[i].y - msg.points[i + 1].y;

                double a = MAX_DBL;
                
                if(dx != 0)
                    a = dy / dx;

                double b = msg.points[i].y - a * msg.points[i].x;
                
                Model model(a,b);

                model.pushPoint(Point(msg.points[i].x, msg.points[i].y));
                model.pushPoint(Point(msg.points[i+1].x, msg.points[i+1].y));

                foundLines.push_back(model);                
            }

            return foundLines;
        }


        friend std::ostream & operator << (std::ostream & out, const RobotControl & rc){
            Utility::printVector(rc.models);

            return out;
        }
};

RobotControl control;


//--------------------------------------------------------------------------------------------------------
void sendLine(const pair<Model, Model> & models){ 
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;

    line_list.header.frame_id = mapName;
    line_list.header.stamp = ros::Time::now();
    line_list.ns = "points_and_lines";
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.id = 0;
    line_list.type = visualization_msgs::Marker::LINE_LIST;

    line_list.scale.x = 0.08;

    line_list.color.g = 1.0;
    line_list.color.a = 0.4;

    if(models.first.isPopulated()){
        pair<Point, Point> points = models.first.getFirstAndLastPoint();

        p.x = points.first.getX();
        p.y = models.first.getSlope()*p.x + models.first.getIntercept();

        line_list.points.push_back(p);

        p.x = points.second.getX();
        p.y = models.first.getSlope()*p.x + models.first.getIntercept();

        line_list.points.push_back(p);
    }

    if(models.second.isPopulated()){
        pair<Point, Point> points = models.second.getFirstAndLastPoint();

        p.x = points.first.getX();
        p.y = models.second.getSlope()*p.x + models.second.getIntercept();

        line_list.points.push_back(p);

        p.x = points.second.getX();
        p.y = models.second.getSlope()*p.x + models.second.getIntercept();

        line_list.points.push_back(p);
    }
    
    pubSelectedLines.publish(line_list);
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ 
    while(!endProgram){

        critSec.lock();
        control.clearModels();
        critSec.unlock();

        usleep(SLEEP_TIME * TO_MILLISECOND);

        critSec.lock();  

        pair<Model, Model> selectedModels = control.selectModels();
        sendLine(selectedModels);
        pair<std_msgs::Float64, std_msgs::Float64> wheels = control.getWheelsCommand(selectedModels);

        //cout << control << endl;
        critSec.unlock();

        pubLeftControl.publish(wheels.first);
        pubRightControl.publish(wheels.second);

        cout << "Left: " << wheels.first.data <<  ", Right: " << wheels.second.data << endl << endl << endl << endl << endl;

    }
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;

    critSec.lock();
    control.backMessage(msg);
    critSec.unlock();
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;
    
    critSec.lock();
    control.frontMessage(msg);
    critSec.unlock();
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    ros::init(argc, argv, "robot_move_node");
    ros::NodeHandle node;

    string subTopicFront, subTopicBack, pubTopicLeft, pubTopicRight, pubTopicSelected, node_name = ros::this_node::getName();

    if(!node.getParam(node_name + "/subscribe_topic_front", subTopicFront) || !node.getParam(node_name + "/subscribe_topic_back", subTopicBack) || 
       !node.getParam(node_name + "/publish_topic_left", pubTopicLeft) || !node.getParam(node_name + "/publish_topic_right", pubTopicRight)){

        ROS_ERROR_STREAM("Argument missing in node " << node_name << ", expected 'subscribe_topic_front', 'subscribe_topic_back', " <<
                         "'publish_topic_left', 'publish_topic_right' and [optional: 'rviz_topic' and 'rviz_frame']\n\n");
        return -1;
    }

    node.param<string>(node_name + "/rviz_frame", mapName, "world");
    node.param<string>(node_name + "/rviz_topic", pubTopicSelected, "/engrais/robot_move/selected_lines");

    ros::Subscriber subFront = node.subscribe(subTopicFront, 10, FrontLinesMsg);
    ros::Subscriber subBack = node.subscribe(subTopicBack, 10, BackLinesMsg);

    pubLeftControl = node.advertise<std_msgs::Float64>(pubTopicLeft, 10);
    pubRightControl = node.advertise<std_msgs::Float64>(pubTopicRight, 10);


    pubSelectedLines = node.advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread);

    Utility::printInColor("Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);

    endProgram = true;
    control_t.join();

    subFront.shutdown();
    subBack.shutdown();
    pubLeftControl.shutdown();
    pubRightControl.shutdown();

    pubSelectedLines.shutdown();
    ros::shutdown();

    Utility::printInColor("Code ended without errors", BLUE);

    return 0;
}

//********************************************************************************************************