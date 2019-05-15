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
#include <StateMachine.h>

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

        pair<Point, Point> positivePoints;
        pair<Point, Point> negativePoints;


    public:
        WeightedModel() {}

        WeightedModel(const Model & m) : a(m.getSlope()), b(m.getIntercept()) {
            assignPoints(m);
            //
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
            double slopeRatio = this->a / m.getSlope();
            double interceptRatio = this->b / m.getIntercept();

            double slopeDifference = fabs(this->a - m.getSlope());
            double interceptDifference = fabs(this->b - m.getIntercept());

            bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold);
            bool isInterceptTheSame = ((1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= 1 + Pearl::sameInterceptThreshold) || interceptDifference <= Pearl::sameInterceptThreshold);

            if(isSlopeTheSame && isInterceptTheSame){
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

class RobotControl{ 
    public:
        StateMachine robotFSM;
        std::vector<WeightedModel> models;


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