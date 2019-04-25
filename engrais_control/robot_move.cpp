//********************************************************************************************************
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

#define KP 1
#define DISTANCE_REFERENCE 1.5
#define TO_MILLISECOND 1000
#define PI 3.141592
#define BODY_SIZE 2

#define MAX_VEL 3

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
            this->a = (this->getSlope() * cont + m.getSlope()) / (cont + 1);
            this->b = (this->getIntercept() * cont + m.getIntercept()) / (cont + 1);

            this->assignPoints(m);

            this->cont++;
        }

        Model toModel(){
            Model ret(a,b);

            if(negativePoints.second.getX() != MIN_DBL && negativePoints.second.getY() != MIN_DBL)
                ret.pushPoint(negativePoints.second);
            else if(positivePoints.first.getX() != MIN_DBL && positivePoints.first.getY() != MIN_DBL)
                ret.pushPoint(positivePoints.first);
                

            if(positivePoints.second.getX() != MIN_DBL && positivePoints.second.getY() != MIN_DBL)
                ret.pushPoint(positivePoints.second);
            else if(negativePoints.first.getX() != MIN_DBL && negativePoints.first.getY() != MIN_DBL)
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

class LineSelector{
    public:
        std::vector<WeightedModel> models;


    public:
        LineSelector(){}

        void clearModels(){
            models.clear();
            //
        }

        void frontMessage(const visualization_msgs::Marker & msg){
            const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0);

            vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

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

        void backMessage(const visualization_msgs::Marker & msg){
            const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, PI);
            const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0);

            vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

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

        vector<Model> selectModels(){
            vector<Model> ret(2);
            int bestCounterLeft = 0, bestCounterRight = 0;

            for(int i = 0; i < models.size(); i++){
                if(models[i].getIntercept() >= 0 && models[i].getCounter() > bestCounterLeft){
                    bestCounterLeft = models[i].getCounter();
                    ret[0] = models[i].toModel();
                }

                if(models[i].getIntercept() < 0 && models[i].getCounter() > bestCounterRight){
                    bestCounterRight = models[i].getCounter();
                    ret[1] = models[i].toModel();
                }
            }

            return ret;
        }


    public:
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

        friend std::ostream & operator << (std::ostream & out, const LineSelector & ls){
            Utility::printVector(ls.models);

            return out;
        }
};

LineSelector selector;
FuzzyController fuzzy;

//--------------------------------------------------------------------------------------------------------
void sendLine(const vector<Model> & models){ 
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

    for(Model m : models){
        if(m.isPopulated()){

            pair<Point, Point> points = m.getFirstAndLastPoint();

            p.x = points.first.getX();
            p.y = m.getSlope()*p.x + m.getIntercept();

            line_list.points.push_back(p);

            p.x = points.second.getX();
            p.y = m.getSlope()*p.x + m.getIntercept();

            line_list.points.push_back(p);
        }
    }
    
    pubSelectedLines.publish(line_list);
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ 
    while(!endProgram){

        critSec.lock();
        selector.clearModels();
        critSec.unlock();

        usleep(250 * TO_MILLISECOND);

        critSec.lock();  

        vector<Model> selectModels = selector.selectModels();
        sendLine(selectModels);

        critSec.unlock();

        double distToCenter = (selectModels[0].getIntercept() + selectModels[1].getIntercept()) / 2.0;
        double angle = atan((selectModels[0].getSlope() + selectModels[1].getSlope()) / 2.0);

        pair<double, double> controls = fuzzy.getOutputValues(distToCenter, angle*180.0/PI);

        std_msgs::Float64 lControl, rControl;

        lControl.data = controls.first * MAX_VEL;
        rControl.data = controls.second * MAX_VEL;

        pubLeftControl.publish(lControl);
        pubRightControl.publish(rControl);

    }
}
//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;

    critSec.lock();
    selector.backMessage(msg);
    critSec.unlock();
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ 
    if(msg.type != 5)
        return;
    
    critSec.lock();
    selector.frontMessage(msg);
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