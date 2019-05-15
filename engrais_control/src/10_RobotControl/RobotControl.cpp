//********************************************************************************************************
#include "RobotControl.h"


//--------------------------------------------------------------------------------------------------------
void RobotControl::frontMessage(const visualization_msgs::Marker & msg){
    const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0);

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

    addMsgModels(modelsInMsg);
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::backMessage(const visualization_msgs::Marker & msg){
    const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI);
    const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0);

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg);

    addMsgModels(modelsInMsg);
}     
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RobotControl::selectModels(){
    std::pair<Model, Model> ret;
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
//--------------------------------------------------------------------------------------------------------
std::pair<std_msgs::Float64, std_msgs::Float64> RobotControl::getWheelsCommand(const std::pair<Model, Model> & selectedModels){
    std::pair<double, double> controls = robotFSM.makeTransition(selectedModels);

    std::pair<std_msgs::Float64, std_msgs::Float64> ret;

    ret.first.data = controls.first;
    ret.second.data = controls.second;

    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::addMsgModels(const std::vector<Model> & modelsInMsg){
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
//--------------------------------------------------------------------------------------------------------
visualization_msgs::Marker RobotControl::translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY){
    visualization_msgs::Marker ret;
    geometry_msgs::Point p;

    for(int i = 0; i < msg.points.size(); i++){
        p.x = msg.points[i].x - newOX;
        p.y = msg.points[i].y - newOY;

        ret.points.push_back(p);
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
visualization_msgs::Marker RobotControl::rotateAxis(const visualization_msgs::Marker & msg, const double angleRot){
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
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RobotControl::getFoundLines(const visualization_msgs::Marker & msg){ 
    std::vector<Model> foundLines;

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

//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const RobotControl & rc){
    Utility::printVector(rc.models);

    return out;
}
//********************************************************************************************************