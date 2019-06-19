//********************************************************************************************************
#include "RobotControl.h"


//--------------------------------------------------------------------------------------------------------
void RobotControl::frontMessage(const visualization_msgs::Marker & msg){ //Receive message from front lines (found by front LIDAR) 
    const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0); //Translate from front LIDAR to robot's center

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg); //Extract models

    addMsgModels(modelsInMsg); //Add models
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::backMessage(const visualization_msgs::Marker & msg){ //Receive message from back lines (found by back LIDAR) 
    const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI); //Rotate from back LIDAR to same orientation as the robot
    const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0); //Translate from back LIDAR to robot's center

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg); //Extract models

    addMsgModels(modelsInMsg); //Add models
}     
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RobotControl::selectModels() const { //Select left and right models 
    std::pair<Model, Model> ret; //Declare return
    int bestCounterLeft = 0, bestCounterRight = 0; //Store best counter for each model

    for(int i = 0; i < this->models.size(); i++){ //For each model
        if(this->models[i].getIntercept() >= 0 && this->models[i].getCounter() > bestCounterLeft){ //If intercept is positive and has a better counter
            bestCounterLeft = this->models[i].getCounter(); //Store best left modelcounter
            ret.first = this->models[i].toModel(); //Convert to regular model
        }

        if(models[i].getIntercept() < 0 && models[i].getCounter() > bestCounterRight){ //If intercept is negative and has a better counter
            bestCounterRight = models[i].getCounter(); //Store best right modelcounter
            ret.second = models[i].toModel(); //Convert to regular model
        }
    }  

    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::pair<std_msgs::Float64, std_msgs::Float64> RobotControl::getWheelsCommand(const std::pair<Model, Model> & selectedModels){ //Get wheel command from finite state machine 
    std::pair<double, double> controls = robotFSM.makeTransition(selectedModels); //Calculate FSM's transition based on selected models

    std::pair<std_msgs::Float64, std_msgs::Float64> ret;

    ret.first.data = controls.first; //Cenvert to ROS message
    ret.second.data = controls.second;

    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::addMsgModels(const std::vector<Model> & modelsInMsg){ //Add models found to weighted models 
    for(int i = 0; i < modelsInMsg.size(); i++){ //For every model in message
        bool existsInModels = false; //Check if this model exists

        for(int j = 0; j < this->models.size(); j++){ //For each weighted model
            if(this->models[j].checkIfSameModel(modelsInMsg[i])){ //Check if model alredy exists
                models[j].fuseModels(modelsInMsg[i]); //If so, fuse it with existing model
                existsInModels = true;
            }
        }

        if(!existsInModels){ //If it doesn't exist
            models.push_back(WeightedModel(modelsInMsg[i])); //Add to the list
        }
    }
}
//--------------------------------------------------------------------------------------------------------
visualization_msgs::Marker RobotControl::translateAxis(const visualization_msgs::Marker & msg, const double newOX, const double newOY) const { //Translate points from origin 
    visualization_msgs::Marker ret;
    geometry_msgs::Point p;

    for(int i = 0; i < msg.points.size(); i++){ //For every point in message 
        p.x = msg.points[i].x - newOX; //Translate X coordinate 
        p.y = msg.points[i].y - newOY; //Translate Y coordinate 

        ret.points.push_back(p); //Add points
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
visualization_msgs::Marker RobotControl::rotateAxis(const visualization_msgs::Marker & msg, const double angleRot) const { //Rotate points from origin 
    const double zMatrix[2][2] = { {cos(angleRot), -sin(angleRot)}, {sin(angleRot), cos(angleRot)} }; //Declare rotating matrix

    visualization_msgs::Marker ret;
    geometry_msgs::Point p;

    for(int i = 0; i < msg.points.size(); i++){ //For each point
        p.x = zMatrix[0][0] * msg.points[i].x + zMatrix[0][1] * msg.points[i].y; //Rotate X coordinate
        p.y = zMatrix[1][0] * msg.points[i].x + zMatrix[1][1] * msg.points[i].y; //Rotate Y coordinate

        ret.points.push_back(p); //Add points
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RobotControl::getFoundLines(const visualization_msgs::Marker & msg) const { //Extract models from message 
    std::vector<Model> foundLines;

    for(int i = 0; i < msg.points.size(); i += 2){ //Get 2 points from ROS message
        double dx = msg.points[i].x - msg.points[i + 1].x; //Calculate Δx
        double dy = msg.points[i].y - msg.points[i + 1].y; //Calculate Δy

        double a = MAX_DBL; //Declare slope
        
        if(dx != 0)
            a = dy / dx; //Calculate Δy/Δx

        double b = msg.points[i].y - a * msg.points[i].x; //Calculate intercept
        
        Model model(a,b);

        model.pushPoint(Point(msg.points[i].x, msg.points[i].y)); //Push negative-most point (x-coordinate)
        model.pushPoint(Point(msg.points[i+1].x, msg.points[i+1].y)); //Push positive-most point (x-coordinate)

        foundLines.push_back(model); //Push model
    }

    return foundLines;
}
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const RobotControl & rc){ //Print object 
    Utility::printVector(rc.models);

    return out;
}
//********************************************************************************************************