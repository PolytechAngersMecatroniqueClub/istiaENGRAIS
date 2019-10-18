//********************************************************************************************************
#include "RobotControl.h"

using namespace std;


//--------------------------------------------------------------------------------------------------------
void RobotControl::frontPointsMessage(const visualization_msgs::Marker & msg){ //Receive message from front lines (found by front LIDAR) 
    this->frontPoints[0].clear();
    this->frontPoints[1].clear();

    const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0); //Translate from front LIDAR to robot's center

    for(int i = 0; i < finalMsg.points.size(); i++){
        Point p(finalMsg.points[i].x, finalMsg.points[i].y);

        if(p.getY() >= 0)
            this->frontPoints[0].insert(this->frontPoints[0].begin(), p);
        else
            this->frontPoints[1].insert(this->frontPoints[1].end(), p);
    }
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::backPointsMessage(const visualization_msgs::Marker & msg){ //Receive message from back lines (found by back LIDAR) 
    this->backPoints[0].clear();
    this->backPoints[1].clear();

    const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI); //Rotate from back LIDAR to same orientation as the robot
    const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0); //Translate from back LIDAR to robot's center

    for(int i = 0; i < finalMsg.points.size(); i++){
        Point p(finalMsg.points[i].x, finalMsg.points[i].y);

        if(p.getY() >= 0)
            this->backPoints[0].insert(this->backPoints[0].begin(), p);
        else
            this->backPoints[1].insert(this->backPoints[1].end(), p);
    }
}   


//--------------------------------------------------------------------------------------------------------
void RobotControl::frontLinesMessage(const visualization_msgs::Marker & msg){ //Receive message from front lines (found by front LIDAR) 
    const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -BODY_SIZE/2.0, 0); //Translate from front LIDAR to robot's center

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg); //Extract models

    addMsgModels(modelsInMsg, true); //Add models
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::backLinesMessage(const visualization_msgs::Marker & msg){ //Receive message from back lines (found by back LIDAR) 
    const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI); //Rotate from back LIDAR to same orientation as the robot
    const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, BODY_SIZE/2.0, 0); //Translate from back LIDAR to robot's center

    std::vector<Model> modelsInMsg = this->getFoundLines(finalMsg); //Extract models

    addMsgModels(modelsInMsg, false); //Add models
}


//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RobotControl::selectModels(const std::vector<int> & msgCounter) { //Select left and right models 

    std::pair<Model, Model> ret; //Declare return

    int minCounter = std::min(msgCounter[0], msgCounter[1]);

    for(int i = 0; i < this->models.size(); i++){ //For each model
        //std::cout << "ContFront: " << msgCounter[1] << ", contBack: " << msgCounter[0] << std::endl << std::endl;

        this->models[i].normalizeModel(msgCounter);

        //cout << this->models[i] << endl << endl << endl << endl;

        if(this->models[i].getBackCounter() < 0.7 * minCounter && this->models[i].getFrontCounter() < 0.7 * minCounter){
            this->models.erase(this->models.begin() + i);
            i--;
        }

        else if(this->models[i].getIntercept() >= 0 && fabs(this->models[i].getIntercept()) < fabs(ret.first.getIntercept())){ //If intercept is positive and has a better counter
            ret.first = this->models[i].toModel(); //Convert to regular model
        }

        else if(models[i].getIntercept() < 0 && fabs(this->models[i].getIntercept()) < fabs(ret.second.getIntercept())){ //If intercept is negative and has a better counter
            ret.second = this->models[i].toModel(); //Convert to regular model
        }
    } 

    std::vector<Point> field;

    field.insert(field.end(), this->frontPoints[0].begin(), this->frontPoints[0].end());
    field.insert(field.end(), this->frontPoints[1].begin(), this->frontPoints[1].end());

    field.insert(field.end(), this->backPoints[0].begin(), this->backPoints[0].end());
    field.insert(field.end(), this->backPoints[1].begin(), this->backPoints[1].end());

    ret.first.clearPoints();
    ret.second.clearPoints();

    for(int i = 0; i < field.size(); i++){
        double distLeft = Utility::distFromPointToLine(field[i], ret.first.getSlope(), ret.first.getIntercept());
        double distRight = Utility::distFromPointToLine(field[i], ret.second.getSlope(), ret.second.getIntercept());

        if(distLeft < Pearl::distanceForOutlier && ret.first.isPopulated())
            ret.first.pushPoint(field[i]);

        else if(distRight < Pearl::distanceForOutlier && ret.second.isPopulated())
            ret.second.pushPoint(field[i]);
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
bool RobotControl::getInitialModels(){
    bool found = false;

    /*for(int i = 0; i < this->models.size(); i++){ //For each model
        if(fabs(this->models[i].getSlope()) < 0.01){
            if(this->models[i].getIntercept() >= 0 && fabs(this->models[i].getIntercept()) < fabs(firstModels.first.getIntercept())){
                firstModels.first = this->models[i].toModel();

                found = true;
            }

            else if(this->models[i].getIntercept() < 0 && fabs(this->models[i].getIntercept()) < fabs(firstModels.second.getIntercept())){
                firstModels.second = this->models[i].toModel();

                found = true;
            }
        }
    }*/

    for(int i = 0; i < this->models.size(); i++){
        for(int j = i + 1; j < this->models.size(); j++){

            double slopeRatio = this->models[i].getSlope() / this->models[j].getSlope();
            double interceptRatio = this->models[i].getIntercept() / this->models[j].getIntercept(); //Calculate intercept ratio

            double slopeDifference = fabs(this->models[i].getSlope() - this->models[j].getSlope()); //Calculate slope difference
            double interceptSum = fabs(this->models[i].getIntercept() + this->models[j].getIntercept()); //Calculate intercept difference

            bool isParallel = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold); //Checks if slope is approximately the same
            bool isEquidistant = ((-1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= -1 + Pearl::sameInterceptThreshold) || interceptSum <= Pearl::sameInterceptThreshold); //Checks if interpect is approximately the same


            if(fabs(this->models[i].getSlope()) < 0.1 && isParallel && isEquidistant){

                if(this->models[i].getIntercept() >= 0 && fabs(this->models[i].getIntercept()) < fabs(firstModels.first.getIntercept())){
                    firstModels.first = this->models[i].toModel();
                    firstModels.second = this->models[j].toModel();

                    found = true;
                }
                else if(this->models[i].getIntercept() < 0 && fabs(this->models[i].getIntercept()) < fabs(firstModels.second.getIntercept())){
                    firstModels.first = this->models[j].toModel();
                    firstModels.second = this->models[i].toModel();

                    found = true;
                }
            }
        }
    }

    return found;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RobotControl::getHorizontalModels(){ 
    std::vector<Model> ret;

    /*for(int i = 0; i < this->models.size(); i++){
        Model m = this->models[i].toModel();


    }*/

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
void RobotControl::addMsgModels(const std::vector<Model> & modelsInMsg, bool isFrontMsg){ //Add models found to weighted models 
    for(int i = 0; i < modelsInMsg.size(); i++){ //For every model in message
        bool existsInModels = false; //Check if this model exists

        for(int j = 0; j < this->models.size(); j++){ //For each weighted model
            if(this->models[j].checkIfSameModel(modelsInMsg[i])){ //Check if model alredy exists
                models[j].fuseModels(modelsInMsg[i], isFrontMsg); //If so, fuse it with existing model
                existsInModels = true;
            }
        }

        if(!existsInModels){ //If it doesn't exist
            models.push_back(WeightedModel(modelsInMsg[i], isFrontMsg)); //Add to the list
        }
    }
}


//--------------------------------------------------------------------------------------------------------
Model RobotControl::translateLine(const Model m, const double newOX, const double newOY) const {

    double newSlope = m.getSlope(); //a' = a
    double newIntercept = m.getSlope() * newOX - newOY + m.getIntercept(); //b' = a * x0 - y0 + b

    std::vector<Point> newPoints = translateAxis(m.getPointsInModel(), newOX, newOY);

    Model ret(newSlope, newIntercept);

    for(int i = 0; i < newPoints.size(); i++)
        ret.pushPoint(newPoints[i]);

    return ret;
}
//--------------------------------------------------------------------------------------------------------
Model RobotControl::rotateLine(const Model m, const double angleRot) const {
    double div = cos(angleRot) - m.getSlope() * sin(angleRot); //cos(θ) - a*sin(θ)

    double newSlope = div != 0 ? (m.getSlope() * cos(angleRot) + sin(angleRot)) / div : MAX_DBL; //a' = (a*cos(θ) + sin(θ)) / (cos(θ) - a*sin(θ))
    double newIntercept = div != 0 ? m.getIntercept() / div : MAX_DBL; //b' = b / (cos(θ) - a*sin(θ))

    std::vector<Point> newPoints = rotateAxis(m.getPointsInModel(), angleRot);

    Model ret(newSlope, newIntercept);

    for(int i = 0; i < newPoints.size(); i++)
        ret.pushPoint(newPoints[i]);

    return ret;
}


//--------------------------------------------------------------------------------------------------------
std::vector<Point> RobotControl::translateAxis(const std::vector<Point> & points, const double newOX, const double newOY) const { //Translate points from origin 
    std::vector<Point> ret;

    for(int i = 0; i < points.size(); i++){ //For every point in message 
        double x = points[i].getX() - newOX; //Translate X coordinate 
        double y = points[i].getY() - newOY; //Translate Y coordinate 

        ret.push_back(Point(x, y)); //Add points
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Point> RobotControl::rotateAxis(const std::vector<Point> & points, const double angleRot) const { //Rotate points from origin 
    const double zMatrix[2][2] = { {cos(angleRot), -sin(angleRot)}, {sin(angleRot), cos(angleRot)} }; //Declare rotating matrix

    std::vector<Point> ret;

    for(int i = 0; i < points.size(); i++){ //For each point
        double x = zMatrix[0][0] * points[i].getX() + zMatrix[0][1] * points[i].getY(); //Rotate X coordinate
        double y = zMatrix[1][0] * points[i].getX() + zMatrix[1][1] * points[i].getY(); //Rotate Y coordinate

        ret.push_back(Point(x, y)); //Add points
    }

    return ret;
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
    out << "Models: \n";
    Utility::printVector(rc.models);

    /*out << "\nFront Positive: \n";
    Utility::printVector(rc.frontPoints[0]);

    out << "\nFront Negative: \n";
    Utility::printVector(rc.frontPoints[1]);

    out << "\nBack Positive: \n";
    Utility::printVector(rc.backPoints[0]);

    out << "\nBack Negative: \n";
    Utility::printVector(rc.backPoints[1]);*/

    return out;
}
//********************************************************************************************************