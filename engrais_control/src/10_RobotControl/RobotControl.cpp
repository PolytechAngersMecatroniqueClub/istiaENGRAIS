//********************************************************************************************************
#include "RobotControl.h"

using namespace std;
//--------------------------------------------------------------------------------------------------------
void RobotControl::SavedInfos::initializeValues(const std::vector<Model> & models, int numLines){
    double model_a = (models[numLines/2 - 1].getSlope() + models[numLines/2].getSlope()) / 2.0; //Calculate mean slope

    double model_dist = fabs(models[numLines/2 - 1].getIntercept() - models[numLines/2].getIntercept()) / sqrt(pow(model_a, 2) + 1); //Calculate distance between models

    this->a = (this->a * this->cont + model_a) / (double)(this->cont + 1); //Ajust slope
    this->dist = (this->dist * this->cont + model_dist) / (double)(this->cont + 1); //Ajust Interept

    this->cont++; //Increment counter
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::frontPointsMessage(const visualization_msgs::Marker & msg){ //Receive message from front lines (found by front LIDAR) 
    this->frontPoints.clear(); //Clear front points

    const visualization_msgs::Marker finalMsg = this->translateAxis(msg, -this->BODY_SIZE/2.0, 0); //Translate from front LIDAR to robot's center

    for(int i = 0; i < finalMsg.points.size(); i++){ //Adds points to field
        Point p(finalMsg.points[i].x, finalMsg.points[i].y);

        this->frontPoints.push_back(p);
    }
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::backPointsMessage(const visualization_msgs::Marker & msg){ //Receive message from back lines (found by back LIDAR) 
    this->backPoints.clear(); //Clear Back Points

    const visualization_msgs::Marker movedMsg = this->rotateAxis(msg, -PI); //Rotate from back LIDAR to same orientation as the robot
    const visualization_msgs::Marker finalMsg = this->translateAxis(movedMsg, this->BODY_SIZE/2.0, 0); //Translate from back LIDAR to robot's center

    for(int i = 0; i < finalMsg.points.size(); i++){ //Adds points to field
        Point p(finalMsg.points[i].x, finalMsg.points[i].y);

        this->backPoints.push_back(p);
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
std::pair<std::vector<Model>, std::vector<bool>> RobotControl::selectModels(){ //Finds and calculates best models 
    cout << si << endl;
    if(this->si.cont < 10){ //If counter < 10, continue to initialize
        selected = initializeRobot(); //Searches for models that are coherent with initialization phase

        if(selected[this->NUM_OF_LINES/2 - 1].isPopulated() && selected[this->NUM_OF_LINES/2].isPopulated()){ //If those models were found
            si.initializeValues(selected, this->NUM_OF_LINES); //Update saved values

            if(si.cont == 10){ //End initialization

                for(int i = 0; i < this->NUM_OF_LINES/2 - 1; i++) //All left models
                    selected[i] = Model(this->si.a, selected[this->NUM_OF_LINES/2 - 1].getIntercept() + ((this->NUM_OF_LINES/2 - 1) - i) * (si.dist * sqrt(pow(si.a, 2) + 1))); //Calculate

                for(int i = this->NUM_OF_LINES/2 + 1; i < this->NUM_OF_LINES; i++) //All right models
                    selected[i] = Model(this->si.a, selected[this->NUM_OF_LINES/2].getIntercept() + ((this->NUM_OF_LINES/2) - i) * (si.dist * sqrt(pow(si.a, 2) + 1))); //Calculate
            }
        }

        return std::pair<std::vector<Model>, std::vector<bool>>(std::vector<Model>(), std::vector<bool>()); //Return empty vector
    }

    else{ //After initialization
        std::pair<std::vector<Model>, std::vector<bool>> ret = findBestModels(); //Selects and calculates models
        selected = ret.first; //Save models

        addPointsToModels(ret); //Find 

        return ret; //Return models
    }
}
//--------------------------------------------------------------------------------------------------------
void RobotControl::addPointsToModels(std::pair<std::vector<Model>, std::vector<bool>> & m) const { //Add points to models 
    std::vector<Point> field; //Declare field

    field.insert(field.end(), this->frontPoints.begin(), this->frontPoints.end()); //Insert front points
    field.insert(field.end(), this->backPoints.begin(), this->backPoints.end()); //Insert back points

    for(int i = 0; i < m.first.size(); i++) //For all models
        m.first[i].clearPoints(); //Clears points

    for(int i = 0; i < field.size(); i++){ //For each point
        for(int j = 0; j < m.first.size(); j++){ //For each model
            double dist = Utility::distFromPointToLine(field[i], m.first[j].getSlope(), m.first[j].getIntercept()); //Get distance from point to model

            if(m.first[j].isPopulated() && dist < Pearl::distanceForOutlier && !m.second[j]){ //If point was found and not calculated, and exists
                m.first[j].pushPoint(field[i]); //Push point
            }
        }
    }

    for(int i = 0; i < m.first.size(); i++){ //For each model
        if(m.second[i]){ //If model was calculated
            for(int j = 0; j < m.first.size(); j++){ //For every model
                if(m.first[j].getPointsSize() >= 2 && i != j && !m.second[j]){ //If model was found, has more than 2 points
                    std::pair<Point, Point> p = m.first[j].getFirstAndLastPoint();

                    m.first[i].pushPoint(p.first); //Add first and last point to calculated model (this works because only the x-coordinate matters)
                    m.first[i].pushPoint(p.second);
                }
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RobotControl::initializeRobot() const { //Do the initialization process 
    std::vector<Model> ret(this->NUM_OF_LINES); //Declare return

    for(int i = 0; i < this->models.size(); i++){ //For each model
        for(int j = i + 1; j < this->models.size(); j++){ //For each model

            double slopeRatio = this->models[i].getSlope() / this->models[j].getSlope(); //Calculate slope ratio
            double interceptRatio = this->models[i].getIntercept() / this->models[j].getIntercept(); //Calculate intercept ratio

            double slopeDifference = fabs(this->models[i].getSlope() - this->models[j].getSlope()); //Calculate slope difference
            double interceptSum = fabs(this->models[i].getIntercept() + this->models[j].getIntercept()); //Calculate intercept difference

            bool isParallel = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold); //Checks if slope is approximately the same
            bool isEquidistant = ((-1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= -1 + Pearl::sameInterceptThreshold) || interceptSum <= Pearl::sameInterceptThreshold); //Checks if interpect is approximately the same


            if(fabs(this->models[i].getSlope()) < 0.05 && isParallel && isEquidistant){ //if a pair of models is approximately the parallel, equidistant and have slope ~ 0

                if(this->models[i].getIntercept() >= 0 && fabs(this->models[i].getIntercept()) < fabs(ret[this->NUM_OF_LINES/2 - 1].getIntercept())){ //If the intercept is smaller,
                    ret[this->NUM_OF_LINES/2 - 1] = this->models[i].toModel(); //Save models
                    ret[this->NUM_OF_LINES/2] = this->models[j].toModel();
                }
                else if(this->models[i].getIntercept() < 0 && fabs(this->models[i].getIntercept()) < fabs(ret[this->NUM_OF_LINES/2].getIntercept())){ //If the intercept is smaller,
                    ret[this->NUM_OF_LINES/2 - 1] = this->models[j].toModel(); //Save models
                    ret[this->NUM_OF_LINES/2] = this->models[i].toModel();
                }
            }
        }
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::pair<std::vector<Model>, std::vector<bool>> RobotControl::findBestModels() { //Search models received to find models that are more coherent and calculates models that are not found
    std::pair<std::vector<Model>, std::vector<bool>> ret(std::vector<Model>(this->NUM_OF_LINES), std::vector<bool>(this->NUM_OF_LINES, false)); //Declare return

    int findCont = 0; //Number of models found
    double newSlope = 0; //field's mean slope

    double minErr = MAX_DBL; //Stores minimum error
    int minErrPos = MAX_INT; //Stores minimum error position

    for(double delta = 0.05; findCont < 1 && delta <= 0.3; delta += 0.05){ //Increment delta until something is found
        findCont = newSlope = 0;

        for(int i = 0; i < selected.size(); i++){ //For each selected model position

            for(WeightedModel m : this->models){ //For each model
                double deltaSlope = fabs(m.getSlope() - si.a); //|Δslope|
                double deltaIntercept = fabs(m.getIntercept() - selected[i].getIntercept()); //|Δb|

                double totalDelta = deltaSlope + deltaIntercept; //Total delta

                if(deltaSlope <= delta && deltaIntercept <= 1.2 * delta){ //If Δslope and Δb are smaller than the maximum allowed delta
                    ret.first[i] = m.toModel(); //Store model into position

                    newSlope += m.getSlope(); //Sums its slope

                    findCont++; //increment found counter

                    if(minErr >= totalDelta){ //Save minimum error
                        minErr = totalDelta;
                        minErrPos = i;
                    }
                }
            }
        }
    }

    if(findCont > 0){ //If something was found
        si.a = newSlope / (double)findCont; //Update field's slope
        for(int i = 0; i < ret.first.size(); i++){ //For each selected model

            if(!ret.first[i].isPopulated()){ //If it's not populated
                ret.first[i] = Model(si.a, selected[minErrPos].getIntercept() + (minErrPos - i) * (si.dist * sqrt(pow(si.a, 2) + 1))); //Calculate models using the minimum error 
                ret.second[i] = true; //Raises calculated flag
            }
        }
    }

    return ret;
}


//--------------------------------------------------------------------------------------------------------
std::pair<std_msgs::Float64, std_msgs::Float64> RobotControl::getWheelsCommand(){ //Get wheel command from finite state machine 
    std::pair<double, double> controls = si.cont >= 10 ? robotFSM.makeTransition(selected, si.dist) : std::pair<double, double> (0,0); //Calculate FSM's transition based on selected models

    std::pair<std_msgs::Float64, std_msgs::Float64> ret;

    ret.first.data = controls.first; //Cenvert to ROS message
    ret.second.data = controls.second;

    return ret; cout << endl;
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

    return out;
}
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream & out, const RobotControl::SavedInfos & si){ //Print object 
    out << "Saved Infos: [ a: " << si.a << ", dist: " << si.dist << ", cont: " << si.cont << " ]\n";

    return out;
}
//********************************************************************************************************