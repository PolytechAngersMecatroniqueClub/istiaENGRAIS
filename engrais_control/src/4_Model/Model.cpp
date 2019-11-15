//********************************************************************************************************
#include "Model.h"

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(){ //Using the points attached, calculate the best line possible 
    Model best = Model::linearFit(pointsInModel); //Find best model using its points

    if(best.getSlope() != MAX_DBL) //If model is different than the default, assign slope and intercept
        this->a = best.getSlope();

    if(best.getIntercept() != MAX_DBL)
        this->b = best.getIntercept();

    this->energy = 0; //Resets energy
    for (Point p : this->pointsInModel){ //For every point
        this->energy += fabs(this->a * p.getX() - p.getY() + this->b) / sqrt(pow(this->a, 2) + 1.0); //Sums its distance to the line
    }
}
//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(const std::vector<Point> & rPointsInModel){ //Using the points passed, calculate the best line possible and store points inside model 
    this->pointsInModel.clear(); //Clear previous point
    this->positivePoints = 0; 
    this->pointsInModel = rPointsInModel; //Attach passed points to the model

    for(Point p : rPointsInModel){ //Counts number of positive points 
        if(p.getY() >= 0)
            this->positivePoints++;
    }

    this->findBestModel();    
}
//--------------------------------------------------------------------------------------------------------
Model Model::linearFit(const std::vector<Point> & vec){ //Uses gauss' linear regression to find best model 
    double xsum = 0, ysum = 0, xysum = 0, xxsum = 0; //Initialize regression sums

    if(vec.size() <= 1){ //If only one point, return default Model
        
        return Model();
    }

    for (Point p : vec){ //For every point
        xsum += p.getX(); //Sum x
        ysum += p.getY(); //Sum y
        xysum += p.getX() * p.getY(); //Sum x*y
        xxsum += p.getX() * p.getX(); //Sum x²
    }

    double a = (double)(vec.size()*xysum - xsum*ysum) / (double)(vec.size()*xxsum - xsum*xsum); //Calculate slope
    double b = (double)(ysum - a*xsum) / (double)vec.size(); //Calculate Intercept

    return (Model(a,b)); //Return best possible model
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::pair<Point, Point> Model::getFirstAndLastPoint(const bool isRotated) const { //Get the closest point (first) and the farthest point (second) using only x-coordinate and rotates it 180º if argument is true
    int mult = isRotated ? -1 : 1;

    std::pair<Point, Point> ret;

    if(this->pointsInModel.size() == 0) //Default return to avoid crashes
        return ret;

    ret.first = ret.second = this->pointsInModel[0]; 

    for(Point p : this->pointsInModel){ //For each point
        if(mult * p.getX() < mult * ret.first.getX()) //Stores closest point
            ret.first = p;
        if(mult * p.getX() > mult * ret.second.getX()) //Stores farthest point
            ret.second = p;
    }

    return ret;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Model::fuseModel(const Model & m){ //Fuse two models, combining point and calculating the new best model 
    this->pointsInModel.insert(this->pointsInModel.begin() + this->positivePoints, m.getPointsVecBegin(), m.getPointsVecBegin() + m.getPositivePointsNum()); //Insert positive points in the positive part
    
    this->pointsInModel.insert(this->pointsInModel.end(), m.getPointsVecBegin() + m.getPositivePointsNum(), m.getPointsVecEnd()); //Insert negative points at the end
    
    this->positivePoints += m.getPositivePointsNum(); //Increments number of positive points
    
    this->findBestModel(); //Recalculates best model
}   
//--------------------------------------------------------------------------------------------------------
void Model::pushPoint(const Point & p) { //Interts a point in the model
    if(this->pointsInModel.size() == 0) //If it's the first point, reset energy
        this->energy = 0;

    if(this->a != MAX_DBL && this->b != MAX_DBL){ //If the model is populated, add the distance
        this->energy += fabs(this->a * p.getX() - p.getY() + this->b) / sqrt(pow(this->a, 2) + 1.0);
    }

    if(p.getY() >= 0){ //Insert it in the positive point part, and increment number of positive points
        this->pointsInModel.insert(pointsInModel.begin() + this->positivePoints, 1, p);
        this->positivePoints++;
    }

    else //Else, insert at end
        this->pointsInModel.push_back(p);
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Model &m){ //Print model 
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy << ", parallelCount: " << m.parallelCount << ", fitness: " << m.fitness;
    out << "\n\t Positive Points: " << m.positivePoints << ", Points: Vector {";
    for(int i = 0; i < m.pointsInModel.size(); i++){
        out << "\n\t\t [" << i << "]: " << m.pointsInModel[i];
    }
    out << "\n\t }\n       ]";

    return out; 
}

//********************************************************************************************************