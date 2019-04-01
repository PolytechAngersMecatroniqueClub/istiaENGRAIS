//********************************************************************************************************

#include "Model.h"


Model Model::linearFit(const std::vector<Point> & vec){ //Checked 
    double xsum = 0, ysum = 0, xysum = 0, xxsum = 0;

    if(vec.size() <= 1){
        
        return Model();
    }

    for (Point p : vec){
        xsum += p.getX(); 
        ysum += p.getY();
        xysum += p.getX() * p.getY();
        xxsum += p.getX() * p.getX();
    }

    double a = (double)(vec.size()*xysum - xsum*ysum) / (double)(vec.size()*xxsum - xsum*xsum);
    double b = (double)(ysum - a*xsum) / (double)vec.size();

    return (Model(a,b));
}
//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(){ //Checked 
    Model best = Model::linearFit(pointsInModel);
    if(best.getSlope() != MAX_DBL)
        this->a = best.getSlope();

    if(best.getIntercept() != MAX_DBL)
        this->b = best.getIntercept();

    this->energy = 0;
    for (Point p : pointsInModel){
        this->energy += fabs(this->a * p.getX() - p.getY() + this->b) / sqrt(pow(this->a, 2) + 1.0);
    }
}
//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(const std::vector<Point> & rPointsInModel){ //Checked 
	this->pointsInModel.clear();
    this->positivePoints = 0;
    this->pointsInModel = rPointsInModel;

    for(Point p : rPointsInModel){
        if(p.getY() >= 0)
            this->positivePoints++;
    }

    findBestModel();    
}
//--------------------------------------------------------------------------------------------------------
void Model::pushPoint(const Point & p) { 
    if(this->pointsInModel.size() == 0)
        this->energy = 0;

    if(this->a != MAX_DBL && this->b != MAX_DBL){
        this->energy += fabs(this->a * p.getX() - p.getY() + this->b) / sqrt(pow(this->a, 2) + 1.0);
    }

    if(p.getY() >= 0){
        pointsInModel.insert(pointsInModel.begin(), 1, p);
        positivePoints++;
    }

    else
        pointsInModel.push_back(p);
}
//--------------------------------------------------------------------------------------------------------
void Model::fuseModel(const Model & m){ //Checked 
    this->positivePoints += m.getPositivePointsNum();

    this->pointsInModel.insert(this->pointsInModel.begin(), m.getPointsVecBegin(), m.getPointsVecBegin() + m.getPositivePointsNum());
    this->pointsInModel.insert(this->pointsInModel.end(), m.getPointsVecBegin() + m.getPositivePointsNum(), m.getPointsVecEnd());

    this->findBestModel();
}   
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Model &m){ //Checked 
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy << ", parallelCount: " << m.parallelCount << ", fitness: " << m.fitness;
    out << "\n\t Positive Points: " << m.positivePoints << ", Points: Vector {";
    for(int i = 0; i < m.pointsInModel.size(); i++){
        out << "\n\t\t [" << i << "]: " << m.pointsInModel[i];
    }
    out << "\n\t }\n       ]";

    return out; 
}

//********************************************************************************************************