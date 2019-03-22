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
void Model::findBestModel(){
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
void Model::findBestModel(const std::vector<Point> & rPointsInModel){
	this->pointsInModel.clear();
    this->pointsInModel = rPointsInModel;

    findBestModel();    
}
//--------------------------------------------------------------------------------------------------------
void Model::fuseModel(const Model & m){
    this->pointsInModel.insert(this->pointsInModel.begin(), m.getPointsVecBegin(), m.getPointsVecBegin() + m.getPositivePointsNum());
    this->pointsInModel.insert(this->pointsInModel.end(), m.getPointsVecBegin() + m.getPositivePointsNum(), m.getPointsVecEnd());

    this->findBestModel();
}   
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Model &m){ //Checked
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy << ", parallelCount: " << m.parallelCount << ", fitness: " << m.fitness;
    out << "\n\tPoints: Vector {";
    for(int i = 0; i < m.pointsInModel.size(); i++){
        out << "\n\t\t[" << i << "]: " << m.pointsInModel[i];
    }
    out << "\n\t}\n]";

    return out; 
}

//********************************************************************************************************