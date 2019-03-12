//********************************************************************************************************

#include "Model.h"

using namespace std;


//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(){ //Checked
    double xsum = 0, ysum = 0, xysum = 0, xxsum = 0;

    if(pointsInModel.size() <= 1)
        return;

    for (Point p : pointsInModel){
        xsum += p.getX(); 
        ysum += p.getY();
        xysum += p.getX() * p.getY();
        xxsum += p.getX() * p.getX();
    }

    this->a = (double)(pointsInModel.size()*xysum - xsum*ysum) / (double)(pointsInModel.size()*xxsum - xsum*xsum);
    this->b = (double)(ysum - this->a*xsum) / (double)pointsInModel.size();

    this->energy = 0;
    for (Point p : pointsInModel){
        this->energy += fabs(this->a * p.getX() - p.getY() + this->b) / sqrt(pow(this->a, 2) + 1.0);
    }
}
//--------------------------------------------------------------------------------------------------------
void Model::findBestModel(const std::vector<Point> & rPointsInModel){ //Checked
	this->pointsInModel.clear();
    this->pointsInModel = rPointsInModel;

    findBestModel();    
}
//--------------------------------------------------------------------------------------------------------
void Model::fuseModel(const Model & m){
    pointsInModel.insert(pointsInModel.end(), m.getPointsVecBegin(), m.getPointsVecEnd());
    this->findBestModel();
}   
//--------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Model &m){ //Checked
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy;
    out << "\n\tPoints: Vector {";
    for(int i = 0; i < m.pointsInModel.size(); i++){
        out << "\n\t\t[" << i << "]: " << m.pointsInModel[i];
    }
    out << "\n\t}\n]";

    return out; 
}

//********************************************************************************************************