//********************************************************************************************************
#include "1_RubyPure.h"


//--------------------------------------------------------------------------------------------------------
void RubyPure::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    outliers.clear();
    models.clear();

    if(this->savedModels.first.getSlope() != MAX_DBL && this->savedModels.first.getIntercept() != MAX_DBL && this->savedModels.first.getEnergy() == 0 && this->savedModels.first.getPointsSize() == 0){
        models.push_back(this->savedModels.first);
    }

    if(this->savedModels.second.getSlope() != MAX_DBL && this->savedModels.second.getIntercept() != MAX_DBL && this->savedModels.second.getEnergy() == 0 && this->savedModels.second.getPointsSize() == 0){
        models.push_back(this->savedModels.second);
    }

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }

    this->savedModels = std::pair<Model, Model>();
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<int> RubyPure::countParallelLines(){
    std::vector<int> ret(models.size(), 1);

    for(int model = 0; model < models.size(); model++){
        for(int model2 = model + 1; model2 < models.size(); model2++){
            fabs(models[model].getSlope() - models[model].getSlope());
            if(fabs(models[model].getSlope() - models[model2].getSlope()) < this->sameSlopeThreshold){
                ret[model]++;
                ret[model2]++;
            }
        }
    }
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyPure::eraseBadModels(const double threshRatio) {
    std::vector<int> parrallel = countParallelLines();

    for(int model = 0; model < models.size(); model++){
        if ((models[model].getEnergy() / (double)models[model].getPointsSize() * parrallel[model]) >= threshRatio){
            removeModel(model);
            model--;
        }
    }
}
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RubyPure::findBestModels() { 
    std::pair<Model, Model> ret; //first = left, second = right
    
    double fitness;
    double bestFitnessLeft = MAX_DBL;
    double bestFitnessRight = MAX_DBL;

    int bestLeftPos = MIN_INT;
    int bestRightPos = MIN_INT; //MAX_INT

    std::vector<int> parrallel = countParallelLines();
    
    for(int model = 0; model < models.size(); model++){
        fitness = models[model].getPointsSize() != 0 ? (fabs(models[model].getIntercept()) + models[model].getEnergy()) / (double)(models[model].getPointsSize() * parrallel[model]) : MAX_DBL;

        if(fitness < bestFitnessLeft && models[model].getIntercept() >= 0){
            bestFitnessLeft = fitness;
            bestLeftPos = model;
        }
        else if(fitness < bestFitnessRight && models[model].getIntercept() < 0){
            bestFitnessRight = fitness;
            bestRightPos = model;
        }
    }

    if(bestLeftPos != MIN_INT){
        ret.first.clearPoints();
        ret.first.setEnergy(0);
    }

    if(bestRightPos != MIN_INT){
        ret.second.clearPoints();
        ret.second.setEnergy(0);
    }

    this->savedModels = ret;

    return ret;
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyPure &r){ 
    out << "RubyPure: [\n\tModels: Vector {\n";
    for(int m = 0; m < r.models.size(); m++){
        out << "\t\t[" << m << "]: Model [ a: " << r.models[m].getSlope() << ", b: " << r.models[m].getIntercept() << ", energy: " << r.models[m].getEnergy();
        out << "\n\t\t\tPoints: Vector {";
        int pos = 0;
        for(Point p : r.models[m].getPointsInModel()){
            out << "\n\t\t\t\t[" << pos << "]: " << p;
            pos++;
        }
        out << "\n\t\t\t}\n\t\t]\n";
    }

    out << "\t}\n\n\tOutliers: Vector {";

    for(int outP = 0; outP < r.outliers.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << r.outliers[outP];
    }

    out << "\n\t}";
    out << "\n]";
    return out; 
}

//********************************************************************************************************