//********************************************************************************************************
#include "1_RubyPure.h"


//--------------------------------------------------------------------------------------------------------
void RubyPure::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    outliers.clear();

    for(int i = 0; i < models.size(); i++)
        models[i].clearPoints();

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyPure::countParallelLines(){
    for(int model = 0; model < models.size(); model++){
        for(int model2 = model + 1; model2 < models.size(); model2++){
            if(fabs(models[model].getSlope() - models[model2].getSlope()) < this->sameSlopeThreshold){
                models[model].incrementParallelCount();
                models[model2].incrementParallelCount();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyPure::eraseBadModels(const double threshRatio) {
    countParallelLines();

    std::vector<int> modelsToBeDeleted;

    for(int model = 0; model < models.size(); model++){
        if ((models[model].getEnergy() / (double)(models[model].getPointsSize() * models[model].getParallelCount())) >= threshRatio){
            modelsToBeDeleted.push_back(model);
        }
    }

    for(int i = 0; i < modelsToBeDeleted.size(); i++)
        removeModel(modelsToBeDeleted[i]);
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