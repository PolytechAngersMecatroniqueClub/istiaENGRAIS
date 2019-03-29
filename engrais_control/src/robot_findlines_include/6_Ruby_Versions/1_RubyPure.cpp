//********************************************************************************************************
#include "1_RubyPure.h"


//--------------------------------------------------------------------------------------------------------
void RubyPure::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    //std::cout << "RubyPure" << std::endl;
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
void RubyPure::countParallelLines(){ //Checked
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
void RubyPure::eraseBadModels(const double threshRatio) { //Checked
    countParallelLines();

    std::vector<int> modelsToBeDeleted;

    for(int model = 0; model < models.size(); model++){
        if ((models[model].getEnergy() / (double)(models[model].getPointsSize() * models[model].getParallelCount())) >= threshRatio){
            modelsToBeDeleted.push_back(model);
        }
    }

    for(int i = 0; i < modelsToBeDeleted.size(); i++){
        removeModel(modelsToBeDeleted[i]);
        for(int j = i + 1; j < modelsToBeDeleted.size(); j++){
            modelsToBeDeleted[j]--;
        }
    }
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyPure &r){ //Checked
    out << "RubyPure: [\n\t  Models: Vector {\n";

    for(int i = 0; i < r.models.size(); i++){
        out << "\t\t[" << i << "]: Model: [ a: " << r.models[i].getSlope() << ", b: " << r.models[i].getIntercept() << ", energy: " << r.models[i].getEnergy() << ", parallelCount: " << r.models[i].getParallelCount() << ", fitness: " << r.models[i].getFitness();
        out << "\n\t\t\t\tPositive Points: " << r.models[i].getPositivePointsNum() << ", Points: Vector {";
        int pos = 0;
        for(Point r : r.models[i].getPointsInModel()){
            out << "\n\t\t\t\t\t[" << pos << "]: " << r;
            pos++;
        }
        out << "\n\t\t\t\t}\n\t\t\t    ]\n";
    }

    out << "\t  }\n\n\t  Outliers: Vector {";

    for(int outP = 0; outP < r.outliers.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << r.outliers[outP];
    }

    out << "\n\t  }";
    out << "\n       ]\n";
    return out;  
}

//********************************************************************************************************