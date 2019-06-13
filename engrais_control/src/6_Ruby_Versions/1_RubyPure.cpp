//********************************************************************************************************
#include "1_RubyPure.h"


//--------------------------------------------------------------------------------------------------------
void RubyPure::populateOutliers(const sensor_msgs::LaserScan & msg){ //Populate outliers vector with laser scan message 
    double angle = msg.angle_min; //Gets minimum angle

    this->outliers.clear(); //Ckear outliers

    for(int i = 0; i < this->models.size(); i++) //For each model
        this->models[i].clearPoints(); //Clear its points, but not the model

    for(int i = 0; i < msg.ranges.size(); i++){ //For each ray
        if(!isinf(msg.ranges[i])) //If it touches something
            this->outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle))); //Convert it to (X,Y) coordiates
        
        angle += msg.angle_increment; //Increment angle
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyPure::countParallelLines(){ //Calculates parallel count for each model 
    for(int model = 0; model < this->models.size(); model++){ //For each model
        for(int model2 = model + 1; model2 < this->models.size(); model2++){
            double slopeRatio = this->models[model].getSlope() / this->models[model2].getSlope(); //Calculate the ratio of the 2 slopes, this is usefull when the slope is big
            double slopeDifference = fabs(this->models[model].getSlope() - this->models[model2].getSlope()); //Calculate the difference of the 2 slopes, this is usefull when the slope is small

            bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold); //If one of the two criteria meet the threshold for slope, store true

            if(isSlopeTheSame){ //If slope is the same, increment parallel count for both models
                this->models[model].incrementParallelCount();
                this->models[model2].incrementParallelCount();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyPure::eraseBadModels(const double threshRatio) { //Erases models that are considered bad, using Energy / (nPoints * parallel count)

    for(int i = 0; i < this->models.size(); i++) //For each model
        this->models[i].resetParallelCount(); //Reset parallel count

    this->countParallelLines(); //Calculate parallel count

    for(int model = 0; model < this->models.size(); model++){ //For each model
        if ((this->models[model].getEnergy() / (double)(this->models[model].getPointsSize() * this->models[model].getParallelCount())) >= threshRatio){ //If ratio is bad, delete model
            this->removeModel(model); //Delete model
            model--; //Decrement index
        }
    }
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyPure &r){ //Print object
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