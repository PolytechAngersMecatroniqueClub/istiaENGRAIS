//********************************************************************************************************
#include "Pearl.h"

//--------------------------------------------------------------------------------------------------------
std::vector<Model> Pearl::findLines() { //Find the best lines into the cloud of points 
    double energy;

    if(this->outliers.size() != 0){ //If the vector is not empty
        int nModels = int(this->outliers.size() / Pearl::divideFactor); //Number of models to be searched is the nu,ber of outliers / 6. Because each model gets 3 points and we want 50% of outliers still available

        if(nModels > 0){ 
            this->searchModels(nModels); //Search for this number of models

            double newEnergy = this->expansionEnergy(); //Calculates expansion energy and total set energy

            newEnergy += this->removeTinyModels((int)(1.5*nModels)); //Remove models that have too few points attached

            energy = newEnergy; //Store current energy

            std::vector<Model> bestModels = this->models; //Store current models
            std::vector<Point> bestOutliers = this->outliers; //Store current set of outliers

            for(int nbOfIteractions = 0; nbOfIteractions < Pearl::maxNumberOfIterations; nbOfIteractions++) { //For 'maxNumberOfIterations' iterations
                this->searchModels((int)(this->outliers.size() / Pearl::divideFactor)); //Search for models

                this->fuseEqualModels(); //Fuse equal models

                this->reEstimation(); //Re calculate best models

                this->eraseBadModels(Pearl::worstEnergySizeRatioAllowed); //Erase models that have energy/nOfPoint

                newEnergy = this->expansionEnergy(); //Calculates expansion energy and total set energy

                if ((newEnergy > energy)){ //Store best state
                    this->models = bestModels;
                    this->outliers = bestOutliers;
                    energy = newEnergy;
                }
                else{ //Restore best state
                    bestModels = this->models;
                    bestOutliers = this->outliers;

                    newEnergy = energy;
                }
            }
        }
    }

    return this->models; //Return models
}
//--------------------------------------------------------------------------------------------------------
void Pearl::populateOutliers(const sensor_msgs::LaserScan & msg){ //Receive LaserScan message and puts all the points into outlier vector 
    //std::cout << "Pearl" << std::endl;
    double angle = msg.angle_min; 

    this->outliers.clear(); //Clear Points
    this->models.clear(); //Clear past models

    for(int i = 0; i < msg.ranges.size(); i++){ //For each ray
        if(!isinf(msg.ranges[i])) //If it touches anything
            this->outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle))); //Calculate x and y coordinates
        
        angle += msg.angle_increment; //Increment angle
    }

    initialField = this->outliers;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Pearl::removePointsInModels(){ //Removes all the points attached to all the models 
    for(int i = 0; i < this->models.size(); i++){ //For each model
        this->outliers.insert(this->outliers.end(), this->models[i].getPointsVecBegin(), this->models[i].getPointsVecEnd()); //Insert points into outliers 
        this->models[i].clearPoints(); //Clear points
    }
}
//--------------------------------------------------------------------------------------------------------    
void Pearl::removeModel(const int modelIndex){ //Removes model from the vector 
	if (!(0 <= modelIndex && modelIndex < models.size())){ //If index is out of range, return error
        Utility::printInColor("Wrong index for removing model", RED);
		exit(-1); //Exit application
    }

    this->outliers.insert(this->outliers.end(), this->models[modelIndex].getPointsVecBegin(), this->models[modelIndex].getPointsVecEnd()); //Insert model's points into outliers
    this->models[modelIndex].clearPoints(); //Clear points
    this->models.erase(this->models.begin() + modelIndex); //Erase model fom vector
}


//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Pearl::searchModels(const int nbOfModels) { //Searches for 'nbOfModels' models that are possible 
    Model model;
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++){ //Search for 'nbOfModels' models
        std::vector<Point> points = this->randomPointsInField(INITIAL_NUMBER_OF_POINTS); //Pick points
        if(points.size() != 0){
            model.findBestModel(points); //If points vector is populated, assign it to model and add it to vector

            this->models.push_back(model);
        }
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Point> Pearl::randomPointsInField(const int num) { //Picks 'num' different points in the outlier vector 
    std::vector<Point> ret;

    if(this->outliers.size() < num){ //Return empty vector if not enough points 
        return std::vector<Point>();
    }

    for(int i = 0; i < num; i++){ //For the amount of points needed 
        int randomNum = Utility::randomInt(0, this->outliers.size() - 1); //Picks a random number for the index

        if(this->outliers[randomNum].getY() >= 0) //If point has positive Y, insert it at the beginning
            ret.insert(ret.begin(), 1, this->outliers[randomNum]);
        else //If point has negative Y, insert it at the end
            ret.push_back(this->outliers[randomNum]);

        this->outliers.erase(this->outliers.begin() + randomNum); //Erase element from outliers 
    }

    return ret; //Return points
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double Pearl::expansionEnergy() { //Redistributes every point to the closest one, removes tiny models and calculates set's final energy 
    double newEnergy = this->redistributePoints(); //Redistributes points
    newEnergy += this->removeTinyModels(INITIAL_NUMBER_OF_POINTS); //Remove tiny models, adjusting energy

    newEnergy += this->calculateAdditionalEnergy(); //Calculates additional energy

    return newEnergy; //Return set's energy
}
//--------------------------------------------------------------------------------------------------------
double Pearl::redistributePoints() { //Reattach points to the closest model 
    this->removePointsInModels(); //Clear points 

    double newEnergy = 0;

    if(this->outliers.size() > 0){ //If there are points in outliers
	    newEnergy = this->outliers.size() * Pearl::outlierPenalty; //Initialize energy with outlier penalty

	    int dominantModelPos;
	    for(int p = 0; p < this->outliers.size(); p++){ //For each point in outliers 
	        double minDist = MAX_DBL; //Declare minimum distance to be 10^20 

	        for(int model = 0; model < this->models.size(); model++){ //For each model
	            
	            double distAt = fabs(this->models[model].getSlope() * this->outliers[p].getX() - this->outliers[p].getY() + this->models[model].getIntercept()) / sqrt(pow(this->models[model].getSlope(), 2) + 1.0); //Calculates distance from a point to all models
	            
	            if (distAt < minDist){ //Stores minimum distance and model index
	                minDist = distAt;
	                dominantModelPos = model;
	            }
	        }

	        if (minDist < Pearl::distanceForOutlier){ //If minimum distance is smaller than distance to be considered outlier
	            newEnergy += minDist - Pearl::outlierPenalty; //Recalculate energy by removing the penalty and the distance from point to model

	            this->models[dominantModelPos].pushPoint(this->outliers[p]); //Push this point to the closest model
	            this->outliers.erase(this->outliers.begin() + p); //Erase point from vector
	            p--; //Decrement position
	        }
	    }
	}

    return newEnergy; //Return energy
}
//--------------------------------------------------------------------------------------------------------
double Pearl::removeTinyModels(const int points) { //Remove models that have 'points' points or less 
    double gainEnergy = 0;
    for(int model = 0; model < this->models.size(); model++){ //For each model
        if(this->models[model].getPointsSize() <= points){ //If this model has little points attached
            gainEnergy += this->models[model].getPointsSize() * Pearl::outlierPenalty - this->models[model].getEnergy(); //Calculates additional energy by removing this model
            removeModel(model); //Remove this model
            model--; //Return position
        }
    }
    return gainEnergy; //Return extra energy
}
//--------------------------------------------------------------------------------------------------------
double Pearl::calculateAdditionalEnergy() const { //Calculates additional energy 
    double addEnergy = 0; 
    for(int model = 0; model < this->models.size(); model++) //For each model
        for(int model2 = model + 1; model2 < this->models.size(); model2++) //Compare with another model
            for(Point p : this->models[model].getPointsInModel()) //Get points in the first model
                for(Point q : this->models[model2].getPointsInModel()) //Get points in the second model
                    addEnergy += Pearl::additionalEnergyLambda * exp(-(pow(p.getX() - q.getX(), 2) + pow(p.getY() - q.getY(), 2)) / pow(Pearl::additionalEnergyCsi, 2)); //Uses extra energy equation for each pair of points

    return addEnergy; //Returns extra energy                         
}


//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Pearl::reEstimation() { //Uses linearFit in every model to recalculete the best line for each set of points 
    for(int model = 0; model < this->models.size(); model++){ //For each model 
        this->models[model].findBestModel(); //Find best possible line for each set of points
    }
}

//########################################################################################################

void Pearl::eraseBadModels(const double threshRatio) { //Erase all models that the ratio energy / number of points is lower than 'threshRatio' 
    for(int model = 0; model < this->models.size(); model++){ //For each model
        if ((this->models[model].getEnergy() / (double)this->models[model].getPointsSize()) >= threshRatio){ //If ratio is bad, delete it
            this->removeModel(model); //Remove
            model--; //Decrement index
        }
    }
}

//########################################################################################################

void Pearl::fuseEqualModels(){ //Fuse models that are considered to be the same, if they have close slope and intercept 
    for(int model = 0; model < this->models.size(); model++){ //For each model
        for(int model2 = model + 1; model2 < this->models.size() && model >= 0; model2++){ //Selects a different one
            if(model == model2 || model < 0 || model2 < 0) //If the models are the same or the position is invalid, continue
                continue;
            
            double slopeRatio = this->models[model].getSlope() / this->models[model2].getSlope(); //Calculate the ratio of the 2 slopes, this is usefull when the slope is big
            double interceptRatio = this->models[model].getIntercept() / this->models[model2].getIntercept(); //Calculate the ratio of the 2 intercepts, this is usefull when the intercept is big

            double slopeDifference = fabs(this->models[model].getSlope() - this->models[model2].getSlope()); //Calculate the difference of the 2 slopes, this is usefull when the slope is small
            double interceptDifference = fabs(this->models[model].getIntercept() - this->models[model2].getIntercept()); //Calculate the difference of the 2 intercepts, this is usefull when the intercept is small

            bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold); //If one of the two criteria meet the threshold for slope, store true
            bool isInterceptTheSame = ((1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= 1 + Pearl::sameInterceptThreshold) || interceptDifference <= Pearl::sameInterceptThreshold); //If one of the two criteria meet the threshold for intercept, store true

            if(isSlopeTheSame && isInterceptTheSame){ //If both are true, the models will be fused 
                int model1Size = this->models[model].getPointsSize(); //Get model 1 number of points
                int model2Size = this->models[model2].getPointsSize(); //Get model 2 number of points

                if(model1Size >= model2Size){ //Fuse smaller model into the bigger model
                	this->models[model].fuseModel(this->models[model2]);
                	this->models.erase(this->models.begin() + model2); //Erase smaller model
                	model2--; //Decrement model 2 position
                }
                else{
                	this->models[model2].fuseModel(this->models[model]);
                	this->models.erase(this->models.begin() + model); //Erase smaller model
                	model--; //Decrement model 1 position
                }
            }
        }
    }
}
// -------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Pearl &p){ //Print object 
    out << "Pearl: [\n\t  Models: Vector {\n";

    for(int i = 0; i < p.models.size(); i++){
        out << "\t\t[" << i << "]: Model: [ a: " << p.models[i].getSlope() << ", b: " << p.models[i].getIntercept() << ", energy: " << p.models[i].getEnergy() << ", parallelCount: " << p.models[i].getParallelCount() << ", fitness: " << p.models[i].getFitness();
        out << "\n\t\t\t\tPositive Points: " << p.models[i].getPositivePointsNum() << ", Points: Vector {";
        int pos = 0;
        for(Point p : p.models[i].getPointsInModel()){
            out << "\n\t\t\t\t\t[" << pos << "]: " << p;
            pos++;
        }
        out << "\n\t\t\t\t}\n\t\t\t    ]\n";
    }

    out << "\t  }\n\n\t  Outliers: Vector {";

    for(int outP = 0; outP < p.outliers.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << p.outliers[outP];
    }

    out << "\n\t  }";
    out << "\n       ]\n";
    return out; 
}

//********************************************************************************************************