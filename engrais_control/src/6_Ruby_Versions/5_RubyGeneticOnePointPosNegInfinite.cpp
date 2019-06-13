//********************************************************************************************************
#include "5_RubyGeneticOnePointPosNegInfinite.h"


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::populateOutliers(const sensor_msgs::LaserScan & msg){ //Populate field vector with laser scan message 
    double angle = msg.angle_min; //Get the minimum angle

    this->numberOfPositivePointsInField = 0; //Resets number of positive points
    this->field.clear(); //Clears field vector

    for(int i = 0; i < this->models.size(); i++){ //For every model
        this->models[i].clearPoints(); //Clear models points
        this->models[i].resetParallelCount(); //Reset parallel count
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){ //For every ray 
        if(!isinf(msg.ranges[i])){ //If it touches something
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){ //If already assigned
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)); //Second point converted to (X,Y) coordinates

                if(!fusedPoint.fusePoint(p, RubyGeneticOnePointPosNegInfinite::distanceToBeConsideredSamePoint)){ //If points are close enough, fuse them and don't add to vector 
                    if(fusedPoint.getY() >= 0){ //If they are far, add the center of mass of this cluster of points
                        this->field.insert(this->field.begin(), 1, fusedPoint); //Add at the first half if positive Y
                        this->numberOfPositivePointsInField++; //Increment counter
                    }
                    else
                        this->field.push_back(fusedPoint); //Push it to the end otherwise
 
                    fusedPoint = p; //Assign new cluster's first point
                }
            }
        
            else{ //If it's first iteration, assign first falue
                fusedPoint = WeightedPoint(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)); //Convert to (X,Y) coordinates
            }
        }
        angle += msg.angle_increment; //Increment angle
    }

    if(fusedPoint.getX() != MIN_DBL && fusedPoint.getY() != MIN_DBL && fusedPoint.getY() >= 0){ //As the last step, add the last point if the fused point is assign, push it into the vector 
        this->field.insert(this->field.begin(), 1, fusedPoint); //Add to te first half if positive Y
        this->numberOfPositivePointsInField++; //Increment
    }
    else if(fusedPoint.getX() != MIN_DBL && fusedPoint.getY() != MIN_DBL){ //Add to second half if negative Y
        this->field.push_back(fusedPoint);
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNegInfinite::findLines() { //Find the best lines into the cloud of points 
    double newEnergy = MAX_DBL, energy = MAX_DBL;

    std::vector<Model> bestModels;

    if(this->field.size() != 0){ //If field have points

        for (int it = 0; it < Pearl::maxNumberOfIterations; it++) { //For each iteration
            this->removePointsInModels(); //Remove points in models

            this->searchModels(RubyGeneticOnePointPosNegInfinite::numberOfModelsToSearch - this->models.size()); //Search for models until 40 total

            this->fuseEqualModels(); //Fuse models that are considered equal

            this->redistributePoints(); //Redistribute points to closest models

            //int numberMinOfPoints = std::max((int)(this->meanNumOfPoints() * RubyGeneticOnePointPosNegInfinite::factorToDeletePoints), 2);

            this->removeTinyModels(3); //Remove models that have too few points

            this->reEstimation(); //Re estimate models using linear fit

            this->eraseBadModels(); //Erase models that are considered bad

            newEnergy = this->calculateEnergy(); //Calculate final energy

            if ((newEnergy >= energy)){ //If energy gets bigger, recover last iteration
                this->models = bestModels;

                energy = newEnergy;
            }
            else{ //Else, save it
                bestModels = this->models;

                newEnergy = energy;
            }
        }
    }
    
    return bestModels;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::removeModel(const int modelIndex){ //Removes model from the vector 
	if (!(0 <= modelIndex && modelIndex < this->models.size())){ //If index is out of range, return error
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}

    this->models[modelIndex].clearPoints(); //Clear points
    this->models.erase(this->models.begin() + modelIndex); //Erase model
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::removePointsInModels(){ //Removes all the points attached to all the models 
    for(int i = 0; i < this->models.size(); i++){ //For each model      
        this->models[i].clearPoints(); //Clear points in model
        this->models[i].resetParallelCount(); //Reset pallarel count
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGeneticOnePointPosNegInfinite::randomPointsInField(const int minNum, const int maxNum, const int num) const { //Selects 'num' points in outliers between index 'minNum' and 'maxNum' 
    std::vector<Point> ret(num); //Return

    std::vector<int> randomNums = Utility::randomDiffVector(minNum, maxNum, num); //Select random indexes

    int pos = 0; //Position initial 0
    for(int i : randomNums){
        ret[pos] = this->field[i]; //Get points
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::searchModels(const int nbOfModels) { //Searches for 'nbOfModels' models that are possible 
    int numberOfPositiveModels = (int)(nbOfModels/2); //Number of positive models (models that has a positive 'b')
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels; //Number of negative models (models that has a negative 'b')

    if(this->field.size() < INITIAL_NUMBER_OF_POINTS) //If the outliers vector is smaller than the minimum
        return;

    if(this->numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS) //If there are too few positive points in the model
        numberOfPositiveModels = 0;

    if(this->field.size() - this->numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS) //If there are too few negative points in the model
        numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels;){ //For each positive model to search
        std::vector<Point> positivePoints = this->randomPointsInField(0, this->numberOfPositivePointsInField - 1, INITIAL_NUMBER_OF_POINTS); //Selects positive points
        if(positivePoints.size() > 0){ //If found something
            this->models.push_back(Model::linearFit(positivePoints)); //Push model
            modelPosNum++;
        }
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels;){
        std::vector<Point> negativePoints = this->randomPointsInField(this->numberOfPositivePointsInField, this->field.size() - 1, INITIAL_NUMBER_OF_POINTS);
        if(negativePoints.size() > 0){ //If found something
            this->models.push_back(Model::linearFit(negativePoints)); //Push model
            modelNegNum++; //Increment
        } 
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::calculateEnergy() const { //Calculate set's total energy 
    double energy = 0; //Sets energy = 0

    for(Model m : this->models) //For each model
        energy += m.getEnergy(); //Gets its energy

    energy += this->calculateAdditionalEnergy(); //Calculate additional energy

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::meanNumOfPoints() const { //Calculate mean number of points in the models 
    double mean = 0;
    for(Model m : this->models) //For each model
        mean += m.getPointsSize(); //Adds the number of points

    return mean/(double)this->models.size(); //Calculates average
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::redistributePoints() { //Reattach points to all models that are close 
    for(int p = 0; p < this->field.size(); p++){ //For each point

        for(int model = 0; model < this->models.size(); model++){ //For each model
            double distAt = fabs(this->models[model].getSlope() * this->field[p].getX() - this->field[p].getY() + this->models[model].getIntercept()) / sqrt(pow(this->models[model].getSlope(), 2) + 1.0); //Calculate distance
            
            if (distAt < Pearl::distanceForOutlier){ //If distance is small, add it to model
                this->models[model].pushPoint(this->field[p]); //Add point 
            }
        }
    }

    return 0;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::countParallelLines() { //Calculate set's total energy 
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
void RubyGeneticOnePointPosNegInfinite::eraseBadModels(){ //Calculate mean number of points in the models 
    this->countParallelLines(); //Calculate parallel count
    
    for(int i = 0; i < this->models.size(); i++) //Calculate fitness for every model
        this->models[i].calculateFitness();

    std::sort(this->models.begin(), this->models.end()); //sort it from better to worse

    int initialPos = std::max((int)(this->models.size()*0.25), 6);

    for(int i = initialPos; i < this->models.size(); i++){ //Keeps the 25% best models, or at least 4
        this->removeModel(i);
        i--;
    }
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNegInfinite &r){ //Print Object 
    out << "RubyGeneticOnePointPosNegInfinite: [\n\t  Models: Vector {\n";

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

    out << "\t  }\n\n\t  Field: Vector {";

    for(int outP = 0; outP < r.field.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << r.field[outP];
    }

    out << "\n\t  }";
    out << "\n       ]\n";
    return out; 
}

//********************************************************************************************************