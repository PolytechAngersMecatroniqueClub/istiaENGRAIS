//********************************************************************************************************
#include "5_RubyGeneticOnePointPosNegInfinite.h"


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked
    double angle = msg.angle_min;

    this->numberOfPositivePointsInField = 0;
    this->field.clear();

    for(int i = 0; i < models.size(); i++){
        this->models[i].clearPoints();
        this->models[i].resetParallelCount();
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, this->distanceToBeConsideredSamePoint)){
                    if(fusedPoint.getY() >= 0){
                        this->field.insert(this->field.begin(), 1, fusedPoint);
                        this->numberOfPositivePointsInField++;
                    }
                    else
                        this->field.push_back(fusedPoint);

                    fusedPoint = p;
                }
            }
        
            else{
                fusedPoint = WeightedPoint(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));
            }
        }
        angle += msg.angle_increment;
    }

    if(fusedPoint.getY() >= 0){
        this->field.insert(this->field.begin(), 1, fusedPoint);
        this->numberOfPositivePointsInField++;
    }
    else{
        this->field.push_back(fusedPoint);
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNegInfinite::findLines() { //Checked 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(field.size() != 0){

        std::vector<Model> bestModels;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            clearPointsInModels();

            searchModels(this->numberOfModelsToSearch - models.size());

            fuseEqualModels();

            redistributePoints();

            numberMinOfPoints = std::max((int)(meanNumOfPoints() * this->factorToDeletePoints), 3);
            removeTinyModels(numberMinOfPoints);

            reEstimation();

            eraseBadModels();

            newEnergy = calculateEnergy();

            if ((newEnergy >= energy)){
                this->models = bestModels;

                energy = newEnergy;
            }
            else{
                bestModels = this->models;

                newEnergy = energy;
            }
        }
    }

    else{
        Utility::printInColor("No data in field, please verify", RED);
    }   
    
    return this->models;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::removeModel(const int modelIndex){ //Checked 
	if (!(0 <= modelIndex && modelIndex < models.size())){
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}

    models[modelIndex].clearPoints();
    models.erase(models.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::clearPointsInModels(){ //Checked 
    for(int i = 0; i < models.size(); i++){       
        models[i].clearPoints();
        this->models[i].resetParallelCount();
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGeneticOnePointPosNegInfinite::randomPointsInField(const int minNum, const int maxNum, const int num) const { //Checked 
    std::vector<Point> ret(num);

    std::vector<int> randomNums = Utility::randomDiffVector(minNum, maxNum, num);

    int pos = 0;
    for(int i : randomNums){
        ret[pos] = field[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::searchModels(const int nbOfModels) { //Checked 
    int numberOfPositiveModels = (int)(nbOfModels/2);
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels;

    if(field.size() < INITIAL_NUMBER_OF_POINTS)
        return;

    if(numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS)
        numberOfPositiveModels = 0;

    if(field.size() - numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS)
        numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels; modelPosNum++){
        std::vector<Point> positivePoints = randomPointsInField(0, numberOfPositivePointsInField - 1, INITIAL_NUMBER_OF_POINTS);
        if(positivePoints.size() > 0)
            models.push_back(Model::linearFit(positivePoints));
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels; modelNegNum++){
        std::vector<Point> negativePoints = randomPointsInField(numberOfPositivePointsInField, field.size() - 1, INITIAL_NUMBER_OF_POINTS);
        if(negativePoints.size() > 0)
            models.push_back(Model::linearFit(negativePoints));
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::calculateEnergy() const { //Checked 
    double energy = 0;

    for(Model m : models)
        energy += m.getEnergy();

    energy += calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::meanNumOfPoints() const { //Checked 
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)models.size();
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::redistributePoints() { //Checked
    double newEnergy = 0;

    for(int p = 0; p < field.size(); p++){

        for(int model = 0; model < models.size(); model++){
            double distAt = fabs(models[model].getSlope() * field[p].getX() - field[p].getY() + models[model].getIntercept()) / sqrt(pow(models[model].getSlope(), 2) + 1.0);
            
            if (distAt < this->distanceForOutlier){
                models[model].pushPoint(field[p]);
            }
        }
    }

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::countParallelLines() { //Checked 
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
void RubyGeneticOnePointPosNegInfinite::eraseBadModels(){ //Checked  
    countParallelLines();
    
    for(int i = 0; i < models.size(); i++)
    	models[i].calculateFitness();

    std::sort(models.begin(), models.end());

    int initialPos = std::max((int)(models.size()*0.25), 6);

    for(int i = initialPos; i < models.size(); i++){
        removeModel(i);
        i--;
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::fuseEqualModels(){ //Checked 
    for(int model = 0; model < models.size(); model++){
        for(int model2 = model + 1; model2 < models.size() && model >= 0; model2++){
            if(model2 == model || model2 < 0)
                continue;
        
            if(fabs(models[model].getSlope() - models[model2].getSlope()) < this->sameSlopeThreshold && fabs(models[model].getIntercept() - models[model2].getIntercept()) < this->sameInterceptThreshold){
                int model1Size = models[model].getPointsSize();
                int model2Size = models[model2].getPointsSize();

                if(model1Size >= model2Size){
                	models[model].fuseModel(models[model2]);                	
                	models.erase(models.begin() + model2);
                	model2--;
                }
                else{
                	models[model2].fuseModel(models[model]);                    
                	models.erase(models.begin() + model);
                	model--;
                }
            }
        }
    }
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNegInfinite &r){ //Checked 
    out << "RubyGeneticOnePointPosNegInfinite: [\n\t  Models: Vector {\n";

    for(int i = 0; i < r.models.size(); i++){
        out << "\t\t[" << i << "]: Model: [ a: " << r.models[i].getSlope() << ", b: " << r.models[i].getIntercept() << ", energy: " << r.models[i].getEnergy() << ", parallelCount: " << r.models[i].parallelCount << ", fitness: " << r.models[i].fitness;
        out << "\n\t\t\t\tPositive Points: " << r.models[i].positivePoints << ", Points: Vector {";
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