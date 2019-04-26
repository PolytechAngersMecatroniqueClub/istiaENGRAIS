//********************************************************************************************************
#include "5_RubyGeneticOnePointPosNegInfinite.h"


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked
    double angle = msg.angle_min;

    this->numberOfPositivePointsInField = 0;
    this->field.clear();

    for(int i = 0; i < this->models.size(); i++){
        this->models[i].clearPoints();
        this->models[i].resetParallelCount();
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, RubyGeneticOnePointPosNegInfinite::distanceToBeConsideredSamePoint)){
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
    else if(fusedPoint.getX() != MIN_DBL && fusedPoint.getY() != MIN_DBL){
        this->field.push_back(fusedPoint);
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNegInfinite::findLines() { //Checked 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(this->field.size() != 0){

        std::vector<Model> bestModels;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            this->clearPointsInModels();

            this->searchModels(RubyGeneticOnePointPosNegInfinite::numberOfModelsToSearch - this->models.size());

            this->fuseEqualModels();

            this->redistributePoints();

            numberMinOfPoints = std::max((int)(this->meanNumOfPoints() * RubyGeneticOnePointPosNegInfinite::factorToDeletePoints), 2);

            this->removeTinyModels(3);

            this->reEstimation();

            this->eraseBadModels();

            newEnergy = this->calculateEnergy();

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
    
    return this->models;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::removeModel(const int modelIndex){ //Checked 
	if (!(0 <= modelIndex && modelIndex < this->models.size())){
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}

    this->models[modelIndex].clearPoints();
    this->models.erase(this->models.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::clearPointsInModels(){ //Checked 
    for(int i = 0; i < this->models.size(); i++){       
        this->models[i].clearPoints();
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
        ret[pos] = this->field[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::searchModels(const int nbOfModels) { //Checked 
    int numberOfPositiveModels = (int)(nbOfModels/2);
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels;

    if(this->field.size() < INITIAL_NUMBER_OF_POINTS)
        return;

    if(this->numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS)
        numberOfPositiveModels = 0;

    if(this->field.size() - this->numberOfPositivePointsInField < INITIAL_NUMBER_OF_POINTS)
        numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels; modelPosNum++){
        std::vector<Point> positivePoints = this->randomPointsInField(0, this->numberOfPositivePointsInField - 1, INITIAL_NUMBER_OF_POINTS);
        if(positivePoints.size() > 0)
            this->models.push_back(Model::linearFit(positivePoints));
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels; modelNegNum++){
        std::vector<Point> negativePoints = this->randomPointsInField(this->numberOfPositivePointsInField, this->field.size() - 1, INITIAL_NUMBER_OF_POINTS);
        if(negativePoints.size() > 0)
            this->models.push_back(Model::linearFit(negativePoints));
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::calculateEnergy() const { //Checked 
    double energy = 0;

    for(Model m : this->models)
        energy += m.getEnergy();

    energy += this->calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::meanNumOfPoints() const { //Checked 
    double mean = 0;
    for(Model m : this->models)
        mean += m.getPointsSize();

    return mean/(double)this->models.size();
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNegInfinite::redistributePoints() { //Checked
    double newEnergy = 0;

    for(int p = 0; p < this->field.size(); p++){

        for(int model = 0; model < this->models.size(); model++){
            double distAt = fabs(this->models[model].getSlope() * this->field[p].getX() - this->field[p].getY() + this->models[model].getIntercept()) / sqrt(pow(this->models[model].getSlope(), 2) + 1.0);
            
            if (distAt < this->distanceForOutlier){
                this->models[model].pushPoint(this->field[p]);
            }
        }
    }

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::countParallelLines() { //Checked 
    for(int model = 0; model < this->models.size(); model++){
        for(int model2 = model + 1; model2 < this->models.size(); model2++){
            if(fabs(this->models[model].getSlope() - this->models[model2].getSlope()) < this->sameSlopeThreshold){
                this->models[model].incrementParallelCount();
                this->models[model2].incrementParallelCount();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::eraseBadModels(){ //Checked  
    this->countParallelLines();
    
    for(int i = 0; i < this->models.size(); i++)
    	this->models[i].calculateFitness();

    std::sort(this->models.begin(), this->models.end());

    int initialPos = std::max((int)(this->models.size()*0.25), 6);

    for(int i = initialPos; i < this->models.size(); i++){
        this->removeModel(i);
        i--;
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNegInfinite::fuseEqualModels(){ //Checked 
    for(int model = 0; model < this->models.size(); model++){
        for(int model2 = model + 1; model2 < this->models.size() && model >= 0; model2++){
            if(model2 == model || model2 < 0)
                continue;
        
            if(fabs(this->models[model].getSlope() - this->models[model2].getSlope()) < this->sameSlopeThreshold && fabs(this->models[model].getIntercept() - this->models[model2].getIntercept()) < this->sameInterceptThreshold){
                int model1Size = this->models[model].getPointsSize();
                int model2Size = this->models[model2].getPointsSize();

                if(model1Size >= model2Size){
                	this->models[model].fuseModel(this->models[model2]);                	
                	this->models.erase(this->models.begin() + model2);
                	model2--;
                }
                else{
                	this->models[model2].fuseModel(this->models[model]);                    
                	this->models.erase(this->models.begin() + model);
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