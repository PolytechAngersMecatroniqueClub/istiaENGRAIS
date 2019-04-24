//********************************************************************************************************
#include "4_RubyGeneticOnePointPosNeg.h"


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    this->numberOfPositivePointsInOutliers = 0;
    outliers.clear();

    for(int i = 0; i < models.size(); i++){
        models[i].clearPoints();
        models[i].resetParallelCount();
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, this->distanceToBeConsideredSamePoint)){
                    if(fusedPoint.getY() >= 0){
                        outliers.insert(outliers.begin(), 1, fusedPoint);
                        this->numberOfPositivePointsInOutliers++;
                    }
                    else
                        outliers.push_back(fusedPoint);

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
        outliers.insert(outliers.begin(), 1, fusedPoint);
        this->numberOfPositivePointsInOutliers++;
    }
    else if(fusedPoint.getX() != MIN_DBL && fusedPoint.getY() != MIN_DBL){
        outliers.push_back(fusedPoint);
    }

    initialField = outliers;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNeg::findLines() { //Checked 
    //std::cout << "RubyGeneticOnePointPosNeg" << std::endl;
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            searchModels(this->numberOfModelsToSearch - models.size());

            fuseEqualModels();

            redistributePoints();

            numberMinOfPoints = std::max((int)(meanNumOfPoints() * this->factorToDeletePoints), 2);

            removeTinyModels(2);

            reEstimation();

            eraseBadModels();

            newEnergy = calculateEnergy();

            if ((newEnergy >= energy)){
                this->models = bestModels;
                this->outliers = bestOutliers;

                energy = newEnergy;
            }
            else{
                bestModels = this->models;
                bestOutliers = this->outliers;

                newEnergy = energy;
            }
        }
    }

    return this->models;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::removeModel(const int modelIndex){ //Checked 
	if (!(0 <= modelIndex && modelIndex < models.size())){
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}
	
    outliers.insert(outliers.begin(), models[modelIndex].getPointsVecBegin(), models[modelIndex].getPointsVecBegin() + models[modelIndex].getPositivePointsNum());
	this->numberOfPositivePointsInOutliers += models[modelIndex].getPositivePointsNum();

    outliers.insert(outliers.end(), models[modelIndex].getPointsVecBegin() + models[modelIndex].getPositivePointsNum(), models[modelIndex].getPointsVecEnd());

    models[modelIndex].clearPoints();
    models.erase(models.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::clearPointsInModels(){ //Checked 
    for(int i = 0; i < models.size(); i++){
	    outliers.insert(outliers.begin(), models[i].getPointsVecBegin(), models[i].getPointsVecBegin() + models[i].getPositivePointsNum());
		numberOfPositivePointsInOutliers += models[i].getPositivePointsNum();

	    outliers.insert(outliers.end(), models[i].getPointsVecBegin() + models[i].getPositivePointsNum(), models[i].getPointsVecEnd());
	        
        models[i].clearPoints();
        this->models[i].resetParallelCount();
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGeneticOnePointPosNeg::randomPointsInField(const int minNum, const int maxNum, const int num) const { //Checked 
    std::vector<Point> ret(num);

    std::vector<int> randomNums = Utility::randomDiffVector(minNum, maxNum, num);

    int pos = 0;
    for(int i : randomNums){
        ret[pos] = outliers[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::searchModels(const int nbOfModels) { //Checked 
    int numberOfPositiveModels = (int)(nbOfModels/2);
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels;

    if(outliers.size() < INITIAL_NUMBER_OF_POINTS)
        return;

    if(numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
        numberOfPositiveModels = 0;

    if(outliers.size() - numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
    	numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels; modelPosNum++){
        std::vector<Point> positivePoints = randomPointsInField(0, numberOfPositivePointsInOutliers - 1, INITIAL_NUMBER_OF_POINTS);
        if(positivePoints.size() > 0)
            models.push_back(Model::linearFit(positivePoints));
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels; modelNegNum++){
        std::vector<Point> negativePoints = randomPointsInField(numberOfPositivePointsInOutliers, outliers.size() - 1, INITIAL_NUMBER_OF_POINTS);
        if(negativePoints.size() > 0)
            models.push_back(Model::linearFit(negativePoints));
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::calculateEnergy() const { //Checked
    double energy = outliers.size() * this->outlierPenalty;

    for(Model m : models)
        energy += m.getEnergy();

    energy += calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::meanNumOfPoints() const { //Checked
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)models.size();
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::redistributePoints() { //Checked 

    clearPointsInModels();

    double newEnergy = 0;

    if(outliers.size() > 0){
	    newEnergy = outliers.size() * this->outlierPenalty;

	    int dominantModelPos;
	    for(int p = 0; p < outliers.size(); p++){
	        double minDist = MAX_DBL;

	        for(int model = 0; model < models.size(); model++){
	            double distAt = fabs(models[model].getSlope() * outliers[p].getX() - outliers[p].getY() + models[model].getIntercept()) / sqrt(pow(models[model].getSlope(), 2) + 1.0);
	            
	            if (distAt < minDist){
	                minDist = distAt;
	                dominantModelPos = model;
	            }
	        }

	        if (minDist < this->distanceForOutlier){
	            newEnergy += minDist - this->outlierPenalty;

	            if(outliers[p].getY() >= 0){
	            	models[dominantModelPos].pushPoint(outliers[p]);
	            	numberOfPositivePointsInOutliers--;
	            }
	            else
	            	models[dominantModelPos].pushPoint(outliers[p]);

	            outliers.erase(outliers.begin() + p);
	            p--;               
	        }
	    }
	}

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::countParallelLines() { //Checked 
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
void RubyGeneticOnePointPosNeg::eraseBadModels(){ //Checked 
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
void RubyGeneticOnePointPosNeg::fuseEqualModels(){ //Checked 
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

std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNeg &r){ //Checked 
    out << "RubyGeneticOnePointPosNeg: [\n\t  Models: Vector {\n";

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