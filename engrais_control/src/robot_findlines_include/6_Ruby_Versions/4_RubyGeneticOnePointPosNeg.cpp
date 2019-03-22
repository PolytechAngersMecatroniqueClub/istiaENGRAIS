//********************************************************************************************************
#include "4_RubyGeneticOnePointPosNeg.h"

using namespace std;


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::populateOutliers(const sensor_msgs::LaserScan & msg){ 
    double angle = msg.angle_min;

    numberOfPositivePointsInOutliers = 0;
    outliers.clear();

    for(int i = 0; i < models.size(); i++){
        models[i].clearPoints();
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
            	//cout << *this << endl<< endl<< endl;
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, this->distanceToBeConsideredSamePoint)){
                    if(fusedPoint.getY() >= 0){
                        outliers.insert(outliers.begin(), 1, fusedPoint);
                        numberOfPositivePointsInOutliers++;
                    }
                    else
                        outliers.insert(outliers.end(), 1, fusedPoint);

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
        numberOfPositivePointsInOutliers++;
    }
    else{
        outliers.push_back(fusedPoint);
    }

    initialField = outliers;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNeg::findLines() { 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < 1; it++) {
            searchModels(this->numberOfModelsToSearch - models.size());
            //cout << "search ok" << endl;
            redistributePoints();
            //cout << "fuse ok" << endl;
            fuseEqualModels();
            //cout << "redistri ok" << endl;
            numberMinOfPoints = std::max((int)(meanNumOfPoints() * this->factorToDeletePoints), 3);

            removeTinyModels(numberMinOfPoints);
            //cout << "remove ok" << endl;
            reEstimation();
            //cout << "reestim ok" << endl;

            eraseBadModels();
            //cout << "eraseBad ok" << endl;

            newEnergy = calculateEnergy();
            //cout << "calculate ok" << endl;

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

    else{
        Utility::printInColor("No data in field, please verify", RED);
    }   

    return this->models;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::removeModel(const int modelIndex){
	if (!(0 <= modelIndex && modelIndex < models.size())){
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}
	
    outliers.insert(outliers.begin(), models[modelIndex].getPointsVecBegin(), models[modelIndex].getPointsVecBegin() + numberOfPositivePointsInModels[modelIndex]);
	numberOfPositivePointsInOutliers += numberOfPositivePointsInModels[modelIndex];

    outliers.insert(outliers.end(), models[modelIndex].getPointsVecBegin() + numberOfPositivePointsInModels[modelIndex], models[modelIndex].getPointsVecEnd());


    models[modelIndex].clearPoints();
    models.erase(models.begin() + modelIndex);
    numberOfPositivePointsInModels.erase(numberOfPositivePointsInModels.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::clearPointsInModels(){
    for(int i = 0; i < models.size(); i++){
	    outliers.insert(outliers.begin(), models[i].getPointsVecBegin(), models[i].getPointsVecBegin() + numberOfPositivePointsInModels[i]);
		numberOfPositivePointsInOutliers += numberOfPositivePointsInModels[i];

	    outliers.insert(outliers.end(), models[i].getPointsVecBegin() + numberOfPositivePointsInModels[i], models[i].getPointsVecEnd());
	        
        models[i].clearPoints();
        models[i].setEnergy(0);

        numberOfPositivePointsInModels[i] = 0;
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGeneticOnePointPosNeg::randomPointsInField(const int minNum, const int maxNum, const int num) const { 
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
void RubyGeneticOnePointPosNeg::searchModels(const int nbOfModels) { 
    int numberOfPositiveModels = (int)(nbOfModels/2);
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels;

    if(outliers.size() < INITIAL_NUMBER_OF_POINTS)
        return;

    if(numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
        numberOfPositiveModels = 0;

    if(outliers.size() - numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
    	numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels; modelPosNum++){
        models.push_back(Model::linearFit(randomPointsInField(0, numberOfPositivePointsInOutliers - 1, INITIAL_NUMBER_OF_POINTS)));
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels; modelNegNum++){
        models.push_back(Model::linearFit(randomPointsInField(numberOfPositivePointsInOutliers, outliers.size() - 1, INITIAL_NUMBER_OF_POINTS)));
    }

    numberOfPositivePointsInModels = vector<int>(models.size(), 0);
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::calculateEnergy() const { 
    double energy = 0;

    for(Model m : models)
        energy += m.getEnergy();

    energy += calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::meanNumOfPoints() const { 
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)models.size();
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::redistributePoints() {

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

	            models[dominantModelPos].addEnergy(minDist);
	            if(outliers[p].getY() >= 0){
	            	models[dominantModelPos].pushPointAtBeginning(outliers[p]);
	            	numberOfPositivePointsInModels[dominantModelPos]++;
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
void RubyGeneticOnePointPosNeg::countParallelLines() { 
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
void RubyGeneticOnePointPosNeg::eraseBadModels(){  
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

                	numberOfPositivePointsInModels[model] += numberOfPositivePointsInModels[model2];
                	numberOfPositivePointsInModels.erase(numberOfPositivePointsInModels.begin() + model2);

                	models.erase(models.begin() + model2);
                	model2--;
                }
                else{
                	models[model2].fuseModel(models[model]);

                	numberOfPositivePointsInModels[model2] += numberOfPositivePointsInModels[model];
                	numberOfPositivePointsInModels.erase(numberOfPositivePointsInModels.begin() + model);

                	models.erase(models.begin() + model);
                	model--;
                }
            }
        }
    }
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNeg &r){ 
    out << "RubyGeneticOnePointPosNeg: [ Positive Points In Outliers : " << r.numberOfPositivePointsInOutliers << "\n\tModels: Vector {\n";
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

    out << "\t}\n\nNumber Positive Points: Vector {";

    for(int outP = 0; outP < r.numberOfPositivePointsInModels.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << r.numberOfPositivePointsInModels[outP];
    }

    out << "\n\t}";
    out << "\n]";

    return out; 
}

//********************************************************************************************************