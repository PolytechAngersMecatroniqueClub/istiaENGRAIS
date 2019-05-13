//********************************************************************************************************
#include "4_RubyGeneticOnePointPosNeg.h"


//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    this->numberOfPositivePointsInOutliers = 0;
    this->outliers.clear();

    for(int i = 0; i < this->models.size(); i++){
        this->models[i].clearPoints();
        this->models[i].resetParallelCount();
    }

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, RubyGeneticOnePointPosNeg::distanceToBeConsideredSamePoint)){
                    if(fusedPoint.getY() >= 0){
                        this->outliers.insert(this->outliers.begin() + this->numberOfPositivePointsInOutliers, 1, fusedPoint);
                        this->numberOfPositivePointsInOutliers++;
                    }
                    else
                        this->outliers.push_back(fusedPoint);

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
        this->outliers.insert(this->outliers.begin() + this->numberOfPositivePointsInOutliers, 1, fusedPoint);
        this->numberOfPositivePointsInOutliers++;
    }
    else if(fusedPoint.getX() != MIN_DBL && fusedPoint.getY() != MIN_DBL){
        this->outliers.push_back(fusedPoint);
    }

    initialField = this->outliers;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGeneticOnePointPosNeg::findLines() { //Checked 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(this->outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < 1; it++) {
            this->searchModels(RubyGeneticOnePointPosNeg::numberOfModelsToSearch - this->models.size());
            //cout << "Search ok" << endl;
            //cout << *this << endl;
            this->fuseEqualModels();
            //cout << "fuse ok" << endl;
            //cout << *this << endl;
            this->redistributePoints();
            //cout << "redist ok" << endl;
            //cout << *this << endl;
            numberMinOfPoints = std::max((int)(this->meanNumOfPoints() * RubyGeneticOnePointPosNeg::factorToDeletePoints), 2);

            this->removeTinyModels(2);
            //cout << "remove tiny ok" << endl;
            //cout << *this << endl;
            this->reEstimation();
            //cout << "estim ok" << endl;
            //cout << *this << endl;
            this->eraseBadModels();
            //cout << "erase ok" << endl;
            //cout << *this << endl;
            newEnergy = this->calculateEnergy();
            //cout << "calclate ok" << endl;
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
	if (!(0 <= modelIndex && modelIndex < this->models.size())){
		Utility::printInColor("Model index does not exist", RED); 
		return;
	}
	
    this->outliers.insert(this->outliers.begin() + this->numberOfPositivePointsInOutliers, this->models[modelIndex].getPointsVecBegin(), this->models[modelIndex].getPointsVecBegin() + this->models[modelIndex].getPositivePointsNum());
	this->numberOfPositivePointsInOutliers += this->models[modelIndex].getPositivePointsNum();

    this->outliers.insert(this->outliers.end(), this->models[modelIndex].getPointsVecBegin() + this->models[modelIndex].getPositivePointsNum(), this->models[modelIndex].getPointsVecEnd());

    this->models[modelIndex].clearPoints();
    this->models.erase(this->models.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::clearPointsInModels(){ //Checked 
    for(int i = 0; i < this->models.size(); i++){
	    this->outliers.insert(this->outliers.begin() + this->numberOfPositivePointsInOutliers, this->models[i].getPointsVecBegin(), this->models[i].getPointsVecBegin() + this->models[i].getPositivePointsNum());
		this->numberOfPositivePointsInOutliers += this->models[i].getPositivePointsNum();

	    this->outliers.insert(this->outliers.end(), this->models[i].getPointsVecBegin() + this->models[i].getPositivePointsNum(), this->models[i].getPointsVecEnd());
	        
        this->models[i].clearPoints();
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
        ret[pos] = this->outliers[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::searchModels(const int nbOfModels) { //Checked 
    int numberOfPositiveModels = (int)(nbOfModels/2);
    int numberOfNegativeModels = nbOfModels - numberOfPositiveModels;

    //cout << numberOfNegativeModels << " " << numberOfPositiveModels << endl;

    //cout << this->numberOfPositivePointsInOutliers << " " << this->outliers.size() << endl;

    if(this->outliers.size() < INITIAL_NUMBER_OF_POINTS)
        return;

    if(this->numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
        numberOfPositiveModels = 0;

    if(this->outliers.size() - this->numberOfPositivePointsInOutliers < INITIAL_NUMBER_OF_POINTS)
    	numberOfNegativeModels = 0;
        
    for(int modelPosNum = 0; modelPosNum < numberOfPositiveModels; modelPosNum++){
        //cout << "a" << endl;
        std::vector<Point> positivePoints = this->randomPointsInField(0, this->numberOfPositivePointsInOutliers - 1, INITIAL_NUMBER_OF_POINTS);
        if(positivePoints.size() > 0)
            this->models.push_back(Model::linearFit(positivePoints));
    }

    for(int modelNegNum = 0; modelNegNum < numberOfNegativeModels; modelNegNum++){
        //cout << "b" << endl;
        std::vector<Point> negativePoints = this->randomPointsInField(this->numberOfPositivePointsInOutliers, this->outliers.size() - 1, INITIAL_NUMBER_OF_POINTS);
        if(negativePoints.size() > 0)
            this->models.push_back(Model::linearFit(negativePoints));
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::calculateEnergy() const { //Checked
    double energy = this->outliers.size() * Pearl::outlierPenalty;

    for(Model m : this->models)
        energy += m.getEnergy();

    energy += this->calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::meanNumOfPoints() const { //Checked
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)this->models.size();
}
//--------------------------------------------------------------------------------------------------------
double RubyGeneticOnePointPosNeg::redistributePoints() { //Checked 

    this->clearPointsInModels();

    double newEnergy = 0;

    if(this->outliers.size() > 0){
	    newEnergy = this->outliers.size() * Pearl::outlierPenalty;

	    int dominantModelPos;
	    for(int p = 0; p < this->outliers.size(); p++){
	        double minDist = MAX_DBL;

	        for(int model = 0; model < this->models.size(); model++){
	            double distAt = fabs(this->models[model].getSlope() * this->outliers[p].getX() - this->outliers[p].getY() + this->models[model].getIntercept()) / sqrt(pow(this->models[model].getSlope(), 2) + 1.0);
	            
	            if (distAt < minDist){
	                minDist = distAt;
	                dominantModelPos = model;
	            }
	        }

	        if (minDist < Pearl::distanceForOutlier){
	            newEnergy += minDist - Pearl::outlierPenalty;

	            if(this->outliers[p].getY() >= 0){
	            	this->models[dominantModelPos].pushPoint(this->outliers[p]);
	            	this->numberOfPositivePointsInOutliers--;
	            }
	            else
	            	this->models[dominantModelPos].pushPoint(this->outliers[p]);

	            this->outliers.erase(this->outliers.begin() + p);
	            p--;               
	        }
	    }
	}

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::countParallelLines() { //Checked 
    for(int model = 0; model < this->models.size(); model++){
        for(int model2 = model + 1; model2 < this->models.size(); model2++){
            if(fabs(this->models[model].getSlope() - this->models[model2].getSlope()) < Pearl::sameSlopeThreshold){
                this->models[model].incrementParallelCount();
                this->models[model2].incrementParallelCount();
            }
        }
    }
}
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::eraseBadModels(){ //Checked 
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
void RubyGeneticOnePointPosNeg::fuseEqualModels(){ //Checked 
    //cout << "Models size " << models.size() << endl;

    //cout << *this << endl;
    for(int model = 0; model < this->models.size(); model++){
        for(int model2 = model + 1; model2 < this->models.size() && model >= 0; model2++){
            if(model2 == model || model2 < 0)
                continue;
        
            double slopeRatio = this->models[model].getSlope() / this->models[model2].getSlope();
            double interceptRatio = this->models[model].getIntercept() / this->models[model2].getIntercept();

            double slopeDifference = fabs(this->models[model].getSlope() - this->models[model2].getSlope());
            double interceptDifference = fabs(this->models[model].getIntercept() - this->models[model2].getIntercept());

            bool isSlopeTheSame = ((1 - Pearl::sameSlopeThreshold <= slopeRatio && slopeRatio <= 1 + Pearl::sameSlopeThreshold) || slopeDifference <= Pearl::sameSlopeThreshold);
            bool isInterceptTheSame = ((1 - Pearl::sameInterceptThreshold <= interceptRatio && interceptRatio <= 1 + Pearl::sameInterceptThreshold) || interceptDifference <= Pearl::sameInterceptThreshold);

            if(isSlopeTheSame && isInterceptTheSame){

                int model1Size = this->models[model].getPointsSize();
                int model2Size = this->models[model2].getPointsSize();

                //cout << "entrou if" << endl;

                if(model1Size >= model2Size){
                    //cout << "fuse error" << endl;
                	this->models[model].fuseModel(this->models[model2]); 
                    //cout << "erase error" << endl;               	
                	this->models.erase(this->models.begin() + model2);
                	model2--;
                }
                else{
                    //cout << "fuse error 2" << endl;
                	this->models[model2].fuseModel(this->models[model]); 
                    //cout << "erase error 2" << endl;                   
                	this->models.erase(this->models.begin() + model);
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