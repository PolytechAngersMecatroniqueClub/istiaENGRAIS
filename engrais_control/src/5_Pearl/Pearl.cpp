//********************************************************************************************************
#include "Pearl.h"


//--------------------------------------------------------------------------------------------------------
void Pearl::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    //std::cout << "Pearl" << std::endl;
    double angle = msg.angle_min;

    this->outliers.clear();
    this->models.clear();

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            this->outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> Pearl::findLines() { //Checked 
	double energy = MAX_DBL;

	if(this->outliers.size() != 0){
	    int nModels = int(this->outliers.size() / Pearl::divideFactor);

	    if(nModels > 0){
	        this->searchModels(nModels);

	        double newEnergy = this->expansionEnergy();

	        newEnergy += this->removeTinyModels((int)(1.5*nModels));

	        energy = newEnergy;

	        std::vector<Model> bestModels = this->models;
	        std::vector<Point> bestOutliers = this->outliers;

	        for(int nbOfIteractions = 0; nbOfIteractions < Pearl::maxNumberOfIterations; nbOfIteractions++) {
	            this->searchModels((int)(this->outliers.size() / Pearl::divideFactor));

	            this->reEstimation();

	            this->eraseBadModels(Pearl::worstEnergySizeRatioAllowed);

	            this->fuseEqualModels();

	            newEnergy = this->expansionEnergy();

	            if ((newEnergy > energy)){
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
	}

	return this->models;
}
//--------------------------------------------------------------------------------------------------------    
void Pearl::removeModel(const int modelIndex){ //Checked 
	if (!(0 <= modelIndex && modelIndex < models.size())){
        Utility::printInColor("Wrong index for removing model", RED);
		exit(1);
    }

    this->outliers.insert(this->outliers.end(), this->models[modelIndex].getPointsVecBegin(), this->models[modelIndex].getPointsVecEnd());
    this->models[modelIndex].clearPoints();
    this->models.erase(this->models.begin() + modelIndex);
}
//--------------------------------------------------------------------------------------------------------
void Pearl::removePointsInModels(){ //Checked 
    for(int i = 0; i < this->models.size(); i++){
        this->outliers.insert(this->outliers.end(), this->models[i].getPointsVecBegin(), this->models[i].getPointsVecEnd());
        this->models[i].clearPoints();
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> Pearl::randomPointsInField(const int num) { //Checked 
    std::vector<Point> ret;

    if(this->outliers.size() < num){
        Utility::printInColor("Not enough outliers, please verify", RED);
        return std::vector<Point>();
    }

    for(int i = 0; i < num; i++){
        int randomNum = Utility::randomInt(0, this->outliers.size() - 1);

        if(this->outliers[randomNum].getY() >= 0)
            ret.insert(ret.begin(), 1, this->outliers[randomNum]);
        else
            ret.push_back(this->outliers[randomNum]);

        this->outliers.erase(this->outliers.begin() + randomNum);
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
void Pearl::searchModels(const int nbOfModels) { //Checked 
    Model model;
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++){
        std::vector<Point> points = this->randomPointsInField(INITIAL_NUMBER_OF_POINTS);
        if(points.size() != 0){
            model.findBestModel(points);

            this->models.push_back(model);
        }
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double Pearl::redistributePoints() { //Checked 
    this->removePointsInModels();

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

	            this->models[dominantModelPos].pushPoint(this->outliers[p]);
	            this->outliers.erase(this->outliers.begin() + p);
	            p--;               
	        }
	    }
	}

    return newEnergy;
}
//--------------------------------------------------------------------------------------------------------
double Pearl::removeTinyModels(const int minimum_points) { //Checked 
    double gainEnergy = 0;
    for(int model = 0; model < this->models.size(); model++){
        if(this->models[model].getPointsSize() <= minimum_points){
            gainEnergy += this->models[model].getPointsSize() * Pearl::outlierPenalty - this->models[model].getEnergy();
            removeModel(model);
            model--;
        }
    }
    return gainEnergy;
}
//--------------------------------------------------------------------------------------------------------
double Pearl::calculateAdditionalEnergy() const { //Checked 
    double addEnergy = 0;
    for(int model = 0; model < this->models.size(); model++)
        for(int model2 = model + 1; model2 < this->models.size(); model2++)
            for(Point p : this->models[model].getPointsInModel())
                for(Point q : this->models[model2].getPointsInModel())
                    addEnergy += Pearl::additionalEnergyLambda * exp(-(pow(p.getX() - q.getX(), 2) + pow(p.getY() - q.getY(), 2)) / pow(Pearl::additionalEnergyCsi, 2));

    return addEnergy;                           
}
//--------------------------------------------------------------------------------------------------------
double Pearl::expansionEnergy() { //Checked 
    double newEnergy = this->redistributePoints();
    newEnergy += this->removeTinyModels(INITIAL_NUMBER_OF_POINTS);

    newEnergy += this->calculateAdditionalEnergy();

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Pearl::reEstimation() { //Checked 
    for(int model = 0; model < this->models.size(); model++){ 
        this->models[model].findBestModel();
    }
}

//########################################################################################################

void Pearl::eraseBadModels(const double threshRatio) { //Checked 
    for(int model = 0; model < this->models.size(); model++){
        if ((this->models[model].getEnergy() / (double)this->models[model].getPointsSize()) >= threshRatio){
            this->removeModel(model);
            model--;
        }
    }
}

//########################################################################################################

void Pearl::fuseEqualModels(){ //Checked 
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
// -------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Pearl &p){ //Checked 
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