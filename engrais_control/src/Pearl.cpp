//********************************************************************************************************
#include "Pearl.h"

//--------------------------------------------------------------------------------------------------------
void Pearl::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    outliers.clear();
    models.clear();

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }
}
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> Pearl::findLines() { //Checked 
	std::pair<Model, Model> ret;

	double energy = MAX_DBL;

	if(outliers.size() != 0){
	    int nModels = int(outliers.size() / this->divideFactor);

	    if(nModels > 0){
	        searchModels(nModels);

	        double newEnergy = expansionEnergy();

	        newEnergy += removeTinyModels((int)(1.5*nModels));

	        int nbOfIteractions = 0;

	        energy = newEnergy;

	        std::vector<Model> bestModels = this->models;
	        std::vector<Point> bestOutliers = this->outliers;

	        while (nbOfIteractions < this->maxNumberOfIterations) {
	            searchModels((int)(outliers.size() / this->divideFactor));

	            reEstimation();

	            eraseBadModels(this->worstEnergySizeRatioAllowed);
	            
	            fuseEqualModels();

	            newEnergy = expansionEnergy();

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

	            nbOfIteractions++;
	        }
	    }
	}

	else{
	    Utility::printInColor("No data in field, please verify", RED);
	}

	return findBestModels();
}
//--------------------------------------------------------------------------------------------------------    
void Pearl::removeModel(const int modelIndex){ //Checked 
	if (!(0 <= modelIndex && modelIndex < models.size()))
		return;

    outliers.insert(outliers.end(), models[modelIndex].getPointsVecBegin(), models[modelIndex].getPointsVecEnd());
    models[modelIndex].clearPoints();
    models.erase(models.begin() + modelIndex);
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> Pearl::randomPointsInField(const int num) { //Checked 
    std::vector<Point> ret(num);

    if(outliers.size() < num){
        Utility::printInColor("Not enough outliers, please verify", RED);
        return ret;
    }

    for(int i = 0; i < num; i++){
        int randomNum = Utility::randomInt(0, outliers.size() - 1);

        ret[i] = outliers[randomNum];
        outliers.erase(outliers.begin() + randomNum);
    }

    return ret;
}
//--------------------------------------------------------------------------------------------------------
void Pearl::searchModels(const int nbOfModels) { //Checked 
    Model model;
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++){
        model.findBestModel(randomPointsInField(INITIAL_NUMBER_OF_POINTS));
        models.push_back(model);
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double Pearl::redistributePoints() { //Checked 
    for(int i = 0; i < models.size(); i++){
        outliers.insert(outliers.end(), models[i].getPointsVecBegin(), models[i].getPointsVecEnd());
        models[i].clearPoints();
        models[i].setEnergy(0);
    }

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
	            models[dominantModelPos].pushPoint(outliers[p]);
	            outliers.erase(outliers.begin() + p);
	            p--;               
	        }
	    }
	}

    return newEnergy;
}
//--------------------------------------------------------------------------------------------------------
double Pearl::removeTinyModels(const int minimum_points) { //Checked 
    double gainEnergy = 0;
    for(int model = 0; model < models.size(); model++){
        if(models[model].getPointsSize() <= minimum_points){
            gainEnergy += models[model].getPointsSize()*this->outlierPenalty - models[model].getEnergy();
            removeModel(model);
            model--;
        }
    }
    return gainEnergy;
}
//--------------------------------------------------------------------------------------------------------
double Pearl::calculateAdditionalEnergy() const { //Checked 
    double addEnergy = 0;
    for(int model = 0; model < models.size(); model++)
        for(int model2 = model + 1; model2 < models.size(); model2++)
            for(Point p : models[model].getPointsInModel())
                for(Point q : models[model2].getPointsInModel())
                    addEnergy += this->additionalEnergyLambda * exp(-(pow(p.getX() - q.getX(), 2) + pow(p.getY() - q.getY(), 2)) / pow(this->additionalEnergyCsi, 2));

    return addEnergy;                           
}
//--------------------------------------------------------------------------------------------------------
double Pearl::expansionEnergy() { //Checked
    double newEnergy = redistributePoints();
    newEnergy += removeTinyModels(2);

    newEnergy += calculateAdditionalEnergy();

    return newEnergy;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void Pearl::reEstimation() { //Checked 
    for(int model = 0; model < models.size(); model++){ 
        models[model].findBestModel();
    }
}

//########################################################################################################

void Pearl::eraseBadModels(const double threshRatio) { //Checked
    for(int model = 0; model < models.size(); model++){
        if ((models[model].getEnergy() / (double)models[model].getPointsSize()) >= threshRatio){
            removeModel(model);
            model--;
        }
    }
}

//########################################################################################################

void Pearl::fuseEqualModels(){ //Checked 
    for(int model = 0; model < models.size(); model++){
        for(int model2 = model + 1; model2 < models.size() && model2 != model && model >= 0 && model2 >= 0; model2++){
        
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
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> Pearl::findBestModels() const { //Checked
    std::pair<Model, Model> ret; //first = left, second = right
    
    double fitness;
    double bestFitnessLeft = MAX_DBL;
    double bestFitnessRight = MAX_DBL;

    int bestLeftPos = MIN_INT;
    int bestRightPos = MIN_INT; //MAX_INT
    
    for(int model = 0; model < models.size(); model++){
        fitness = models[model].getPointsSize() != 0 ? (fabs(models[model].getIntercept()) + models[model].getEnergy()) / (double)(models[model].getPointsSize()) : MAX_DBL;

        if(fitness < bestFitnessLeft && models[model].getIntercept() >= 0){
            bestFitnessLeft = fitness;
            bestLeftPos = model;
        }
        else if(fitness < bestFitnessRight && models[model].getIntercept() < 0){
            bestFitnessRight = fitness;
            bestRightPos = model;
        }
    }

    if(bestLeftPos != MIN_INT){
        ret.first = models[bestLeftPos];
        ret.first.clearPoints();
        ret.first.setEnergy(0);
    }

    if(bestRightPos != MIN_INT){
        ret.second = models[bestRightPos];
        ret.second.clearPoints();
        ret.second.setEnergy(0);
    }
}
// -------------------------------------------------------------------------------------------------------
std::ostream & operator << (std::ostream &out, const Pearl &p){ //Checked
    out << "Pearl: [\n\tModels: Vector {\n";
    for(Model m : p.models){
        out << "\t\tModel: [ a: " << m.getSlope() << ", b: " << m.getIntercept() << ", energy: " << m.getEnergy();
        out << "\n\t\t\tPoints: Vector {";
        int pos = 0;
        for(Point p : m.getPointsInModel()){
            out << "\n\t\t\t\t[" << pos << "]: " << p;
            pos++;
        }
        out << "\n\t\t\t}\n\t\t]\n";
    }

    out << "\t}\n\n\tOutliers: Vector {";

    for(int outP = 0; outP < p.outliers.size(); outP++){
        out << "\n\t\t[" << outP << "]: " << p.outliers[outP];
    }

    out << "\n\t}";
    out << "\n]";
    return out; 
}

//********************************************************************************************************