//********************************************************************************************************
#include "2_RubyGenetic.h"


//--------------------------------------------------------------------------------------------------------
void RubyGenetic::populateOutliers(const sensor_msgs::LaserScan & msg){ //Checked 
    double angle = msg.angle_min;

    this->outliers.clear();

    for(int i = 0; i < this->models.size(); i++){
        this->models[i].clearPoints();
        this->models[i].resetParallelCount();
    }

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            this->outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGenetic::findLines() { //Checked 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(this->outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            this->searchModels(RubyGenetic::numberOfModelsToSearch - models.size());

            this->fuseEqualModels();
            
            this->redistributePoints();

            numberMinOfPoints = std::max((int)(this->meanNumbOfPoints() * RubyGenetic::factorToDeletePoints), 3);

            this->removeTinyModels(numberMinOfPoints);

            this->reEstimation();
            
            this->eraseBadModels();
            
            newEnergy = this->calculateEnergy();

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

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGenetic::randomPointsInField(const int num) const { //Checked 
    std::vector<Point> ret(num);

    std::vector<int> randomNums = Utility::randomDiffVector(0, this->outliers.size() - 1, num);

    int pos = 0;
    for(int i : randomNums) {
        ret[pos] = this->outliers[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGenetic::searchModels(const int nbOfModels) { //Checked 
    if(this->outliers.size() < INITIAL_NUMBER_OF_POINTS)
        return;
        
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++){
        std::vector<Point> points = this->randomPointsInField(INITIAL_NUMBER_OF_POINTS);
        if(points.size() > 0)
            this->models.push_back(Model::linearFit(points));
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGenetic::calculateEnergy() const { //Checked 
    double energy = this->outliers.size() * this->outlierPenalty;

    for(Model m : models)
        energy += m.getEnergy();

    energy += this->calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGenetic::meanNumbOfPoints() const { //Checked 
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)this->models.size();
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGenetic::countParallelLines(){ //Checked 
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
void RubyGenetic::eraseBadModels(){ //Checked 
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

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGenetic &r){ //Checked 
    out << "RubyGenetic: [\n\t  Models: Vector {\n";

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