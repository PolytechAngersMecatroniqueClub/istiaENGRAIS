//********************************************************************************************************
#include "2_RubyGenetic.h"


//--------------------------------------------------------------------------------------------------------
void RubyGenetic::populateOutliers(const sensor_msgs::LaserScan & msg){ //Populate outliers vector with laser scan message 
    double angle = msg.angle_min; //Gets minimum angle 

    this->outliers.clear(); //Clear outliers

    for(int i = 0; i < this->models.size(); i++){ //For each model
        this->models[i].clearPoints(); //Clear points in models
    }

    for(int i = 0; i < msg.ranges.size(); i++){ //For each ray in LIDAR
        if(!isinf(msg.ranges[i])) //If it finds something
            this->outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle))); //Convert to (X,Y) coordinates
        
        angle += msg.angle_increment; //Increment angle
    }

    initialField = this->outliers;
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGenetic::findLines() { //Find the best lines into the cloud of points 
    double newEnergy = MAX_DBL, energy = MAX_DBL;

    std::vector<Model> bestModels;
    std::vector<Point> bestOutliers;

    if(this->outliers.size() != 0){ //If outliers are populated

        for (int it = 0; it < Pearl::maxNumberOfIterations; it++){ //For each iteration
            this->searchModels(RubyGenetic::numberOfModelsToSearch); //Search for models until 40 total

            this->fuseEqualModels(); //Fuse models that are considered equal
            
            this->redistributePoints(); //Redistribute points to closest models

            //int numberMinOfPoints = std::max((int)(this->meanNumOfPoints() * RubyGenetic::factorToDeletePoints), 3);

            this->removeTinyModels(3); //Remove models that have too few points

            this->reEstimation(); //Re estimate models using linear fit
            
            this->eraseBadModels(); //Erase models that are considered bad
            
            newEnergy = this->calculateEnergy(); //Calculate final energy

            if ((newEnergy >= energy)){ //If energy gets bigger, recover last iteration
                this->models = bestModels;
                this->outliers = bestOutliers;
            }
            else{ //Else, save it
                bestModels = this->models;
                bestOutliers = this->outliers;

                energy = newEnergy;
            }
        }
    }  

    return bestModels; //Return best models
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGenetic::randomPointsInField(const int num) const { //Picks 'num' different points in the outlier vector 
    std::vector<Point> ret(num); //Declares return

    std::vector<int> randomNums = Utility::randomDiffVector(0, this->outliers.size() - 1, num); //Selects 'num' random integers

    int pos = 0; //Starting position is 0
    for(int i : randomNums) { //For each random integer
        ret[pos] = this->outliers[i]; //Assing this set of points to the return vector
        pos++;
    }
    
    return ret; //Return random points
}
//--------------------------------------------------------------------------------------------------------
void RubyGenetic::searchModels(const int nbOfModels) { //Searches for 'nbOfModels' models that are possible 
    if(this->outliers.size() < INITIAL_NUMBER_OF_POINTS) //If outliers vector doesn't have enough points
        return;
        
    while(this->models.size() < nbOfModels){ //Search for models
        std::vector<Point> points = this->randomPointsInField(INITIAL_NUMBER_OF_POINTS); //Get a random set of points
        if(points.size() > 0) //If the function returned something useful 
            this->models.push_back(Model::linearFit(points)); //Add the linear fit for this vector of points to the models vector
    }
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGenetic::calculateEnergy() const { //Calculate set's total energy 
    double energy = this->outliers.size() * Pearl::outlierPenalty; //Add the outliers' penalty energy

    for(Model m : this->models) //For each model
        energy += m.getEnergy(); //Gets its energy

    energy += this->calculateAdditionalEnergy(); //Calculate additional energy

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGenetic::meanNumOfPoints() const { //Calculate mean number of points in the models 
    double mean = 0;
    for(Model m : this->models) //For each model
        mean += m.getPointsSize(); //Adds the number of points

    return mean/(double)this->models.size(); //Calculates average
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGenetic::countParallelLines(){ //Calculates parallel count for each model 
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
void RubyGenetic::eraseBadModels(){ //Erases models that are considered bad 

    for(int i = 0; i < this->models.size(); i++) //For each model
        this->models[i].resetParallelCount(); //Reset parallel count
    
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

std::ostream & operator << (std::ostream &out, const RubyGenetic &r){ //Print object 
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