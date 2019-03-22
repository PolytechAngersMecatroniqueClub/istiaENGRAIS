//********************************************************************************************************
#include "2_RubyGenetic.h"

using namespace std;
//--------------------------------------------------------------------------------------------------------
void RubyGenetic::populateOutliers(const sensor_msgs::LaserScan & msg){ 
    double angle = msg.angle_min;

    outliers.clear();

    for(int i = 0; i < models.size(); i++)
        models[i].clearPoints();

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i]))
            outliers.push_back(Point(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle)));
        
        angle += msg.angle_increment;
    }
}
//--------------------------------------------------------------------------------------------------------
std::vector<Model> RubyGenetic::findLines() { 
    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            searchModels(this->numberOfModelsToSearch - models.size());

            redistributePoints();
            
            fuseEqualModels();

            numberMinOfPoints = std::max((int)(meanNumbOfPoints() * this->factorToDeletePoints), 3);

            removeTinyModels(numberMinOfPoints);

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

    else{
        Utility::printInColor("No data in field, please verify", RED);
    }   

    return models;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGenetic::randomPointsInField(const int num) const { 
    std::vector<Point> ret(num);

    std::vector<int> randomNums = Utility::randomDiffVector(0, outliers.size() - 1, num);

    int pos = 0;
    for(int i : randomNums) {
        ret[pos] = outliers[i];
        pos++;
    }
    
    return ret;
}
//--------------------------------------------------------------------------------------------------------
void RubyGenetic::searchModels(const int nbOfModels) { 
    if(outliers.size() < INITIAL_NUMBER_OF_POINTS)
            return;
        
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++)
        models.push_back(Model::linearFit(randomPointsInField(INITIAL_NUMBER_OF_POINTS)));
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
double RubyGenetic::calculateEnergy() const { 
    double energy = 0;

    for(Model m : models)
        energy += m.getEnergy();

    energy += calculateAdditionalEnergy();

    return energy;
}
//--------------------------------------------------------------------------------------------------------
double RubyGenetic::meanNumbOfPoints() const { 
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)models.size();
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
void RubyGenetic::countParallelLines(){ 
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
void RubyGenetic::eraseBadModels(){  
    countParallelLines();
    
    for(int i = 0; i < models.size(); i++)
    	models[i].calculateFitness();

    std::sort(models.begin(), models.end());

    int initialPos = max((int)(models.size()*0.25), 6);

    for(int i = initialPos; i < models.size(); i++){
    	removeModel(i);
    	i--;
    }
}	

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGenetic &r){ 
    out << "RubyGenetic: [\n\tModels: Vector {\n";
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
    return out; 
}

//********************************************************************************************************