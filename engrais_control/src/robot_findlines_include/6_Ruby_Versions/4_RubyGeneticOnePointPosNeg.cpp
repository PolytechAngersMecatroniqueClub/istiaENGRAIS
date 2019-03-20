//********************************************************************************************************
#include "4_RubyGeneticOnePointPosNeg.h"

using namespace std;
//--------------------------------------------------------------------------------------------------------
void RubyGeneticOnePointPosNeg::populateOutliers(const sensor_msgs::LaserScan & msg){ 
    double angle = msg.angle_min;

    outliers.clear();

    for(int i = 0; i < models.size(); i++)
        models[i].clearPoints();

    WeightedPoint fusedPoint;

    for(int i = 0; i < msg.ranges.size(); i++){
        if(!isinf(msg.ranges[i])){
            if(!(fusedPoint.getX() == MIN_DBL && fusedPoint.getY() == MIN_DBL)){
                WeightedPoint p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                if(!fusedPoint.fusePoint(p, this->distanceToBeConsideredSamePoint)){
                    if(p.getY() >= 0){
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

    if(fusedPoint.getY() >= 0)
        outliers.insert(outliers.begin(), 1, fusedPoint);
    else
        outliers.insert(outliers.end(), 1, fusedPoint);

    initialField = outliers;
}
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RubyGeneticOnePointPosNeg::findLines() { 
    std::pair<Model, Model> bestPair;
    std::pair<Model, Model> tempPair;

    int numberMinOfPoints;

    double newEnergy = MAX_DBL, energy = MAX_DBL;

    if(outliers.size() != 0){

        std::vector<Model> bestModels;
        std::vector<Point> bestOutliers;

        for (int it = 0; it < this->maxNumberOfIterations; it++) {
            searchModels(this->numberOfModelsToSearch);

            redistributePoints();

            fuseEqualModels();

            numberMinOfPoints = std::max((int)(meanNumbOfPoints() * this->factorToDeletePoints), 3);

            removeTinyModels(numberMinOfPoints);

            reEstimation();
            
            tempPair = eraseBadModels();
            
            newEnergy = calculateEnergy();

            if ((newEnergy >= energy)){
                this->models = bestModels;
                this->outliers = bestOutliers;
                tempPair = bestPair;

                energy = newEnergy;
            }
            else{
                bestModels = this->models;
                bestOutliers = this->outliers;
                bestPair = tempPair;

                newEnergy = energy;
            }
        }
    }

    else{
        Utility::printInColor("No data in field, please verify", RED);
    }   

    return bestPair;
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<Point> RubyGeneticOnePointPosNeg::randomPointsInField(const int num) const { 
    std::vector<Point> ret(num);

    std::vector<int> randomNums = Utility::randomDiffVector(0, outliers.size() - 1, num);

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

    if()
        
    for(int modelNum = 0; modelNum < nbOfModels; modelNum++)
        models.push_back(Model::linearFit(randomPointsInField(INITIAL_NUMBER_OF_POINTS)));
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
double RubyGeneticOnePointPosNeg::meanNumbOfPoints() const { 
    double mean = 0;
    for(Model m : models)
        mean += m.getPointsSize();

    return mean/(double)models.size();
}

//########################################################################################################

//--------------------------------------------------------------------------------------------------------
std::vector<int> RubyGeneticOnePointPosNeg::countParallelLines() const { 
    std::vector<int> ret(models.size(), 1);

    for(int model = 0; model < models.size(); model++){
        for(int model2 = model + 1; model2 < models.size(); model2++){
            fabs(models[model].getSlope() - models[model].getSlope());
            if(fabs(models[model].getSlope() - models[model2].getSlope()) < this->sameSlopeThreshold){
                ret[model]++;
                ret[model2]++;
            }
        }
    }
    return ret;
}
//--------------------------------------------------------------------------------------------------------
std::pair<Model, Model> RubyGeneticOnePointPosNeg::eraseBadModels() { 
    std::pair<Model, Model> ret; //first = left, second = right
    
    double fitness;
    double bestFitnessLeft = MAX_DBL;
    double bestFitnessRight = MAX_DBL;

    int bestLeftPos = MIN_INT;
    int bestRightPos = MIN_INT; //MAX_INT

    std::vector<int> parrallel = countParallelLines();

    /*Utility::printInColor("Before Erase Bad Models", RED);
    std::cout << (*this) << std::endl;*/
    
    for(int model = 0; model < models.size(); model++){
        fitness = models[model].getPointsSize() != 0 ? (models[model].getIntercept()*models[model].getIntercept() + models[model].getEnergy()*10) / (double)(models[model].getPointsSize()*models[model].getPointsSize()*parrallel[model]) : MAX_DBL;

        if(fitness < bestFitnessLeft && models[model].getIntercept() >= 0){
            bestFitnessLeft = fitness;

            if(bestLeftPos != MIN_INT){
                removeModel(bestLeftPos);

                if(bestRightPos > bestLeftPos)
                    bestRightPos--;

                bestLeftPos = --model;
            }
            else{
                bestLeftPos = model;
            }

        }
        else if(fitness < bestFitnessRight && models[model].getIntercept() < 0){
            bestFitnessRight = fitness;

            if(bestRightPos != MIN_INT){
                removeModel(bestRightPos);

                if(bestLeftPos > bestRightPos)
                    bestLeftPos--;

                bestRightPos = --model;
            }
            else{
                bestRightPos = model;
            }
        }
        else {
            removeModel(model);
            model--;
        }
    }

    /*Utility::printInColor("After Erase Bad Models", RED);
    std::cout << (*this) << std::endl;*/

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

    return ret;
}

//########################################################################################################

std::ostream & operator << (std::ostream &out, const RubyGeneticOnePointPosNeg &r){ 
    out << "RubyGeneticOnePointPosNeg: [\n\tModels: Vector {\n";
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