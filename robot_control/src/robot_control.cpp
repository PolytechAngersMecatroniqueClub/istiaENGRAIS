#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <fstream> 

#include <cmath>
#include <time.h>  
#include <algorithm>
#include <signal.h>
#include <unistd.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include <chrono>
#include <thread>

using namespace std;

#define pi 3.1415926535

#define BLUE 34
#define RED 31
#define CYAN 36

#define ROBOT_HEIGHT 0.46
#define RANGE 5

#define INITIAL_NUMBER_OF_POINTS 4

#define OUTLIER INT_MIN



// *******************************************************************************************************

class Point{ 

    public: 
        double x;
        double y;

        // --------------------------------------------------------------------------------------------------
        Point(){
            x = y = DBL_MIN;
            return;
        }
        // --------------------------------------------------------------------------------------------------
        Point(const double xx, const double yy){
            x = xx;
            y = yy;
        }
        // --------------------------------------------------------------------------------------------------
        friend ostream & operator << (ostream &out, const Point &p); // Operator to display class on screen
        // --------------------------------------------------------------------------------------------------
};

// ****************** *************************************************************************************

class Utility{ 
    private:
    public:
        static double calcSomDist(const vector<Point> & vec_p, const double a, const double b) { // OK 
            double dist = 0;
            for(Point p : vec_p)
                dist +=  abs(a * p.x - p.y + b) / sqrt(pow(a, 2) + 1.0);

            return dist;
        }
        //--------------------------------------------------------------------------------------------------------
        static double calcSomDist(const vector<double> & x, const vector<double> & y, const double a_mod, const double b_mod) { // OK 
            double dist = 0;
            for(int element  = 0; element < x.size(); element++)
                dist +=  abs((y[element] - a_mod * x[element] - b_mod) / sqrt(pow(a_mod, 2) + 1.0));

            return dist;
        }
        //--------------------------------------------------------------------------------------------------------
        static int randomInt(const int min, const int max){
            int r = (rand() % (max - min + 1)) + min;
            return r;
        }
        // -------------------------------------------------------------------------------------------------------
        static vector<int> randomDiffVector(const int min, const int max, const int size){
            vector<int> r(size, -1);
            int randNum;

            if((max-min+1) < size){
                printInColor("Wrong usage of randomDiffVector, max - min + 1 should be greater than the size", RED);
                return r;
            }

            for(int i = 0; i < size; i++){
                while(true){
                    randNum = randomInt(min, max);
                    if(findIndex(r, randNum) == -2)
                        break;
                }
                r[i] = randNum;
            }

            return r;
        }
        // -------------------------------------------------------------------------------------------------------
        static double sum(const vector<double> & vec){
            double s = 0;

            for(int i = 0; i < vec.size(); i++)
                s += vec[i];

            return s;
        }
        // -------------------------------------------------------------------------------------------------------
        template < typename T> static void printVector(const vector<T> & vec){ 
            cout << endl << "Vector {" << endl;
            for(int i = 0; i < vec.size(); i++){
                cout << "\t" << i << ": [" << vec[i] << "]" << endl; 
            }
            cout << "}" << endl;
        }
        // -------------------------------------------------------------------------------------------------------
        template < typename T> static void printVector(const vector<T> & vec, ostream &out){ 
            out << "Vector {" << endl;
            for(int i = 0; i < vec.size(); i++){
                out << "\t" << i << ": [" << vec[i] << "]" << endl; 
            }
            out << "}" << endl;
        }
        // -------------------------------------------------------------------------------------------------------
        template < typename T> static void printVector(const vector<T> & vec, fstream &out){ 
            out << "Vector {" << endl;
            for(int i = 0; i < vec.size(); i++){
                out << "\t" << i << ": [" << vec[i] << "]" << endl; 
            }
            out << "}" << endl;
        }
        // -------------------------------------------------------------------------------------------------------
        template < typename T> static int findIndex(const vector<T>  & vec, const T  & element){
            int index = -2;
            auto it = find(vec.begin(), vec.end(), element);
            if (it != vec.end())
                index = distance(vec.begin(), it);

            return index;
        }
        // -------------------------------------------------------------------------------------------------------
        static double d2r(const double degrees){ // Deegrees to radians 
            double radians = pi/180.0 * degrees;
            return radians; 
        }
        // -------------------------------------------------------------------------------------------------------
        static double r2d(const double radians){ // Radians to degrees 
            double degrees = 180.0/pi * radians;
            return degrees; 
        }
        // -------------------------------------------------------------------------------------------------------
        static void printInColor(const string msg, const int color){
            string msgType = "[MSG]";
            if(color == RED)
                msgType = "[ERR]";
            else if(color == CYAN || color == BLUE)
                msgType = "[OK]";

            if(color == RED || color == BLUE)
                cout << endl;
            cout << endl << "\033[1;" << color << "m" << msgType << " " << msg << "\033[0m";

            if(color == RED || color == BLUE)
                cout << endl;
        }
        // -------------------------------------------------------------------------------------------------------
};

// *******************************************************************************************************

class Model{ 
    public: 
        double a;
        double b;
        double energy;

        vector<int> refPointsInModel;

        // -------------------------------------------------------------------------------------------------------
        Model(){
            a = b = energy = DBL_MAX;
            //
        }
        // -------------------------------------------------------------------------------------------------------
        Model(const double a, const double b, const double energy){
            this->a = a;
            this->b = b;
            this->energy = energy;
        }
        // -------------------------------------------------------------------------------------------------------
        Model(const int vectorSize) : refPointsInModel(vectorSize) {}
        // -------------------------------------------------------------------------------------------------------
        bool operator == (const Model& m) const {
            if(refPointsInModel.size() != m.refPointsInModel.size())
                return false;

            for(int i = 0; i < refPointsInModel.size(); i++){
                if(refPointsInModel[i] != m.refPointsInModel[i])
                    return false;
            }
                
            return ((a == m.a) && (b == m.b) && (energy == m.energy));            
        }
        //--------------------------------------------------------------------------------------------------------
        friend ostream & operator << (ostream &out, const Model &m);
        // -------------------------------------------------------------------------------------------------------
        void setVariables(const double a, const double b, const double energy){
            this->a = a;
            this->b = b;
            this->energy = energy;
        }
        // -------------------------------------------------------------------------------------------------------
        vector<Point> getPoints(const vector<Point> & field) const {
            vector<Point> ret(refPointsInModel.size());
            int pos = 0;

            for(int i : refPointsInModel){
                ret[pos] = field[i];
                pos++;
            }
            return ret;
        }
        // -------------------------------------------------------------------------------------------------------
        void findBestModel(const vector<Point> & field){
            vector<Point> pointsInModel = getPoints(field);

            for(int point1 = 0; point1 < pointsInModel.size(); point1++){
                for(int point2 = point1 + 1; point2 < pointsInModel.size(); point2++){

                    double dx = pointsInModel[point2].x - pointsInModel[point1].x;
                    double dy = pointsInModel[point2].y - pointsInModel[point1].y;

                    double a_mod = this->a;

                    if(dx != 0)
                        a_mod = dy/dx;

                    double b_mod = pointsInModel[point2].y - (a_mod * pointsInModel[point2].x);

                    double totalDist = Utility::calcSomDist(pointsInModel, a_mod, b_mod);

                    if(totalDist < this->energy){
                        this->energy = totalDist;
                        this->a = a_mod;
                        this->b = b_mod;
                    }   
                }
            }
        }
        // -------------------------------------------------------------------------------------------------------
        void findBestModel(const vector<int> & rPointsInModel, const vector<Point> & field){

            this->refPointsInModel = rPointsInModel;

            vector<Point> pointsInModel = getPoints(field);

            this->energy = DBL_MAX;

            for(int point1 = 0; point1 < pointsInModel.size(); point1++){
                for(int point2 = point1 + 1; point2 < pointsInModel.size(); point2++){

                    double dx = pointsInModel[point2].x - pointsInModel[point1].x;
                    double dy = pointsInModel[point2].y - pointsInModel[point1].y;

                    double a_mod = this->a;

                    if(dx != 0)
                        a_mod = dy/dx;

                    double b_mod = pointsInModel[point2].y - (a_mod * pointsInModel[point2].x);

                    double totalDist = Utility::calcSomDist(pointsInModel, a_mod, b_mod);

                    if(totalDist < this->energy){
                        this->energy = totalDist;
                        this->a = a_mod;
                        this->b = b_mod;
                    }   
                }
            }
        }
};

// *******************************************************************************************************

class Collection{ 

    public: 
        vector<Model> models;
        vector<int> outliers;

        double outlierPenalty = 3;
        double lambda = 4;
        double csi = 3;

        double aThresh = 0.5;
        double bThresh = 0.5;


        //--------------------------------------------------------------------------------------------------------
        Collection(){}
        //--------------------------------------------------------------------------------------------------------
        friend ostream & operator << (ostream &out, const Collection &c);
        friend fstream & operator << (fstream &arq, const Collection &c);
        // -------------------------------------------------------------------------------------------------------
        bool operator == (const Collection& c){
            bool b = (models == c.models) && (outliers == c.outliers);
            return b;
        }
        // -------------------------------------------------------------------------------------------------------
        bool operator != (const Collection& c){
            bool b = (*this == c);
            return !b;
        }
        // -------------------------------------------------------------------------------------------------------
        void clearPointsInModels(){
            for(int i = 0; i < models.size(); i++)
                clearPointsInModel(i);
        }
        // -------------------------------------------------------------------------------------------------------
        void clearPointsInModel(const int index){
            outliers.insert(outliers.end(), models[index].refPointsInModel.begin(), models[index].refPointsInModel.end());
            models[index].refPointsInModel.clear();
            models[index].energy = 0;
        }
        // -------------------------------------------------------------------------------------------------------
        void pushModel(const Model & m){
            models.push_back(m);
            //
        }
        // -------------------------------------------------------------------------------------------------------
        void pushOutliers(const int pos){
            outliers.push_back(pos);
            //
        }
        // -------------------------------------------------------------------------------------------------------
        int pullOutlierPoint(const int pos){
            int r = outliers[pos];
            outliers.erase(outliers.begin() + pos);
            return r;
        }
        // -------------------------------------------------------------------------------------------------------
        void pullModelPoint(const int modelPos, const int outlier, const double pointDistance){
            models[modelPos].energy += pointDistance;

            models[modelPos].refPointsInModel.push_back(outliers[outlier]);
            outliers.erase(outliers.begin() + outlier);
        }
        // -------------------------------------------------------------------------------------------------------
        void removeModel(const int modelIndex){
            clearPointsInModel(modelIndex);
            models.erase(models.begin() + modelIndex);
        }
        // -------------------------------------------------------------------------------------------------------
        double calculateAdditionalEnergy(const vector<Point> & field) {
            double addEnergy = 0;
            for(int model = 0; model < models.size(); model++)
                for(int model2 = model + 1; model2 < models.size(); model2++)
                    for(Point p : models[model].getPoints(field))
                        for(Point q : models[model2].getPoints(field))
                            addEnergy += this->lambda * exp(-(pow(p.x - q.x, 2) + pow(p.y - q.y, 2)) / pow(this->csi, 2));

            return addEnergy;                           
        }
        // -------------------------------------------------------------------------------------------------------
        void removeEqualModels(){
            for(int model = 0; model < models.size(); model++){
                for(int model2 = model + 1; model2 < models.size(); model2++){
                    if(abs(models[model].a - models[model2].a) < aThresh && abs(models[model].b - models[model2].b) < bThresh){
                        int model1Size = models[model].refPointsInModel.size();
                        int model2Size = models[model2].refPointsInModel.size();

                        if(model1Size >= model2Size){
                            models[model].a = (models[model].a * model1Size + models[model2].a * model2Size)/double(model1Size + model2Size);
                            models[model].b = (models[model].b * model1Size + models[model2].b * model2Size)/double(model1Size + model2Size);
                            models[model].refPointsInModel.insert(models[model].refPointsInModel.end(), models[model2].refPointsInModel.begin(), models[model2].refPointsInModel.end());
                            models.erase(models.begin() + model2);
                            model2--;
                        }
                        else{
                            models[model2].a = (models[model].a * model1Size + models[model2].a * model2Size)/double(model1Size + model2Size);
                            models[model2].b = (models[model].b * model1Size + models[model2].b * model2Size)/double(model1Size + model2Size);
                            models[model2].refPointsInModel.insert(models[model2].refPointsInModel.end(), models[model].refPointsInModel.begin(), models[model].refPointsInModel.end());
                            models.erase(models.begin() + model);
                            model--;
                        }
                    }
                }
            }
        }
};

// *******************************************************************************************************

class Pearl{ 
    public: 
        vector<Point> field;
        Collection collection;    

        ros::Publisher pubLineNode;

        int nbMaxIteractions = 10;

        double distOutlier = 1.5;
        double threshRatio = 0.7;
        double divideFactor = 8;

        // -------------------------------------------------------------------------------------------------------
        Pearl(){}
        // -------------------------------------------------------------------------------------------------------
        void populateField(const sensor_msgs::LaserScan & msg) { 
            double angle = msg.angle_min;
            int pos = 0;
            field.clear();
            collection.outliers.clear();
            collection.models.clear();

            for(int i = 0; i < msg.ranges.size(); i++){
                if(!isinf(msg.ranges[i])){
                    Point p(msg.ranges[i] * cos(angle), msg.ranges[i] * sin(angle));

                    collection.pushOutliers(pos);
                    field.push_back(p);
                    pos++;
                }
                angle += msg.angle_increment;
            }
        }
        // -------------------------------------------------------------------------------------------------------
        void sendLine(const Model & model) const { 
            const double topX = 0.6;
            const double botX = -1.2;

            const double topY = 0.8;
            const double botY = -0.8;

            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "/map";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = "points_and_lines";
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;


            line_list.id = 0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = 0.1;


            // Line list is red
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;

            geometry_msgs::Point p;

            p.x = botX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = botX;
            p.y = botY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = botY;
            line_list.points.push_back(p);



            p.x = botX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = botX;
            p.y = botY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = botY;
            line_list.points.push_back(p);

            p.x = 0;
            p.y = model.a*0 + model.b;
            p.z = 0;

            line_list.points.push_back(p);

            p.x = RANGE;
            p.y = model.a*RANGE + model.b;

            line_list.points.push_back(p);
            
            pubLineNode.publish(line_list);
        }
        // -------------------------------------------------------------------------------------------------------
        void sendLine(const vector<Model> & models) const { 
            const double topX = 0.6;
            const double botX = -1.2;

            const double topY = 0.8;
            const double botY = -0.8;

            visualization_msgs::Marker line_list;
            line_list.header.frame_id = "/map";
            line_list.header.stamp = ros::Time::now();
            line_list.ns = "points_and_lines";
            line_list.action = visualization_msgs::Marker::ADD;
            line_list.pose.orientation.w = 1.0;


            line_list.id = 0;
            line_list.type = visualization_msgs::Marker::LINE_LIST;

            // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
            line_list.scale.x = 0.1;


            // Line list is red
            line_list.color.r = 1.0;
            line_list.color.a = 1.0;
            geometry_msgs::Point p;

            p.x = botX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = botX;
            p.y = botY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = botY;
            line_list.points.push_back(p);



            p.x = botX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = topY;
            line_list.points.push_back(p);

            p.x = botX;
            p.y = botY;
            line_list.points.push_back(p);

            p.x = topX;
            p.y = botY;
            line_list.points.push_back(p);

            for (int i = 0; i < models.size(); i++){

                p.x = 0;
                p.y = models[i].a*0 + models[i].b;
                p.z = 0;

                line_list.points.push_back(p);

                p.x = 5000;
                p.y = models[i].a*p.x + models[i].b;

                line_list.points.push_back(p);

            }
            pubLineNode.publish(line_list);
        }
        // -------------------------------------------------------------------------------------------------------
        vector<Model> findLines() { 

            Collection oldCollection;
            double energy = DBL_MAX;
            double newEnergy = 0.0;

            if(field.size() != 0){
                int nModels = int(field.size() / this->divideFactor);

                if(nModels > 0){
                    searchModels(nModels);

                    newEnergy = expansionEnergy();

                    newEnergy += removeTinyModels(int(1.5*nModels));

                    int nbOfIteractions = 0;

                    oldCollection = collection;

                    energy = newEnergy;

                    while (nbOfIteractions < this->nbMaxIteractions) {
                        searchModels(int(collection.outliers.size() / this->divideFactor));

                        reEstimation();

                        eraseBadModels(this->threshRatio);
                        
                        modelsMS2();

                        newEnergy = expansionEnergy();

                        //cout << collection << endl;

                        if ((newEnergy > energy)){
                            collection = oldCollection;
                            energy = newEnergy;
                        }
                        else{
                            oldCollection = collection;
                            newEnergy = energy;
                        }

                        nbOfIteractions++;
                    }
                }
            }

            else{
                Utility::printInColor("No data in field, please verify", RED);
            }

            cout << oldCollection << endl << endl << endl << endl;

            return findModels();
        }


    //protected:
        // #######################################################################################################
        vector<int> randomPointsInField(const int num) { //Checked 
            vector<int> ret(num);

            if(collection.outliers.size() < num){
                Utility::printInColor("Not enough outliers, please verify", RED);
                return ret;
            }

            for(int i = 0; i < num; i++){
                int randomNum = Utility::randomInt(0, collection.outliers.size() - 1);

                ret[i] = collection.pullOutlierPoint(randomNum);
            }

            return ret;
        }
        // -------------------------------------------------------------------------------------------------------
        void searchModels(const int nbOfModels) { // Ckecked 
            for(int modelNum = 0; modelNum < nbOfModels; modelNum++){
                Model model;
                model.findBestModel(randomPointsInField(INITIAL_NUMBER_OF_POINTS), field);

                collection.pushModel(model);
            }
        }
        // #######################################################################################################
        double redistributePoints() { //Ckecked 
            collection.clearPointsInModels();

            double newEnergy = collection.outliers.size() * collection.outlierPenalty;

            int dominantModelPos;
            for(int p = 0; p < collection.outliers.size(); p++){
                double minDist = DBL_MAX;

                for(int model = 0; model < collection.models.size(); model++){
                    
                    double distAt = abs(collection.models[model].a * field[collection.outliers[p]].x - field[collection.outliers[p]].y + collection.models[model].b) / sqrt(pow(collection.models[model].a, 2) + 1.0);
                    
                    if (distAt < minDist){
                        minDist = distAt;
                        dominantModelPos = model;
                    }
                }

                if (minDist < this->distOutlier){
                    newEnergy += minDist - collection.outlierPenalty;
                    collection.pullModelPoint(dominantModelPos, p, minDist);
                    p--;               
                }
            }

            return newEnergy;
        }
        // -------------------------------------------------------------------------------------------------------
        double removeTinyModels(const int minimum_points = INITIAL_NUMBER_OF_POINTS) { //Ckecked 
            double gainEnergy = 0;
            for(int model = 0; model < collection.models.size(); model++){
                if(collection.models[model].refPointsInModel.size() <= minimum_points){
                    gainEnergy += collection.models[model].refPointsInModel.size() - collection.models[model].energy;
                    collection.removeModel(model);
                    model--;
                }
            }
            return gainEnergy;
        }
        // -------------------------------------------------------------------------------------------------------
        double expansionEnergy() { //Ckecked 
            double newEnergy = redistributePoints();
            newEnergy += removeTinyModels();

            //newEnergy += collection.calculateAdditionalEnergy(field);

            return newEnergy;
        }
        // #######################################################################################################
        void reEstimation() { 
            for(int model = 0; model < collection.models.size(); model++){ 
                collection.models[model].findBestModel(field);
            }
        }
        // #######################################################################################################
        Model eraseBadModels(const double threshRatio) { 

            double bestRatio = numeric_limits<double>::max();
            Model bestModel;

            for(int model = 0; model < collection.models.size(); model++){
                if (collection.models[model].energy / collection.models[model].refPointsInModel.size() < bestRatio){
                    bestModel = collection.models[model];
                    bestRatio = collection.models[model].energy / collection.models[model].refPointsInModel.size();
                }

                if (collection.models[model].energy / collection.models[model].refPointsInModel.size() >= threshRatio){
                    collection.removeModel(model);
                    model--;
                }
            }

            if (collection.models.size() == 0)
                collection.pushModel(bestModel);
            return bestModel;
        }
        // #######################################################################################################
        void modelsMS2() {
            collection.removeEqualModels();
            //
        }
        vector<Model> findModels(){
            vector<Model> ret(2); //first = left, second = right
            
            for(Model m : collection.models){
                if(ret[0].refPointsInModel.size() < m.refPointsInModel.size() && m.b >= 0){
                    ret[0] = m;
                }
                if(ret[1].refPointsInModel.size() < m.refPointsInModel.size() && m.b < 0){
                    ret[1] = m;
                }
            }
            return ret;
        }
};

// *******************************************************************************************************

class Global{ 
    private: 
        static Global* GlobalPointer;
        Global();

    public: 
        
        Pearl pearl;

        static Global* getInstance(){
            if(GlobalPointer == NULL)
                GlobalPointer = new Global();
            return GlobalPointer;
        }
};

Global::Global(){}
Global* Global::GlobalPointer = NULL;
Global* GV = Global::getInstance();

// *******************************************************************************************************

void OnRosMsg(const sensor_msgs::LaserScan & msg){
    auto start = std::chrono::system_clock::now();

    GV->pearl.populateField(msg);
    vector<Model> lines = GV->pearl.findLines();
    GV->pearl.sendLine(lines);
    /*if(abs(lines[0].a) > 2 || abs(lines[1].a) > 2){
        Utility::printVector(lines);
        std::this_thread::sleep_for(2s);
        exit(1);
    }*/
    //Utility::printVector(GV->pearl.field);
    //cout << endl << endl << GV->pearl.collection.models[0] << endl << endl << endl;

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";

    //exit(1);
}
// -------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){
    Utility::printInColor("Initializing Robot Control Ros Node", CYAN);

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/robot/sensor/data", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    GV->pearl.pubLineNode = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    Utility::printInColor("Code Running, press Control+C to end", BLUE);
    ros::spin();
    Utility::printInColor("Shitting down...", CYAN);


    sub.shutdown();
    GV->pearl.pubLineNode.shutdown();
    ros::shutdown();


    Utility::printInColor("Code ended without errors", BLUE);

    return 0;
}

// *******************************************************************************************************




// -------------------------------------------------------------------------------------------------------
ostream & operator << (ostream &out, const Point &p){ 
    out << "Point: [ x: " << p.x << ", y: " << p.y << " ]"; 

    return out; 
}
// ------------------------------------------------------------------------------------------------------- 
ostream & operator << (ostream &out, const Model &m){ 
    out << "Model: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy;
    out << "\n\tPoints: Vector {";
    for(int i = 0; i < m.refPointsInModel.size(); i++){
        out << "\n\t\t[" << i << "]: [ Ref: " << m.refPointsInModel[i] << ", " << GV->pearl.field[m.refPointsInModel[i]] << "]";
    }
    out << "\n\t}\n]";

    return out; 
} 
// -------------------------------------------------------------------------------------------------------
ostream & operator << (ostream &out, const Collection &c){ 
    out << "Collection: [\n\tModels: Vector {\n";
    for(Model m : c.models){
        out << "\t\tModel: [ a: " << m.a << ", b: " << m.b << ", energy: " << m.energy;
        out << "\n\t\t\tPoints: Vector {";
        for(int i = 0; i < m.refPointsInModel.size(); i++){
            out << "\n\t\t\t\t[" << i << "]: [ Ref: " << m.refPointsInModel[i] << ", " << GV->pearl.field[m.refPointsInModel[i]] << "]";
        }
        out << "\n\t\t\t}\n\t\t]\n";
    }

    out << "\t}\n\n\tOutliers: Vector {";

    for(int outP = 0; outP < c.outliers.size(); outP++){
        out << "\n\t\t[" << outP << "]: [ Ref: " << c.outliers[outP] << ", " << GV->pearl.field[c.outliers[outP]] << "]";
    }

    out << "\n\t}";
    out << "\n]";
    return out; 
} 