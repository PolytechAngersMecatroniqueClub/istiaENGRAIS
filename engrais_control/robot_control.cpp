#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include <cmath>
#include <chrono>
#include <time.h>  
#include <algorithm>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>

#include "src/Point.h"
#include "src/Utility.h"
#include "src/Model.h"
#include "src/Pearl.h"

using namespace std;


// -------------------------------------------------------------------------------------------------------
template < typename T> int findIndex(const vector<T>  & vec, const T  & element){
    int index = -2;
    auto it = find(vec.begin(), vec.end(), element);
    if (it != vec.end())
        index = distance(vec.begin(), it);

    return index;
}
// -------------------------------------------------------------------------------------------------------
vector<int> randomDiffVector(const int min, const int max, const int size){
    vector<int> r(size, -1);
    int randNum;

    if((max-min+1) < size){
        Utility::printInColor("Wrong usage of randomDiffVector, max - min + 1 should be greater than the size", RED);
        return r;
    }

    for(int i = 0; i < size; i++){
        while(true){
            randNum = Utility::randomInt(min, max);
            if(findIndex(r, randNum) == -2)
                break;
        }
        r[i] = randNum;
    }

    return r;
}

class Ruby : public Pearl{
	public:
		int nOfPositivePoints = 0;
		int nOfNegativePoints = 0;
		pair <Model, Model> savedModels;

	public:
		//--------------------------------------------------------------------------------------------------------
		void populateOutliers(const sensor_msgs::LaserScan & msg){
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
		vector<Point> randomPointsInField(const int num) { 
		    vector<Point> ret(num);

	        vector<int> randomNums = randomDiffVector(0, outliers.size() - 1, num);

	        int pos = 0;
	        for(int i : randomNums){
	        	ret[pos] = outliers[i];
	        	pos++;
	        }
		    
		    return ret;
		}
		//--------------------------------------------------------------------------------------------------------
		void searchModels(const int nbOfModels) { 
		    for(int modelNum = 0; modelNum < nbOfModels; modelNum++){
		    	if(outliers.size() < INITIAL_NUMBER_OF_POINTS)
		    		return;

		        models.push_back(Model::linearFit(randomPointsInField(INITIAL_NUMBER_OF_POINTS)));
		    }
		}
		//--------------------------------------------------------------------------------------------------------
		std::pair<Model, Model> findLines() { 
			std::pair<Model, Model> bestPair, tempPair;
			int numberMinOfPoints;

			double newEnergy = MAX_DBL, energy = MAX_DBL;

			cout << *this << endl;

			if(outliers.size() != 0){

		        vector<Model> bestModels;
		        vector<Point> bestOutliers;

		        for (int it = 0; it < this->maxNumberOfIterations; it++) {
		            searchModels(50);

		            redistributePoints();

		            fuseEqualModels();

		            numberMinOfPoints = min((int)(meanNumbOfPoints() * 0.8),  3);

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

			cout << *this << endl;

			return bestPair;
		}
		//--------------------------------------------------------------------------------------------------------
		double calculateEnergy(){ 
			double energy = 0;

			for(Model m : models)
				energy += m.getEnergy();

			energy += calculateAdditionalEnergy();

			return energy;
		}
		//--------------------------------------------------------------------------------------------------------
		double meanNumbOfPoints(){ 
			double mean = 0;
			for(Model m : models)
				mean += m.getPointsSize();

			return mean/(double)models.size();
		}
		//--------------------------------------------------------------------------------------------------------
		vector<int> countParallelLines(){
			vector<int> ret(models.size(), 1);

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
		pair<Model, Model> eraseBadModels(){ 
		    pair<Model, Model> ret; //first = left, second = right
		    
			double fitness;
			double bestFitnessLeft = MAX_DBL;
			double bestFitnessRight = MAX_DBL;

			int bestLeftPos = MIN_INT;
			int bestRightPos = MIN_INT; //MAX_INT

			vector<int> parrallel = countParallelLines();
			
		    for(int model = 0; model < models.size(); model++){
		    	fitness = models[model].getPointsSize() != 0 ? (fabs(models[model].getIntercept()) + models[model].getEnergy()) / (double)(models[model].getPointsSize()*parrallel[model]) : MAX_DBL;

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
		//--------------------------------------------------------------------------------------------------------
		friend ostream & operator << (ostream &out, const Ruby &r);
};


Ruby ruby;

ros::Subscriber sub;
ros::Publisher pubLineNode;


//--------------------------------------------------------------------------------------------------------
void prepareLineList(visualization_msgs::Marker & line_list, const double botX, const double topX, const double botY, const double topY) { 
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
}
//--------------------------------------------------------------------------------------------------------
void sendLine(const pair<Model, Model> & models) { 
    visualization_msgs::Marker line_list;
    geometry_msgs::Point p;

    prepareLineList(line_list, 0.6, -1.2, 0.8, -0.8);
    
    if(models.first.getSlope() != MAX_DBL && models.first.getIntercept() != MAX_DBL){
	    p.x = 0;
	    p.y = models.first.getSlope()*0 + models.first.getIntercept();
	    p.z = 0;

	    line_list.points.push_back(p);

	    p.x = 20;
	    p.y = models.first.getSlope()*p.x + models.first.getIntercept();

	    line_list.points.push_back(p);
	}

	if(models.second.getSlope() != MAX_DBL && models.second.getIntercept() != MAX_DBL){
	    p.x = 0;
	    p.y = models.second.getSlope()*0 + models.second.getIntercept();
	    p.z = 0;

	    line_list.points.push_back(p);

	    p.x = 20;
	    p.y = models.second.getSlope()*p.x + models.second.getIntercept();

	    line_list.points.push_back(p);
   }
    
    pubLineNode.publish(line_list);
}

int cont = 0;
//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){
    auto start = std::chrono::system_clock::now();

    ruby.populateOutliers(msg);

    pair <Model, Model> lines = ruby.findLines();

    sendLine(lines);

    auto end = std::chrono::system_clock::now();

    std::chrono::duration<double> elapsed_seconds = end - start;
    std::time_t end_time = std::chrono::system_clock::to_time_t(end);

    std::cout << "finished computation at " << std::ctime(&end_time) << "elapsed time: " << elapsed_seconds.count() << "s\n";

    cont++;
    if( cont == 2 )
    	exit(1);
}
// -------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){

    ROS_INFO("Initializing Robot Control Ros Node");

    srand (time(NULL));
    ros::init(argc, argv, "robot_control_node"); // Initiate a new ROS node named "robot_control_node"

    ros::NodeHandle node;

    sub = node.subscribe("/robot_engrais/lidar_engrais/data", 10, OnRosMsg); // Subscribe to a given topic, in this case "/robot/sensor/data".
    pubLineNode = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);


    ROS_INFO("Code Running, press Control+C to end");
    ros::spin();
    ROS_INFO("Shitting down...");


    sub.shutdown();
    pubLineNode.shutdown();
    ros::shutdown();


    ROS_INFO("Code ended without errors");
    return 0;
}

// *******************************************************************************************************











std::ostream & operator << (std::ostream &out, const Ruby &r){ //Checked
    out << "Ruby: [\n\tModels: Vector {\n";
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



