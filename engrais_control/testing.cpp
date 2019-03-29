#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <fstream>

#include <ros/ros.h>

#include "src/robot_findlines_include/1_Point/Point.h"
#include "src/robot_findlines_include/2_WeightedPoint/WeightedPoint.h"
#include "src/robot_findlines_include/3_Utility/Utility.h"
#include "src/robot_findlines_include/4_Model/Model.h"

#include <engrais_control/Model.h>
#include <engrais_control/Results.h>

#define TRUE_RIGHT_SLOPE 0.220254
#define TRUE_RIGHT_INTERCEPT -0.864839

#define TRUE_LEFT_SLOPE 0.220254
#define TRUE_LEFT_INTERCEPT 2.02784


#define TOLERANCE 0.2

using namespace std;

class MethodResult{
	public:
		string method;

		int executionCount = 0;
		double totalExecutionTime = 0;

		int correctLeftModelInsideVector = 0;
		int correctRightModelInsideVector = 0;

		int correctLeftModelSelected = 0;
		int correctRightModelSelected = 0;

	public:
		MethodResult() {}
		MethodResult(string m) { method = m; }

		bool operator == (const MethodResult & n) const { return (this->method == n.method); }

		friend ostream & operator << (ostream & , const MethodResult & );
};
//--------------------------------------------------------------------------------------------------------
inline std::ostream & operator << (std::ostream &out, const MethodResult &mr){ //Checked
    out << "MethodResult: [ Method: " << mr.method << ", executionCount: " << mr.executionCount << ", totalExecutionTime: " << mr.totalExecutionTime << ", correctLeftModelInsideVector: " << mr.correctLeftModelInsideVector;
    out << ", correctRightModelInsideVector: " << mr.correctRightModelInsideVector << ", correctLeftModelSelected: " << mr.correctLeftModelSelected;
    out << ", correctRightModelSelected: " << mr.correctRightModelSelected << " ]";
    return out; 
}


vector<MethodResult> methodResult;
ros::Subscriber sub;


//--------------------------------------------------------------------------------------------------------
void printResults(const engrais_control::Results & results){

    cout << "Method: "<< results.method << ", Run Time: " << results.runTime << endl;

    cout << "Models: Vector {" << endl;
    for(int i = 0; i < results.foundModels.size(); i++){
        cout << "\t[" << i << "]: Slope: " << results.foundModels[i].slope << ", Intercept: " << results.foundModels[i].intercept << endl;
    }
    cout << "}" << endl << endl;
}
//--------------------------------------------------------------------------------------------------------
pair <engrais_control::Model, engrais_control::Model> findClosestModels(const engrais_control::Results & msg){
	pair <engrais_control::Model, engrais_control::Model> ClosestModels;

    ClosestModels.first.slope = ClosestModels.first.intercept = ClosestModels.second.slope = ClosestModels.second.intercept = MAX_DBL;

    for(int i = 0; i < msg.foundModels.size(); i++){
    	if(msg.foundModels[i].intercept >= 0 && fabs(ClosestModels.first.intercept) > fabs(msg.foundModels[i].intercept)){
    		ClosestModels.first = msg.foundModels[i];
    	}
    	else if(msg.foundModels[i].intercept < 0 && fabs(ClosestModels.second.intercept) > fabs(msg.foundModels[i].intercept)){
    		ClosestModels.second = msg.foundModels[i];
    	}
    }

    return ClosestModels;
}
int cont = 0;
//--------------------------------------------------------------------------------------------------------
void resultsMsg(const engrais_control::Results & msg){
	//printResults(msg);

	bool ex = methodResult.size() == 0 ? false : true;

	for(int i = 0; i < methodResult.size(); i++){
		if(methodResult[i].executionCount != 1000){
			ex = false;
		}
	}

	if(ex){
		std::ofstream file ("./src/istiaENGRAIS/engrais_control/tests/results_easy_angle2.txt");
		
		for(int i = 0; i < methodResult.size(); i++)
			file << methodResult[i] << endl << endl;
		
		file << endl << endl << endl << "---------------------------------------------------------" << endl;
		
		sub.shutdown();
		ros::shutdown();

		file.close();
		exit(1);
	}

	pair <engrais_control::Model, engrais_control::Model> ClosestModels = findClosestModels(msg);

	int indexForMethod = Utility::findIndex(methodResult, MethodResult(msg.method));

	if(indexForMethod == MIN_INT){
		methodResult.push_back(MethodResult(msg.method));
		indexForMethod = methodResult.size() - 1;
	}

	bool lModelFound = false;
	bool rModelFound = false;

	for(int i = 0; i < msg.foundModels.size(); i++){
		if(!lModelFound && fabs(TRUE_LEFT_SLOPE - msg.foundModels[i].slope) < TOLERANCE && fabs(TRUE_LEFT_INTERCEPT - msg.foundModels[i].intercept) < TOLERANCE){
			methodResult[indexForMethod].correctLeftModelInsideVector++;
			lModelFound = true;
		}

		if(!rModelFound && fabs(TRUE_RIGHT_SLOPE - msg.foundModels[i].slope) < TOLERANCE && fabs(TRUE_RIGHT_INTERCEPT - msg.foundModels[i].intercept) < TOLERANCE){
			methodResult[indexForMethod].correctRightModelInsideVector++;
			rModelFound = true;
		}
	}

	if(fabs(TRUE_LEFT_SLOPE - ClosestModels.first.slope) < TOLERANCE && fabs(TRUE_LEFT_INTERCEPT - ClosestModels.first.intercept) < TOLERANCE){
		methodResult[indexForMethod].correctLeftModelSelected++;
	}

	if(fabs(TRUE_RIGHT_SLOPE - ClosestModels.second.slope) < TOLERANCE && fabs(TRUE_RIGHT_INTERCEPT - ClosestModels.second.intercept) < TOLERANCE){
		methodResult[indexForMethod].correctRightModelSelected++;
	}

	methodResult[indexForMethod].executionCount++;
	methodResult[indexForMethod].totalExecutionTime += msg.runTime;

	if(++cont%((1000*6)/100) == 0){
		cout << "[ " << (double)cont/(double)(1000.0*6.0)*100.0 << "] complete" << endl;
	}
}


int main(int argc, char **argv){

	cout << "tests began" << endl;

	ros::init(argc, argv, "testing_node");
	ros::NodeHandle node;

    sub = node.subscribe("/testing/robot_engrais/results", 10, resultsMsg);

    ros::spin();

    sub.shutdown();
    ros::shutdown();
    
    return 0;
}
