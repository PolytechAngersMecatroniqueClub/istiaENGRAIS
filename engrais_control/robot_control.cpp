//********************************************************************************************************
#include <chrono>
#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>
#include <thread>
#include <mutex>
#include <vector>

#include <Point.h>
#include <Model.h>
#include <Utility.h>

#include <WeightedModel.h>
#include <RobotControl.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#define PI 3.1415926535 
#define TO_MILLISECOND 1000 

using namespace std;

mutex critSec; //Critical section mutex

string mapName, node_name, emergecy_topic, mode, changeModeTopic; 

int SLEEP_TIME;

ros::Publisher pubLeftControl;
ros::Publisher pubRightControl;
ros::Publisher pubSelectedLines;

RobotControl* control;

ros::NodeHandle* node;

ros::MultiThreadedSpinner spinner(2);

//--------------------------------------------------------------------------------------------------------
void sendLine(const std::pair<vector<Model>, std::vector<bool>> & models){ //Send model's first and last point using the visualization marker, green means it was found, yellow means it was estimated
    visualization_msgs::Marker line_list_found, line_list_calculated;
    geometry_msgs::Point p;

    line_list_found.header.frame_id = line_list_calculated.header.frame_id = mapName; //Assign map name
    line_list_found.header.stamp = line_list_calculated.header.stamp = ros::Time::now(); //Assign time
    line_list_found.ns = line_list_calculated.ns = "points_and_lines"; //Message type 
    line_list_found.action = line_list_calculated.action = visualization_msgs::Marker::ADD; //Add points
    line_list_found.pose.orientation.w = line_list_calculated.pose.orientation.w = 1.0;

    line_list_found.id = 0; //Set id
    line_list_calculated.id = 1; //Set id

    line_list_found.type = line_list_calculated.type = visualization_msgs::Marker::LINE_LIST; //Message line list flag

    line_list_found.scale.x = line_list_calculated.scale.x = 0.08; //Line list scale

    line_list_found.color.g = 1.0; //Color

    line_list_calculated.color.g = 1.0; //Line list color
    line_list_calculated.color.r = 1.0; //Line list color

    line_list_found.color.a = line_list_calculated.color.a = 0.4; //Alpha

    for(int i = 0; i < models.first.size(); i++){ //For every model found

        if(!models.second[i] && models.first[i].isPopulated() && models.first[i].getPointsSize() >= 2){ //If model was found
            pair<Point, Point> points = models.first[i].getFirstAndLastPoint(); //Finds negative and positive-most points (x-axis)

            p.x = points.first.getX(); //Get first point X coordinate
            p.y = models.first[i].getSlope()*p.x + models.first[i].getIntercept(); //Calculate Y using the model's information

            line_list_found.points.push_back(p); //Push point

            p.x = points.second.getX(); //Last point X coordinate
            p.y = models.first[i].getSlope()*p.x + models.first[i].getIntercept(); //Calculate using model's information

            line_list_found.points.push_back(p); //Push
        }
        else if(models.second[i] && models.first[i].isPopulated()){ //If model was estimated
            pair<Point, Point> points = models.first[i].getFirstAndLastPoint(); //Finds negative and positive-most points (x-axis)

            p.x = points.first.getX(); //Get first point X coordinate
            p.y = models.first[i].getSlope()*p.x + models.first[i].getIntercept(); //Calculate Y using the model's information

            line_list_calculated.points.push_back(p); //Push point

            p.x = points.second.getX(); //Last point X coordinate
            p.y = models.first[i].getSlope()*p.x + models.first[i].getIntercept(); //Calculate using model's information

            line_list_calculated.points.push_back(p);
        }
    }
    
    pubSelectedLines.publish(line_list_found); //Push lines list
    pubSelectedLines.publish(line_list_calculated); //Push lines list
}

//--------------------------------------------------------------------------------------------------------
void ModeChangeMsg(const std_msgs::String & msg){ //Message to change mode 
    if(msg.data != "automatic"){ //If received something that is not automatic
        mode = "manual"; //Change to manual
    }
    else{
        mode = "automatic"; //Change to automatic
    }

    Utility::printInColor(node_name + ": Mode changed to " + mode + "\n", CYAN);
}
//--------------------------------------------------------------------------------------------------------
void changeModeThread(){ //Thread to chenge mode 
    ros::Subscriber subMode = node->subscribe(changeModeTopic, 10, ModeChangeMsg); //Subscribe to topic

    spinner.spin(); //Spin

    subMode.shutdown(); //shutdown
}


//--------------------------------------------------------------------------------------------------------
ros::Time lastMsg; //Store last time
bool comReady = false; //Flag to say the communication is ready
bool emergencyCalled = false; //Flag to signal exit

//--------------------------------------------------------------------------------------------------------
void OnEmergencyBrake(const std_msgs::Bool & msg){ //Emergency message
    lastMsg = ros::Time::now(); //Store last time
    
    comReady = true; //Sets flag

    if(msg.data == true){ //If message is true, exit
        Utility::printInColor(node_name + ": Emergency Shutdown Called", RED);
        emergencyCalled = true;
        ros::shutdown();
    }
}
//--------------------------------------------------------------------------------------------------------
void emergencyThread(){ //Emergency exit 
    lastMsg = ros::Time::now(); //Sets time to now

    ros::Subscriber emergencySub = node->subscribe(emergecy_topic, 10, OnEmergencyBrake); //Subscribe to topics
    ros::Publisher emergencyPub = node->advertise<std_msgs::Bool>(emergecy_topic, 10); //Topic to publish if automatic

    std_msgs::Bool msg; //Sets message
    msg.data = false;
    
    while(ros::ok() && !comReady && mode != "automatic") //If mode is manual, wait for comunication to be set
        ros::Duration(0.01).sleep();
    
    while(ros::ok() && !emergencyCalled){ //Until emergency is called
        if(mode != "automatic" && !emergencyCalled){ //If it's manual
            ros::Time now = ros::Time::now(); //get time
            
            ros::Duration delta_t = now - lastMsg;

            if(!emergencyCalled && delta_t.toSec() > 0.2){ //If last emergency message was received more than 200ms ago, shutdown
                Utility::printInColor(node_name + ": Emergency Timeout Shutdown", RED);
                ros::shutdown();
            }

        }
        else{ //If automatic, send message to not shutdown
            emergencyPub.publish(msg);
        }

        ros::Duration(0.05).sleep(); //Sleep 50ms
    }

    emergencySub.shutdown(); //Shutdown everything
    emergencyPub.shutdown();
}


//--------------------------------------------------------------------------------------------------------
void controlThread(){ //Control Thread 
    while(ros::ok()){ //While the program runs

        critSec.lock(); //Lock critical section
        control->clearModels(); //Clear all models
        critSec.unlock(); //Unlock critical section

        usleep(SLEEP_TIME * TO_MILLISECOND); //Sleep for a time

        critSec.lock();  //Lock critical section

        std::pair<vector<Model>, std::vector<bool>> selectedModels = control->selectModels(); //Select models
        sendLine(selectedModels); //Send selected lines
        pair<std_msgs::Float64, std_msgs::Float64> wheels = control->getWheelsCommand(); //Calculates wheels' commands
        
        critSec.unlock(); //Unlock

        if(mode == "automatic"){
            pubLeftControl.publish(wheels.first); //Send left wheel command
            pubRightControl.publish(wheels.second); //Send right wheel command
        }

        cout << "First: " << wheels.first.data << ", second: " << wheels.second.data << endl;

        cout << "----------------------------------------------------------" << endl; 
    }
}


//--------------------------------------------------------------------------------------------------------
void BackLinesMsg(const visualization_msgs::Marker & msg){ //Back node message received 
    critSec.lock(); //Lock critical section

    if(msg.type == visualization_msgs::Marker::LINE_LIST) //Check flag to see if it's line list
        control->backLinesMessage(msg); //Feed back message to control
    
    else if(msg.type == visualization_msgs::Marker::POINTS)
        control->backPointsMessage(msg); //Feed back message to control

    critSec.unlock(); //Unlock critical section
}
//--------------------------------------------------------------------------------------------------------
void FrontLinesMsg(const visualization_msgs::Marker & msg){ //Front node message received 
    critSec.lock(); //Lock critical section
    
    if(msg.type == visualization_msgs::Marker::LINE_LIST) //Check flag to see if it's line list    
        control->frontLinesMessage(msg); //Feed front message to control
 
    else if(msg.type == visualization_msgs::Marker::POINTS)
        control->frontPointsMessage(msg); //Feed front message to control
    
    critSec.unlock(); //Unlock critical section
}


//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main funcion

    ros::init(argc, argv, "robot_move_node"); //Initialize node
    node = new ros::NodeHandle();

    int NUM_LINES, N_TIMES_TURN;
    double MAX_VEL, BODY_SIZE;

    string subTopicFront, subTopicBack, pubTopicLeft, pubTopicRight, pubTopicSelected;
   
    node_name = ros::this_node::getName();


    node->param<string>(node_name + "/subscribe_topic_front", subTopicFront, "/default/frontLines"); //Get all parameters, with default values 
    node->param<string>(node_name + "/subscribe_topic_back", subTopicBack, "/default/backLines");

    node->param<string>(node_name + "/publish_topic_left", pubTopicLeft, "/default/leftWheel");
    node->param<string>(node_name + "/publish_topic_right", pubTopicRight, "/default/rightWheel");

    node->param<string>(node_name + "/mode", mode, "automatic");
    node->param<string>(node_name + "/change_mode_topic", changeModeTopic, "none");

    node->param<int>(node_name + "/number_lines", NUM_LINES, 4);
    node->param<int>(node_name + "/turn_times", N_TIMES_TURN, 2);
    node->param<int>(node_name + "/sleep_time_ms", SLEEP_TIME, 250);

    node->param<double>(node_name + "/max_velocity", MAX_VEL, 1.0);
    node->param<double>(node_name + "/body_size", BODY_SIZE, 1.0);

    node->param<string>(node_name + "/rviz_frame", mapName, "world");
    node->param<string>(node_name + "/rviz_topic", pubTopicSelected, "/default/selectedLines");
    node->param<string>(node_name + "/emergency_topic", emergecy_topic, "none");

    if(NUM_LINES % 2 == 1){
        ROS_ERROR_STREAM(node_name << "/PARAMETER ERROR: NUMBER OF LINES MUST BE PAIR\n");
        return -5;
    }


    control = new RobotControl(NUM_LINES, N_TIMES_TURN, MAX_VEL, BODY_SIZE); //Create control object

    ros::Subscriber subFront = node->subscribe(subTopicFront, 10, FrontLinesMsg); //Subscribe to topic
    ros::Subscriber subBack = node->subscribe(subTopicBack, 10, BackLinesMsg);

    pubLeftControl = node->advertise<std_msgs::Float64>(pubTopicLeft, 10); //Topic to publish
    pubRightControl = node->advertise<std_msgs::Float64>(pubTopicRight, 10);

    pubSelectedLines = node->advertise<visualization_msgs::Marker>(pubTopicSelected, 10);

    thread control_t(controlThread); //launch control thread

    thread* emergency_t; //launch emergency thread
    if(emergecy_topic != "none")
        emergency_t = new thread(emergencyThread);


    thread* changeMode_t; //launch change mode thread
    if(changeModeTopic != "none")
        changeMode_t = new thread(changeModeThread);

    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    spinner.spin(); //Spin
    Utility::printInColor(node_name + ": Shutting down...", CYAN);

    if(emergecy_topic != "none"){ //Wait for threads to finish
        emergency_t->join();
        delete emergency_t;
    }

    if(changeModeTopic != "none"){
        changeMode_t->join();
        delete changeMode_t;
    }

    control_t.join();

    delete control;

    subFront.shutdown(); //Shutdown topics
    subBack.shutdown();

    pubLeftControl.shutdown();
    pubRightControl.shutdown();
    pubSelectedLines.shutdown();

    ros::shutdown();
    
    delete node;    

    Utility::printInColor(node_name + ": Code ended without errors", BLUE);

    return 0;
}

//********************************************************************************************************
