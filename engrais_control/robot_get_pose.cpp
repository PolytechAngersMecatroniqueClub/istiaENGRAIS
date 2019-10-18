//********************************************************************************************************
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>


using namespace std;

ofstream arq;

ros::Publisher pubExit;

//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const nav_msgs::Odometry & msg){ //ROS message received
    static auto first = msg.header.stamp;

    arq << setprecision(3) << msg.header.stamp - first << ";;";

    arq << int(msg.pose.pose.position.x*1000)/1000.0 << ";" << int(msg.pose.pose.position.y*1000)/1000.0 << ";" << int(msg.pose.pose.position.z*1000)/1000.0 << ";;";

    arq << int(msg.pose.pose.orientation.x*1000)/1000.0 << ";" << int(msg.pose.pose.orientation.y*1000)/1000.0 << ";" << int(msg.pose.pose.orientation.z*1000)/1000.0 << ";;" << endl;
}
//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main function 

    ros::init(argc, argv, "robot_get_pose"); //Initialize ROS
    ros::NodeHandle node;

    string arq_name, node_name = ros::this_node::getName();

    node.param<string>(node_name + "/arq_name", arq_name, "simulation_results.csv");

    cout << arq_name << endl;

    arq.open(arq_name, std::ofstream::out | std::ofstream::trunc);

    arq << "t;;x;y;z;;r;p;y" << endl;

    ros::Subscriber sub = node.subscribe("/gazebo_robot_pose", 10, OnRosMsg);
    pubExit = node.advertise<std_msgs::Bool>("/exit_app_topic", 10);// /engrais/laser_front/lines or /engrais/laser_back/lines

    ros::spin();
    
    arq.close();
   
}

//********************************************************************************************************