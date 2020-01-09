//********************************************************************************************************
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>

#include <Point.h>
#include <Utility.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

ros::NodeHandle* node;

ros::Publisher pub;

string node_name, groupOperator;

double radius_min, radius_max, box_x_minimum, box_x_maximum, box_y_minimum, box_y_maximum;

//--------------------------------------------------------------------------------------------------------
sensor_msgs::LaserScan filterMsg(const sensor_msgs::LaserScan & msg, const double r_min = 0, const double box_x_min = -INFINITY, const double box_y_min = -INFINITY,
                                 const double r_max = INFINITY, const double box_x_max = INFINITY, const double box_y_max = INFINITY)
{
	sensor_msgs::LaserScan filteredMsg = msg;

    double angle = msg.angle_min; 

	for(int i = 0; i < filteredMsg.ranges.size(); i++){ //For each ray
		Point p(filteredMsg.ranges[i] * cos(angle), filteredMsg.ranges[i] * sin(angle));

		if(groupOperator == "union" && !((box_x_min <= p.getX() && p.getX() <= box_x_max && box_y_min <= p.getY() && p.getY() <= box_y_max) || (r_min <= filteredMsg.ranges[i] && filteredMsg.ranges[i] <= r_max))){
			filteredMsg.ranges[i] = INFINITY;
		}
		else if(groupOperator == "intersection" && !((box_x_min <= p.getX() && p.getX() <= box_x_max && box_y_min <= p.getY() && p.getY() <= box_y_max) && (r_min <= filteredMsg.ranges[i] && filteredMsg.ranges[i] <= r_max))){
			filteredMsg.ranges[i] = INFINITY;
		}

        angle += msg.angle_increment; //Increment angle
	}

	return filteredMsg;
}
//--------------------------------------------------------------------------------------------------------
void OnRosMsg(const sensor_msgs::LaserScan & msg){ //ROS message received 
	sensor_msgs::LaserScan filteredMsg = filterMsg(msg, radius_min, box_x_minimum, box_y_minimum, radius_max, box_x_maximum, box_y_maximum);

	pub.publish(filteredMsg);
}

//--------------------------------------------------------------------------------------------------------
int main(int argc, char **argv){ //Main function 
    ros::init(argc, argv, "engrais_lidar_filter");

    node = new ros::NodeHandle();

    node_name = ros::this_node::getName();

    string sub_topic, pub_topic;

    node->param<string>(node_name + "/subscribe_topic", sub_topic, "default/filteredLidarMsg"); //Get parameters or set default values
    node->param<string>(node_name + "/publish_topic", pub_topic, "default/filteredLidarMsg"); //Get parameters or set default values

    node->param<double>(node_name + "/radius_minimum", radius_min, -INFINITY);
    node->param<double>(node_name + "/radius_maximum", radius_max, INFINITY);

    node->param<double>(node_name + "/box_x_minimum", box_x_minimum, -INFINITY);
    node->param<double>(node_name + "/box_x_maximum", box_x_maximum, INFINITY);

    node->param<double>(node_name + "/box_y_minimum", box_y_minimum, -INFINITY);
    node->param<double>(node_name + "/box_y_maximum", box_y_maximum, INFINITY);

    node->param<string>(node_name + "/operator", groupOperator, "union");

    ros::Subscriber sub = node->subscribe(sub_topic, 10, OnRosMsg); // /engrais/laser_front/scan or /engrais/laser_back/scan
    pub = node->advertise<sensor_msgs::LaserScan>(pub_topic, 10);// /engrais/laser_front/lines or /engrais/laser_back/lines

    Utility::printInColor(node_name + ": Code Running, press Control+C to end", CYAN);
    ros::spin();
    Utility::printInColor(node_name + ": Shutting down...", CYAN);

    sub.shutdown();
    pub.shutdown();

    ros::shutdown();

    Utility::printInColor(node_name + ": Code ended without errors", BLUE);

    return 0;
}