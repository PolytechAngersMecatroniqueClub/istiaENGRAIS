//********************************************************************************************************
#ifndef CONTROL_H
#define CONTROL_H

#include <stdio.h> 
#include <stdlib.h>   
#include <iostream>

#include "../../include/1_Point/Point.h"
#include "../../include/3_Utility/Utility.h"
#include "../../include/4_Model/Model.h"

#include <visualization_msgs/Marker.h>

class Control{
	public:
		Model ClosestLeftModel;
		Model ClosestRightModel;
	public:
		Control(){}

};

#endif