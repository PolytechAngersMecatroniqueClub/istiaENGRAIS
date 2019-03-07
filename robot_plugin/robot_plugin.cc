#ifndef _ROBOT_PLUGIN_HH_
#define _ROBOT_PLUGIN_HH_

#include <thread>

#include <ros/ros.h> 
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo_client.hh>


#include <iostream>
#include <fstream>
#include <string> 
#include <unistd.h>

using namespace std;

#define pi 3.1415926535

#define BLUE 34
#define RED 31
#define CYAN 36

#define UPDATE_RATE 30

int msgID = 0;

namespace gazebo
{

	// *******************************************************************************************************

	class Utility{ 
	    private:
	    public:
	        
	        template < typename T> static void printVector(const vector<T> & vec){ 
	            for(int i = 0; i < vec.size(); i++){
	                cout << i << ": [" << vec[i] << "]" << endl; 
	            }
	            cout << endl;
	        }
	        // -------------------------------------------------------------------------------------------------------
	        template < typename T> static void printVector(const vector<T> & vec, ostream & out){ 
	            for(int i = 0; i < vec.size(); i++){
	                out << i << ": [" << vec[i] << "]" << endl; 
	            }
	            out << endl;
	        }
	        // -------------------------------------------------------------------------------------------------------
	        template < typename T> static void printVector(const vector<T> & vec, fstream & out){ 
	            for(int i = 0; i < vec.size(); i++){
	                out << i << ": [" << vec[i] << "]" << endl; 
	            }
	            out << endl;
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
	        static double sum(const vector<double> & vec){
	        	double s = 0;

	        	for(int i = 0; i < vec.size(); i++)
	        		s += vec[i];

	        	return s;
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
	            cout << endl << "\033[1;" << color << "m" << "[OK] " << msg << "\033[0m";

	            if(color == RED || color == BLUE)
	                cout << endl;
	        }
	};

	// *******************************************************************************************************

	class Point{ 
		public:
		    double x;
		    double y;

		    // --------------------------------------------------------------------------------------------------
		    Point(){
		        x = 0;
		        y = 0;
		    }
		    // --------------------------------------------------------------------------------------------------
		    Point(const double xx, const double yy){
		        x = xx;
		        y = yy;
		    }
		    // --------------------------------------------------------------------------------------------------
		    friend ostream & operator << (ostream &out, const Point &p); // Operator to display class on screen
		    friend fstream & operator << (fstream &arq, const Point &p); // Operator to store class in a log file
		    // --------------------------------------------------------------------------------------------------
	};
	// -------------------------------------------------------------------------------------------------------
	ostream & operator << (ostream &out, const Point &p){ 
	    out << "[" << p.x << ", " << p.y << "]"; 

	    return out; 
	} 
	// -------------------------------------------------------------------------------------------------------
	fstream & operator << (fstream &out, const Point &p){ 
	    out << "[" << p.x << ", " << p.y << "]"; 

	    return out; 
	} 

	// ********************************************************************************************************

	class RobotPlugin : public ModelPlugin{

		//Name of the ros Node
		private: 
			const string rosName = "gazebo_node";

			//Store robot's parts
			physics::ModelPtr robot;
			physics::ModelPtr sensor;

			//Strore robot's Joints
			physics::JointPtr sensor_top;
			physics::JointPtr wheel_1;
			physics::JointPtr wheel_2;

			//Store PID reference
			common::PID sensor_pid;
			common::PID wheels_pid;

			//Node for communication with cmd
			transport::NodePtr node;

			//Subscribers to change robot's state
			transport::SubscriberPtr sensor_sub;
			transport::SubscriberPtr wheels_sub;
			transport::SubscriberPtr data_sub; //This one in particular to get sensor's / gazebo's data

			//Names to generate the topic
			string sensor_topic_name;
			string wheels_topic_name;
			string data_topic_name;

			//Ros node to communicate with ROS
			unique_ptr<ros::NodeHandle> rosNode;

			//Ros publisher tp publish sensor's / gazebo's data
			ros::Publisher ros_data_values_pub;

			//Ros subscriers to change robot's state with Ros
			ros::Subscriber ros_sensor_sub;
			ros::Subscriber ros_wheels_sub;
			ros::Subscriber ros_keyboard_wheels_sub;

			//Necessary to create callback functions with gazebo
			ros::CallbackQueue rosQueue;

			//Necessary to create callback functions with gazebo
			std::thread rosQueueThread;

			// ---------------------------------------------------------------------------------------------------

			~RobotPlugin(){ //Destroy and unsubscribe everything (otherwise the topic or node would still exist after terminating the program) 
				if(this->data_sub != NULL)
					this->data_sub->Unsubscribe();

				if(this->ros_data_values_pub != NULL)
					this->ros_data_values_pub.shutdown();



				if(this->ros_sensor_sub != NULL)
					this->ros_sensor_sub.shutdown();

				if(this->ros_wheels_sub != NULL)
					this->ros_wheels_sub.shutdown();

				if(this->ros_keyboard_wheels_sub != NULL)
					this->ros_keyboard_wheels_sub.shutdown();



				if(this->rosNode != NULL)
					ros::shutdown();


				if(this->sensor_sub != NULL)
					this->sensor_sub->Unsubscribe();

				if(this->wheels_sub != NULL)
					this->wheels_sub->Unsubscribe();


				if(this->node != NULL)
					this->node->Fini();


				Utility::printInColor("Program Terminated, topics and nodes destroyed", BLUE); 
			}
			// ---------------------------------------------------------------------------------------------------
			physics::ModelPtr GetNestedModel(const physics::ModelPtr& _model, const string name){ // Get model inside a model 
				physics::ModelPtr r = _model->NestedModel(name);

				if(r == NULL){
					Utility::printInColor("Nested Model Named \"" + name + "\" not found in \"" + _model->GetScopedName() + "\"", RED);
					exit(-2);
				}

				return r;
			}
			// ---------------------------------------------------------------------------------------------------
			physics::JointPtr GetJoint(const physics::ModelPtr& _model, const string name){
				physics::JointPtr r = _model->GetJoint(name);

				if(r == NULL){
					Utility::printInColor("Joint Named \"" + name + "\" not found in \"" + _model->GetScopedName() + "\"", RED);
					exit(-3);
				}

				return r;
			}
			// ---------------------------------------------------------------------------------------------------
			common::PID SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, const double P = 0.0, const double I = 0.0, const double D = 0.0){
				// Setup a P-controller, with a gain of P
				common::PID pid = common::PID(P, I, D);		

				// Apply the P-controller to the joint.
				 _model->GetJointController()->SetVelocityPID(_joint->GetScopedName(), pid);

				return pid;
			}
			// ---------------------------------------------------------------------------------------------------
			void SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, common::PID & pid){
				_model->GetJointController()->SetVelocityPID(_joint->GetScopedName(), pid);
				return;
			}
			// ---------------------------------------------------------------------------------------------------
			void InitializeModel(const physics::ModelPtr& _model, const sdf::ElementPtr& _sdf){ //Initialize program getting models, joints, setting PIDs to each joint and initial velocity

				// Safety check
				if (_model->GetJointCount() != 4)
				{
					///std::string s = to_string(_model->GetJointCount());
					Utility::printInColor("Invalid joint count [" + to_string(_model->GetJointCount()) + "], Robot plugin not loaded", RED);
					exit(-1);
				}


				//Save Models and joints in memory for later manipulation 
				this->robot = _model;
				this->sensor = GetNestedModel(_model, "sensor");

				this->sensor_top = GetJoint(this->sensor, "base_JOINT_top");
				this->wheel_1 = GetJoint(this->robot, "project_robot_sphere::body_JOINT_wheel_1");
				this->wheel_2 = GetJoint(this->robot, "project_robot_sphere::body_JOINT_wheel_2");

				Utility::printInColor("Models and Joints found successfully", CYAN);


				//Set PIDs for each joint as P-Controller 0.1
				this->sensor_pid = SetPID(this->sensor, this->sensor_top, 0.1);
				this->wheels_pid = SetPID(this->robot, this->wheel_1, 0.1);
				SetPID(this->robot, this->wheel_2, this->wheels_pid);


				Utility::printInColor("PIDs have been set for each joint", CYAN);
				

				// Default to zero velocity
				double sensor_angular_velocity = 0.0;
				double wheel_1_angular_velocity = 0.0;
				double wheel_2_angular_velocity = 0.0;

				// Check that the velocity element exists, then read the value
				if (_sdf->HasElement("sensor_velocity"))
					sensor_angular_velocity = _sdf->Get<double>("sensor_velocity");
				if (_sdf->HasElement("wheel_1_velocity"))
					wheel_1_angular_velocity = _sdf->Get<double>("wheel_1_velocity");
				if (_sdf->HasElement("wheel_2_velocity"))
					wheel_2_angular_velocity = _sdf->Get<double>("wheel_2_velocity");

				this->sensor->GetJointController()->SetVelocityTarget(sensor_top->GetScopedName(), sensor_angular_velocity);
				this->robot->GetJointController()->SetVelocityTarget(wheel_1->GetScopedName(), wheel_1_angular_velocity);
				this->robot->GetJointController()->SetVelocityTarget(wheel_2->GetScopedName(), wheel_2_angular_velocity);

				Utility::printInColor("Initialization Complete", BLUE);	
			}


			// ---------------------------------------------------------------------------------------------------
			void OnSensorMsg(ConstVector2dPtr &_msg){ // Changes sensor's rotation speed 
				if(!this->sensor->GetJointController()->SetVelocityTarget(this->sensor_top->GetScopedName(), _msg->x()))
					Utility::printInColor("Error assigning velocity to sensor, joint not found", RED);
			}
			// ---------------------------------------------------------------------------------------------------
			void OnWheelsMsg(ConstVector2dPtr &_msg){ // Changes wheel's rotation speed 
				if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_1->GetScopedName(), _msg->x()))
					Utility::printInColor("Error assigning velocity to wheel 1, joint not found", RED);	
				if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_2->GetScopedName(), _msg->y()))
					Utility::printInColor("Error assigning velocity to wheel 2, joint not found", RED);	
			}
			// ---------------------------------------------------------------------------------------------------
			void CreateCommunication(){ // Set nodes to receive commands through cmd file 

				this->node = transport::NodePtr(new transport::Node());

				#if GAZEBO_MAJOR_VERSION < 8
					this->node->Init(this->robot->GetWorld()->GetName());
				#else
					this->node->Init(this->robot->GetWorld()->Name());
				#endif

				Utility::printInColor("Node created", CYAN);

				// Create a topic name

				wheels_topic_name  = "~/" + this->robot->GetName() + "/" + "wheels" + "/vel_cmd"; // ~/robot/wheel_2/vel_cmd
				sensor_topic_name = "~/" + this->robot->GetName() + "/" + this->sensor->GetName() + "/vel_cmd"; // ~/robot/sensor/vel_cmd

				this->wheels_sub = this->node->Subscribe(wheels_topic_name, &RobotPlugin::OnWheelsMsg, this);
				this->sensor_sub = this->node->Subscribe(sensor_topic_name, &RobotPlugin::OnSensorMsg, this);

				Utility::printInColor("Node and Subscription Nodes created successfully", CYAN);
			}


			// ---------------------------------------------------------------------------------------------------
			sensor_msgs::LaserScan gazeboToRos(ConstLaserScanStampedPtr &_msg){
				gazebo::msgs::LaserScan scan = _msg->scan();

				sensor_msgs::LaserScan rosMsg;

				rosMsg.header.seq = msgID;
				msgID++;

				rosMsg.header.stamp.sec = _msg->time().sec();
				rosMsg.header.stamp.nsec = _msg->time().nsec();

				rosMsg.header.frame_id = "map";

				rosMsg.angle_min = scan.angle_min();
				rosMsg.angle_max = scan.angle_max();
				rosMsg.angle_increment  = scan.angle_step();

				rosMsg.range_min = scan.range_min();
				rosMsg.range_max = scan.range_max();

				rosMsg.time_increment = (1 / UPDATE_RATE) / (scan.count());

				rosMsg.ranges.resize(scan.count());
				rosMsg.intensities.resize(scan.count());

				for(int i = 0; i < scan.count(); i++){
					rosMsg.ranges[i] = scan.ranges(i);
					rosMsg.intensities[i] = scan.intensities(i);
				}

				return rosMsg;
			}
			// ---------------------------------------------------------------------------------------------------
			void OnRosSensorMsg(const std_msgs::Float32ConstPtr &_msg){ // Changes sensor's rotation speed with ROS 
				if(!this->sensor->GetJointController()->SetVelocityTarget(this->sensor_top->GetScopedName(), _msg->data))
					Utility::printInColor("Error assigning velocity to sensor, joint not found", RED);
			}
			// ---------------------------------------------------------------------------------------------------
			void OnRosWheelMsg(const std_msgs::Float32MultiArray::ConstPtr &array){ // Changes wheel's rotation speed with ROS 
				float msg[2];

				int size = 0;

				// get message's values and size
				for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
				{
					msg[size%2] = *it;
					size++;
				}

				if(size != 2){
					Utility::printInColor("Expected a 2 element data array on ROS message, received " + to_string(size) + " instead", RED);
					return;
				}

				if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_1->GetScopedName(), msg[0])){
					Utility::printInColor("Error assigning velocity to wheel 1, joint not found", RED);
					return;
				}

				if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_2->GetScopedName(), msg[1]))
					Utility::printInColor("Error assigning velocity to wheel 2, joint not found", RED);
			}
			// ---------------------------------------------------------------------------------------------------
			void OnRosKeyboardMsg(const std_msgs::Float32MultiArray::ConstPtr &array){
				float msg[2];

				int size = 0;

				// get message's values and size
				for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it)
				{
					msg[size%2] = *it;
					size++;
				}

				if(size != 2){
					Utility::printInColor("Expected a 2 element data array on ROS message, received " + to_string(size) + " instead", RED);
					return;
				}

				this->wheel_1->SetVelocity(0, msg[0]);
				this->wheel_2->SetVelocity(0, msg[1]);
			}
			// ---------------------------------------------------------------------------------------------------
			void QueueThread(){ // ROS helper function that processes messages 
			  static const double timeout = 0.01;
			  while (this->rosNode->ok())
			  {
			    this->rosQueue.callAvailable(ros::WallDuration(timeout));
			  }
			}
			// ---------------------------------------------------------------------------------------------------
			void OnDataMsg(ConstLaserScanStampedPtr &_msg){ // Receives gazebo values and publish to ROS 
				sensor_msgs::LaserScan rosMsg = gazeboToRos(_msg);

				ros_data_values_pub.publish(rosMsg);
			}
			// ---------------------------------------------------------------------------------------------------
			void CreateRosCommunication(){ // Create ROS nodes to get commands and send sensor information 
				// Initialize ros, if it has not already bee initialized.
				if (!ros::isInitialized()){
				  int argc = 0;
				  char **argv = NULL;
				  ros::init(argc, argv, rosName, ros::init_options::NoSigintHandler);
				}

				// Create our ROS node. This acts in a similar manner to
				// the Gazebo node
				this->rosNode.reset(new ros::NodeHandle(rosName));

				Utility::printInColor("RosNode created", CYAN);

				// Create a named topic, and subscribe to it.
				ros::SubscribeOptions sensor_so = ros::SubscribeOptions::create<std_msgs::Float32>(

				      "/" + this->robot->GetName() + "/" + this->sensor->GetName() + "/vel_cmd",
				      1,
				      boost::bind(&RobotPlugin::OnRosSensorMsg, this, _1),
				      ros::VoidPtr(), &this->rosQueue);


				ros::SubscribeOptions wheels_so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(

				      "/" + this->robot->GetName() + "/wheels/vel_cmd",
				      1,
				      boost::bind(&RobotPlugin::OnRosWheelMsg, this, _1),
				      ros::VoidPtr(), &this->rosQueue);

				

				ros::SubscribeOptions keyboard_wheels_so = ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(

				      "/" + this->robot->GetName() + "/wheels/instant_vel_cmd",
				      1,
				      boost::bind(&RobotPlugin::OnRosKeyboardMsg, this, _1),
				      ros::VoidPtr(), &this->rosQueue);

				
				data_topic_name = "/gazebo/default/robot/sensor/top/LIDAR/scan";

				this->ros_wheels_sub = this->rosNode->subscribe(wheels_so);
				this->ros_keyboard_wheels_sub = this->rosNode->subscribe(keyboard_wheels_so);

				this->ros_sensor_sub = this->rosNode->subscribe(sensor_so);


				this->rosQueueThread = std::thread(std::bind(&RobotPlugin::QueueThread, this));


				this->ros_data_values_pub = this->rosNode->advertise<sensor_msgs::LaserScan>("/robot/sensor/data", 1000);
				this->data_sub = this->node->Subscribe(data_topic_name, &RobotPlugin::OnDataMsg, this);



				Utility::printInColor("RosTopics Subscribers created", CYAN);		
			}



		/// \brief Constructor
		public:
		 
			RobotPlugin() {}
			// ---------------------------------------------------------------------------------------------------
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){ // This function is called when plugin is called, a "main" function 

				InitializeModel(_model, _sdf);

				CreateCommunication();

				CreateRosCommunication();

				Utility::printInColor("Initialization, normal and ROS communication finished", BLUE);
			}
	};

	// *******************************************************************************************************

	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
	GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif