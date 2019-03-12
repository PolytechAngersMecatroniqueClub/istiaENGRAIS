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


namespace gazebo
{
    class RobotPlugin : public ModelPlugin{

            //Name of the ros Node
        private: 
            const std::string rosName = "gazebo_node";

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
            std::string sensor_topic_name;
            std::string wheels_topic_name;
            std::string data_topic_name;

            //Ros node to communicate with ROS
            std::unique_ptr<ros::NodeHandle> rosNode;

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
            int msgID = 0;

            // ---------------------------------------------------------------------------------------------------

            ~RobotPlugin(); //Destroy and unsubscribe everything (otherwise the topic or node would still exist after terminating the program)

            physics::ModelPtr GetNestedModel(const physics::ModelPtr& _model, const std::string name);   // Get model inside a model
            physics::JointPtr GetJoint(const physics::ModelPtr& _model, const std::string name);
            common::PID SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, double P = 0.0, double I = 0.0, double D = 0.0);
            void SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, common::PID & pid);
            void InitializeModel(const physics::ModelPtr& _model, const sdf::ElementPtr& _sdf);     //Initialize program getting models, joints, setting PIDs to each joint and initial velocity
            void OnSensorMsg(ConstVector2dPtr &_msg);                                               // Changes sensor's rotation speed
            void OnWheelsMsg(ConstVector2dPtr &_msg);                                               // Changes wheel's rotation speed
            void CreateCommunication();                                                             // Set nodes to receive commands through cmd file
            sensor_msgs::LaserScan gazeboToRos(ConstLaserScanStampedPtr &_msg);
            void OnRosSensorMsg(const std_msgs::Float32ConstPtr &_msg);                             // Changes sensor's rotation speed with ROS 
            void OnRosWheelMsg(const std_msgs::Float32MultiArray::ConstPtr &array);                 // Changes wheel's rotation speed with ROS 
            void OnRosKeyboardMsg(const std_msgs::Float32MultiArray::ConstPtr &array);
            void QueueThread();                                                                     // ROS helper function that processes messages 
            void OnDataMsg(ConstLaserScanStampedPtr &_msg);                                         // Receives gazebo values and publish to ROS 
            void CreateRosCommunication();                                                          // Create ROS nodes to get commands and send sensor information 

        public:
            // Constructor
            RobotPlugin();
            virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // This function is called when plugin is called, a "main" function 
    };

    // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
    GZ_REGISTER_MODEL_PLUGIN(RobotPlugin)
}
#endif
