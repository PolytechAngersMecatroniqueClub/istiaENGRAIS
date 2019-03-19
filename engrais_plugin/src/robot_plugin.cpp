#include "robot_plugin.h"

#define pi 3.1415926535
#define UPDATE_RATE 30

using namespace std;
using namespace gazebo;

RobotPlugin::~RobotPlugin(){ //Destroy and unsubscribe everything (otherwise the topic or node would still exist after terminating the program) 
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
    ROS_INFO("Program Terminated, topics and nodes destroyed"); 
}

// ---------------------------------------------------------------------------------------------------

physics::ModelPtr RobotPlugin::GetNestedModel(const physics::ModelPtr& _model, const string name){ // Get model inside a model 
    physics::ModelPtr r = _model->NestedModel(name);
    if(r == NULL){
        string msg = "Nested Model Named \"" + name + "\" not found in \"" + _model->GetScopedName() + "\"";
        ROS_ERROR("%s", msg.c_str());
        exit(-2);
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------

physics::JointPtr RobotPlugin::GetJoint(const physics::ModelPtr& _model, const string name){
    physics::JointPtr r = _model->GetJoint(name);
    if(r == NULL){
        string msg = "Joint Named \"" + name + "\" not found in \"" + _model->GetScopedName() + "\"";
        ROS_ERROR("%s", msg.c_str());
        exit(-3);
    }
    return r;
}

// ---------------------------------------------------------------------------------------------------

common::PID RobotPlugin::SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, double P, double I, double D){
    // Setup a P-controller, with a gain of P
    common::PID pid = common::PID(P, I, D);		
    // Apply the P-controller to the joint.
    _model->GetJointController()->SetVelocityPID(_joint->GetScopedName(), pid);
    return pid;
}

// ---------------------------------------------------------------------------------------------------

common::PID RobotPlugin::SetPID(const physics::ModelPtr& _model, const physics::JointPtr& _joint, const physics::JointPtr& _joint2, double P, double I, double D){
    // Setup a P-controller, with a gain of P
    common::PID pid = common::PID(P, I, D);     
    // Apply the P-controller to the joint.
    _model->GetJointController()->SetVelocityPID(_joint->GetScopedName(), pid);
    _model->GetJointController()->SetVelocityPID(_joint2->GetScopedName(), pid);
    return pid;
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::InitializeModel(const physics::ModelPtr& _model, const sdf::ElementPtr& _sdf){ //Initialize program getting models, joints, setting PIDs to each joint and initial velocity
    // Safety check
    if (_model->GetJointCount() != 4){
        string msg = "Invalid joint count [" + to_string(_model->GetJointCount()) + "], Robot plugin not loaded";
        ROS_ERROR("%s", msg.c_str());
        exit(-1);
    }

    //Save Models and joints in memory for later manipulation 
    this->robot = _model;
    this->sensor = GetNestedModel(_model, "lidar_engrais");

    this->sensor_top = GetJoint(this->sensor, "base_JOINT_top");
    this->wheel_1 = GetJoint(this->robot, "robot_engrais::body_JOINT_wheel_1");
    this->wheel_2 = GetJoint(this->robot, "robot_engrais::body_JOINT_wheel_2");

    ROS_INFO("Models and Joints found successfully");

    //Set PIDs for each joint as P-Controller 10
    this->sensor_pid = SetPID(this->sensor, this->sensor_top, 10.0);
    this->wheels_pid = SetPID(this->robot, this->wheel_1, this->wheel_2, 10.0);

    ROS_INFO("PIDs have been set for each joint");

    ROS_INFO("Initialization Complete");
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::OnSensorMsg(ConstVector2dPtr &_msg){ // Changes sensor's rotation speed 
    if(!this->sensor->GetJointController()->SetVelocityTarget(this->sensor_top->GetScopedName(), _msg->x()))
        ROS_ERROR("Error assigning velocity to sensor, joint not found");
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::OnWheelsMsg(ConstVector2dPtr &_msg){ // Changes wheel's rotation speed 
    if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_1->GetScopedName(), _msg->x()))
        ROS_ERROR("Error assigning velocity to wheel 1, joint not found");
    if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_2->GetScopedName(), _msg->y()))
        ROS_ERROR("Error assigning velocity to wheel 2, joint not found");
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::CreateCommunication(){ // Set nodes to receive commands through cmd file 

    this->node = transport::NodePtr(new transport::Node());

    #if GAZEBO_MAJOR_VERSION < 8
        this->node->Init(this->robot->GetWorld()->GetName());
    #else
        this->node->Init(this->robot->GetWorld()->Name());
    #endif

    ROS_INFO("Node created");

    // Create a topic name

    wheels_topic_name  = "~/" + this->robot->GetName() + "/" + "wheels" + "/vel_cmd"; // ~/robot/wheel/vel_cmd
    sensor_topic_name = "~/" + this->robot->GetName() + "/" + this->sensor->GetName() + "/vel_cmd"; // ~/robot/sensor/vel_cmd

    this->wheels_sub = this->node->Subscribe(wheels_topic_name, &RobotPlugin::OnWheelsMsg, this);
    this->sensor_sub = this->node->Subscribe(sensor_topic_name, &RobotPlugin::OnSensorMsg, this);

    ROS_INFO("Node and Subscription Nodes created successfully");
}

// ---------------------------------------------------------------------------------------------------

sensor_msgs::LaserScan RobotPlugin::gazeboToRos(ConstLaserScanStampedPtr &_msg){
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

void RobotPlugin::OnRosSensorMsg(const std_msgs::Float32ConstPtr &_msg){ // Changes sensor's rotation speed with ROS 
    if(!this->sensor->GetJointController()->SetVelocityTarget(this->sensor_top->GetScopedName(), _msg->data))
        ROS_ERROR("Error assigning velocity to sensor, joint not found");
}

// ---------------------------------------------------------------------------------------------------
void RobotPlugin::OnRosWheelMsg(const std_msgs::Float32MultiArray::ConstPtr &array){ // Changes wheel's rotation speed with ROS 
    float msg[2];

    int size = 0;

    // get message's values and size
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
        msg[size%2] = *it;
        size++;
    }

    if(size != 2){
        // ROS_ERROR("Expected a 2 element data array on ROS message, received " + to_string(size) + " instead");
        ROS_ERROR("Expected a 2 element data array on ROS message, received X instead");
        return;
    }

    if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_1->GetScopedName(), msg[0])){
        ROS_ERROR("Error assigning velocity to wheel 1, joint not found");
        return;
    }

    if(!this->robot->GetJointController()->SetVelocityTarget(this->wheel_2->GetScopedName(), msg[1]))
        ROS_ERROR("Error assigning velocity to wheel 2, joint not found");
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::OnRosKeyboardMsg(const std_msgs::Float32MultiArray::ConstPtr &array){
    float msg[2];

    int size = 0;

    // get message's values and size
    for(std::vector<float>::const_iterator it = array->data.begin(); it != array->data.end(); ++it){
        msg[size%2] = *it;
        size++;
    }

    if(size != 2){
        // ROS_ERROR("Expected a 2 element data array on ROS message, received " + to_string(size) + " instead");
        ROS_ERROR("Expected a 2 element data array on ROS message, received X instead");
        return;
    }

    this->wheel_1->SetVelocity(0, msg[0]);
    this->wheel_2->SetVelocity(0, msg[1]);
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::QueueThread(){ // ROS helper function that processes messages 
    static const double timeout = 0.01;
    while (this->rosNode->ok()){
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
    }
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::OnDataMsg(ConstLaserScanStampedPtr &_msg){ // Receives gazebo values and publish to ROS 
    sensor_msgs::LaserScan rosMsg = gazeboToRos(_msg);
    ros_data_values_pub.publish(rosMsg);
}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::CreateRosCommunication(){ // Create ROS nodes to get commands and send sensor information 
    // Initialize ros, if it has not already bee initialized.
    if (!ros::isInitialized()){
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, rosName, ros::init_options::NoSigintHandler);
    }

    // Create our ROS node. This acts in a similar manner to
    // the Gazebo node
    this->rosNode.reset(new ros::NodeHandle(rosName));

    ROS_INFO("RosNode created");

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

    data_topic_name = "/gazebo/default/robot_engrais/lidar_engrais/top/LIDAR/scan";

    this->ros_wheels_sub = this->rosNode->subscribe(wheels_so);
    this->ros_keyboard_wheels_sub = this->rosNode->subscribe(keyboard_wheels_so);
    this->ros_sensor_sub = this->rosNode->subscribe(sensor_so);
    this->rosQueueThread = std::thread(std::bind(&RobotPlugin::QueueThread, this));
    this->ros_data_values_pub = this->rosNode->advertise<sensor_msgs::LaserScan>("/robot_engrais/lidar_engrais/data", 1000);
    this->data_sub = this->node->Subscribe(data_topic_name, &RobotPlugin::OnDataMsg, this);

    ROS_INFO("RosTopics Subscribers created");
}

// ---------------------------------------------------------------------------------------------------

RobotPlugin::RobotPlugin() {}

// ---------------------------------------------------------------------------------------------------

void RobotPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf){ // This function is called when plugin is called, a "main" function 
    InitializeModel(_model, _sdf);
    CreateCommunication();
    CreateRosCommunication();
    ROS_INFO("Initialization, normal and ROS communication finished");
}
