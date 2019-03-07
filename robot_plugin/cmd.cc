#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

using namespace std;
/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo as a client
  #if GAZEBO_MAJOR_VERSION < 6
    gazebo::setupClient(_argc, _argv);
  #else
    gazebo::client::setup(_argc, _argv);
  #endif

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  
  node->Init();

  // Publish to the  velodyne topic

  gazebo::transport::PublisherPtr sersor_pub =  node->Advertise<gazebo::msgs::Vector2d>("~/robot/sensor/vel_cmd");
  gazebo::transport::PublisherPtr wheels_pub =  node->Advertise<gazebo::msgs::Vector2d>("~/robot/wheels/vel_cmd");

  if(!(_argc == 2 || _argc == 4)){
    cerr << "Wrong arguments, exected [sensor_vel] or [sensor_vel wheel_1_vel wheel_2_vel]" << endl;
    return -1;
  }

  // Wait for a subscriber to connect to this publisher
  if(_argc == 2){
    sersor_pub->WaitForConnection();
  }

  if(_argc == 4){
    sersor_pub->WaitForConnection();
    wheels_pub->WaitForConnection();
  }

  // Create a a vector3 message
  gazebo::msgs::Vector2d sensor_msg;
  gazebo::msgs::Vector2d wheels_msg;

  // Set the velocity in the x-component
  #if GAZEBO_MAJOR_VERSION < 6
    if(_argc == 2){
      gazebo::msgs::Set(&sensor_msg, gazebo::math::Vector2(atof(_argv[1]), 0));
    }

    if(_argc == 4){
      gazebo::msgs::Set(&sensor_msg, gazebo::math::Vector2(atof(_argv[1]), 0));
      gazebo::msgs::Set(&wheels_msg, gazebo::math::Vector2(atof(_argv[2]), atof(_argv[3])));
    }

  #else
    if(_argc == 2){
      gazebo::msgs::Set(&sensor_msg, ignition::math::Vector2d(atof(_argv[1]), 0));
    }

    if(_argc == 4){
      gazebo::msgs::Set(&sensor_msg, ignition::math::Vector2d(atof(_argv[1]), 0));
      gazebo::msgs::Set(&wheels_msg, ignition::math::Vector2d(atof(_argv[2]), atof(_argv[2])));
    }

  #endif


  // Send the message
  if(_argc == 2)
    sersor_pub->Publish(sensor_msg);

  if(_argc == 4){
    sersor_pub->Publish(sensor_msg);
    wheels_pub->Publish(wheels_msg);
  }



  // Make sure to shut everything down.
  #if GAZEBO_MAJOR_VERSION < 6
    gazebo::shutdown();
  #else
    gazebo::client::shutdown();
  #endif
}