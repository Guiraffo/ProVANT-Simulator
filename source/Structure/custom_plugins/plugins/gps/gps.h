#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include "ros/ros.h"
#include "simulator_msgs/Sensor.h"
#include <random>
#include <update_timer.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "XMLRead.h"



namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class gps : public ModelPlugin
  {
    /// \brief Constructor.
    public: gps();

    /// \brief Destructor.
    public: virtual ~gps();

    /// \brief Load the sensor plugin.
    /// \param[in] _sensor Pointer to the sensor that loaded this plugin.
    /// \param[in] _sdf SDF element that describes the plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

    /// \brief Callback that receives the contact sensor's update signal.
    private: void OnUpdate(ConstGPSPtr&);


    physics::WorldPtr world;
    UpdateTimer updateTimer;
    event::ConnectionPtr updateConnection;

    //private: ros::NodeHandle n;

    //private: ros::Publisher gps_pub;

    private: std::string gazebotopic;
    private: std::string rostopic;
    private: double Nbits;
    private: transport::SubscriberPtr sub;
    private: transport::NodePtr node;
    physics::ModelPtr model;

    ros::NodeHandle n;
    ros::Publisher publisher_;
  };
}
#endif
