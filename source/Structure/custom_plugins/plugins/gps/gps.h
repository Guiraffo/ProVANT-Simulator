/*
* File: gps.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement GPS. It gets information from Gazebo topics and publishes in Ros topic.
*/

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
  class gps : public ModelPlugin
  {
    /// Constructor.
    public: gps();
    /// Destructor.
    public: virtual ~gps();
    /// Load the sensor plugin.
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    /// Callback that receives the gps's update signal.
    private: void OnUpdate(ConstGPSPtr&);


    physics::WorldPtr world; // pointer to the world
    UpdateTimer updateTimer; // pointer to time
    event::ConnectionPtr updateConnection; // pointer to connection

    private: std::string gazebotopic; // name of Gazebo's topic
    private: std::string rostopic; // name of ROS's topic
    private: transport::SubscriberPtr sub; // Gazebo's subscriber
    private: transport::NodePtr node; // Gazebo node handle
    physics::ModelPtr model; // pointer to model
    ros::NodeHandle n; // ros node handle
    ros::Publisher publisher_; // Ros publisher
  };
}
#endif
