/*
* File: sonar.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sonar
*/

#ifndef _GAZEBO_SONAR_PLUGIN_HH_
#define _GAZEBO_SONAR_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/sensors/sensors.hh>
#include "ros/ros.h"
#include <random>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"


namespace gazebo
{
  class sonar : public ModelPlugin
  {
    ///  Constructor.
    public: sonar();
    ///  Destructor.
    public: virtual ~sonar();
    /// initial setup
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    /// Callback that receives the sonar's update signal.
    private: virtual void OnUpdate(ConstSonarPtr&);
    // world's pointer
    physics::WorldPtr world;

    private: std::string gazebotopic;// Gazebo's topic
    private: std::string rostopic; // ROS's topic
    private: transport::SubscriberPtr sub; // Gazebo's subscriber
    private: transport::NodePtr node; // Gazebo's node
    physics::ModelPtr model; // pointer of model
    ros::NodeHandle n; // ROS node handle
    ros::Publisher publisher_; // ROS's publisher

  };
}
#endif
