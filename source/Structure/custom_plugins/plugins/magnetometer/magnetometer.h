/*
* File: magnetometer.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement magnetometer. It gets information from Gazebo and quantizes the data.
*/


#ifndef _GAZEBO_GPS_PLUGIN_HH_
#define _GAZEBO_GPS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include "ros/ros.h"
#include "simulator_msgs/Sensor.h"
#include <random>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include "XMLRead.h"


namespace gazebo
{
  class magnetometer : public ModelPlugin
  {
    // constructor
    public: magnetometer();
    /// destructor.
    public: virtual ~magnetometer();
    // initial setup 
    public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    // Callback that receives the magnetometer sensor's update signal.
    private: void OnUpdate(ConstMagnetometerPtr&);
    //pointer to the world 
    physics::WorldPtr world;
    private: std::string gazebotopic; // name of Gazebo's topic
    private: std::string rostopic; // name of ROS's topic
    private: transport::SubscriberPtr sub; // ROS subscriber
    private: transport::NodePtr node; // Gazebo's node handle
    physics::ModelPtr model; // pounter to the model
    ros::NodeHandle n; // ROS's node handle
    ros::Publisher publisher_; // publisher
  };
}
#endif
