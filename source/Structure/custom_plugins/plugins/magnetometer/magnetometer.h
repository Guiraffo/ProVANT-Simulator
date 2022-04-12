/*
 * File: magnetometer.h
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description:  This library is responsable to implement magnetometer. It gets information from Gazebo and quantizes
 * the data.
 */

#ifndef PROVANT_MAGNETOMETER_PLUGIN_H
#define PROVANT_MAGNETOMETER_PLUGIN_H

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
public:
  magnetometer() = default;
  virtual ~magnetometer() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  
private:
// Callback that receives the magnetometer sensor's update signal.
  void OnUpdate(ConstMagnetometerPtr&);

  // pointer to the world
  physics::WorldPtr world;
  std::string gazebotopic;  // name of Gazebo's topic
  std::string rostopic;  // name of ROS's topic
  transport::SubscriberPtr sub;  // ROS subscriber
  transport::NodePtr node;    // Gazebo's node handle
  physics::ModelPtr model;    // pounter to the model
  ros::NodeHandle n;          // ROS's node handle
  ros::Publisher publisher_;  // publisher
};
}  // namespace gazebo

#endif // PROVANT_MAGNETOMETER_PLUGIN_H
