/*
 * File: sonar.h
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description:  This library is responsable to implement a sonar
 */

#ifndef PROVANT_SONAR_PLUGIN_H
#define PROVANT_SONAR_PLUGIN_H

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
public:
  sonar() = default;
  virtual ~sonar() = default;
  /// initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  /// Callback that receives the sonar's update signal.
  virtual void OnUpdate(ConstSonarPtr&);
  // world's pointer
  physics::WorldPtr world;

private:
  std::string gazebotopic;  // Gazebo's topic
  std::string rostopic;  // ROS's topic
  transport::SubscriberPtr sub;  // Gazebo's subscriber
  transport::NodePtr node;    // Gazebo's node
  physics::ModelPtr model;    // pointer of model
  ros::NodeHandle n;          // ROS node handle
  ros::Publisher publisher_;  // ROS's publisher
};
}  // namespace gazebo

#endif  // PROVANT_SONAR_PLUGIN_H
