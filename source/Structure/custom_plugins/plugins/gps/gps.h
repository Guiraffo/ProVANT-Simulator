/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gps.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads GPS data from Gazebo and publishes it in a ROS topic as a sensor message.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_GPS_H
#define PROVANT_GPS_H

#include <ros/ros.h>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Event.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

namespace gazebo
{
class gps : public ModelPlugin
{
public:
  gps() = default;
  virtual ~gps() = default;
  // Load the sensor plugin.
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate(const ConstGPSPtr& msg);

  physics::WorldPtr world;                // pointer to the world
  event::ConnectionPtr updateConnection;  // pointer to connection
  std::string gazebotopic;                // name of Gazebo's topic
  std::string rostopic;                   // name of ROS's topic
  transport::SubscriberPtr sub;           // Gazebo's subscriber
  transport::NodePtr node;                // Gazebo node handle
  physics::ModelPtr model;                // pointer to model
  ros::NodeHandle n;                      // ros node handle
  ros::Publisher publisher_;              // Ros publisher
};
}  // namespace gazebo

#endif  // PROVANT_GPS_H
