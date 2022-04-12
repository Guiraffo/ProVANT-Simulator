/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file brushless.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * represents a brushless motor.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_BRUSHLESS_PLUGIN_H
#define PROVANT_BRUSHLESS_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class Aerodinamica : public ModelPlugin
{
public:
  Aerodinamica() = default;  // constructor
  virtual ~Aerodinamica() = default;  // destructor
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;  // initial setup
  void CallbackFR(std_msgs::Float64);  // callback to apply forces at right brushless
  void CallbackFL(std_msgs::Float64);  // callback to apply forces at left brushless

private:
  ros::NodeHandle node_handle_;  // node handle of ROS
  physics::WorldPtr world;       // pointer to the simulation world
  physics::LinkPtr linkR;        // pointer to the right brushless's link
  physics::LinkPtr linkL;        // pointer to the left brushless's link
  std::string topic_FR;          // name of topic of right brushless
  std::string topic_FL;          // name of topic of left brushless
  // ROS subscribers
  ros::Subscriber motor_subscriberFL_;
  ros::Subscriber motor_subscriberFR_;
  std::string NameOfLinkDir_;  // name of right brushless's link
  std::string NameOfLinkEsq_;  // name of left brushless's link
  double Fr, Fl;               // Lift Forces
};
}  // namespace gazebo

#endif  // PROVANT_BRUSHLESS_PLUGIN_H
