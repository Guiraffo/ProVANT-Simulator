/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file perturbation.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * simulate a force perturbation.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_PERTURBATION_H
#define PROVANT_PERTURBATION_H

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class pertubation : public ModelPlugin
{
public:
  pertubation() = default;
  virtual ~pertubation() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  // Callback to provide a pertubation in x direction
  void CallbackX(std_msgs::Float64);
  // Callback to provide a pertubation in y direction
  void CallbackY(std_msgs::Float64);
  // Callback to provide a pertubation in z direction
  void CallbackZ(std_msgs::Float64);

private:
  ros::NodeHandle node_handle_;  // ROS node handle
  physics::WorldPtr world;       // world's pointer
  physics::LinkPtr link;         // link's world
  // topics
  std::string topicX;
  std::string topicY;
  std::string topicZ;
  // name of node handle
  std::string NameOfNode_;
  // subscriber
  ros::Subscriber pertubation_subscriberX;
  ros::Subscriber pertubation_subscriberY;
  ros::Subscriber pertubation_subscriberZ;
  // name of link
  std::string NameOfLink;
  // values of pertubation
  double Fx, Fy, Fz;
};
}  // namespace gazebo

#endif  // PROVANT_PERTURBATION_H
