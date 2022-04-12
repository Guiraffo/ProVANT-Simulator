/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file aerodinamica5.h
 * @brief This file contains the implementation of the aerodinamica5 class.
 *
 * @author Jonatan Mota Campos
 */

#ifndef PROVANT_AERODINAMICA5_H
#define PROVANT_AERODINAMICA5_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <eigen3/Eigen/Eigen>

#include "XMLRead.h"

namespace gazebo
{
class aerodinamica5 : public ModelPlugin
{
public:
  aerodinamica5();  // constructor
public:
  virtual ~aerodinamica5();                                            // destructor
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;  // initial setup
  void CallbackFR2(std_msgs::Float64);                                 // callback to get the force at brushless 1
  void CallbackFL3(std_msgs::Float64);                                 // callback to get the force at brushless 2
  void CallbackFR4(std_msgs::Float64);                                 // callback to get the force at brushless 3
  void CallbackFL5(std_msgs::Float64);                                 // callback to get the force at brushless 4

private:
  ros::NodeHandle node_handle_;  // node handle of ROS
  physics::WorldPtr world;       // pointer to the simulation world
  physics::LinkPtr linkFr2;      // pointer to the rotor 2 link
  physics::LinkPtr linkFl3;      // pointer to the rotor 3 link
  physics::LinkPtr linkFr4;      // pointer to the rotor 4 link
  physics::LinkPtr linkFl5;      // pointer to the rotor 5 link

  std::string topic_FR2;  // name of topic of brushless 1
  std::string topic_FL3;  // name of topic of brushless 2
  std::string topic_FR4;  // name of topic of brushless 3
  std::string topic_FL5;  // name of topic of brushless 4

  // ROS subscribers
  ros::Subscriber motor_subscriberFR2_;
  ros::Subscriber motor_subscriberFL3_;
  ros::Subscriber motor_subscriberFR4_;
  ros::Subscriber motor_subscriberFL5_;

  std::string NameOfLink2_;  // name of rotor 2
  std::string NameOfLink3_;  // name of rotor 3
  std::string NameOfLink4_;  // name of rotor 4
  std::string NameOfLink5_;  // name of rotor 5

  double DragConst;
};
}  // namespace gazebo

#endif  // PROVANT_AERODINAMICA5_H
