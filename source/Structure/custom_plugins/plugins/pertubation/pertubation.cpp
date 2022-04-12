/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file perturbation.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * simulate a force perturbation.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <pertubation.h>

#include "XMLRead.h"

namespace gazebo
{
// initial setup
void pertubation::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    std::cout << "Aerodinamica nao inicializado!" << std::endl;
    return;
  }

  // get ROS topics name
  topicX = XMLRead::ReadXMLString("topicX", _sdf);
  topicY = XMLRead::ReadXMLString("topicY", _sdf);
  topicZ = XMLRead::ReadXMLString("topicZ", _sdf);
  // get link name
  NameOfLink = XMLRead::ReadXMLString("Link", _sdf);

  // get simulation link
  link = _model->GetLink(NameOfLink);

  // subscribers
  pertubation_subscriberX = node_handle_.subscribe(topicX, 1, &gazebo::pertubation::CallbackX, this);
  pertubation_subscriberY = node_handle_.subscribe(topicY, 1, &gazebo::pertubation::CallbackY, this);
  pertubation_subscriberZ = node_handle_.subscribe(topicZ, 1, &gazebo::pertubation::CallbackZ, this);
}

// Callback to provide a pertubation in x direction
void pertubation::CallbackX(std_msgs::Float64 msg)
{
  Fx = msg.data;
  ignition::math::Vector3d force(Fx, 0, 0);
  // applying
  link->AddRelativeForce(force);
}

// Callback to provide a pertubation in y direction
void pertubation::CallbackY(std_msgs::Float64 msg)
{
  Fy = msg.data;
  ignition::math::Vector3d force(0, Fy, 0);
  // applying
  link->AddRelativeForce(force);
}

// Callback to provide a pertubation in z direction
void pertubation::CallbackZ(std_msgs::Float64 msg)
{
  Fz = msg.data;
  ignition::math::Vector3d force(0, 0, Fz);
  // applying
  link->AddRelativeForce(force);
}

GZ_REGISTER_MODEL_PLUGIN(pertubation)
}  // namespace gazebo
