/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file brushless.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * represents a brushless motor.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <brushless.h>

#include "XMLRead.h"

namespace gazebo
{
// to load initial setup
void Aerodinamica::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    std::cout << "Aerodynamics not initialized!" << std::endl;
    return;
  }

  topic_FR = XMLRead::ReadXMLString("topic_FR", _sdf);       // name of right brushless's topic
  topic_FL = XMLRead::ReadXMLString("topic_FL", _sdf);       // name of left brushless's topic
  NameOfLinkDir_ = XMLRead::ReadXMLString("LinkDir", _sdf);  // name of right brushless's link
  NameOfLinkEsq_ = XMLRead::ReadXMLString("LinkEsq", _sdf);  // name of left brushless's link

  // get elements of the simulation
  linkR = _model->GetLink(NameOfLinkDir_);
  linkL = _model->GetLink(NameOfLinkEsq_);

  // subscribers of data to apply in simulator
  motor_subscriberFR_ = node_handle_.subscribe(topic_FR, 1, &gazebo::Aerodinamica::CallbackFR, this);
  motor_subscriberFL_ = node_handle_.subscribe(topic_FL, 1, &gazebo::Aerodinamica::CallbackFL, this);
}

// callback to apply forces at right brushless
void Aerodinamica::CallbackFR(std_msgs::Float64 msg)
{
  Fr = msg.data;
  ignition::math::Vector3d forceR(0, 0, Fr);                  // Right force in the left brushless
  ignition::math::Vector3d torqueR(0, 0, 0.0178947368 * Fr);  // drag torque
  // Applying
  linkR->AddRelativeForce(forceR);
  linkR->AddRelativeTorque(torqueR);
}

// callback to apply forces at left brushless
void Aerodinamica::CallbackFL(std_msgs::Float64 msg)
{
  Fl = msg.data;
  ignition::math::Vector3d forceL(0, 0, Fl);                   // Lift force in the left brushless
  ignition::math::Vector3d torqueL(0, 0, -0.0178947368 * Fl);  // drag torque
  // Applying
  linkL->AddRelativeForce(forceL);
  linkL->AddRelativeTorque(torqueL);
}

GZ_REGISTER_MODEL_PLUGIN(Aerodinamica)
}  // namespace gazebo
