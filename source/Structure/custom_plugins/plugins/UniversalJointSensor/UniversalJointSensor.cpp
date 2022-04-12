/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file UniversalJointSensor.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * reads all of the data for a specific joint.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <UniversalJointSensor.h>

#include <simulator_msgs/Sensor.h>

#include "XMLRead.h"

namespace gazebo
{
// initial setup
void UniversalJointSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_INFO("Nao inicializado!");
    return;
  }

  NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic", _sdf);   // name of topic to publish data
  NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint", _sdf);  // name of joint
  axis = XMLRead::ReadXMLString("Axis", _sdf);                 // axis of joint
  world = _model->GetWorld();                                  // get pointer's world
  junta = _model->GetJoint(NameOfJoint_);                      // get pointer's joint

  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });
  // publisher
  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
}

// for each step time
void UniversalJointSensor::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  //

  simulator_msgs::Sensor newmsg;
  newmsg.name = NameOfNode_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = "1";
  int index;  // axis of joint
  if (axis == "axis")
    index = 0;
  else
  {
    if (axis == "axis2")
      index = 1;
    else
    {
      std::cout << "Erro no indice da junta, em UniversalJointSensor";
      exit(1);
    }
  }
  newmsg.values.push_back(junta->Position(index));     // get angle
  newmsg.values.push_back(junta->GetVelocity(index));  // get velocity
  newmsg.values.push_back(junta->GetForce(index));     // get force

  // publish data
  publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(UniversalJointSensor)
}  // namespace gazebo
