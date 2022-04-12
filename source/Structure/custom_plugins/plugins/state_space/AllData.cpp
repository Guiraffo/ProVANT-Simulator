/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file AllData.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * reads the state-space data of a UAV.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <AllData.h>

#include <simulator_msgs/Sensor.h>

#include "XMLRead.h"

namespace gazebo
{
// constructor
AllData::AllData() : PhipThetapPsip(3, 1), RIB(3, 3), W_n(3, 3), WIIB(3, 1)
{
}

// initial setup
void AllData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  std::cout << "entrei";
  if (!ros::isInitialized())
  {
    ROS_INFO("Nao inicializado!");
    return;
  }

  NameOfNode_ = XMLRead::ReadXMLString("NameOfTopic", _sdf);     // Get name of topic to publish data
  NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR", _sdf);  // name of the right joint
  NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL", _sdf);  // name of the left joint

  world = _model->GetWorld();                // pointer to the world
  juntaR = _model->GetJoint(NameOfJointR_);  // pointer to the right joint
  juntaL = _model->GetJoint(NameOfJointL_);  // pointer to the left joint

  link_name_ = XMLRead::ReadXMLString("bodyName", _sdf);  // name of the main body
  link = _model->GetLink(link_name_);                     // pointer to the main body

  // notifying when occurs new step time
  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });
  // ROS publisher
  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
}

// new step time
void AllData::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  // mutex

  simulator_msgs::Sensor newmsg;
  newmsg.name = NameOfNode_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = "1";
  ignition::math::Pose3d pose = link->WorldPose();
  newmsg.values.push_back(pose.Pos().X());          // x
  newmsg.values.push_back(pose.Pos().Y());          // y
  newmsg.values.push_back(pose.Pos().Z());          // z
  newmsg.values.push_back(pose.Rot().Euler().X());  // roll
  newmsg.values.push_back(pose.Rot().Euler().Y());  // pitch
  newmsg.values.push_back(pose.Rot().Euler().Z());  // yaw
  newmsg.values.push_back(juntaR->Position(0));     // aR
  newmsg.values.push_back(juntaL->Position(0));     // aL
  ignition::math::Vector3d linear = link->WorldLinearVel();
  newmsg.values.push_back(linear.X());  // vx
  newmsg.values.push_back(linear.Y());  // vy
  newmsg.values.push_back(linear.Z());  // vz
  ignition::math::Vector3d angular = link->WorldAngularVel();

  // Maps to the body
  Phi = pose.Rot().Euler().X();
  Theta = pose.Rot().Euler().Y();
  Psi = pose.Rot().Euler().Z();

  RIB << (cos(Psi) * cos(Theta)), (cos(Psi) * sin(Phi) * sin(Theta) - cos(Phi) * sin(Psi)),
      (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)), (cos(Theta) * sin(Psi)),
      (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)),
      (cos(Phi) * sin(Psi) * sin(Theta) - cos(Psi) * sin(Phi)), (-sin(Theta)), (cos(Theta) * sin(Phi)),
      (cos(Phi) * cos(Theta));

  W_n << 1.0, 0.0, -sin(Theta), 0.0, cos(Phi), cos(Theta) * sin(Phi), 0.0, -sin(Phi), cos(Phi) * cos(Theta);

  WIIB << angular.X(), angular.Y(), angular.Z();
  PhipThetapPsip = W_n.inverse() * RIB.transpose() * WIIB;

  newmsg.values.push_back(PhipThetapPsip(0));
  newmsg.values.push_back(PhipThetapPsip(1));
  newmsg.values.push_back(PhipThetapPsip(2));

  newmsg.values.push_back(juntaR->GetVelocity(0));  // daR
  newmsg.values.push_back(juntaL->GetVelocity(0));  // daL

  // publish data
  publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(AllData)
}  // namespace gazebo
