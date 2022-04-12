/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file AllData5.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * publishes the state vector of the UAV5 on a ROS topic as a sensor message.
 *
 * @author Jonatan Campos
 * @author Júnio Eduardo de Morais Aquino
 */

#include "AllData5.h"

#include <simulator_msgs/Sensor.h>

#include "XMLRead.h"

namespace gazebo
{
AllData5::AllData5() : distributionX(0, 0), distributionY(0, 0), distributionZ(0, 0)
{
}

void AllData5::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    ROS_INFO("Nao inicializado!");
    return;
  }

  NameOfNode_ = XMLRead::ReadXMLString("NameOfNode", _sdf);
  NameOfJointR_ = XMLRead::ReadXMLString("NameOfJointR", _sdf);
  NameOfJointL_ = XMLRead::ReadXMLString("NameOfJointL", _sdf);

  world = _model->GetWorld();
  juntaR = _model->GetJoint(NameOfJointR_);
  juntaL = _model->GetJoint(NameOfJointL_);

  link_name_ = XMLRead::ReadXMLString("bodyName", _sdf);
  link = _model->GetLink(link_name_);

  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });
  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_, 1);
}

void AllData5::Update()
{
  common::Time sim_time = world->SimTime();
  boost::mutex::scoped_lock scoped_lock(lock);
  //			data.header.stamp = ros::Time(sim_time.sec, sim_time.nsec);

  simulator_msgs::Sensor newmsg;
  newmsg.name = NameOfNode_;
  newmsg.header.stamp = ros::Time::now();
  newmsg.header.frame_id = "1";
  ignition::math::Pose3d pose = link->WorldPose();

  newmsg.values.push_back(pose.Pos().X());          // 0
  newmsg.values.push_back(pose.Pos().Y());          // 1
  newmsg.values.push_back(pose.Pos().Z());          // 2
  newmsg.values.push_back(pose.Rot().Euler().X());  // 3
  newmsg.values.push_back(pose.Rot().Euler().Y());  // 4
  newmsg.values.push_back(pose.Rot().Euler().Z());  // 5
  newmsg.values.push_back(juntaR->Position(0));     // aR
  newmsg.values.push_back(juntaL->Position(0));     // aL

  ignition::math::Vector3d linear = link->WorldLinearVel();

  newmsg.values.push_back(linear.X());  // 8
  newmsg.values.push_back(linear.Y());  // 9
  newmsg.values.push_back(linear.Z());  // 10

  ignition::math::Vector3d angular = link->WorldAngularVel();
  newmsg.values.push_back(angular.X());             // 11
  newmsg.values.push_back(angular.Y());             // 12
  newmsg.values.push_back(angular.Z());             // 13
  newmsg.values.push_back(juntaR->GetVelocity(0));  // 14
  newmsg.values.push_back(juntaL->GetVelocity(0));  // 15

  publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(AllData5)
}  // namespace gazebo
