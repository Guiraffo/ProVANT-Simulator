/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file imu.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * reads the IMU Data for a particular model.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include <imu.h>

#include "XMLRead.h"

namespace gazebo
{
// initial setup
void imu::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // GET INFO XML
  Topic_ = XMLRead::ReadXMLString("Topic", _sdf);
  link_name_ = XMLRead::ReadXMLString("bodyName", _sdf);

  // OTHERS TOOLS (ROS, GAZEBO)
  imu_pub = n.advertise<simulator_msgs::Sensor>(Topic_, 1);
  link = _model->GetLink(link_name_);
  world = _model->GetWorld();
  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });
}

// callback to publish IMU data
void imu::Update()
{
  // GET DATA
  common::Time sim_time = world->SimTime();                            // simulation time
  ignition::math::Pose3d pose = link->WorldPose();                     // world pose
  ignition::math::Vector3d angular = link->WorldAngularVel();          // angular velocity
  ignition::math::Vector3d AccelLinear = link->RelativeLinearAccel();  // linear acceleration

  // FILL MSG
  simulator_msgs::Sensor newmsg;
  newmsg.name = Topic_;
  newmsg.header.stamp = ros::Time::now();
  newmsg.header.frame_id = "1";
  newmsg.values.push_back(fmod(pose.Rot().Euler().X(), 360));  // data range -pi to pi
  newmsg.values.push_back(fmod(pose.Rot().Euler().Y(), 360));  // data range -pi to pi
  newmsg.values.push_back(fmod(pose.Rot().Euler().Z(), 360));  // data range -pi to pi
  newmsg.values.push_back(angular.X());
  newmsg.values.push_back(angular.Y());
  newmsg.values.push_back(angular.Z());
  newmsg.values.push_back(AccelLinear.X());
  newmsg.values.push_back(AccelLinear.Y());
  newmsg.values.push_back(AccelLinear.Z());

  // SEND MSG
  imu_pub.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(imu)
}  // namespace gazebo
