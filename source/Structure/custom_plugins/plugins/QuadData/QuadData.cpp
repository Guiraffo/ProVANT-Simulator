/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file QuadData.cpp
 * @brief This file contains the implementation of the QuadData class.
 *
 * See the header documentation for more details on the QuadData class.
 *
 * @author Jonatan Mota Campos
 */

#include "QuadData.h"

#include <simulator_msgs/Sensor.h>

namespace gazebo
{
// constructor
QuadData::QuadData() : PhipThetapPsip(3, 1), RIB(3, 3), W_n(3, 3), WIIB(3, 1), XpYpZp(3, 1), UVW(3, 1), PQR(3, 1)
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// initial setup
void QuadData::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  _logMsg = "[QuadDataPlugin";
  if (_sdf->HasAttribute("name"))
  {
    _logMsg += ": ";
    _logMsg += _sdf->GetAttribute("name")->GetAsString();
  }
  _logMsg += "] ";
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << _sdf->GetAttribute("name")->GetAsString() << "] "
                                           << "Starting plugin loading process.");
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. Load the Gazebo system plugin "
                                                 "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and "
                                                 "try again. Note: This plugin will be automatically loaded using "
                                                 "the \"roslaunch gazebo_ros empty_world\" launch command.");
    return;
  }

  if (_sdf->HasElement("NameOfTopic"))
  {
    NameOfNode_ = _sdf->Get<std::string>("NameOfTopic");
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while loading the plugin. The NameOfTopic tag is not "
                                                 "defined. "
                                                 "Please define this tag that must contain the name of the Topic to "
                                                 "publish the state vector in the plugin SDF description and try "
                                                 "again.");
    return;
  }

  world = _model->GetWorld();  // pointer to the world

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->Get<std::string>("bodyName");
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while loading the plugin. The bodyName tag is not defined. "
                                                 "Please define this tag taht must contain the name of the link "
                                                 "containing the quadrotor UAV center of rotation in the plugin SDF "
                                                 "description and try again.");
    return;
  }

  link = _model->GetLink(link_name_);  // pointer to the main body

  // ROS publisher
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the /" << NameOfNode_ << " ROS topic.");
  publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(NameOfNode_,
                                                              1);  // NameofNode is actually the name of the topic.

  // Connecting to the update timer.
  updateConnection = event::Events::ConnectWorldUpdateEnd([this]() { this->Update(); });

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The loading process finished successfully.");
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// new step time
void QuadData::Update()
{
  boost::mutex::scoped_lock scoped_lock(lock);  // mutex

  simulator_msgs::Sensor newmsg;
  newmsg.name = NameOfNode_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = link_name_;
  ignition::math::Pose3d pose = link->WorldPose();
  newmsg.values.push_back(pose.Pos().X());          // x
  newmsg.values.push_back(pose.Pos().Y());          // y
  newmsg.values.push_back(pose.Pos().Z());          // z
  newmsg.values.push_back(pose.Rot().Euler().X());  // roll
  newmsg.values.push_back(pose.Rot().Euler().Y());  // pitch
  newmsg.values.push_back(pose.Rot().Euler().Z());  // yaw

  // compute the generalized velocities
  ignition::math::Vector3d linear = link->WorldLinearVel();
  ignition::math::Vector3d angular = link->WorldAngularVel();

  Phi = pose.Rot().Euler().X();
  Theta = pose.Rot().Euler().Y();
  Psi = pose.Rot().Euler().Z();

  RIB << (cos(Psi) * cos(Theta)), (cos(Psi) * sin(Phi) * sin(Theta) - cos(Phi) * sin(Psi)),
      (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)), (cos(Theta) * sin(Psi)),
      (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)),
      (cos(Phi) * sin(Psi) * sin(Theta) - cos(Psi) * sin(Phi)), (-sin(Theta)), (cos(Theta) * sin(Phi)),
      (cos(Phi) * cos(Theta));

  W_n << 1.0, 0.0, -sin(Theta), 0.0, cos(Phi), cos(Theta) * sin(Phi), 0.0, -sin(Phi), cos(Phi) * cos(Theta);

  // Get the angular velocity w.r.t I expressed in I and maps to obtain the time derivative of Euler angles
  WIIB << angular.X(), angular.Y(), angular.Z();
  PhipThetapPsip = W_n.inverse() * (RIB.transpose() * WIIB);
  XpYpZp << linear.X(), linear.Y(), linear.Z();

  // Get u v w
  UVW = RIB.transpose() * XpYpZp;

  // Get p q r (angular velocity w.r.t. inertial frame and expressed in the body fixed frame)
  ignition::math::Vector3d relativeAngularVel = link->RelativeAngularVel();
  PQR << relativeAngularVel.X(), relativeAngularVel.Y(), relativeAngularVel.Z();

  newmsg.values.push_back(linear.X());         // dx
  newmsg.values.push_back(linear.Y());         // dy
  newmsg.values.push_back(linear.Z());         // dz
  newmsg.values.push_back(PhipThetapPsip(0));  // phip
  newmsg.values.push_back(PhipThetapPsip(1));  // thetap
  newmsg.values.push_back(PhipThetapPsip(2));  // psip
  newmsg.values.push_back(UVW(0));             // u
  newmsg.values.push_back(UVW(1));             // v
  newmsg.values.push_back(UVW(2));             // w
  newmsg.values.push_back(PQR(0));             // p
  newmsg.values.push_back(PQR(1));             // q
  newmsg.values.push_back(PQR(2));             // r
  newmsg.values.push_back(pose.Rot().X());     // x quaternions
  newmsg.values.push_back(pose.Rot().Y());     // y quaternions
  newmsg.values.push_back(pose.Rot().Z());     // z quaternions
  newmsg.values.push_back(pose.Rot().W());     // w quaternions

  // Add accelerations to the state plugin
  ignition::math::Vector3d worldAccel = link->WorldAngularAccel();
  newmsg.values.push_back(worldAccel.X());  // Angular acceleration around X
  newmsg.values.push_back(worldAccel.Y());  // Angular acceleration around Y
  newmsg.values.push_back(worldAccel.Z());  // Angular acceleration around Z

  // Log the values to help during debug.
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Linear positions of the quadrotor UAV [m] at iteration "
                                            << world->Iterations() << ": " << pose.Pos().X() << ", " << pose.Pos().Y()
                                            << ", " << pose.Pos().Z() << ";");
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Attitude of the quadrotor UAV (Euler Angles) [rad] at iteration "
                                            << world->Iterations() << ": " << pose.Rot().Euler().X() << ", "
                                            << pose.Rot().Euler().Y() << ", " << pose.Rot().Euler().Z() << ";");
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Orientation quaternion of the quadrotor UAV at iteration "
                                            << world->Iterations() << ": " << pose.Rot().X() << "i + " << pose.Rot().Y()
                                            << "j + " << pose.Rot().Z() << "k + " << pose.Rot().W());
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "World acceleration of the quadrotor UAV at iteration "
                                            << world->Iterations() << ": X: " << worldAccel.X()
                                            << ", Y: " << worldAccel.Y() << ", Z: " << worldAccel.Z() << ".");
  // Publish the state vector message.
  publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(QuadData)
}  // namespace gazebo
