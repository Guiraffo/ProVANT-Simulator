/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file user_camera_follower.cpp
 * @brief This file contains the implementation of the UserCameraFollower class.
 *
 * Check the header user_camera_follower.h for more detailed documentation.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_camera_plugins/user_camera_follower/user_camera_follower.h"

#include <gazebo/msgs/vector3d.pb.h>
#include <gazebo/msgs/quaternion.pb.h>

#include <ros/ros.h>

using provant::camera_plugins::UserCameraFollower;

GZ_REGISTER_MODEL_PLUGIN(provant::camera_plugins::UserCameraFollower);

UserCameraFollower::UserCameraFollower() : ModelPlugin()
{
}

void UserCameraFollower::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  // Set the log message
  _logMsg = "[UserCameraFollower: " + GetHandle() + "] ";
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  // Store a pointer to the model
  this->_model = _model;

  // Prepare the topic
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Configuring a gazebo transport node for the plugin.");
  _gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node());
  if (!_gzNode)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while trying to create a gazebo node for the plugin.");
    return;
  }
  _gzNode->Init();

  // Connect to the user_camera/pose topic
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Connecting to the ~/user_camera/pose gazebo topic.");
  _userCameraPoseSubscriber =
      _gzNode->Subscribe("~/user_camera/pose", &provant::camera_plugins::UserCameraFollower::OnPoseUpdate, this, false);

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully");
}

void UserCameraFollower::OnPoseUpdate(const ConstPosePtr& msg)
{
  const auto pos = msg->position();
  const auto orientation = msg->orientation();
  const ignition::math::Pose3d pose{ pos.x(), pos.y(), pos.z(), orientation.w(), orientation.x(),
                                               orientation.y(), orientation.z() };
  const auto currentPose = _model->WorldPose();

  const auto linearDistance = currentPose.Pos().Distance(pose.Pos());
  const auto attDiff = currentPose.Rot() - pose.Rot();
  const auto quaterionNorm = std::sqrt(attDiff.X() * attDiff.X() + attDiff.Y() * attDiff.Y() +
                                       attDiff.Z() * attDiff.Z() + attDiff.W() * attDiff.W());

  if(linearDistance > threshold || quaterionNorm > threshold)
  {
    _model->SetWorldPose(pose);
  }
}
