/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file pose_sensor.cpp
 * @brief This file contains the implementation of the PoseSensorPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/pose_sensor/pose_sensor.h"

#include <gazebo/physics/physics.hh>

#include <simulator_msgs/Sensor.h>

using provant::plugins::PoseSensorPlugin;

void PoseSensorPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  GZ_ASSERT(model, "Receveid a null model pointer");
  GZ_ASSERT(sdf, "Received a null SDF element pointer.");

  // Store the sdf and model pointer
  _sdf = sdf;
  _model = model;

  // Get the model name
  _modelName = _model->GetName();

  // Setup Logging
  SetupLogging();

  // Check if ROS is initialized.
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                                                 "in the gazebo_ros package).");
    return;
  }

  // Parse the SDF for the plugin configuration.
  if (!ParseSDF())
    return;

  // Get the pointer to the link
  if (!SetupLink())
    return;

  // Start the ROS Topic
  if (!SetupROSTopic())
    return;

  // Connect to the Gazebo signals
  if (!ConnectToSignals())
  {
    return;
  }
}

void PoseSensorPlugin::Init()
{
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin Initialized.");
}

void PoseSensorPlugin::SetupLogging()
{
  const std::string className = "PoseSensorPlugin";
  const std::string sensorName = GetHandle();

  // Check that the plugin has a name (this is required by gazebo, so a plugin with a null name should not exist)
  GZ_ASSERT(!sensorName.empty(), "The plugin has a null name.");

  _logMsg = "[" + className + ": " + sensorName + "] ";
}

bool PoseSensorPlugin::ParseSDF()
{
  // Get the link name
  if (_sdf->HasElement("link"))
  {
    _linkName = _sdf->GetElement("link")->GetValue()->GetAsString();
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while parsing the plugin SDF configuration. The "
                                                 "link name is not defined. This element must contain the name of a "
                                                 "valid model link that the plugin will read and publish the pose.");
    return false;
  }

  // Get the topic name
  if (_sdf->HasElement("topic"))
  {
    _topicName = _sdf->GetElement("topic")->GetValue()->GetAsString();
  }
  else
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while parsing the plugin SDF configuration. The "
                                                 "link name is not defined. This element must contain the name of the "
                                                 "ROS topic used by the plugin to publish the link pose.");
    return false;
  }

  return true;
}

bool PoseSensorPlugin::SetupLink()
{
  // Get the pointer to the link under the model
  _link = _model->GetLink(_linkName);

  if (!_link)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while trying to get a pointer the required link. A link with "
                                                 "the name \""
                                              << _linkName << "\" cannot be found under the " << _modelName
                                              << " model.");
    return false;
  }

  return true;
}

bool PoseSensorPlugin::SetupROSTopic()
{
  const std::string fullTopicName = "provant_simulator/" + _modelName + "/" + _topicName;

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the " << fullTopicName << " ROS topic.");
  try
  {
    _publisher = _rosNodeHandle.advertise<simulator_msgs::Sensor>(fullTopicName, 1, false);
    return true;
  }
  catch (const ros::InvalidNameException& e)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while advertising the ROS topic. The provided topic name \""
                                              << _topicName << "\" results in an invalid ROS topic name ("
                                              << fullTopicName << ").");
    return false;
  }
}

bool PoseSensorPlugin::ConnectToSignals()
{
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Connecting to the WorldUpdateEnd signal.");

  _worldUpdateEndConn = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { this->OnWorldUpdate(); });

  if (!_worldUpdateEndConn)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while connecting the WorldUpdateEnd gazebo signal");
    return false;
  }

  return true;
}

void PoseSensorPlugin::OnWorldUpdate()
{
  const auto pose = _link->WorldPose();

  // Positions
  const auto pos = pose.Pos();
  const auto x = pos.X();
  const auto y = pos.Y();
  const auto z = pos.Z();

  // Euler angles
  const auto rot = pose.Rot().Euler();
  const auto roll = rot.X();
  const auto pitch = rot.Y();
  const auto yaw = rot.Z();

  // Create the message
  auto msg = simulator_msgs::Sensor();

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = _linkName;
  msg.name = GetHandle();

  msg.values = { x, y, z, roll, pitch, yaw };

  // Publish the message
  _publisher.publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(provant::plugins::PoseSensorPlugin)
