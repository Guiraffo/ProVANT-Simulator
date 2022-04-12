/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file step_service.cpp
 * @brief This file contains the implementation of the StepServicePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/step_service/step_service.h"

#include <gazebo/common/Events.hh>

#include <chrono>

using namespace gazebo;
using namespace std::chrono_literals;

using provant::plugins::StepServicePlugin;

GZ_REGISTER_WORLD_PLUGIN(StepServicePlugin)

void StepServicePlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_world != nullptr, "Received a null world.");
  (void)_sdf;  // Silence warning against unused variable

  // Initialize the logging message
  _logMsg = "[StepServicePlugin: " + GetHandle() + "] ";

  // Check if ROS is initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. Load the Gazebo system plugin "
                                                 "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and try "
                                                 "again.\n"
                                              << "This plugin should be loaded automatically if Gazebo is started from "
                                                 "roslaunch. The plugin cannot proceed with initialization and will "
                                                 "not be loaded.");
  }

  // Inform system that the plugin started loading
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  // Save pointer to the world
  this->_world = _world;

  // Advertise the service
  const std::string serviceName("provant_simulator/step");
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the " << serviceName << " ROS service.");
  _server = _rosNodeHandle.advertiseService(serviceName, &StepServicePlugin::StepRequestHandler, this);

  // Finish the plugin loading process
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully.");
}

const std::string& StepServicePlugin::getLogMessage() const
{
  return _logMsg;
}

bool StepServicePlugin::StepRequestHandler(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res)
{
  (void)req;  // Silence unused variable warning.

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Received a call to the /provant_simulator/step service");

  // Request a world step
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Requesting one step.");
  const auto current_iter = _world->Iterations();
  const auto step_start = std::chrono::high_resolution_clock::now();
  _world->Step(1);

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Waiting for the step to complete.");
  const auto start = std::chrono::high_resolution_clock::now();
  while (_world->Iterations() < current_iter + 1)
  {
    const auto now = std::chrono::high_resolution_clock::now();
    const auto secs = std::chrono::duration_cast<std::chrono::seconds>(now - start);
    if (secs.count() > 120)
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "A timeout ocurred while waiting for a world step.");

      res.success = false;
      res.message = "A timeout ocurred after waiting for 120 seconds for a world update.";

      return false;
    }
  }

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Step advanced successfully. Waited for "
                                            << std::chrono::duration_cast<std::chrono::microseconds>(
                                                   std::chrono::high_resolution_clock::now() - start)
                                                   .count()
                                            << " microseconds. The full step update took "
                                            << std::chrono::duration_cast<std::chrono::microseconds>(
                                                   std::chrono::high_resolution_clock::now() - step_start)
                                                   .count()
                                            << " microseconds.");
  res.success = true;
  return true;
}
