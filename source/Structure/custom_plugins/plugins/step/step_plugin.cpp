/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the StepPlugin class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include "step_plugin.h"

#include <gazebo/physics/World.hh>

namespace gazebo
{
void StepPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->_world = _world;  // Save world pointer.
  this->_sdf = _sdf;      // Save the SDF Element pointer.

  GZ_ASSERT(_world != nullptr, "The simulation world is null");
  GZ_ASSERT(_sdf != nullptr, "Received a null SDF Element pointer");

  // Pauses the simulation to avoid errors in the timing synchronization.
  _world->SetPaused(true);

  // Assemble the _logMsg
  _logMsg = "[StepPlugin";
  if (_sdf->HasAttribute("name"))
  {
    _logMsg += ": ";
    _logMsg += _sdf->GetAttribute("name")->GetAsString();
  }
  _logMsg += "] ";

  gzmsg << _logMsg << "Starting the plugin loading process.\n";
  bool isHil = false;
  bool okElementFound = false;
  if (_sdf->HasElement("ok"))
  {
    okElementFound = true;
    std::string okValue = _sdf->Get<std::string>("ok");
    if (okValue == "hil")
      isHil = true;
    else
      isHil = false;
  }

  // Check if ROS is initialized.
  if (ros::isInitialized())
  {
    if (!okElementFound)
    {
      ROS_WARN_STREAM_NAMED(PLUGIN_ID, _logMsg << "The \"ok\" element is not defined in the plugin SDF. The "
                                                  "simulation "
                                                  "will be executed in normal mode. If this is the indented behavior "
                                                  "please define the \"ok\" tag with a value different from \"hil\" "
                                                  "in the plugin SDF to silence this warning.");
    }

    // If ROS is initialized, and the simulation is not in HIL mode, subscribe to the /Step topic.
    if (!isHil)
    {
      ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Subscribing to the /Step topic.");
      // Subscriber to receive messages of the /Step topic.
      _stepSubscriber = _nodeHandle.subscribe("Step", 1, &gazebo::StepPlugin::chatterCallback, this);
    }
    else
    {
      ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The simulation is running in Hardware In the Loop (HIL) mode. The "
                                                  "step plugin will take no action in this simulation.");
    }

    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The plugin loading process finished successfully.");
  }
  else
  {
    // If ROS is not initialized, and the simulation is not in HIL mode, an error has ocurred, and the user must be
    // notified.
    if (!isHil)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS Node for Gazebo has not been initialized. The StepPlugin "
                                                   "plugin cannot be loaded. Load the Gazebo system plugin "
                                                   "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package. This is "
                                                   "done automatically if you load gazebo using the \"roslaunch "
                                                   "gazebo_ros empty_world.launch\" command.");
      return;
    }

    gzmsg << _logMsg << "The plugin finished loading successfully.\n";
  }
}

void StepPlugin::chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  (void)msg;  // Silence the unused variable warning.
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advancing one step.");
  _world->Step(1);  // commando to new step time
}

GZ_REGISTER_WORLD_PLUGIN(StepPlugin)
}  // namespace gazebo
