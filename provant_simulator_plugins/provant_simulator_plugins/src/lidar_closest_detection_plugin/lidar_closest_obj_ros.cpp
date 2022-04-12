/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file lidar_closest_obj_ros.cpp
 * @brief This file contains the implementation of the LIDARClosestObjPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/lidar_closest_detection_plugin/lidar_closets_obj_ros.h"

#include <gazebo/sensors/GpuRaySensor.hh>

#include <simulator_msgs/Sensor.h>

#include <provant_simulator_sdf_parser/sdf_parser.h>

#include "provant_simulator_plugins/lidar_closest_detection_plugin/closest_point_finder.h"

using provant::plugins::LIDARClosestObjectPlugin;

GZ_REGISTER_SENSOR_PLUGIN(LIDARClosestObjectPlugin);

void LIDARClosestObjectPlugin::Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  // Format the log message
  _logMsg = "[LIDARClosestObjPlugin: " + GetHandle() + "] ";

  // Log the plugin initialization
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  // Load the base plugin
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Loading base plugin.");
  GpuRayPlugin::Load(_sensor, _sdf);

  // Convert the sensor to a GpuRaySensor
  _raySensor = std::dynamic_pointer_cast<gazebo::sensors::GpuRaySensor>(_sensor);
  if (_raySensor == nullptr)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "The received sensor is not a GPU Ray Sensor. The plugin cannot be "
                                                 "loaded.");
    return;
  }

  // Checks if ROS is initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' "
                                                 "in the gazebo_ros package).");
    return;
  }

  SDFParser parser{ _sdf };
  const auto status = parser.GetElementText("topic", &_topicName);
  if (status.isError())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "The \"topic\" element was not found at the plugin SDF configuration. "
                                                 "This element must contain the name of the ROS topic used to publish "
                                                 "the results.");
    return;
  }

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Defering the plugin loading.");
  _initializationThread = std::thread([this]() { this->SetupNodes(); });
}

void LIDARClosestObjectPlugin::SetupNodes()
{
  _gzNode = boost::make_shared<gazebo::transport::Node>();
  _gzNode->Init(_raySensor->WorldName());

#if GAZEBO_MAJOR_VERSION >= 11
  if (!_gzNode->IsInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to initialize the gazebo transport "
                                                 "node.");
    return;
  }
#endif

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the " << _topicName << " ROS topic.");
  _publisher = _rosNodeHandle.advertise<simulator_msgs::Sensor>(_topicName, 1, false);

  _finder = std::make_unique<provant::plugins::ClosestPointFinder>(_raySensor, _logMsg, PLUGIN_ID);

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID,
                         _logMsg << "Subsribing to the sensor data (" << _raySensor->Topic() << ") gazebo topic.");
  _raySensorSub = _gzNode->Subscribe(_raySensor->Topic(), &LIDARClosestObjectPlugin::OnNewScan, this);
  if (!_raySensorSub)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to connect to the sensor data ("
                                              << _raySensor->Topic() << ") gazebo topic.");
    _rosNodeHandle.shutdown();
    return;
  }
}

void LIDARClosestObjectPlugin::OnNewScan(ConstLaserScanStampedPtr& msg)
{
  if (!_finder)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Trying to process new scan with a null ClosestPointFinder object.");
    return;
  }

  _publisher.publish(_finder->OnNewScan(msg));
}
