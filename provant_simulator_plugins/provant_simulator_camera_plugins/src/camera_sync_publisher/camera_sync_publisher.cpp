/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_sync_publisher.cpp
 * @brief
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_camera_plugins/camera_sync_publisher/camera_sync_publisher.h"

#include <gazebo/common/Events.hh>

using provant::plugins::CameraSyncPublisher;

GZ_REGISTER_SENSOR_PLUGIN(CameraSyncPublisher);

CameraSyncPublisher::CameraSyncPublisher() : _emptyFrameSlots(0), _fullFrameSlots(0)
{
}

void CameraSyncPublisher::Load(gazebo::sensors::SensorPtr sensor, sdf::ElementPtr sdf)
{
  _logMsg = "[CameraSyncPublisher: " + GetHandle() + "] ";
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS node for Gazebo has not been initialized, unable to load "
                                                 "plugin. "
                                              << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the "
                                                 "gazebo_ros package)");
    return;
  }

  CameraPlugin::Load(sensor, sdf);
  // copying from CameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->camera;

  GazeboRosCameraUtils::Load(sensor, sdf);

  // Connect to the WorldUpdate signal
  _updateConn = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { this->OnWorldUpdate(); });

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully.");
}

void CameraSyncPublisher::OnNewFrame(const unsigned char* _image, unsigned int /*_width*/, unsigned int /*_height*/,
                                     unsigned int /*_depth*/, const std::string& /*_format*/)
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Received a camera frame");
  if (!this->parentSensor->IsActive())
  {
    ROS_WARN_STREAM_NAMED(PLUGIN_ID, _logMsg << "An inactive sensor was detected!");
    this->parentSensor->SetActive(true);
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Trying to wait on the empty frames slot semaphore.");
    if (_emptyFrameSlots.try_wait())
    {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Publishing camera frame.");
    auto updateTime = parentSensor->LastMeasurementTime();
    PutCameraData(_image, updateTime);
    PublishCameraInfo(updateTime);

    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Posting on the full frames slot semaphore.");
    _fullFrameSlots.post();
    }
  }
}

void CameraSyncPublisher::OnWorldUpdate()
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Received a World Update End signal");

  if(_workPosted) {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Waiting on the full frames slot semaphore");
    _fullFrameSlots.wait();
  }

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Posting to the empty frames slot semaphore");
  _workPosted = true;
  _emptyFrameSlots.post();
}
