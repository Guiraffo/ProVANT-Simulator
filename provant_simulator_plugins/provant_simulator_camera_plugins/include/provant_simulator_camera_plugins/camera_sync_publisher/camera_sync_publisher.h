/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_sync_publisher.h
 * @brief
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef _PROVANT_SIMUlATOR_CAMERA_SYNC_PUBLISHER_H
#define _PROVANT_SIMUlATOR_CAMERA_SYNC_PUBLISHER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/sensors.hh>

#include <gazebo/plugins/CameraPlugin.hh>
#include <gazebo_plugins/gazebo_ros_camera_utils.h>

#include <boost/interprocess/sync/interprocess_semaphore.hpp>

#include <atomic>

namespace provant
{
namespace plugins
{
/**
 * @brief
 *
 * @todo Add documentation.
 * @todo Add support for step limited recording (starting_from) (ending_in) steps.
 *
 */
class CameraSyncPublisher : public gazebo::CameraPlugin, gazebo::GazeboRosCameraUtils
{
public:
  CameraSyncPublisher();
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

protected:
  virtual void OnNewFrame(const unsigned char* _image, unsigned int _width, unsigned int _height, unsigned int _depth,
                          const std::string& _format);

private:
  gazebo::event::ConnectionPtr _updateConn;
  void OnWorldUpdate();

  /**
   * @brief Semaphore used to indicate the number of available frames slots.
   *
   * When this semaphore has a non zero value it indicates that a new frame must be saved from the camera.
   */
  boost::interprocess::interprocess_semaphore _emptyFrameSlots;
  /**
   * @brief Semaphore used to indicate the number of caputred frames.
   *
   * When this semaphore has a non zero value it indicates that a frame was saved from the camera, and the world
   * update process can continue.
   */
  boost::interprocess::interprocess_semaphore _fullFrameSlots;

  std::atomic<bool> _workPosted = false;

  const std::string PLUGIN_ID = "camera_sync_publisher";
  std::string _logMsg;
};
}  // namespace plugins
}  // namespace provant

#endif  //_PROVANT_SIMUlATOR_CAMERA_SYNC_PUBLISHER_H
