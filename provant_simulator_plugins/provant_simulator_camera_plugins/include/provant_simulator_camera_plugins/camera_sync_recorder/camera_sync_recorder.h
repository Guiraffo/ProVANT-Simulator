/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file camera_sync_recorder.h
 * @brief This file contains the declaration of the SyncCameraRecorderPlugin.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CAMERA_SYNC_RECORDER_H
#define PROVANT_CAMERA_SYNC_RECORDER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>

#include <spdlog/spdlog.h>

#include <memory>
#include <optional>

#include <provant_simulator_camera_plugins/camera_recorder/camera_recorder.h>

namespace gazebo
{
/**
 * @brief The Syn Camera Recorder Plugin is a Gazebo sensor plugin that is able
 * to record a video from a camera sensor synchronizing the frames with the
 * simulation steps.
 *
 * @todo Add method to close the recording on the CameraRecorder class.
 * @todo Connect to the Kill signal and save the recording before Gazebo crashes.
 */
class SyncCameraRecorderPlugin : public SensorPlugin
{
public:
  SyncCameraRecorderPlugin() = default;
  virtual ~SyncCameraRecorderPlugin() = default;
  /**
   * @brief Loads the plugin.
   *
   * @param _sensor Pointer to the sensor managed by this plugin.
   * @param _sdf Pointer to the SDF element that contains this plugin configuration.
   */
  void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Method called at every simulation step. Used to request that the recorder grabs a new camera frame.
   */
  void OnWorldUpdate();
  /**
   * @brief Parse the options from the SDF element and if successfull returns a RecorderOptions object.
   *
   * @param _sdf Pointer to the SDF element that contains the plugin configuration.
   * @return std::optional<provant::camera_plugins::RecorderOptions> RecorderOptions if parsing is successfull and an
   * empty object otherwise.
   */
  std::optional<provant::camera_plugins::RecorderOptions> ParseOptions(sdf::ElementPtr _sdf) const;

private:
  /// Pointer to the world update end connection.
  event::ConnectionPtr _onWorldUpdateEndConn;
  /// Pointer to the camera sensor.
  sensors::CameraSensorPtr _cameraSensor;
  /// Pointer to the camera recorder used to create the video recording of the camera sensor.
  std::unique_ptr<provant::camera_plugins::CameraRecorder> _recorder;
  /// Logger used to output messages on the console.
  std::shared_ptr<spdlog::logger> _logger;
};
}  // namespace gazebo

#endif  // PROVANT_CAMERA_SYNC_RECORDER_H
