/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file user_camera_sync_recorder.h
 * @brief This file contains the implementation of the UserCameraSyncRecorder class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_USER_CAMERA_SYNC_RECORDER_H
#define PROVANT_USER_CAMERA_SYNC_RECORDER_H

// If transport is necessary remove this line and the comment from the next ones
#ifndef Q_MOC_RUN
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/msgs/request.pb.h>
#include <gazebo/msgs/response.pb.h>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

#include "provant_simulator_camera_plugins/camera_recorder/camera_recorder.h"
#endif

#include <memory>
#include <mutex>
#include <thread>

namespace provant::camera_plugins
{
/**
 * @brief The User Camera Sync Recorder class is a Gazebo GUI plugin that allows the user to capture the frames
 * from the Gazebo user camera in sync with the simulation steps.
 *
 * The resulting video is smoother and always has consistent timing as it does not depend on the time Gazebo takes
 * to completely update the simulation with the physics and rendering engines.
 *
 */
class GAZEBO_VISIBLE UserCameraSyncRecorder : public gazebo::GUIPlugin
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new User Camera Sync Recorder object and setups the user interface.
   */
  UserCameraSyncRecorder();
  virtual ~UserCameraSyncRecorder() = default;
  /**
   * @brief Called once during the plugin initialization.
   *
   * This method must check the plugin SDF for its configuration, read and validate its values and perform the
   * registration of the plugin with the sync camera manager.
   *
   * In order to prevent blocking, this method will read and validate the options from the SDF and defer the
   * transport node configuration and registration process to another thread.
   *
   * @param _sdf
   */
  void Load(sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Sets the GUI overlay size and position, and hides the object.
   */
  void SetupUi();

  /**
   * @brief Configure the Gazebo transport node and subscribe to the necessary topics used to communicate with
   * the sync camera manager plugin.
   *
   * @return true If the configuration is successfull.
   * @return false Otherwise.
   */
  virtual bool SetupTransport();
  /**
   * @brief Handler method for new request messages arriving to this plugin.
   *
   * This method will check if the request is for some of the services provided by this plugin, and if so forward the
   * incoming request to the appropriate method.
   *
   * @sa OnGrabFrameRequest(const ConstRequestPtr&)
   * @sa OnEnableOnRequest(const ConstRequestPtr&)
   *
   * @param req
   */
  virtual void OnRequest(const ConstRequestPtr& req);
  /**
   * @brief Handler method for the grab frame service.
   *
   * This service indicates that a new frame should be captured by the recorder.
   *
   * @param req Request received.
   */
  virtual void OnGrabFrameRequest(const ConstRequestPtr& req);
  /**
   * @brief Handler method for the enable_request service.
   *
   * This service is used during the plugin loading process to inform the sync camera manager of the step in which
   * recording should begin.
   *
   * @param req Request received.
   */
  virtual void OnEnableOnRequest(const ConstRequestPtr& req);

  /**
   * @brief Helper method to publish the response messages to the services provided by this plugin.
   *
   * This method will ensure that the appropriate fields are set on the response message in order to ensure that
   * the calling plugin will receive the answer, this fields are id and request.
   *
   * @param req Request message originating the response.
   * @param res Response to send.
   */
  virtual void SendResponse(const ConstRequestPtr& req, gazebo::msgs::Response& res);

  /**
   * @brief Handler method for the PostRender signal.
   *
   * Used to initialize and configure the camera and pass the object to the recorder used to capture the frames.
   */
  virtual void OnPostRender();
  /**
   * @brief Helper method that configures the camera to save its frames and remaining initialization process.
   */
  virtual void SetupCamera();
  /**
   * @brief Method used to register the recorder object with the sync camera manager plugin.
   */
  virtual void RegisterRecorder();

private:
  /// Message used to uniquely identify the log messages emitted by this object in the log output.
  std::string _logMsg;
  /// Name of the ROS child logger used by this plugin.
  const std::string PLUGIN_ID = "user_camera_sync_rec";

  /// Step in which the camera should be enabled.
  int32_t _enableOn = 0;

  /// Mutex used to synchronize access to the camera and recorder objects.
  std::mutex _recorderMutex;
  /// Pointer to the SDF element containing the plugin configuration.
  sdf::ElementPtr _sdfElement;
  /// Recorder object used to capture the frames from the user camera
  std::unique_ptr<CameraRecorder> _recorder;

  /// Connection to on post render event, used to configure the camera.
  gazebo::event::ConnectionPtr _onPostRenderConn;

  /// Thread used to finish the registration of this plugin with the Sync Camera Manager.
  // Note: This is necessary because the load method cannot block, so we defer the loading process to another thread and
  // allow the load method to return.
  std::thread _registrationThread;

  /// Gazebo node to advertise and subscribe to the request and response topics.
  gazebo::transport::NodePtr _gzNode;
  /// Publisher to the ~/response topic. Used to respond to request messages.
  gazebo::transport::PublisherPtr _responsePublisher;
  /// Subscriber connection to the ~/request topic. Used to receive request messages.
  gazebo::transport::SubscriberPtr _requestSubscriber;
};
}  // namespace provant::camera_plugins

#endif  // PROVANT_USER_CAMERA_SYNC_RECORDER_H
