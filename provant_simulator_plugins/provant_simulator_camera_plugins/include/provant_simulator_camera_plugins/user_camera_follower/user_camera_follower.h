/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file user_camera_follower.h
 * @brief This file contains the implementation of the UserCameraFollower class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_USER_CAMERA_FOLLOWER_H
#define PROVANT_USER_CAMERA_FOLLOWER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <gazebo/msgs/pose.pb.h>

namespace provant::camera_plugins
{
/**
 * @brief The User Camera Follower is a model plugin that manages the pose of a given model so that it tracks the user
 * camera pose.
 *
 * This plugin was designed for use during the creation of presentation videos of the experiments created in Gazebo to
 * give more fine grained control of the video settings. For example, the user camera resolution is defined by the size
 * of the Gazebo client window, and is thus limited by the resolution of the monitor containing the window, by using
 * this plugin, it is possible to record a video in any desired resolution and use the Gazebo client to define the
 * camera pose.
 *
 * However, recording videos with this plugin has some limitations compared to a direct record of the user camera:
 *
 * - Some elements shown in the user camera such as the axis, or rays shown by a few sensors will not be rendered, as
 * this elements only exist for visual guidance of the user and are not represtend in the simulation server.
 *
 */
class UserCameraFollower : public gazebo::ModelPlugin
{
public:
  UserCameraFollower();
  virtual ~UserCameraFollower() = default;
  /**
   * @brief Load the plugin configuration.
   *
   * This method is called once during the plugin initialization process. It then creates and initializes a Gazebo
   * transport node on the default namespace, and subscribe to the ~/user_camera/pose gazebo topic.
   *
   * Note that the ~/ will be replaced by the name of the first world found in the Gazebo server.
   *
   * @param _model Pointer to the model that contains this plugin. This model pose is managed by this plugin.
   * @param _sdf Pointer to the SDF element containing this plugin configuration. Unused.
   */
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Handler for new messages on the ~/user_camera/pose gazebo topic.
   *
   * Receives messages in the format of a pose protobuf message, and updates the
   * pose of the model managed by the plugin to the one of the last received
   * message.
   *
   * @param msg Message containing the pose of the user camera with reference to
   * the inertial frame.
   */
  virtual void OnPoseUpdate(const ConstPosePtr& msg);

private:
  //! Pointer to the model managed by this plugin. This model pose will be updated according to the pose of the user
  //! camera.
  gazebo::physics::ModelPtr _model;

  //! Node to connect to the Gazebo transport layer.
  gazebo::transport::NodePtr _gzNode;
  //! Store the connection to the ~/user_camera/pose gazebo topic.
  gazebo::transport::SubscriberPtr _userCameraPoseSubscriber;

  //! Unique identifier of this plugin in the log messages.
  std::string _logMsg;
  //! Name of the ROS child logger used for this plugin
  const std::string PLUGIN_ID = "user_camera_follower";

  //! If the user camera moves in a value that is lower than this threshold, this movement will be ignored.
  double threshold = 1e-2;
};
}  // namespace provant::camera_plugins

#endif  // PROVANT_USER_CAMERA_FOLLOWER_H
