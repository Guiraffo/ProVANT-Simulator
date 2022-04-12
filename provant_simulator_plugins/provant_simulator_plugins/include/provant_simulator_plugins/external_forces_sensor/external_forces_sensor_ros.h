/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file external_forces_sensor_ros.h
 * @brief This file contains the declaration of the ExternalForcesSensorPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef EXTERNAL_FORCES_SENSOR_ROS_H
#define EXTERNAL_FORCES_SENSOR_ROS_H

#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "external_forces_sensor.h"

namespace provant
{
namespace plugins
{
class ExternalForcesSensorPlugin : public gazebo::ModelPlugin
{
public:
  ExternalForcesSensorPlugin();
  virtual ~ExternalForcesSensorPlugin();

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  void Init() override;
  void Reset() override;

  virtual void OnUpdate();

  const std::string GetTopicName() const;

protected:
  gazebo::physics::ModelPtr _model;
  //! Store a pointer to the simulation world
  gazebo::physics::WorldPtr _world;
  sdf::ElementPtr _sdf;

  gazebo::physics::LinkPtr _sensedLink;
  std::string _topicName;
  std::string _linkName;

  std::string _pluginName;
  std::string _logMsg;

  std::unique_ptr<ExternalForcesSensor> _linkSensor;
  //! Stores the values of the forces and torques being applied at the monitored link during the last step update
  std::vector<double> _lastForceAndTorque;

  //! Stores a pointer to the gazebo event used to call the OnUpdate() method at every step
  gazebo::event::ConnectionPtr _updateConnection;

  //! Node handle to manage the ROS Node created for this plugin instance.
  ros::NodeHandle _nodeHandle;
  //! Publisher to send the message updates
  ros::Publisher _publisher;

  //! Stores the number of active subscribers to the ROS topic used to publish the force updates.
  int _numSubscribers = 0;
  //! Mutex to protected the updating of the number of subscribers.
  std::mutex _numSubscribersMutex;

  /**
   * @brief Method called when a new subscriber is listening to the topic used to publish the forces updates.
   *
   * This method increments the count of subscribers.
   * @sa _numSubscribers
   * @sa OnSubscriberDisconnect()
   */
  virtual void OnSubscriberConnect();
  /**
   * @brief Method called when a subscriber stops listening to the topic used to publish the force updates.
   *
   * This method decrements the count of subscribers.
   * @sa _numSubscribers
   * @sa OnSubsriberConnect()
   */
  virtual void OnSubscriberDiscconect();

  virtual void SetupLogging();
  virtual bool ReadTags();
  virtual bool GetLink();
  virtual void SetupLinkSensor();
  virtual void RegisterOnUpdateEvent();
  virtual bool SetupROSTopic();
  virtual void SetupPluginIntrospection();
};
}  // namespace plugins
}  // namespace provant

#endif  // EXTERNAL_FORCES_SENSOR_ROS_H
