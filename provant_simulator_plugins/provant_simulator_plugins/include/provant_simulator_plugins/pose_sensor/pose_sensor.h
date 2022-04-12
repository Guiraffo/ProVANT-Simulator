/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file pose_sensor.h
 * @brief This file contains the declaration of the PoseSensorPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_POSE_SENSOR_H
#define PROVANT_POSE_SENSOR_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>

#include <string>

namespace provant
{
namespace plugins
{
class PoseSensorPlugin : public gazebo::ModelPlugin
{
public:
  PoseSensorPlugin() = default;
  virtual ~PoseSensorPlugin() = default;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;
  void Init() override;

protected:
  ros::Publisher _publisher;

  void OnWorldUpdate();

  void SetupLogging();
  bool ParseSDF();
  bool SetupLink();
  bool SetupROSTopic();
  bool ConnectToSignals();

private:
  sdf::ElementPtr _sdf;
  gazebo::physics::ModelPtr _model;
  gazebo::physics::LinkPtr _link;

  gazebo::event::ConnectionPtr _worldUpdateEndConn;

  std::string _modelName;
  std::string _linkName;
  std::string _topicName;

  ros::NodeHandle _rosNodeHandle;

  std::string _logMsg;
  const std::string PLUGIN_ID = "pose_sensor";
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_POSE_SENSOR_H
