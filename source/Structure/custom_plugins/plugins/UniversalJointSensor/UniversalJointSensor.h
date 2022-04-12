/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file UniversalJointSensor.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * reads all of the data for a specific joint.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_UNIVERSAL_JOINT_SENSOR_H
#define PROVANT_UNIVERSAL_JOINT_SENSOR_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <boost/thread.hpp>

namespace gazebo
{
class UniversalJointSensor : public ModelPlugin
{
  // constructor
public:
  UniversalJointSensor() = default;
  virtual ~UniversalJointSensor() = default;
  // initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  virtual void Update();

private:
  std::string axis;          // kind of axis
  std::string NameOfJoint_;  // name of joint
  std::string NameOfNode_;   // name of node
  physics::WorldPtr world;   // pointer to the world
  physics::JointPtr junta;   // pointer the joint
  event::ConnectionPtr updateConnection;
  ros::NodeHandle node_handle_;  // ROS's node handle
  boost::mutex lock;             // mutex
  ros::Publisher publisher_;     // publisher
};
}  // namespace gazebo

#endif  // PROVANT_UNIVERSAL_JOINT_SENSOR_H
