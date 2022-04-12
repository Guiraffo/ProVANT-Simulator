/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file servo2_motor_plug.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * implements a servo motor.
 *
 * @author Arthur Viana Lima
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SERVO2_PLUGIN_H
#define PROVANT_SERVO2_PLUGIN_H

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include "std_msgs/Float64.h"

#include <boost/thread.hpp>

namespace gazebo
{
/**
 * @todo Why do we need 2 servo implementations?
 */
class Servo2MotorPlugin : public ModelPlugin
{
public:
  Servo2MotorPlugin();
  virtual ~Servo2MotorPlugin() = default;
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  virtual void Update();
  void Controlador();
  void CallbackReferencias(std_msgs::Float64);

private:
  int16_t saturate(float, const float);
  float velocity_feedforward(float);
  float velocity_controller(float, float);
  float position_controller(float, float);

  std::string path;
  std::string NameOfJoint_;
  std::string NameOfNode_;
  std::string TopicSubscriber_;
  std::string TopicPublisher_;
  std::string Modo_;
  physics::WorldPtr world;
  physics::JointPtr junta;
  event::ConnectionPtr updateConnection;
  ros::NodeHandle node_handle_;
  ros::Publisher motor_publisher_;
  ros::Subscriber motor_subscriber_;
  double refvalue;
  double ang;
  double vel_ang;
  double Kpx, Kpv;
  double Kix, Kiv;
  double Kdx, Kdv;
  double torque;
  double torquepub;
  boost::mutex lock;

  physics::ModelPtr model;
  common::PID pid;
};
}  // namespace gazebo

#endif  // PROVANT_SERVO2_PLUGIN_H
