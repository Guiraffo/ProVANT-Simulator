/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file servo2_motor_plug.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * implements a servo motor.
 *
 * @author Arthur Viana Lima
 * @author Júnio Eduardo de Morais Aquino
 */

#include "servo2_motor_plug.h"

#include <simulator_msgs/Sensor.h>

#include "XMLRead.h"

namespace gazebo
{
void Servo2MotorPlugin::CallbackReferencias(std_msgs::Float64 msg)
{
  boost::mutex::scoped_lock scoped_lock(lock);

  if (Modo_ == "Torque")
  {
    junta->SetForce(0, msg.data);
  }
  else
  {
    this->model->GetJointController()->SetPositionTarget(this->junta->GetScopedName(), msg.data);
  }
}

Servo2MotorPlugin::Servo2MotorPlugin()
{
  torque = 0;
}

void Servo2MotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  if (!ros::isInitialized())
  {
    return;
  }

  NameOfJoint_ = XMLRead::ReadXMLString("NameOfJoint", _sdf);
  TopicSubscriber_ = XMLRead::ReadXMLString("TopicSubscriber", _sdf);
  TopicPublisher_ = XMLRead::ReadXMLString("TopicPublisher", _sdf);
  Modo_ = XMLRead::ReadXMLString("Modo", _sdf);

  world = _model->GetWorld();
  junta = _model->GetJoint(NameOfJoint_);

  // -------------------------------------------------------------------------

  this->model = _model;

  if (Modo_ == "Posicao")
  {
    // Setup de um controlador proporcional (P) com um ganho de 0.1:
    //  				this->pid = common::PID(0.010, 0.050, 0.0001);

    if (NameOfJoint_ == "elev")
    {
      this->pid = common::PID(0.010, 0.050, 0.0001);
    }
    else
    {
      this->pid = common::PID(0.001, 0.0050, 0.00001);
    }

    // Aplica o controlador na junta:
    this->model->GetJointController()->SetPositionPID(this->junta->GetScopedName(), this->pid);
  }

  // -------------------------------------------------------------------------

  motor_subscriber_ =
      node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::Servo2MotorPlugin::CallbackReferencias, this);
  motor_publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(TopicPublisher_, 5);

  updateConnection = event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo& info) {
    (void)info;  // Suprress unused variable warning
    this->Update();
  });
}

void Servo2MotorPlugin::Update()
{
  simulator_msgs::Sensor newmsg;
  newmsg.name = TopicPublisher_;
  newmsg.header.stamp = ros::Time::now();
  newmsg.header.frame_id = "1";
  newmsg.values.push_back(junta->Position(0));
  newmsg.values.push_back(junta->GetVelocity(0));
  motor_publisher_.publish(newmsg);
}

GZ_REGISTER_MODEL_PLUGIN(Servo2MotorPlugin)
}  // namespace gazebo
