/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file servo_motor_plug.h
 * @brief This file contains the declaration of a Gazebo model plugin that
 * represents a Servo motor. It works in torque and position modes and can return
 * the angular position and velocuty of the joint as a Sensor message.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 * @author Jonatan Mota Campos
 */

#ifndef PROVANT_SERVO_MOTOR_PLUG_H
#define PROVANT_SERVO_MOTOR_PLUG_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <limits>
#include <string>

namespace gazebo
{
class ServoMotorPlugin : public ModelPlugin
{
public:
  enum class ServoModes
  {
    Torque,
    Position
  };

  ServoMotorPlugin() = default;
  virtual ~ServoMotorPlugin() = default;

  // initial setup
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  // update fpr each step time
  virtual void Update();

  double Saturate(double value, double lowerLim, double upperLim) const;

  // calback for receiving references
  void CallbackReferencias(const std_msgs::Float64& msg);
  void PrintFileForce(double);
  void PrintFileAngle(double);

private:
  std::string NameOfJoint_;      // name of joint
  std::string TopicSubscriber_;  // name of topic for receiving references
  std::string TopicPublisher_;   // name of topic for sending sensor data
  std::string TopicDrag_;
  //! Indicate the servo working mode
  ServoModes _mode;

  bool _saturationEnabled = false;
  double Force_Saturation_ = std::numeric_limits<double>::infinity();
  double Angle_Saturation_ = std::numeric_limits<double>::infinity();

  // Friction coefficent, used to calculate the friction forces that occur during the joint movement.
  double fric_coeff = 0L;

  std::string tag_;
  physics::WorldPtr world;                // world's pointer
  physics::JointPtr junta;                // joint's pointer
  event::ConnectionPtr updateConnection;  // connection pointer
  ros::NodeHandle node_handle_;           // ROS's node handle
  ros::Publisher motor_publisher_;        // ROS publisher
  ros::Subscriber motor_subscriber_;      // ROS subscriber
  ros::Publisher drag_publisher_;
  std_msgs::Float64 ViscDrag;
  //! Log message used to uniquely identify this plugin in the log output
  std::string _logMsg;
  //! Name of the ROS child logger used to emit the log messages sent by this plugin
  const std::string PLUGIN_ID = "servo";
};
}  // namespace gazebo

#endif  // PROVANT_SERVO_MOTOR_PLUG_H
