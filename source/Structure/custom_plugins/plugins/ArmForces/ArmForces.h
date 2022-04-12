/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file ArmForces.h
 * @brief This file contains the implementation of the armforces class.
 *
 * @author Jonatan Mota Campos
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_ARMFORCES_H
#define PROVANT_ARMFORCES_H

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <provant_simulator_sdf_parser/sdf_parser.h>

#include <atomic>
#include <optional>

namespace provant
{
namespace plugins
{
/**
 * @brief The ArmForces is a prototype plugin that applies torques at to a planar manipulator with two links.
 *
 * The plugin expects an SDF configuration with the following parameters:
 * - joint1: String. Name of the first joint.
 * - joint2: String. Name of the second joint.
 * - joint1_torque_topic: String. Name of the ROS topic to receive the torques to apply at the first joint.
 * - joint2_torque_topic: String. Name of the ROS topic to receive the torques to apply at the second joint.
 *
 * This plugin receives the torques to apply, in Nm from two ROS topics that are user configurable, and applies them
 * to the joints using the SetForce method.
 *
 * This plugin was created as a basic test to the model before designing the Gazebo Abstraction Layer that will be used
 * for future sensors and actuator plugins of the ProVANT Simulator.
 */
class ArmForces : public gazebo::ModelPlugin
{
public:
  ArmForces() = default;
  /**
   * @brief Method called when the plugin is loaded.
   *
   * This method will parse the configuration from the plugin SDF, connect to the world update end event and
   * to the user defined ROS topics.
   *
   * @param _model Pointer to the model containing this plugin.
   * @param _sdf Pointer to the SDF elements containing the plugin configuration.
   */
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Update the torques to be applied at the first manipulator joint.
   *
   * @param msg ROS message containing the torque in Nm to apply at the first manipulator joint.
   */
  void CallbackT1(const std_msgs::Float64& msg);
  /**
   * @brief Update the torques to be applied at the second manipulator joint.
   *
   * @param msg ROS message containing the torque in Nm to apply at the second manipulator joint.
   */
  void CallbackT2(const std_msgs::Float64& msg);

  /**
   * @brief Parse a joint from the plugin SDF configuration.
   *
   * If the element with the specified name exists, contains a valid name, and a joint with the specified name exists
   * in the provided model, returns a pointer to the joint. Otherwise returns an empty object.
   *
   * @param parser Parser from which the joint name from.
   * @param elementName Name of the SDF element containing the name of the joint.
   * @param model Pointer to the model which contain the joints.
   * @param elementDescription A description of what should be the content of the element.1
   * @return std::optional<gazebo::physics::JointPtr> If the element contains a valid joint, returns a joint pointer,
   * otherwise returns an empty object.
   */
  std::optional<gazebo::physics::JointPtr> parseJoint(const SDFParser& parser, const std::string& elementName,
                                                      gazebo::physics::ModelPtr model,
                                                      const std::string& elementDescription) const;
  /**
   * @brief Parse the name of a ROS topic from the plugin SDF configuration.
   *
   * If the provided element contains a valid string (non empty) returns the content of the string. Otherwise an empty
   * object is returned.
   *
   * @param parser Parser from which to read the topic from.
   * @param elementName Name of the SDF element containing the topic name.
   * @param elementDescription Description of what should be the content of the element.
   * @return std::optional<std::string> If the element is found and is valid, return a string containing the topic name.
   * If an error occurs the error is logged and an empty object is returned.
   */
  std::optional<std::string> parseTopicName(const SDFParser& parser, const std::string& elementName,
                                            const std::string& elementDescription) const;
  /**
   * @brief Log an error message about a SDF element not being found.
   *
   * @param status SDFStatus containing the error message.
   * @param elementName Name of the element in which the error ocurred.
   * @param elementDescription Description of what should be the content of this element.
   */
  void logElementNotFoundError(const SDFStatus& status, const std::string& elementName,
                               const std::string& elementDescription) const;
  /**
   * @brief Method called at every simulation step.
   *
   * This method will read and apply the last update of the joint torques to the first and second manipulator joints.
   */
  void OnUpdate();

private:
  /// @brief ROS node handle to subscribe to the topics.
  ros::NodeHandle node_handle_;

  /// @brief Pointer to the first manipulator joint (joint between the base link and the first link).
  gazebo::physics::JointPtr _joint1;
  /// @brief Pointer to the second manipulator joint (joint between links one and two).
  gazebo::physics::JointPtr _joint2;

  /// @brief Torque applied to the first joint.
  std::atomic<double> _joint1Torque{ 0.0 };
  /// @brief Torque applied to the second joint.
  std::atomic<double> _joint2Torque{ 0.0 };

  /// @brief Connection to the Gazebo World Update End event.
  gazebo::event::ConnectionPtr _worldUpdateEndConn;

  // ROS subscribers
  /// @brief ROS topic subscriber to receive updates for the torque of the first joint.
  ros::Subscriber motor_subscriber1_;
  /// @brief ROS topic subscriber to receive updates for the torque of the second joint.
  ros::Subscriber motor_subscriber2_;

  /// @brief Name of the ROS child logger used to emit the messages of this plugin.
  const std::string PLUGIN_ID = "arm_forces";
  /// @brief Message used to uniquely identify the messages from this plugin instance in the log output.
  std::string _logMsg;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_ARMFORCES_H
