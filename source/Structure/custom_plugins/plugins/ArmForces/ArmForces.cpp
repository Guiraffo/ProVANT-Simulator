/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file ArmForces.cpp
 * @brief This file contains the implementation of the ArmForces class.
 *
 * @author Jonatan Mota Campos
 * @author Júnio Eduardo de Morais Aquino
 */

#include "ArmForces.h"

#include <gazebo/common/Events.hh>

using provant::plugins::ArmForces;

/**********************************************************************************************************************/
void ArmForces::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  /* Prepare the log message identify the messages from this plugin in the log output.
   * The first part "[ArmForces: " identifies the plugin type, and the second type
   * contain a unique name for the plugin, allowing the identification of which instance
   * generated a given message. The GetHandle() method returns the plugin name that
   * is passed in tha name attribute of the plugin SDF.
   */
  _logMsg = "[ArmForces: " + GetHandle() + "] ";

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS Node for Gazebo has not been initialized. The "
                                                 "plugin cannot be loaded. Load the Gazebo system plugin "
                                                 "\'libgazebo_ros_api_plugin.so\' of the gazebo_ros "
                                                 "package. This is "
                                                 "done automatically if you load gazebo using the "
                                                 "\"roslaunch "
                                                 "gazebo_ros empty_world.launch\" command.");
    return;
  }

  std::string joint1Topic, joint2Topic;

  SDFParser parser{ _sdf };

  auto joint1Res =
      parseJoint(parser, "joint1", _model, "This element must contain the name of the first joint of the manipulator.");
  if (!joint1Res.has_value())
    return;
  _joint1 = joint1Res.value_or(nullptr);

  auto joint2Res = parseJoint(parser, "joint2", _model,
                              "This element must contain the name fo the second joint of the manipulator.");
  if (!joint2Res.has_value())
    return;
  _joint2 = joint2Res.value_or(nullptr);

  const auto joint1TopicName = parseTopicName(parser, "joint1_torque_topic",
                                              "This element must conain the name of the ROS topic used to receive "
                                              "updates with the torque in Nm applied to the first manipulator "
                                              "joint.");
  if (!joint1TopicName.has_value())
    return;

  const auto joint2TopicName = parseTopicName(parser, "joint2_torque_topic",
                                              "This element must conain the name of the ROS topic used to receive "
                                              "updates with the torque in Nm applied to the second manipulator "
                                              "joint.");
  if (!joint2TopicName.has_value())
    return;

  // Subscribe to the ROS topics used to receive torque value updates
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Subcribding to the " << joint1TopicName.value() << " ROS topic.");
  motor_subscriber1_ = node_handle_.subscribe(joint1TopicName.value(), 1, &ArmForces::CallbackT1, this);

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Subcribding to the " << joint1TopicName.value() << " ROS topic.");
  motor_subscriber2_ = node_handle_.subscribe(joint2TopicName.value(), 1, &ArmForces::CallbackT2, this);

  // Connect to the WorldUpdatEnd topic
  _worldUpdateEndConn = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { this->OnUpdate(); });
  if (_worldUpdateEndConn == nullptr)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to connect to the World Update End "
                                                 "topic.");
    // Unsubscribe from the ROS topics
    node_handle_.shutdown();
    return;
  }

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "The plugin loaded successfully.");
}

std::optional<std::string> ArmForces::parseTopicName(const SDFParser& parser, const std::string& elementName,
                                                     const std::string& elementDescription) const
{
  try
  {
    const auto name = parser.GetElementText(elementName);
    if (name.empty())
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID,
                             _logMsg << "The value of the " << elementName << " is empty. " << elementDescription);
      return {};
    }

    return name;
  }
  catch (const SDFStatus& e)
  {
    logElementNotFoundError(e, elementName, elementDescription);
    return {};
  }
}

std::optional<gazebo::physics::JointPtr> ArmForces::parseJoint(const SDFParser& parser, const std::string& elementName,
                                                               gazebo::physics::ModelPtr model,
                                                               const std::string& elementDescription) const
{
  try
  {
    const auto jointName = parser.GetElementText(elementName);
    auto joint = model->GetJoint(jointName);

    if (joint == nullptr)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A joint named \"" << jointName << "\" was not found in the "
                                                << model->GetName() << ". Please fix the value of the" << elementName
                                                << "element at the plugin "
                                                   "configuration.");
      return {};
    }

    return joint;
  }
  catch (const SDFStatus& e)
  {
    logElementNotFoundError(e, elementName, elementDescription);
    return {};
  }
}

void ArmForces::logElementNotFoundError(const SDFStatus& status, const std::string& elementName,
                                        const std::string& elementDescription) const
{
  ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error with message \"" << status.what()
                                            << "\" ocurred while trying to read the " << elementName
                                            << " element from the plugin SDF configuration." << elementDescription);
}

/**********************************************************************************************************************/
// Callback to update the torque at joint 1
void ArmForces::CallbackT1(const std_msgs::Float64& msg)
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Entered the Callback1 method.");
  _joint1Torque.store(msg.data);
}

/**********************************************************************************************************************/
// Callback to update the torque at joint 2
void ArmForces::CallbackT2(const std_msgs::Float64& msg)
{
  //  ignition::math::Vector3d torque2(0, 0, msg.data);
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Entered the Callback2 method.");
  _joint2Torque.store(msg.data);
}

void ArmForces::OnUpdate()
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Entered the OnUpdate method.");

  GZ_ASSERT(_joint1 != nullptr, "The first joint pointer is null.");
  GZ_ASSERT(_joint2 != nullptr, "The second joint pointer is null.");

  _joint1->SetForce(0, _joint1Torque.load());
  _joint2->SetForce(0, _joint2Torque.load());
}

GZ_REGISTER_MODEL_PLUGIN(ArmForces)
