/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file servo_motor_plug.cpp
 * @brief This file contains the implementation of the ServoMotorPlugin class.
 * See the servo_motor_plug.h file for more detailed documentation.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 * @author Jonatan Mota Campos
 */

#include "servo_motor_plug.h"

#include <boost/algorithm/string.hpp>

#include <XMLRead.h>

#include <provant_simulator_sdf_parser/sdf_parser.h>
#include <simulator_msgs/Sensor.h>


namespace gazebo
{
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Callback for references
void ServoMotorPlugin::CallbackReferencias(const std_msgs::Float64& msg)
{
  switch (_mode)
  {
    case ServoModes::Torque: {
      const auto force = Saturate(msg.data, -Force_Saturation_, Force_Saturation_);
      junta->SetForce(0, force);
      break;
    }
    case ServoModes::Position: {
      const auto pos = Saturate(msg.data, -Angle_Saturation_, Angle_Saturation_);
      ///@todo What exactly do we need to fix here?
      junta->SetPosition(0, msg.data);  // TO FIX
      break;
    }
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////

// initial setup
void ServoMotorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  _logMsg = "[ServoMotorPlugin: " + GetHandle() + "] ";

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Ros is not initialized, the plugin will not be loaded.");
    return;
  }

  // Get a pointer to the simulation world
  world = _model->GetWorld();
  GZ_ASSERT(world, "The simulation world is null.");

  SDFParser parser{ _sdf };

  auto errorMessage = [this](const SDFStatus& e, const std::string& elementName,
                             const std::string& additionalMsg = std::string{ "" }) {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error with mesage \"" << e.what()
                                              << "\" was found while reading the " << elementName << " element. "
                                              << additionalMsg);
  };

  // Get the name of the joint and check if a joint with the specified name exists
  try
  {
    NameOfJoint_ = parser.GetElementText("NameOfJoint");
    junta = _model->GetJoint(NameOfJoint_);
    if (!junta)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A joint named \"" << NameOfJoint_ << "\" was not found in the "
                                                << _model->GetName() << " model.");
      return;
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "NameOfJoint",
                 "This element "
                 "must contain the name of the model joint controlled by this servo.");
    return;
  }

  // Get the name of the subscriber topic
  try
  {
    TopicSubscriber_ = parser.GetElementText("TopicSubscriber");
    if (TopicSubscriber_.length() == 0)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "The \"TopicSubscriber\" element is empty. This element must "
                                                   "contain the name of the ROS topic used to receive the position or "
                                                   "torque values the servo must apply.");
      return;
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "TopicSubscriber",
                 "This element must contain the name of the ROS topic used to receive the position or torque values "
                 "the servo must apply.");
    return;
  }

  // Get the name of the publisher topic
  try
  {
    TopicPublisher_ = parser.GetElementText("TopicPublisher");
    if (TopicPublisher_.length() == 0)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "The element \"TopicPublisher\" is empty. This element must contain "
                                                   "the name of the ROS topic used to publish the position of "
                                                   "velocities of the joint controlled by this servo.");
      return;
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "TopicPublisher",
                 "This element must contain the name of the ROS topic used to publish the position and angular "
                 "velocity of the joint controlled by this servo.");
    return;
  }

// Get the name of the publisher topic for the drag
  try
  {
    TopicDrag_ = parser.GetElementText("TopicDrag");
    if (TopicDrag_.length() == 0)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "The element \"TopicDrag_\" is empty. This element must contain "
                                                   "the name of the ROS topic used to publish the viscous drag of "
                                                   "this servo.");
      return;
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "TopicDrag_",
                 "This element must contain the name of the ROS topic used to publish the viscous drag "
                 "of the joint controlled by this servo.");
    return;
  }



  // Check the servo working mode
  try
  {
    // Convert the mode to an all lowercase string in order to simplify comparissons
    using boost::algorithm::to_lower_copy;
    const auto modeStr = to_lower_copy(XMLRead::ReadXMLString("Modo", _sdf));
    if (modeStr == "torque")
    {
      _mode = ServoModes::Torque;
    }
    else if (modeStr == "position")
    {
      _mode = ServoModes::Position;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An unrecognized servo was found while reading the Modo parameter. "
                                                   "Valid modes are either torque or position, please check your "
                                                   "plugin "
                                                   "SDF configuration and try again.");
      return;
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "Modo",
                 "This element must indicate the working mode of this servo plugin. Valid options are either torque or "
                 "position.");
    return;
  }

  try
  {
    if (parser.HasElement("enable_saturation"))
    {
      _saturationEnabled = parser.GetElementBool("enable_saturation");
      if (_saturationEnabled)
      {
        ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saturation is enabled.");
      }
      else
      {
        ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saturation is disabled.");
      }
    }
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "enable_saturation", "This element indicates if saturation is enabled or disabled.");
  }

  // The saturation values are only needed if saturation is enabled
  if (_saturationEnabled)
  {
    switch (_mode)
    {
      case ServoModes::Torque:
        try
        {
          Force_Saturation_ = parser.GetElementDouble("Force_Saturation");
        }
        catch (const SDFStatus& e)
        {
          errorMessage(e, "Force_Saturation",
                       "This element must contain the saturation limit of the torque applied by the servo.");
          return;
        }
        break;
      case ServoModes::Position:
        try
        {
          Angle_Saturation_ = parser.GetElementDouble("Angle_Saturation");
        }
        catch (const SDFStatus& e)
        {
          errorMessage(e, "Angle_Saturation", "This element must contain the position saturation of the servo.");
          return;
        }
        break;
    }
  }

  if (parser.HasElement("friction_coeff"))
  {
    try
    {
      fric_coeff = parser.GetElementDouble("friction_coeff");
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "friction_coeff",
                   "This element must contain the friction of coefficient used to compute the friction torque "
                   "generated by the joint movement. To disable the friction forces, set this parameter to zero.");
      return;
    }
  }

  // subscriber
  motor_subscriber_ = node_handle_.subscribe(TopicSubscriber_, 1, &gazebo::ServoMotorPlugin::CallbackReferencias, this);
  // publisher
  motor_publisher_ = node_handle_.advertise<simulator_msgs::Sensor>(TopicPublisher_, 5);
  // viscous drag publisher
  drag_publisher_ = node_handle_.advertise<std_msgs::Float64>(TopicDrag_,5);

  // starts connection
  updateConnection =
      event::Events::ConnectWorldUpdateBegin([this](const common::UpdateInfo /*info*/) { this->Update(); });

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ServoMotorPlugin::Update()
{
  simulator_msgs::Sensor newmsg;
  newmsg.name = TopicPublisher_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = "1";
  newmsg.values.push_back(junta->Position(0));     // angular position
  newmsg.values.push_back(junta->GetVelocity(0));  // angular velocity

  /*
   * This will add the force contrary to the movement of the joint.
   * Note that a call to the SetForce method will not override the torque that was previously applied to the joint,
   * but will add sum the torque passed as a parameter to the previous one.
   */
  if (std::abs(fric_coeff) > 1e-9)
  {
    ViscDrag.data = -fric_coeff * junta->GetVelocity(0);
    junta->SetForce(0,ViscDrag.data );
    drag_publisher_.publish(ViscDrag);
  }

  // publishes data
  motor_publisher_.publish(newmsg);
}

double ServoMotorPlugin::Saturate(double value, double lowerLim, double upperLim) const
{
  return _saturationEnabled ? std::min(upperLim, std::max(value, lowerLim)) : value;
}

GZ_REGISTER_MODEL_PLUGIN(ServoMotorPlugin)
}  // namespace gazebo
