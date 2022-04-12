/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the QuadForces class.
 *
 * @author Jonatan Mota Campos
 */

#include <QuadForces.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include "XMLRead.h"

namespace gazebo
{
/**********************************************************************************************************************/
// Constructor
QuadForces::QuadForces() : ForceBody(3), TorqueBody(3)
{
}

/**********************************************************************************************************************/

// Method called when the plugin is Loaded by Gazebo
void QuadForces::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  _logMsg = "[QuadForcesPlugin";
  if (_sdf->HasAttribute("name"))
  {
    _logMsg += ": ";
    _logMsg += _sdf->GetAttribute("name")->GetAsString();
  }
  _logMsg += "] ";

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Started the plugin loading process.");

  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "A ROS Node for Gazebo has not been initilized. The "
                                                 "QuadForces"
                                                 "plugin cannot be loaded. Load the Gazebo system plugin "
                                                 "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros "
                                                 "package. This is "
                                                 "done automatically if you load gazebo using the "
                                                 "\"roslaunch "
                                                 "gazebo_ros empty_world.launch\" command.");
    return;
  }

  topic_F1 = XMLRead::ReadXMLString("topic_F1", _sdf);  // name of brushless number 1
  topic_F2 = XMLRead::ReadXMLString("topic_F2", _sdf);  // name of brushless number 2
  topic_F3 = XMLRead::ReadXMLString("topic_F3", _sdf);  // name of brushless number 3
  topic_F4 = XMLRead::ReadXMLString("topic_F4", _sdf);  // name of brushless number 4
  NameOfLink_ = XMLRead::ReadXMLString("body", _sdf);   // name of main link
  DragConst = XMLRead::ReadXMLDouble("DragCte", _sdf);  // Drag constant
  alpha = XMLRead::ReadXMLDouble("alpha", _sdf);        // The tilt angle of the propellers toward the center of the UAV
  length = XMLRead::ReadXMLDouble("length", _sdf);      // The arm length (distance from the center to the propeller).

  // Setup saturation
  SetupSaturation(_sdf);

  _world = _model->GetWorld();
  GZ_ASSERT(_world != nullptr, "The model is a null world.");

  // get elements of the simulation
  link = _model->GetLink(NameOfLink_);

  // subscribers of data to apply in simulator
  motor_subscriberF1_ = node_handle_.subscribe(topic_F1, 1, &gazebo::QuadForces::CallbackF1, this);
  motor_subscriberF2_ = node_handle_.subscribe(topic_F2, 1, &gazebo::QuadForces::CallbackF2, this);
  motor_subscriberF3_ = node_handle_.subscribe(topic_F3, 1, &gazebo::QuadForces::CallbackF3, this);
  motor_subscriberF4_ = node_handle_.subscribe(topic_F4, 1, &gazebo::QuadForces::CallbackF4, this);

  // Connect to the WorldUpdateEnd event to call the Update method at every simulation step
  updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&gazebo::QuadForces::Update, this));
  if (updateConnection == nullptr)
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while loading the plugin. It was not possible to "
                                                 "register "
                                                 "the connection to the Gazebo WorldUpdateBegin event. A "
                                                 "null "
                                                 "pointer was returned.");
    return;
  }

  // Send log message informing the plugin was successfully loaded
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Successfully loaded the " << _sdf->GetAttribute("name")->GetAsString()
                                           << " plugin.");
}

/**********************************************************************************************************************/

// Callback to set the force at the brushless 1
void QuadForces::CallbackF1(const std_msgs::Float64& msg)
{
  F1 = SaturateForce(msg.data);
}

/**********************************************************************************************************************/

// Callback to set the force at the brushless 2
void QuadForces::CallbackF2(const std_msgs::Float64& msg)
{
  F2 = SaturateForce(msg.data);
}

/**********************************************************************************************************************/

// Callback to set the force at the brushless 3
void QuadForces::CallbackF3(const std_msgs::Float64& msg)
{
  F3 = SaturateForce(msg.data);
}

/**********************************************************************************************************************/

// Callback to set the force at the brushless 4
void QuadForces::CallbackF4(const std_msgs::Float64& msg)
{
  F4 = SaturateForce(msg.data);
}

/**********************************************************************************************************************/

// Method called at every simulation step
void QuadForces::Update()
{
  TorqueBody << (F2 - F4) * length * cos(alpha), (F3 - F1) * length * cos(alpha),
      DragConst * (F1 + F3 - F2 - F4) * cos(alpha);
  ForceBody << (F3 - F1) * sin(alpha), (F4 - F2) * sin(alpha), (F1 + F2 + F3 + F4) * cos(alpha);

  ignition::math::Vector3d Torque(TorqueBody(0), TorqueBody(1), TorqueBody(2));
  ignition::math::Vector3d Force(ForceBody(0), ForceBody(1), ForceBody(2));

  // Applies force and torque w.r.t the body frame
  link->AddRelativeForce(Force);
  link->AddRelativeTorque(Torque);

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Forces applied by the propellers at iteration " << _world->Iterations()
                                            << ": F1: " << F1 << ", F2: " << F2 << ", F3: " << F3 << ", F4: " << F4
                                            << ".");
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Applying forces to the UAV at iteration " << _world->Iterations()
                                            << ": Fx: " << Force.X() << ", Fy: " << Force.Y() << ", Fz: " << Force.Z()
                                            << ", Tau_x: " << Torque.X() << ", Tau_y: " << Torque.Y()
                                            << ", Tau_z: " << Torque.Z() << ".");
}

/**********************************************************************************************************************/

// Method used to apply saturation to the forces
double QuadForces::SaturateForce(double force) const
{
  if (_saturate)
  {
    return std::max(_satMin, std::min(_satMax, force));
  }
  // If saturation is disabled, return the unsaturated force value.
  return force;
}

/**********************************************************************************************************************/

// Method used to configure the saturation
void QuadForces::SetupSaturation(sdf::ElementPtr sdf)
{
  if (sdf->HasElement("saturate"))
  {
    std::string saturate_str = sdf->Get<std::string>("saturate");
    // Convert string to lower case
    std::transform(saturate_str.begin(), saturate_str.end(), saturate_str.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (saturate_str == "true" || saturate_str == "1")
    {
      _saturate = true;
    }
    else if (saturate_str == "false" || saturate_str == "0")
    {
      _saturate = false;
      ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saturation is disabled");
      return;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Error while configuring the forces saturation, and "
                                                   "invalid value was read from the "
                                                   "saturate tag: \""
                                                << saturate_str
                                                << " \". The valid options are \"true\" or \"false\". "
                                                   "Please verify this tag value and "
                                                   "try again. "
                                                   "Saturation will be disabled in the current simulation.");
      return;
    }
  }
  else
  {
    // If the saturate element, we warn the user of no saturation will be applied, and inform the user how to silence
    // this warning
    ROS_WARN_STREAM_NAMED(PLUGIN_ID, _logMsg << "The saturate tag was not found at the plugin SDF "
                                                "configuration. The "
                                                "saturation will be disabled. If this is the intended "
                                                "behavior of this "
                                                "simulation, please add the \"<saturate>false</saturate>\" "
                                                "tag to the "
                                                "plugin SDF to silence this warning.");
    return;
  }

  // If saturation is enabled, read the maximum and minimum values
  if (sdf->HasElement("satmin"))
  {
    _satMin = XMLRead::ReadXMLDouble("satmin", sdf);
  }

  if (sdf->HasElement("satmax"))
  {
    _satMax = XMLRead::ReadXMLDouble("satmax", sdf);
  }

  // If saturation is enabled, but the limits are not setup, notify the user
  if (_satMin == -std::numeric_limits<double>::infinity() && _satMax == std::numeric_limits<double>::infinity())
  {
    _saturate = false;
    ROS_WARN_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saturation is enabled, but both the minium and maximum "
                                                "values are set "
                                                "for infinity or not configured, so saturation will not be "
                                                "enabled. If "
                                                "this is the intended behavior, please disable saturation "
                                                "to silence "
                                                "this warning. If this is an error, please define at least "
                                                "one the "
                                                "\"satmin\" or \"satmax\" tags in the plugin SDF with the "
                                                "desired "
                                                "values.");
  }
  else
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Saturation is enabled, the applied forces will be limited "
                                                "between ["
                                             << _satMin << ", " << _satMax << "].");
  }
}

GZ_REGISTER_MODEL_PLUGIN(QuadForces)
}  // namespace gazebo
