/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file external_forces_sensor_ros.cpp
 * @brief This file contains the implementation of the ExternalForcesSensorPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/external_forces_sensor/external_forces_sensor_ros.h"

#include <gazebo/physics/physics.hh>
#include <gazebo/util/IntrospectionManager.hh>
#include <ros/time.h>

#include <geometry_msgs/WrenchStamped.h>

#include <cassert>
#include <sstream>
#include <typeinfo>

#include <boost/core/demangle.hpp>

using namespace gazebo;
using provant::plugins::ExternalForcesSensorPlugin;

ExternalForcesSensorPlugin::ExternalForcesSensorPlugin()
{
  _lastForceAndTorque.resize(6);
}

ExternalForcesSensorPlugin::~ExternalForcesSensorPlugin()
{
  _nodeHandle.shutdown();
}

void ExternalForcesSensorPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  // Verify if the model and sdf pointers are valid
  GZ_ASSERT(_model != nullptr, "Received a null model");
  GZ_ASSERT(_sdf != nullptr, "Received a null SDF");
  _world = _model->GetWorld();
  GZ_ASSERT(_world != nullptr, "The model is in a null world");

  // Verify that ROS is initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("ExternalForcesSensorPlugin", "A ROS Node for Gazebo has not been initilized, unable to "
                                                         "load "
                                                         "plugin. Load the Gazebo system plugin "
                                                         "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package. "
                                                         "This is "
                                                         "done automatically if you load gazebo using the \"roslaunch "
                                                         "gazebo_ros empty_world.launch\" command.");
    return;
  }

  // Stores the pointers to the model and sdf
  this->_model = _model;
  this->_sdf = _sdf;

  // Reads the name and creates a log message identifier for the current node
  SetupLogging();
  // Reads the link name and toppic name from the plugin SDF
  if (!ReadTags())
    return;
  // Get a pointer to the monitored link
  if (!GetLink())
    return;
  // Create a force sensor object for the specified link
  SetupLinkSensor();
  // Advertise a ROS topic to publish the updates of the forces and torques
  if (!SetupROSTopic())
    return;
  SetupPluginIntrospection();
  // Connect to gazebo WorldUpdateEnd
  RegisterOnUpdateEvent();
}

void ExternalForcesSensorPlugin::Init()
{
  ModelPlugin::Init();
  ROS_INFO_STREAM(_logMsg << "Finished plugin loading process with success");
}

void ExternalForcesSensorPlugin::Reset()
{
  ModelPlugin::Reset();
}

void ExternalForcesSensorPlugin::OnUpdate()
{
  if (_linkSensor == nullptr)
  {
    ROS_ERROR_STREAM(_logMsg << "Error in the OnUpdate method. A null link sensor pointer was found.");
    return;
  }

  // Get the forces currently acting on the link
  std::vector<double> _wrench = _linkSensor->GetForcesAndTorques();
  if (_wrench.size() != 6)
  {
    // Format vector for string impression
    std::stringstream _vecstr;
    _vecstr << "[";
    for (std::size_t i = 0; i < _wrench.size(); i++)
    {
      _vecstr << _wrench.at(i) << ", ";
    }
    _vecstr.seekp(-2, _vecstr.cur);
    _vecstr << "]";

    ROS_ERROR_STREAM(_logMsg << "Error, malformed forces and torques vector returned. Vector with elements: "
                             << _vecstr.str());
    return;
  }

  _lastForceAndTorque = _wrench;

  if (_publisher && _numSubscribers > 0)
  {
    // Create the message
    geometry_msgs::WrenchStamped msg;
    // The seq is equal to the number of steps (Iterations)
    msg.header.seq = _world->Iterations();
    msg.header.frame_id = _linkName;
    msg.header.stamp = ros::Time::now();
    // Update the forces
    msg.wrench.force.x = _lastForceAndTorque.at(0);
    msg.wrench.force.y = _lastForceAndTorque.at(1);
    msg.wrench.force.z = _lastForceAndTorque.at(2);
    // Update the torques
    msg.wrench.torque.x = _lastForceAndTorque.at(3);
    msg.wrench.torque.y = _lastForceAndTorque.at(4);
    msg.wrench.torque.z = _lastForceAndTorque.at(5);

    _publisher.publish(msg);
  }
}

const std::string ExternalForcesSensorPlugin::GetTopicName() const
{
  return _topicName;
}

void ExternalForcesSensorPlugin::SetupLogging()
{
  char const* className = typeid(this).name();
  _logMsg = "[" + boost::core::demangle(className);

  if (_sdf->HasAttribute("name"))
  {
    _pluginName = _sdf->GetAttribute("name")->GetAsString();
    _logMsg += ": " + _pluginName;
  }
  _logMsg += "] ";
}

bool ExternalForcesSensorPlugin::ReadTags()
{
  if (_sdf->HasElement("topic"))
  {
    _topicName = _sdf->Get<std::string>("topic");
  }
  else
  {
    ROS_FATAL_STREAM(_logMsg << "The topic name for publishing the forces and torques was not found. Please define the "
                                "\"topic\" tag on the plugin SDF configuration and try again.");
    return false;
  }

  if (_sdf->HasElement("link"))
  {
    _linkName = _sdf->Get<std::string>("link");
  }
  else
  {
    ROS_FATAL_STREAM(_logMsg << "The link name this plugin should monitor the applied forces and torques was not "
                                "defined in the plugin SDF. Please add the \"link\" element on the plugin SDF "
                                "definition and try again.");
    return false;
  }

  return true;
}

bool ExternalForcesSensorPlugin::GetLink()
{
  if (_linkName.empty())
    return false;

  _sensedLink = _model->GetLink(_linkName);
  if (_sensedLink == nullptr)
    return false;

  return true;
}

void ExternalForcesSensorPlugin::SetupLinkSensor()
{
  _linkSensor = std::unique_ptr<ExternalForcesSensor>(new ExternalForcesSensor(_sensedLink));
}

void ExternalForcesSensorPlugin::RegisterOnUpdateEvent()
{
  _updateConnection = event::Events::ConnectWorldUpdateEnd(std::bind(&ExternalForcesSensorPlugin::OnUpdate, this));
  if (_updateConnection == nullptr)
  {
    ROS_FATAL_STREAM(_logMsg << "Error on RegisterOnUpdateEvent, a null connection pointer was returned.");
  }
  else
  {
    ROS_INFO_STREAM(_logMsg << "Connected to the WorldUpdateEnd event");
  }
}

void ExternalForcesSensorPlugin::OnSubscriberConnect()
{
  std::lock_guard<std::mutex> lock(_numSubscribersMutex);
  _numSubscribers++;
}

void ExternalForcesSensorPlugin::OnSubscriberDiscconect()
{
  std::lock_guard<std::mutex> lock(_numSubscribersMutex);
  _numSubscribers--;
  if (_numSubscribers < 0)
  {
    ROS_WARN_STREAM(_logMsg << "Negative number of subscribers detected on the \"" << _topicName << "\" topic!");
  }
}

bool ExternalForcesSensorPlugin::SetupROSTopic()
{
  ROS_INFO_STREAM(_logMsg << "Advertising topic with name: " << _topicName);
  try
  {
    _publisher = _nodeHandle.advertise<geometry_msgs::WrenchStamped>(
        _topicName, 1, std::bind(&ExternalForcesSensorPlugin::OnSubscriberConnect, this),
        std::bind(&ExternalForcesSensorPlugin::OnSubscriberDiscconect, this));
    return true;
  }
  catch (const ros::InvalidNameException& e)
  {
    ROS_FATAL_STREAM(_logMsg << "InvalidNameException when trying to register the topic with name \"" << _topicName
                             << "\". Exception message: " << e.what());
    return false;
  }
  return false;
}

void ExternalForcesSensorPlugin::SetupPluginIntrospection()
{
  std::string identifier;
  if (_pluginName.empty())
  {
    identifier = "ExternalForcesSensorPlugin/" + _linkName;
  }
  else
  {
    identifier = _pluginName + "/" + _linkName;
  }

  auto forceCallback_x = [this]() { return this->_lastForceAndTorque.at(0); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/force/x",
                                                                   forceCallback_x);

  auto forceCallback_y = [this]() { return this->_lastForceAndTorque.at(1); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/force/y",
                                                                   forceCallback_y);

  auto forceCallback_z = [this]() { return this->_lastForceAndTorque.at(2); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/force/z",
                                                                   forceCallback_z);

  auto torqueCallback_x = [this]() { return this->_lastForceAndTorque.at(2); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/torque/x",
                                                                   torqueCallback_x);

  auto torqueCallback_y = [this]() { return this->_lastForceAndTorque.at(2); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/torque/y",
                                                                   torqueCallback_y);

  auto torqueCallback_z = [this]() { return this->_lastForceAndTorque.at(2); };
  gazebo::util::IntrospectionManager::Instance()->Register<double>("data://" + identifier + "/torque/z",
                                                                   torqueCallback_z);
}

GZ_REGISTER_MODEL_PLUGIN(ExternalForcesSensorPlugin)
