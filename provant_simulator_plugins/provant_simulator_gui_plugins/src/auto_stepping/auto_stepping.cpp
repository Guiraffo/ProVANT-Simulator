/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the AutoSteppingGUIPlugin
 * class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "auto_stepping.h"

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>

#include <QDebug>

using namespace gazebo;

AutoSteppingGUIPlugin::AutoSteppingGUIPlugin() : GUIPlugin()
{
  qInfo() << "Starting the AutoSteppingGUIPlugin";
  SetupUI();
}

void AutoSteppingGUIPlugin::SetupUI()
{
  setWindowFlag(Qt::WindowType::Window, true);
  setWindowFlag(Qt::WindowType::FramelessWindowHint, true);
  setAttribute(Qt::WidgetAttribute::WA_TranslucentBackground, true);
  setWindowOpacity(0);
  // Set the Qt stylesheet to match the gazebo standard
  this->setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white;}");

  _mainLayout = new QHBoxLayout(this);
  QFrame* mainFrame = new QFrame(this);
  QVBoxLayout* frameLayout = new QVBoxLayout();

  QHBoxLayout* buttonsLayout = new QHBoxLayout();

  _playPausePushbutton = new QPushButton(this);
  _stepPushbutton = new QPushButton(this);

  _playPausePushbutton->setIcon(_playIcon);

  QIcon stepIcon(":/images/step.svg");
  _stepPushbutton->setIcon(stepIcon);

  connect(_playPausePushbutton, SIGNAL(clicked()), this, SLOT(OnPlayPausePushbuttonClick()));
  connect(_stepPushbutton, SIGNAL(clicked()), this, SLOT(OnStepPushbuttonClick()));

  buttonsLayout->addWidget(_playPausePushbutton);
  buttonsLayout->addWidget(_stepPushbutton);

  buttonsLayout->addStretch(1);

  QLabel* provantLabel = new QLabel(tr("ProVANT Simulator"));

  frameLayout->addWidget(provantLabel);
  frameLayout->addLayout(buttonsLayout);

  mainFrame->setLayout(frameLayout);
  _mainLayout->addWidget(mainFrame);

  frameLayout->setContentsMargins(4, 4, 4, 4);
  _mainLayout->setContentsMargins(0, 0, 0, 0);

  setLayout(_mainLayout);

  move(5, 5);
  // From the original example code, kept for reference if needed in the future
  // resize(200, 40);
  Disable();
}

AutoSteppingGUIPlugin::~AutoSteppingGUIPlugin()
{
  // Finish ROS Node
  nodeHandle.shutdown();
}

void AutoSteppingGUIPlugin::Load(sdf::ElementPtr sdf)
{
  // Set log message
  _logMsg = "[AutoSteppingGUIPlugin";
  if (sdf != NULL)
  {
    if (sdf->HasAttribute("name"))
    {
      std::string pluginName = sdf->GetAttribute("name")->GetAsString();
      _logMsg += ": " + pluginName;
    }
  }
  _logMsg += "] ";

  gzdbg << _logMsg << "Initializing the AutoSteppingGUIPlugin.\n";
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED(PLUGIN_ID, "A ROS node for Gazebo has not been initialized, unable to load "
                                      "plugin. Load the Gazebo system plugin "
                                      "\'libgazebo_ros_api_plugin.so\' in the gazebo_ros package and "
                                      "try again. Note: This plugin will be automatically loaded using "
                                      "the \"roslaunch gazebo_ros empty_world\" launch command.");
    return;
  }

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loading process started.");

  SetupStepTopic();
  SetupSimulationStateTopic();
  SetupPlayPauseService();

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded with success.");
}

void AutoSteppingGUIPlugin::SetupStepTopic()
{
  const std::string stepTopicName("Step");
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Advertising the " << stepTopicName << " topic.");
  _stepPublisher = nodeHandle.advertise<std_msgs::String>(stepTopicName, 1);
}

void AutoSteppingGUIPlugin::SetupSimulationStateTopic()
{
  const std::string topicName("provant_simulator/simulation_state");
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Subscribing to the " << topicName << " topic.");
  _simulationStateSubscriber =
      nodeHandle.subscribe(topicName, 1, &gazebo::AutoSteppingGUIPlugin::SimulationStatusCallback, this);
}

void AutoSteppingGUIPlugin::SetupPlayPauseService()
{
  const std::string serviceName("enable_autostepping");
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Creating client for the " << serviceName << " ROS service.");
  _playPauseClient = nodeHandle.serviceClient<std_srvs::SetBool>(serviceName);
}

void AutoSteppingGUIPlugin::SetEnabled(bool enabled)
{
  if (enabled)
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Enabling the GUI.");
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Disabling the GUI.");
  }
  _playPausePushbutton->setEnabled(enabled);
  _stepPushbutton->setEnabled(enabled);
}

void AutoSteppingGUIPlugin::Enable()
{
  SetEnabled(true);
}

void AutoSteppingGUIPlugin::Disable()
{
  SetEnabled(false);
}

void AutoSteppingGUIPlugin::OnPlayPausePushbuttonClick()
{
  // Lock the paused mutex
  std::unique_lock<std::mutex> updateGuard(_pausedMutex);

  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Play/Pause pushbutton clicked.");
  std_srvs::SetBool service;

  if (_paused)
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Playing simulation.");
    service.request.data = true;
    _playPauseClient.call(service);
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Pausing simulation.");
    service.request.data = false;
    _playPauseClient.call(service);
  }

  if (service.response.success)
  {
    _paused = !_paused;
    UpdatePlayPausePushbuttonIcon(_paused);

    if (!_paused)
    {
      // Add one step to allow the simulation to begin
      AdvanceStep();
    }
  }
  else
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Call to the enable_autostepping service failed with message: \""
                                              << service.response.message << "\".");
}

void AutoSteppingGUIPlugin::AdvanceStep()
{
  std_msgs::String msg;
  _stepPublisher.publish(msg);
}

void AutoSteppingGUIPlugin::OnStepPushbuttonClick()
{
  if (_paused)
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Stepping.");
    AdvanceStep();
  }
}

bool AutoSteppingGUIPlugin::IsPaused() const
{
  return _paused;
}

bool AutoSteppingGUIPlugin::IsRunning() const
{
  return !_paused;
}

void AutoSteppingGUIPlugin::SimulationStatusCallback(const std_msgs::String::ConstPtr& msg)
{
  std::string state = msg->data;

  if (state != _prevState)
  {
    // Lock the paused mutex
    std::lock_guard<std::mutex> updateGuard(_pausedMutex);

    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Changing the simulation state to " << state << ".");

    if (state == "starting")
    {
      Disable();
      _paused = true;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else if (state == "ready")
    {
      Enable();
      _paused = true;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else if (state == "running")
    {
      Enable();
      _paused = false;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else if (state == "paused")
    {
      Enable();
      _paused = true;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else if (state == "finished")
    {
      Disable();
      _paused = true;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else if (state == "shuttingdown")
    {
      Disable();
      _paused = true;
      UpdatePlayPausePushbuttonIcon(_paused);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An unrecognized value (" << state
                                                << ") was received on the /simulation_step topic.");
    }
    _prevState = state;
  }
}

void AutoSteppingGUIPlugin::UpdatePlayPausePushbuttonIcon(bool paused)
{
  if (paused)
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Showing the play icon on the pushbutton.");
    _playPausePushbutton->setIcon(_playIcon);
  }
  else
  {
    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Showing the pause icon on the pushbutton.");
    _playPausePushbutton->setIcon(_pauseIcon);
  }
}

// Register the plguin
GZ_REGISTER_GUI_PLUGIN(AutoSteppingGUIPlugin)
