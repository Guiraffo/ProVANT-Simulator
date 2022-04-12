/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the StepPlugin class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_STEP_PLUGIN_H
#define PROVANT_STEP_PLUGIN_H

#include <gazebo/common/Plugin.hh>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>

namespace gazebo
{
/**
 * @brief The StepPlugin is a class that exposes a topic that allows a ROS node
 * to control the steps of the Gazebo simulation, and thus control the pace of
 * the simulation.
 *
 * This plugin is a fundamental component of the ProVANT Simulator, and it is
 * the piece that allows the controller node to assume control of simulation
 * timing, and thus allowing the control strategies to take as much time as it
 * needs to compute the control inputs to apply in the model, while the
 * simulation time is advanced only the time specified in the physics paramters.
 *
 * If the simulation is in Hardware In the Loop (HIL) mode, as indicated by the
 * "ok" parameter in the plugin SDF options, this plugin loads, but does not
 * do anything.
 *
 * @todo Change the name of the ok parameter to something which has at least
 * a ressemblance of meaning.
 */
class StepPlugin : public WorldPlugin
{
public:
  /**
   * @brief Construct a new Step Plugin object.
   * The constructor does not initializes the plugin because of Gazebo internal
   * logic, see the Load method for more details on plugin initialization.
   * @sa Load()
   */
  StepPlugin() = default;
  /**
   * @brief Destroy the Step Plugin object.
   * Shutdowns the subscription to the /Step topic.
   */
  virtual ~StepPlugin() = default;
  /**
   * @brief Method called when this plugin is loaded by Gazebo.
   *
   * If the simulation is in normal mode, this method will setup the subscriber
   * to the /Step node.
   * If the simulation is in Hardware In the Loop (HIL) mode, the plugin loads,
   * but does nothing.
   *
   * @param _world Pointer to the simulation world which contains this plugin.
   * @param _sdf Pointer to the SDF element which instantiates this plugin and
   * contains its settings.
   */
  void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Method called when a new message is published on the /Step topic.
   *
   * Used to advance the simulation steps.
   *
   * @todo Change the name of the /Step topic to /step to follow ROS naming standards.
   * @todo Change the type of message of this topic to an EmptyMessage, as it currently does nothing with the string
   * received in the message parameter.
   *
   * @param msg Message sent to the /Step topic. Currently this is an unused parameter.
   */
  void chatterCallback(const std_msgs::String::ConstPtr& msg);

private:
  //! Pointer to the simulation world.
  physics::WorldPtr _world;
  //! Pointer to the SDF element that instantiates this plugin.
  sdf::ElementPtr _sdf;
  //! Subscriber to the /Step topic.
  ros::Subscriber _stepSubscriber;
  //! Node handle to manage the subscription to the topics and services published and subscribed by this node.
  ros::NodeHandle _nodeHandle;
  //! Name of the child logger used by this plugin.
  const std::string PLUGIN_ID = "step_plugin";
  //! Message used to identify the log messages of this plugin in the output screen.
  std::string _logMsg;
};
}  // namespace gazebo

#endif  // PROVANT_STEP_PLUGIN_H
