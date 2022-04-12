/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the VisualPropellers class.
 *
 * @author Jonatan Mota Campos
 */

#ifndef VISUAL_PROPELLERS_H
#define VISUAL_PROPELLERS_H

#include <ros/ros.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <string>

namespace gazebo
{
/**
 * @brief The VisualPropellers class is a pure Gazebo plugin (has no dependency on ROS) that adds an angular velocity to
 * the propeller blades, enhancing the visual feedback of the simulation.
 *
 * This plugin expects the SDF to define the Propeller1, Propeller2, and Propellers_Velocity tags in its SDF, the first
 * two must contain the name of the propellers links and the last one the desired angular velocity in radians per second
 * to be applied at the propellers.
 */
class VisualPropellers : public ModelPlugin
{
public:
  /**
   * @brief Construct a new Visual Propellers object.
   */
  VisualPropellers() = default;
  /**
   * @brief Destroy the Visual Propellers object.
   *
   * Note: Previous versions of the destructor used to set the speed of the propellers to zero on destruction.
   * This is the expected behavior. However, Gazebo does not follow an appropriate order of destruction and deletes
   * some links prior to the deletion of the model plugins, thus when we tried to set the speed of the links
   * it resulted in an error.
   *
   * Up to this point there is no way to detect if the link is still available during this destruction, so the
   * destructor functionality was just removed.
   */
  virtual ~VisualPropellers() = default;
  /**
   * @brief Method called when the plugin is loaded by Gazebo.
   *
   * Reads the name of the propellers links and angular speed from the plugin SDF
   * configuration and applies the desired speed to the propellers.
   *
   * @param _model Pointer to the model that contains this plugin.
   * @param _sdf Pointer to the SDF element that instantiates this plugin.
   */
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
  /**
   * @brief Method called when the simulator is reseted.
   *
   * Reapplies the angular velocities to the propellers.
   */
  virtual void Reset() override;

private:
  physics::WorldPtr world;
  event::ConnectionPtr updateConnection;  // connection pointer

protected:
  event::ConnectionPtr _worldUpdateBeginConn;

  void OnWorldUpdateBegin();

private:
  //! Name of the first propeller joint.
  std::string _propeller1Name;
  //! Name of the second propeller joint.
  std::string _propeller2Name;
  //! Pointer to the joint of the first propeller.
  physics::JointPtr _propeller1;
  //! Pointer to the joint of the second propeller.
  physics::JointPtr _propeller2;
  //! Angular speed applied to the propellers.
  double _velocity;
  //! Holds a pointer that mantains the plugin connected to the world update end signal.
  event::ConnectionPtr _worldUpdateEndConn;
  //! Message used to identify the messages from this plugin in the output screen.
  std::string _logMsg;
};
}  // namespace gazebo

#endif  // VISUAL_PROPELLERS_H
