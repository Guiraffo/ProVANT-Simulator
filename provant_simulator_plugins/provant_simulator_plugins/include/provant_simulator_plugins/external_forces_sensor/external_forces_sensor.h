/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration for the ExternalForcesSensor class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef EXTERNAL_FORCES_SENSOR_H
#define EXTERNAL_FORCES_SENSOR_H

#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

#include <vector>

namespace provant
{
namespace plugins
{
/**
 * @brief The ExternalForcesSensor class is the internal implementation of a
 * Gazebo plugin that reads the forces and torques acting on a given link,
 * and return their values in a STL vector.
 *
 * This vector can in turn be published to ROS nodes to allow verification
 * of the forces acting on a link.
 *
 * This plugin is specially usefull during the validation and debugging process
 * of new actuators.
 */
class ExternalForcesSensor
{
public:
  /**
   * @brief Construct a new External Forces Sensor object
   */
  ExternalForcesSensor(gazebo::physics::LinkPtr link);
  /**
   * @brief Destroy the External Forces Sensor object.
   */
  virtual ~ExternalForcesSensor();

  /**
   * @brief Return a pointer of the link whose forces are read by this object.
   *
   * @return physics::LinkPtr
   */
  gazebo::physics::LinkPtr GetLink() const;

  /**
   * @brief Get the forces and torques acting on the center of mass of the link
   * relative to the link coordinate system.
   *
   * The vector is the following order
   * Force applied on the X direction
   * Force applied on the Y direction
   * Force applied on the Z direction
   * Torque applied around the X direction (roll torque)
   * Torque applied around the Y direction (pitch torque)
   * Torque applied around the Z direction (yaw torque)
   *
   * @return const std::vector<double>&
   */
  virtual std::vector<double> GetForcesAndTorques() const;
  /**
   * @brief Get the Forces And Torques acting on the center of mass of the link
   * relative to the inertial coordinate system.
   *
   * The vector is in the same order as the vector returned by
   * GetForcesAndTorques().
   *
   * @sa GetForcesAndTorques()
   *
   * @return std::vector<double>
   */
  virtual std::vector<double> GetForcesAndTorquesInertial() const;

protected:
  //! Pointer to the Link this object should read the forces and torques
  gazebo::physics::LinkPtr _link;

  /**
   * @brief Helper function to convert a force and a torque ignition math vector
   * to a single std::vector.
   *
   * @param force Tridimensional force vector to convert.
   * @param torque Tridimensional torque vector to convert.
   * @return std::vector<double>
   */
  virtual std::vector<double> ToStdVector(ignition::math::Vector3d force, ignition::math::Vector3d torque) const;
};
}  // namespace plugins
}  // namespace provant

#endif  // EXTERNAL_FORCES_SENSOR_H
