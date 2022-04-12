/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation for the ExternalForcesSensor class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/external_forces_sensor/external_forces_sensor.h"

#include <ignition/math/Vector3.hh>
#include <gazebo/util/IntrospectionManager.hh>

#include <cassert>

using namespace gazebo;
using ignition::math::Vector3d;
using provant::plugins::ExternalForcesSensor;

ExternalForcesSensor::ExternalForcesSensor(physics::LinkPtr link)
{
  _link = link;
}

ExternalForcesSensor::~ExternalForcesSensor()
{
}

physics::LinkPtr ExternalForcesSensor::GetLink() const
{
  return _link;
}

std::vector<double> ExternalForcesSensor::ToStdVector(Vector3d force, Vector3d torque) const
{
  std::vector<double> genForces;
  genForces.resize(6);

  genForces.at(0) = force.X();
  genForces.at(1) = force.Y();
  genForces.at(2) = force.Z();

  genForces.at(3) = torque.X();
  genForces.at(4) = torque.Y();
  genForces.at(5) = torque.Z();

  return genForces;
}

std::vector<double> ExternalForcesSensor::GetForcesAndTorques() const
{
  return ToStdVector(_link->RelativeForce(), _link->RelativeTorque());
}

std::vector<double> ExternalForcesSensor::GetForcesAndTorquesInertial() const
{
  return ToStdVector(_link->WorldForce(), _link->WorldTorque());
}
