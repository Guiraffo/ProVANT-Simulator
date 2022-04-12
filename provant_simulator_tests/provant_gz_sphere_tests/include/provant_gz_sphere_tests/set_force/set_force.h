/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file set_force.h
 * @brief This file contains the implementation of the SetForcePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SET_FORCE_H
#define PROVANT_SET_FORCE_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <spdlog/spdlog.h>

#include <memory>

#include <provant_simulator_log_utils/gzlog/gzlog.h>

namespace provant
{
namespace plugins
{
class SetForcePlugin : public gazebo::ModelPlugin
{
public:
  SetForcePlugin(std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("Set"
                                                                                                       "Forc"
                                                                                                       "e"
                                                                                                       "Plug"
                                                                                                       "in"));
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  void OnUpdate();

private:
  std::shared_ptr<spdlog::logger> _logger;

  gazebo::event::ConnectionPtr _conn;
  gazebo::physics::LinkPtr _link;
  ignition::math::Vector3d _forceVector;

  int _counter = 0;
  int applyEveryNSteps = 1;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_SET_FORCE_H
