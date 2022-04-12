/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file set_force_once.h
 * @brief This file contains the declaration of the SetForceOncePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SET_FORCE_ONCE_H
#define PROVANT_SET_FORCE_ONCE_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <spdlog/spdlog.h>

#include <memory>

#include <provant_simulator_log_utils/gzlog/gzlog.h>

namespace provant
{
namespace plugins
{
class SetForceOncePlugin : public gazebo::ModelPlugin
{
public:
  SetForceOncePlugin(std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("SetForceOnc"
                                                                                                           "ePlugin"));
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  std::shared_ptr<spdlog::logger> _logger;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_SET_FORCE_ONCE_H
