/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file apply_force_once.h
 * @brief This file contains the declaration of the ApplyForceOncePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_APPLY_FORCE_ONCE_H
#define PROVANT_APPLY_FORCE_ONCE_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <spdlog/spdlog.h>

#include <memory>

#include <provant_simulator_log_utils/gzlog/gzlog.h>

namespace provant
{
namespace plugins
{
class ApplyForceOncePlugin : public gazebo::ModelPlugin
{
public:
  ApplyForceOncePlugin(std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("ApplyForc"
                                                                                                             "eOncePlug"
                                                                                                             "in"));
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  std::shared_ptr<spdlog::logger> _logger;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_APPLY_FORCE_ONCE_H
