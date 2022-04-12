/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file apply_force_once.cpp
 * @brief
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_gz_sphere_tests/apply_force_once/apply_force_once.h"

#include <provant_simulator_sdf_parser/sdf_parser.h>

#include <sstream>

using provant::plugins::ApplyForceOncePlugin;

GZ_REGISTER_MODEL_PLUGIN(ApplyForceOncePlugin);

ApplyForceOncePlugin::ApplyForceOncePlugin(std::shared_ptr<spdlog::logger> logger) : _logger(logger)
{
}

void ApplyForceOncePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Received a null model");
  GZ_ASSERT(_sdf, "Received a null sdf configuration");

  _logger->info("Starting the plugin loading process.");

  SDFParser parser{ _sdf };

  try
  {
    const auto linkName = parser.GetElementText("link");
    const auto force = parser.GetElementDouble("force");

    auto linkPtr = _model->GetLink(linkName);
    if (!linkPtr)
    {
      _logger->critical("A link named " + linkName + " does not exist in the " + _model->GetName() + " model.");
      return;
    }

    std::stringstream ss;
    ss << "Applying a force of " << force << "N to the " << linkName << " link.";
    _logger->info(ss.str());

    const ignition::math::Vector3d forceVector{ 0, 0, force };
    linkPtr->AddRelativeForce(forceVector);

    _logger->info("The plugin loaded successfully.");
  }
  catch (const SDFStatus& e)
  {
    _logger->critical("An exception with message \"" + std::string{ e.what() } +
                      "\" occurred while reading the plugin parameters from its SDF.");
    return;
  }
}
