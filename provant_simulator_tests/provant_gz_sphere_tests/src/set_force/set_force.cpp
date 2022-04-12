/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file set_force.cpp
 * @brief This file contains the implementation of the SetForcePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_gz_sphere_tests/set_force/set_force.h"

#include <gazebo/common/Events.hh>

#include <sstream>

#include <provant_simulator_sdf_parser/sdf_parser.h>

using provant::plugins::SetForcePlugin;

GZ_REGISTER_MODEL_PLUGIN(SetForcePlugin)

SetForcePlugin::SetForcePlugin(std::shared_ptr<spdlog::logger> logger) : _logger(logger)
{
}

void SetForcePlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Received a null model");
  GZ_ASSERT(_sdf, "Received a null sdf configuration");

  _logger->info("Starting the plugin loading process.");

  SDFParser parser{ _sdf };

  try
  {
    const auto linkName = parser.GetElementText("link");
    const auto force = parser.GetElementDouble("force");

    if (parser.HasElement("apply_every_n_steps"))
    {
      applyEveryNSteps = parser.GetElementInt("apply_every_n_steps");
    }
    else
    {
      applyEveryNSteps = 1;
    }

    _link = _model->GetLink(linkName);
    if (!_link)
    {
      _logger->critical("A link named " + linkName + " does not exist in the " + _model->GetName() + " model.");
      return;
    }

    std::stringstream ss;
    ss << "Applying a force of " << force << "N to the " << linkName << " link.";
    _logger->info(ss.str());

    _forceVector = ignition::math::Vector3d{ 0, 0, force };

    _logger->info("Connecting to the WorldUpdateBegin signal.");
    _conn = gazebo::event::Events::ConnectWorldUpdateBegin(
        [this](const gazebo::common::UpdateInfo& /*info*/) { this->OnUpdate(); });

    _logger->info("The plugin loaded successfully.");
  }
  catch (const SDFStatus& e)
  {
    _logger->critical("An exception with message \"" + std::string{ e.what() } +
                      "\" occurred while reading the plugin parameters from its SDF.");
    return;
  }
}

void SetForcePlugin::OnUpdate()
{
  if (_counter % applyEveryNSteps == 0)
  {
    _link->SetForce(_forceVector);
  }
  _counter++;
}
