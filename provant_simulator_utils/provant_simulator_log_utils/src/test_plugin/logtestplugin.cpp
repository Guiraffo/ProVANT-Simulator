/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file logtestplugin.cpp
 * @brief This file contains the implementation of the LogTestPlugin class.
 *
 * See the logtestplugin.h header for more detailed documentation.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "logtestplugin.h"

using provant::gz_plugins::LogTestPlugin;

GZ_REGISTER_WORLD_PLUGIN(provant::gz_plugins::LogTestPlugin);

LogTestPlugin::LogTestPlugin(std::shared_ptr<spdlog::logger> logger) : _logger(std::move(logger))
{
}

void LogTestPlugin::Load(gazebo::physics::WorldPtr /*_world*/, sdf::ElementPtr /*_sdf*/)
{
  _logger->set_level(spdlog::level::level_enum::trace);

  _logger->trace("Hello, this is a trace message.");
  _logger->debug("Hello, this is a debug message.");
  _logger->info("Hello, this is an info message.");
  _logger->warn("Hello, this is a warning message.");
  _logger->error("Hello, this is an error message");
  _logger->critical("Hello, this is a critical message.");
}
