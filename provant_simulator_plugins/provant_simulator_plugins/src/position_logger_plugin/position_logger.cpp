/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file position_logger.cpp
 * @brief This file contains the implementation of the PositionLoggerPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/position_logger_plugin/position_logger_plugin.h"

#include <gazebo/common/Events.hh>

using provant::plugins::PositionLoggerPlugin;

GZ_REGISTER_MODEL_PLUGIN(PositionLoggerPlugin);

PositionLoggerPlugin::PositionLoggerPlugin(std::shared_ptr<spdlog::logger> logger) : _logger(logger)
{
}

void PositionLoggerPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_model, "Received a null model.");
  GZ_ASSERT(_sdf, "The plugin SDF was empty.");

  _logger->info("Starting the plugin loading process.");

  // Read the options from the plugin SDF
  SDFParser parser{ _sdf };
  const auto filename = ParseFilename(parser);
  if (!filename.has_value())
    return;

  _logger->info("Saving the data to the " + filename.value() + " file.");

  const auto linkName = ParseLinkName(parser);
  if (!linkName.has_value())
    return;

  // Check if a link with the provide name exists
  _link = _model->GetLink(linkName.value());
  if (!_link)
  {
    _logger->critical("A link with the specified name (" + linkName.value() + ") does not exist in the " +
                      _model->GetName() + " model.");
    return;
  }

  // Initialize the file
  try
  {
    using provant::csv::CSVFileWriter;
    _writer = std::unique_ptr<CSVFileWriter<double, 9>>(
        new CSVFileWriter<double, 9>{ CSVFileWriter<double, 9>::relativeToMatlabData(filename.value()),
                                      { "x", "y", "z", "dot_x", "dot_y", "dot_z", "ddot_x", "ddot_y", "ddot_z" } });
  }
  catch (const std::exception& e)
  {
    _logger->critical("An exception with message \"" + std::string{ e.what() } +
                      "\" occurred while trying to create the simulation results file.");
    return;
  }

  _worldUpdateConn = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { this->OnUpdate(); });
  _pauseConn = gazebo::event::Events::ConnectPause([this](bool /*paused*/) { this->FlushFileContents(); });
  _sigIntConn = gazebo::event::Events::ConnectSigInt([this]() { this->FlushFileContents(); });

  _logger->info("Plugin loaded successfully.");
}

std::optional<std::string> PositionLoggerPlugin::ParseFilename(const SDFParser& parser) const
{
  try
  {
    return parser.GetElementText("filename");
  }
  catch (const SDFStatus& e)
  {
    _logger->critical("An error ocurred while reading the value of the filename element. This element should contain "
                      "the path to the file that will contain the simulation results.");
    return std::optional<std::string>{};
  }
}

std::optional<std::string> PositionLoggerPlugin::ParseLinkName(const SDFParser& parser) const
{
  try
  {
    return parser.GetElementText("link");
  }
  catch (const SDFStatus& e)
  {
    _logger->critical("An error ocurred while reading the value of the link element. This element should contain the "
                      "name of a model link to monitor during the simulation.");
    return std::optional<std::string>{};
  }
}

void PositionLoggerPlugin::OnUpdate()
{
  const auto pos = _link->WorldPose().Pos();
  const auto vel = _link->WorldLinearVel();
  const auto acc = _link->WorldLinearAccel();

  try
  {
    _writer->writeLine({ pos.X(), pos.Y(), pos.Z(), vel.X(), vel.Y(), vel.Z(), acc.X(), acc.Y(), acc.Z() });
  }
  catch (const std::exception& e)
  {
    _logger->critical("An unexpected exception with message \"" + std::string{ e.what() } +
                      "\" occurred while writing the values to the file.");
  }
}

void PositionLoggerPlugin::FlushFileContents()
{
  _logger->info("Flushing the " + _writer->path() + " file contents.");
  _writer->flush();
}
