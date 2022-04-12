/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file position_logger_plugin.h
 * @brief This file contains the declaration of the PositionLoggerPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_POSITION_LOGGER_H
#define PROVANT_POSITION_LOGGER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <memory>
#include <optional>

#include <spdlog/spdlog.h>

#include <provant_simulator_csv_writer/csvwriter.h>
#include <provant_simulator_log_utils/gzlog/gzlog.h>
#include <provant_simulator_sdf_parser/sdf_parser.h>

namespace provant
{
namespace plugins
{
class PositionLoggerPlugin : public gazebo::ModelPlugin
{
public:
  PositionLoggerPlugin(std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("PositionL"
                                                                                                             "oggerPlug"
                                                                                                             "in"));
  virtual ~PositionLoggerPlugin() = default;
  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

protected:
  void OnUpdate();
  void FlushFileContents();

  std::optional<std::string> ParseFilename(const SDFParser& parser) const;
  std::optional<std::string> ParseLinkName(const SDFParser& parser) const;

private:
  gazebo::physics::LinkPtr _link;
  std::ofstream _os;
  std::unique_ptr<provant::csv::CSVFileWriter<double, 9>> _writer;
  std::shared_ptr<spdlog::logger> _logger;

  gazebo::event::ConnectionPtr _worldUpdateConn;
  gazebo::event::ConnectionPtr _pauseConn;
  gazebo::event::ConnectionPtr _sigIntConn;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_POSITION_LOGGER_H
