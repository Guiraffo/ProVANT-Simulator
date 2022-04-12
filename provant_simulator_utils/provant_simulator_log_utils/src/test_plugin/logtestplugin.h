/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file logtestplugin.h
 * @brief This file contains the declaration of the LogTestPlugin class.
 *
 * The LogTestPlugin class implements a Gazebo World Plugin created to the test
 * the ProVANT Logger.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_LOG_TEST_PLUGIN_H
#define PROVANT_LOG_TEST_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <provant_simulator_log_utils/gzlog/gzlog.h>

#include <memory>

namespace provant
{
namespace gz_plugins
{
/**
 * @brief The LogTestPlugin implements a Gazebo World plugin developed in order to test the various loggers provided by
 * the provant_simulator_log_utils package.
 *
 * This plugin provides no functionality that is usefull outside of the development of this package.
 *
 */
class LogTestPlugin : public gazebo::WorldPlugin
{
public:
  /**
   * @brief Construct a new Log Test Plugin object.
   *
   * @param logger Logger object used to output the log messages of this plugin to the screen. By the default uses
   * a logger with a Gazebo sink.
   */
  LogTestPlugin(std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("log_test_"
                                                                                                      "plugin"));
  virtual ~LogTestPlugin() = default;
  /**
   * @brief Called once during the plugin loading process. This method is used to output one log message of each
   * severity level to the screen in order to test the behavior of the logger object passed to the class constructor.
   *
   * This method will ouput one message of the following severity levels to the screen:
   *
   * - trace
   * - debug
   * - info
   * - warning
   * - error
   * - critical
   *
   * @param _world Unused. World containing this plugin.
   * @param _sdf Unused. Pointer to the SDF element that instantiated this plugin.
   */
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

private:
  //! Store a logger used to output messages to the screen.
  std::shared_ptr<spdlog::logger> _logger{};
};
}  // namespace gz_plugins
}  // namespace provant

#endif  // PROVANT_LOG_TEST_PLUGIN_H
