/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gzlog.h
 * @brief This file contains the template of a spdlog sink that allows the integration of the spdlog library with the
 * Gazebo standard logging facilities.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#pragma once

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>
#include <spdlog/spdlog.h>

#include <gazebo/common/Console.hh>

#include <mutex>

namespace provant
{
namespace log
{
/**
 * @brief GazeboLogSinkBase.
 * @tparam Mutex Mutex type used to enable or disable thread-safety of the sink.
 *
 * Thread-safe sinks must receive a Mutex type similar to std::mutex, and sinks that do not require
 * such feature must receive an empty object that implements the mutex concept.
 */
template <typename Mutex>
/**
 * @brief Provides an SPDLog sink that outputs the messages to the Gazebo Console output.
 *
 * This sink uses the Gazebo Console methods instead of the macros to provide a consistent output format for the
 * messages. Some of the macros output the message with the originating line and file information and others do not.
 * This inconsistent behavior is strange, and furthermore this information is not that helpfull and does not allow
 * unique identification of the plugin instance originating the message, therefore, it was removed from the message
 * outputs.
 *
 */
class GazeboLogSinkBase : public spdlog::sinks::base_sink<Mutex>
{
public:
  /**
   * @brief Construct a new Gazebo Log Sink Base object.
   *
   * @param name Name of the sink. Used to uniquely identify the message in the log output.
   */
  explicit GazeboLogSinkBase(const std::string& name = "") : spdlog::sinks::base_sink<Mutex>()
  {
    spdlog::sinks::base_sink<Mutex>::set_pattern("[%n] %v");
  }

protected:
  /**
   * @brief Formats the message objects received to a string ready to ouput to the Gazebo console outputs.
   *
   * @param msg Message object to format.
   * @return std::string Formatted log message.
   */
  std::string formatMsg(const spdlog::details::log_msg& msg) const
  {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
    return fmt::to_string(formatted);
  }

  /**
   * @brief Sinks messages objects and sends them to the appropriate Gazebo console output.
   *
   * As Gazebo does not have output methods to handle all of the severity levels provided by the spdlog library,
   * the following mapping is used:
   *
   * - trace -> dbg
   * - debug -> dbg
   * - info -> msg
   * - warning -> warn
   * - error -> err
   * - critical -> err
   *
   * @param msg Message object to output.
   */
  void sink_it_(const spdlog::details::log_msg& msg) override
  {
    using gazebo::common::Console;
    using spdlog::level::level_enum;

    switch (msg.level)
    {
      case level_enum::trace:
        Console::dbg() << formatMsg(msg);
        break;
      case level_enum::debug:
        Console::dbg() << formatMsg(msg);
        break;
      case level_enum::info:
        Console::msg() << formatMsg(msg);
        break;
      case level_enum::warn:
        Console::warn() << formatMsg(msg);
        break;
      case level_enum::err:
        Console::err() << formatMsg(msg);
        break;
      case level_enum::critical:
        Console::err() << formatMsg(msg);
        break;
      case level_enum::off:
        break;
      case level_enum::n_levels:
        break;
    }
  }

  /**
   * @brief Flush the message buffer to the output.
   *
   * As Gazebo logging does not have any buffer, this method doesn't do anything, but is provided as it is required by
   * the super class.
   */
  void flush_() override
  {
  }
};

using GazeboLogSinkMT = GazeboLogSinkBase<std::mutex>;
using GazeboLogSink = GazeboLogSinkBase<spdlog::details::null_mutex>;

/**
 * @brief Creates an SPDLog logger object that contains a GazeboLogSink.
 *
 * This logger will output messages to the console using the default Gazebo logging mechanism, it is similar to the
 * use of the methods gzdbg(), gzmsg() and gzerr().
 *
 */
class GazeboLogger : public spdlog::logger
{
public:
  // Inherit constructors from spdlog::logger
  using spdlog::logger::logger;
  /**
   * @brief Construct a new Gazebo Logger object.
   *
   * @param name Name of the logger. Used to uniquely identify this logger in the log outputs.
   */
  explicit GazeboLogger(std::string name);
  virtual ~GazeboLogger() = default;
};

}  // namespace log
}  // namespace provant
