/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file rossink.h
 * @brief
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#pragma once

#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/base_sink.h>

#include <ros/console.h>

#include <mutex>

namespace provant
{
namespace log
{
template <typename Mutex>
class ROSNamedSinkBase : public spdlog::sinks::base_sink<Mutex>
{
public:
  explicit ROSNamedSinkBase(const std::string& name) : spdlog::sinks::base_sink<Mutex>(), _name(name)
  {
    spdlog::sinks::base_sink<Mutex>::set_pattern("[%n] %v");
  }

  explicit ROSNamedSinkBase(const std::string& name, std::unique_ptr<spdlog::formatter> formatter)
    : spdlog::sinks::base_sink<Mutex>(formatter), _name(name)
  {
    spdlog::sinks::base_sink<Mutex>::set_pattern("[%n] %v");
  }

  const std::string& getLogName() const
  {
    return _name;
  }

protected:
  std::string formatMsg(const spdlog::details::log_msg& msg)
  {
    spdlog::memory_buf_t formatted;
    spdlog::sinks::base_sink<Mutex>::formatter_->format(msg, formatted);
    return fmt::to_string(formatted);
  }

  void sink_it_(const spdlog::details::log_msg& msg) override
  {
    using spdlog::level::level_enum;
    auto level = msg.level;
    switch (msg.level)
    {
      case level_enum::trace:
        ROS_DEBUG_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::debug:
        ROS_DEBUG_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::info:
        ROS_INFO_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::warn:
        ROS_WARN_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::err:
        ROS_ERROR_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::critical:
        ROS_FATAL_NAMED(_name, "%s", formatMsg(msg));
        break;
      case level_enum::off:
        break;
      case level_enum::n_levels:
        break;
    }
  }

  void flush_() final
  {
  }

private:
  const std::string _name;
};

using ROSNamedSinkMT = ROSNamedSinkBase<std::mutex>;
using ROSNamedSink = ROSNamedSinkBase<spdlog::details::null_mutex>;

}  // namespace log
}  // namespace provant
