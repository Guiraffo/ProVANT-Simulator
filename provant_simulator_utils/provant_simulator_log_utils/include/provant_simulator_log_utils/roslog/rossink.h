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
class ROSSinkBase : public spdlog::sinks::base_sink<Mutex>
{
public:
  ROSSinkBase() : spdlog::sinks::base_sink<Mutex>()
  {
    // set_pattern("[%n] %v");
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
        ROS_DEBUG("%s", formatMsg(msg));
        break;
      case level_enum::debug:
        ROS_DEBUG("%s", formatMsg(msg));
        break;
      case level_enum::info:
        ROS_INFO("%s", formatMsg(msg));
        break;
      case level_enum::warn:
        ROS_WARN("%s", formatMsg(msg));
        break;
      case level_enum::err:
        ROS_ERROR("%s", formatMsg(msg));
        break;
      case level_enum::critical:
        ROS_FATAL("%s", formatMsg(msg));
        break;
      case level_enum::off:
        break;
      case level_enum::n_levels:
        break;
    }
  }

  void flush_() override
  {
  }
};

using ROSSinkMT = ROSSinkBase<std::mutex>;
using ROSSink = ROSSinkBase<spdlog::details::null_mutex>;

}  // namespace log
}  // namespace provant
