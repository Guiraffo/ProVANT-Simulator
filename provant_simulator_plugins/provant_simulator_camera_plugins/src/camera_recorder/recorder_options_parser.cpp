/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file recorder_options_parser.cpp
 * @brief This file contains the implementation of the RecorderOptionsParser class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_camera_plugins/camera_recorder/recorder_options_parser.h"

#include <opencv2/videoio.hpp>

#include <filesystem>
#include <sstream>

using provant::camera_plugins::RecorderOptions;
using provant::camera_plugins::RecorderOptionsParser;

RecorderOptionsParser::RecorderOptionsParser(sdf::ElementPtr sdf, double step_duration,
                                             std::shared_ptr<spdlog::logger> logger)
  : _logger(logger)
{
  _success = parseOptions(sdf, step_duration);
}

std::optional<RecorderOptions> RecorderOptionsParser::getOptions() const
{
  if (_success)
  {
    return _options;
  }
  return std::optional<RecorderOptions>{};
}

bool RecorderOptionsParser::parseOptions(sdf::ElementPtr sdf, double step_duration)
{
  SDFParser parser{ sdf };

  if (!parseDestinationPath(parser) || !parseFps(parser, step_duration))
  {
    _logger->critical("A critical error ocurred while parsing the camera options. The video cannot be recorded.");
    return false;
  }

  (void)parseWidth(parser);
  (void)parseHeight(parser);
  (void)parseNumFrames(parser);
  (void)parseFourcc(parser);

  return true;
}

bool RecorderOptionsParser::parseDestinationPath(const SDFParser& parser)
{
  try
  {
    const auto pathStr = parser.GetElementText("destination_filename");
    if (pathStr.empty())
    {
      _logger->error("The destination_filename element is empty. This element should contain the name of the resulting "
                     "videofile, either as an absolute path or the filename relative to the Matlab folder.");
      return false;
    }

    auto path = std::filesystem::path{ pathStr };
    if (!path.has_extension())
    {
      _logger->warn("The destination filename doesn't have an extension, the default .mp4 extension will be used.");
      path.append(".mp4");
    }

    if (path.is_relative())
    {
      // Get value of the TILT_MATLAB environment variable
      const auto tiltMatlabEnv = std::getenv("TILT_MATLAB");
      if (tiltMatlabEnv == nullptr)
      {
        _logger->error("An error ocurred while reading the value of the TILT_MATLAB environment variable. As the "
                       "informed destination_filename is a relative path, this value was required and a video cannot "
                       "be recorded.");
        return false;
      }

      const std::filesystem::path matlabPath{ tiltMatlabEnv };
      path = matlabPath / path;
    }

    // Check if the path names a file
    if (!path.has_filename())
    {
      _logger->error("The specified filename does not name a file. Please add the desired filename for the resulting "
                     "video.");
      return false;
    }

    // Check if the file location is writable
    auto dir{ path };
    dir.remove_filename();

    if (!std::filesystem::exists(dir))
    {
      _logger->error("The specified filename is inside an inexisting direcotory. Please create this directory or "
                     "choose another path.");
      return false;
    }

    _options.path = std::filesystem::absolute(path).string();
    return true;
  }
  catch (const SDFStatus& e)
  {
    errorMessage(e, "destination_filename",
                 "This element must contain an absolute path or the filename of the resulting video file.");
    return false;
  }
}

bool RecorderOptionsParser::parseWidth(const SDFParser& parser)
{
  if (parser.HasElement("width"))
  {
    try
    {
      const auto width = parser.GetElementUnsignedInt("width");
      _options.width = width;

      return true;
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "width",
                   "This element should contain a positive integer indicating the horizontal resolution (width) of the "
                   "resulting video.");
      return false;
    }
  }

  return true;
}

bool RecorderOptionsParser::parseHeight(const SDFParser& parser)
{
  if (parser.HasElement("height"))
  {
    try
    {
      _options.height = parser.GetElementUnsignedInt("height");
      return true;
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "height",
                   "This element should contain a positive integer indicating the vertical resolution (height) of the "
                   "resulting video.");
      return false;
    }
  }

  return true;
}

bool RecorderOptionsParser::parseFps(const SDFParser& parser, double step_duration)
{
  if (parser.HasElement("fps"))
  {
    try
    {
      const auto fps = parser.GetElementDouble("fps");
      if (fps <= 0)
      {
        _logger->error("A negative fps value was passed as a parameter. The fps must be a positive number indicating "
                       "the desired number of frames per second of the resulting video.");
        return false;
      }

      _options.fps = fps;
      return true;
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "fps",
                   "This element should contain the desired number of frames per second (fps) of the resulting video.");
      return false;
    }
  }

  _options.fps = 1.0 / step_duration;
  return true;
}

bool RecorderOptionsParser::parseNumFrames(const SDFParser& parser)
{
  if (parser.HasElement("num_frames"))
  {
    try
    {
      _options.maxFrames = parser.GetElementUnsignedInt("num_frames");
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "num_frames",
                   "This element should contain the maximum number of frames to record the video for.");
      return false;
    }
  }

  return true;
}

bool RecorderOptionsParser::parseFourcc(const SDFParser& parser)
{
  if (parser.HasElement("fourcc"))
  {
    try
    {
      const auto str = parser.GetElementText("fourcc");
      if (str.length() != 4)
      {
        _logger->error("The fourcc code should contain 4 characters.");
        return false;
      }

      _options.fourcc = cv::VideoWriter::fourcc(str.at(0), str.at(1), str.at(2), str.at(3));

      _logger->warn("Using a non-default codec. This is not guaranteed to work as it is not possible to check if a "
                    "given codec is accessible by OpenCV.");
      return true;
    }
    catch (const SDFStatus& e)
    {
      errorMessage(e, "fourcc", "This element should contain a four letter code ");
      return false;
    }
  }

  return true;
}

void RecorderOptionsParser::errorMessage(const SDFStatus& e, const std::string elementName, const std::string msg)
{
  std::stringstream ss;

  ss << "An error with message \"" << e.what() << "\" ocurred while reading the " << elementName << ". " << msg;

  _logger->error(ss.str());
}
