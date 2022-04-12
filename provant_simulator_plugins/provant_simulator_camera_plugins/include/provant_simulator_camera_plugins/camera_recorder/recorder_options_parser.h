/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file recorder_options_parser.h
 * @brief This file contains the implementation of the RecorderOptions class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_RECORDER_OPTIONS_PARSER_H
#define PROVANT_RECORDER_OPTIONS_PARSER_H

#include <sdf/sdf.hh>

#include <spdlog/spdlog.h>

#include <memory>
#include <optional>

#include <provant_simulator_log_utils/gzlog/gzlog.h>
#include <provant_simulator_sdf_parser/sdf_parser.h>

#include "camera_recorder.h"

namespace provant
{
namespace camera_plugins
{
struct RecorderOptions;

/**
 * @brief The RecorderOptionsParser class will parse the recorder options from a SDF element, and if successfull provide
 * a RecorderOptions object with the parsed options.
 *
 */
class RecorderOptionsParser
{
public:
  /**
   * @brief Construct a new Recorder Options Parser object.
   *
   * @param sdf SDF element that contains the plugin configuration.
   * @param step_duration The simulation step time.
   * @param logger Logger instance used to output the messages generated by this object.
   */
  RecorderOptionsParser(sdf::ElementPtr sdf, double step_duration,
                        std::shared_ptr<spdlog::logger> logger = std::make_shared<provant::log::GazeboLogger>("Recorder"
                                                                                                              "OptionsP"
                                                                                                              "arser"));
  /**
   * @brief Parse the options from the provided SDF element, and if the parse is successfull returns a RecorderOptions
   * object.
   *
   * @return std::optional<RecorderOptions> If the parsing is successfull a RecorderOptions object is returned, and no
   * value is returned otherwise.
   */
  std::optional<RecorderOptions> getOptions() const;

protected:
  /**
   * @brief Parse the options from the plugin SDF configuration.
   *
   * @param sdf SDF element that contains the plugin configuration.
   * @param step_duration The simulation step time.
   * @return true If parsing the options was successfull.
   * @return false Otherwise.
   */
  bool parseOptions(sdf::ElementPtr sdf, double step_duration);
  /**
   * @brief Parse the destination file name of the resulting recording.
   *
   * This method will check if the destination is writable, and assemble a full path for the file relative to the
   * MATLAB_DATA path.
   *
   * @param parser SDFParser to read the parameter from.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseDestinationPath(const SDFParser& parser);
  /**
   * @brief Parse the width of the resulting image.
   *
   * If the width element is not present, the width of the first camera frame captured is used.
   *
   * @param parser SDFParser to read the parameter from.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseWidth(const SDFParser& parser);
  /**
   * @brief Parse the height of the resulting image.
   *
   * If the height element is not present, the height of the first camera frame captured is used.
   *
   * @param parser SDFParser to read the parameter from.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseHeight(const SDFParser& parser);
  /**
   * @brief Parse the number of frames per second of the resulting recording.
   *
   * If the fps element is not present in the SDF configuration, the fps is the simulation frequency.
   *
   * @param parser SDFParser to read the parameter from.
   * @param step_duration The duration of a simulation step. Used as a default value if the fps element is not provided.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseFps(const SDFParser& parser, double step_duration);
  /**
   * @brief Parse the num_frames parameter. The num_frames parameter indicates the maximum number of frames that should
   * be recorded.
   *
   * @param parser SDFParser to read the parameter from.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseNumFrames(const SDFParser& parser);
  /**
   * @brief Parse the fourcc parameter. This parameter indicates the codec used by OpenCV to record the video.
   *
   * @param parser SDFParser to read the parameter from.
   * @return true If the element was found and read successfully, or if the default value is used.
   * @return false Otherwise.
   */
  bool parseFourcc(const SDFParser& parser);

  /**
   * @brief Creates a log message containing an error message.
   *
   * @param e SDFStatus that originated the error message.
   * @param elementName Name of the element in which the error ocurred.
   * @param msg Additional message to include in the error log.
   */
  void errorMessage(const SDFStatus& e, const std::string elementName, const std::string msg = std::string{ "" });

private:
  /// Store the options parsed from the SDF
  RecorderOptions _options;
  /// Log used to emit the messages from this object.
  std::shared_ptr<spdlog::logger> _logger;
  /// Indicates if the parsing was successfull or not
  bool _success = false;
};
}  // namespace camera_plugins
}  // namespace provant

#endif  // PROVANT_RECORDER_OPTIONS_PARSER_H