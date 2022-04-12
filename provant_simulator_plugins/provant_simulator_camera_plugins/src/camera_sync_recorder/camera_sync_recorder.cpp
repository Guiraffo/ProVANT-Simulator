#include "provant_simulator_camera_plugins/camera_sync_recorder/camera_sync_recorder.h"

#include <ros/ros.h>

#include <spdlog/sinks/stdout_color_sinks.h>

#include "provant_simulator_camera_plugins/camera_recorder/recorder_options_parser.h"

using gazebo::SyncCameraRecorderPlugin;

GZ_REGISTER_SENSOR_PLUGIN(gazebo::SyncCameraRecorderPlugin);

void SyncCameraRecorderPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  GZ_ASSERT(_sensor, "Received an invalid sensor pointer");

  const std::string logPattern{ "[%Y/%m/%d-%T.%e] [SyncCameraRecorder:" + GetHandle() + "] [%l] %v" };

  auto stdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  stdoutSink->set_pattern(logPattern);

  auto stderrSink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
  stderrSink->set_pattern(logPattern);

  std::vector<spdlog::sink_ptr> sinks;
  sinks.push_back(stdoutSink);
  sinks.push_back(stderrSink);

  _logger = std::make_shared<spdlog::logger>("logger", std::begin(sinks), std::end(sinks));

  _logger->info("Starting the plugin loading process.");

  // Check if the provided sensor is a camera
  _cameraSensor = std::dynamic_pointer_cast<sensors::CameraSensor>(_sensor);
  if (!_cameraSensor)
  {
    _logger->critical("The plugin requires a camera sensor and received a sensor of a different type.");
    return;
  }

  auto camera = _cameraSensor->Camera();
  if (!camera)
  {
    _logger->critical("An error ocurred while trying to get a pointer to the sensor camera.");
    return;
  }

  // Parse the options
  const auto opts = ParseOptions(_sdf);
  if (!opts.has_value())
  {
    _logger->critical("An error ocurred while trying to parse the plugin configurations from the SDF. The plugin will "
                      "not be loaded.");
    return;
  }

  auto recOpts = opts.value();
  recOpts.width = recOpts.width.value_or(camera->ImageWidth());
  recOpts.height = recOpts.height.value_or(camera->ImageHeight());

  _recorder = std::make_unique<provant::camera_plugins::CameraRecorder>(
      camera, recOpts, "[SynCameraRecorderPlugin: " + GetHandle() + "] ", "sync_camera_recorder");
  if (_recorder == nullptr || !_recorder->SetupCamera())
  {
    _recorder.release();
    _logger->critical("An error ocurred while tyring to setup the camera.");
    return;
  }

  _logger->debug("Connecting to the world update end signal.");
  _onWorldUpdateEndConn = event::Events::ConnectWorldUpdateEnd([this]() { this->OnWorldUpdate(); });

  _logger->info("Plugin loaded successfully.");
}

std::optional<provant::camera_plugins::RecorderOptions>
SyncCameraRecorderPlugin::ParseOptions(sdf::ElementPtr _sdf) const
{
  using provant::camera_plugins::RecorderOptionsParser;

  const auto world = gazebo::physics::get_world(_cameraSensor->WorldName());

  RecorderOptionsParser parser{ _sdf, world->Physics()->GetMaxStepSize(), _logger };
  return parser.getOptions();
}

void SyncCameraRecorderPlugin::OnWorldUpdate()
{
  if (_recorder)
  {
    _recorder->GrabFrame();
  }
}
