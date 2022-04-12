/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file user_camera_sync_recorder.cpp
 * @brief This file contains the implementation of the UserCameraSyncRecorder class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "user_camera_sync_recorder.h"

#include <gazebo/msgs/int.pb.h>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/physics/World.hh>

#include <ros/ros.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <provant_simulator_sdf_parser/sdf_parser.h>

#include <vector>

#include "provant_simulator_camera_plugins/sync_camera_manager/camera_request_ids.h"
#include "provant_simulator_camera_plugins/camera_recorder/recorder_options_parser.h"

using provant::camera_plugins::UserCameraSyncRecorder;

GZ_REGISTER_GUI_PLUGIN(provant::camera_plugins::UserCameraSyncRecorder);

UserCameraSyncRecorder::UserCameraSyncRecorder() : GUIPlugin()
{
  SetupUi();
}

void UserCameraSyncRecorder::Load(sdf::ElementPtr _sdf)
{
  _logMsg = "[UserCameraSyncRecorder: " + GetHandle() + "] ";

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Starting the plugin loading process.");

  SDFParser parser{ _sdf };
  if (parser.HasElement("enabled"))
  {
    try
    {
      const auto res = parser.GetElementBool("enabled");
      if (res == false)
      {
        ROS_WARN_STREAM_NAMED(PLUGIN_ID, _logMsg << "The plugin is disabled.");
        return;
      }
    }
    catch (const SDFStatus& e)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to read the enabled element from the "
                                                   "plugin SDF configuration.");
      return;
    }
  }

  if (!SetupTransport())
    return;

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Connecting to the PostRender gazebo signal.");
  _onPostRenderConn = gazebo::event::Events::ConnectPostRender([this]() { this->OnPostRender(); });

  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Plugin loaded successfully.");
}

void UserCameraSyncRecorder::SetupUi()
{
  // Apply the gazebo default stylesheet
  setStyleSheet("QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");
  // Hide the overlay
  move(0, 0);
  resize(1, 1);
  hide();
}

bool UserCameraSyncRecorder::SetupTransport()
{
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Configuring the gazebo transport node and topics for the plugin.");
  _gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node{});
  if (!_gzNode)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to create a gazebo transport node for "
                                                 "the plugin.");
    return false;
  }
  _gzNode->Init();
#if GAZEBO_MAJOR_VERSION >= 11
  if (!_gzNode->IsInitialized())
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to initialize the transport node.");
    return false;
  }
#endif

  _responsePublisher = _gzNode->Advertise<gazebo::msgs::Response>("~/response");
  if (!_responsePublisher)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to create a connection to the "
                                                 "~/response gazebo topic.");
    return false;
  }

  _requestSubscriber =
      _gzNode->Subscribe("~/request", &provant::camera_plugins::UserCameraSyncRecorder::OnRequest, this, true);
  if (!_requestSubscriber)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to connect to the ~/request gazebo "
                                                 "topic.");
    return false;
  }

  return true;
}

void UserCameraSyncRecorder::OnRequest(const ConstRequestPtr& req)
{
  // Check if the request is for this plugin instance
  if (req->has_data() && req->data() == GetHandle())
  {
    // Check if the request is for a service provided by this plugin
    if (req->request() == provant::camera_plugins::request_names::ENABLE_ON)
    {
      OnEnableOnRequest(req);
    }
    else if (req->request() == provant::camera_plugins::request_names::GRAB_FRAME)
    {
      OnGrabFrameRequest(req);
    }
  }
}

void UserCameraSyncRecorder::OnGrabFrameRequest(const ConstRequestPtr& req)
{
  ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "A new frame was requested");

  auto res_msg = gazebo::msgs::Response{};

  if (_recorder)
  {
    if (_recorder->GrabFrame())
    {
      res_msg.set_response(provant::camera_plugins::response_status::SUCCESS);
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to record a new frame.");
      res_msg.set_response(provant::camera_plugins::response_status::ERROR);
    }
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "A grab frame request was received, but the plugin still does not "
                                                 "have a initialized recorder object.");
    res_msg.set_response(provant::camera_plugins::response_status::ERROR);
  }

  SendResponse(req, res_msg);
}

void UserCameraSyncRecorder::OnEnableOnRequest(const ConstRequestPtr& req)
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Received a request for the enable on service.");

  auto res_msg = gazebo::msgs::Response{};
  res_msg.set_response(provant::camera_plugins::response_status::SUCCESS);

  auto int_msg = gazebo::msgs::Int{};
  int_msg.set_data(_enableOn);

  // Add the encoded int message to the response
  res_msg.set_type(int_msg.GetTypeName());
  auto serialized_data_ptr = res_msg.mutable_serialized_data();
  int_msg.SerializeToString(serialized_data_ptr);

  SendResponse(req, res_msg);
}

void UserCameraSyncRecorder::SendResponse(const ConstRequestPtr& req, gazebo::msgs::Response& res)
{
  res.set_id(req->id());
  res.set_request(req->request());

  _responsePublisher->Publish(res);
}

void UserCameraSyncRecorder::OnPostRender()
{
  std::scoped_lock<std::mutex> lock(_recorderMutex);

  if (!_recorder)
  {
    SetupCamera();
  }
}

void UserCameraSyncRecorder::SetupCamera()
{
  if (_recorder)
    return;

  auto _userCam = gazebo::gui::get_active_camera();
  if (_userCam)
  {
    const auto world = gazebo::physics::get_world(gazebo::gui::get_world());
    if (!world)
    {
      ROS_FATAL_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while trying to get a pointer to the simulation "
                                                   "world.");
      return;
    }

    const std::string logFormat{ "[%Y/%m/%d-%T.%e] [UserCameraSyncRecorder: " + GetHandle() + "] [%l] %v" };
    auto stdoutSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    stdoutSink->set_pattern(logFormat);

    auto stderrSink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    stderrSink->set_pattern(logFormat);

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(stdoutSink);
    sinks.push_back(stderrSink);

    auto logger = std::make_shared<spdlog::logger>("console", std::begin(sinks), std::end(sinks));

    using provant::camera_plugins::RecorderOptionsParser;
    RecorderOptionsParser parser{ _sdfElement, world->Physics()->GetMaxStepSize(), logger };
    const auto opts = parser.getOptions();

    if (!opts.has_value())
    {
      logger->critical("An error ocurred while parsing the options for this recorder.");
      return;
    }

    auto recorderOpts = opts.value();
    recorderOpts.width = recorderOpts.width.value_or(_userCam->ImageWidth());
    recorderOpts.height = recorderOpts.height.value_or(_userCam->ImageHeight());

    _recorder = std::make_unique<CameraRecorder>(_userCam, recorderOpts, _logMsg, PLUGIN_ID);
    if (!_recorder->SetupCamera())
    {
      _recorder.reset();
      ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "An error ocurred while configuring the user camera.");
      return;
    }

    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "The camera is configured, dropping connection to the PostRender "
                                                 "signal");
    _onPostRenderConn.reset();

    ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Defering the recorder registration process.");
    _registrationThread = std::thread{ [this]() { this->RegisterRecorder(); } };
  }
}

void UserCameraSyncRecorder::RegisterRecorder()
{
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "Sending the registration request.");
  ROS_DEBUG_STREAM_NAMED(PLUGIN_ID, _logMsg << "World name: " << gazebo::gui::get_world() << ".");

  const auto res = gazebo::transport::request(gazebo::gui::get_world(),
                                              provant::camera_plugins::request_names::ENABLE_RECORDER, GetHandle());
  if (res->response() == provant::camera_plugins::response_status::SUCCESS)
  {
    ROS_INFO_STREAM_NAMED(PLUGIN_ID, _logMsg << "Recorder registered successfully.");
  }
  else
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID,
                           _logMsg << "An error ocurred while trying to enable the " << GetHandle() << " recorder.");
  }
}
