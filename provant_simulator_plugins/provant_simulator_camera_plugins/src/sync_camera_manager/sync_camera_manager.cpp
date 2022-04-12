/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sync_camera_manager.cpp
 * @brief This file contains the implementation of the SyncCameraManager class.
 *
 * See the sync_camera_manger.h header for more detailed documentation.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_camera_plugins/sync_camera_manager/sync_camera_manager.h"

#include <gazebo/common/Events.hh>
#include <gazebo/msgs/int.pb.h>
#include <gazebo/msgs/response.pb.h>

#include "provant_simulator_camera_plugins/sync_camera_manager/camera_request_ids.h"

using provant::camera_plugins::SyncCameraManager;

GZ_REGISTER_WORLD_PLUGIN(provant::camera_plugins::SyncCameraManager);

SyncCameraManager::SyncCameraManager() : gazebo::WorldPlugin()
{
}

void SyncCameraManager::Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr /*_sdf*/)
{
  _logMsg = "[SyncCameraManager: " + GetHandle() + "] ";

  // Store the pointer to the simulation world
  this->_world = _world;

  gzmsg() << _logMsg << "Starting the plugin loading process.\n";

  // Configure the gazebo transport
  if (!SetupTransportNode())
    return;

  gzmsg() << _logMsg << "Connecting to the WorldUpdateEnd signal.\n";
  _worldUpdateEndConn = gazebo::event::Events::ConnectWorldUpdateEnd([this]() { this->OnWorldUpdateEnd(); });

  gzmsg() << _logMsg << "Plugin loaded successfully.\n";
}

bool SyncCameraManager::SetupTransportNode()
{
  gzmsg() << _logMsg << "Configuring the gazebo transport node and topics for the plugin.\n";
  _gzNode = gazebo::transport::NodePtr(new gazebo::transport::Node{});
  if (!_gzNode)
  {
    gzerr() << _logMsg << "An error ocurred while trying to create a gazebo transport node for the plugin.\n";
    return false;
  }
  _gzNode->Init();
#if GAZEBO_MAJOR_VERSION >= 11
  if (!_gzNode->IsInitialized())
  {
    gzerr() << _logMsg << "An error ocurred while trying to initialize the transport node.\n";
    return false;
  }
#endif

  _responsePublisher = _gzNode->Advertise<gazebo::msgs::Response>("~/response");
  if (!_responsePublisher)
  {
    gzerr() << _logMsg << "An error ocurred while trying to create a connection to the ~/response gazebo topic.\n";
    return false;
  }

  _requestSubscriber =
      _gzNode->Subscribe("~/request", &provant::camera_plugins::SyncCameraManager::OnRequest, this, true);
  if (!_requestSubscriber)
  {
    gzerr() << _logMsg << "An error ocurred while trying to connect to the ~/request gazebo topic.\n";
    return false;
  }

  return true;
}

void SyncCameraManager::OnRequest(const ConstRequestPtr& req)
{
  const auto req_name = req->request();
  if (req_name == provant::camera_plugins::request_names::ENABLE_RECORDER)
  {
    OnEnableRecorderRequest(req);
  }
  else if (req_name == provant::camera_plugins::request_names::DISABLE_RECORDER)
  {
    OnDisableRecorderRequest(req);
  }
}

void SyncCameraManager::OnEnableRecorderRequest(const ConstRequestPtr& req)
{
  if (req->has_data() && !req->data().empty())
  {
    const auto recorder_name = req->data();

    // Send the response to the initial request. This must be done before the request to the enable_on service in order
    // to prevent a deadlock.
    SendResponse(req, provant::camera_plugins::response_status::SUCCESS);

    {
      std::scoped_lock<std::mutex> lock(_enableOnThreadsMutex);
      _enableOnThreads.push_back(std::thread{ [this, recorder_name]() { this->RequestInitialStep(recorder_name); } });
    }
  }
  else
  {
    // Handle error: Request failed
    gzerr() << _logMsg << "An invalid " << provant::camera_plugins::request_names::ENABLE_RECORDER
            << " request was received.\n";
    SendResponse(req, provant::camera_plugins::response_status::ERROR);
  }
}

void SyncCameraManager::RequestInitialStep(std::string recorder_name)
{
  // Query the step number in which the recorder must be enabled
  const auto res =
      gazebo::transport::request(_world->Name(), provant::camera_plugins::request_names::ENABLE_ON, recorder_name);
  if (res->response() == provant::camera_plugins::response_status::SUCCESS)
  {
    if (res->has_type() && !res->type().empty() && res->has_serialized_data() && !res->serialized_data().empty())
    {
      auto intMsg = gazebo::msgs::Int{};

      if (res->type() == intMsg.GetTypeName())
      {
        intMsg.ParseFromString(res->serialized_data());

        const auto step_number = intMsg.data();

        if (step_number >= 0)
        {
          // If successfull, log the action, and add the recorder to the active list.
          gzmsg() << _logMsg << "Enabling the " << recorder_name << " on step number " << step_number << ".\n";
          std::scoped_lock<std::mutex> lock(_activeRecordersMutex);
          _activeRecorders[recorder_name] = step_number;
        }
        else
        {
          // Handle Error: invalid step number
          gzerr() << _logMsg << "The " << recorder_name << " recorder request activation on a illegal step number ("
                  << step_number << "). The step number must be a positive integer, therefore, the " << recorder_name
                  << " will not be enabled.\n";
        }
      }
      else
      {
        // Handle Error: Invalid response data type
        gzerr() << _logMsg << "The " << recorder_name << " responded to the "
                << provant::camera_plugins::request_names::ENABLE_ON << " request with an invalid response data type ("
                << res->type() << "). The response to the " << provant::camera_plugins::request_names::ENABLE_ON
                << " request must contain a serialized message of type int. Due to this error the " << recorder_name
                << " will not be enabled.\n";
      }
    }
    else
    {
      // Handle Error: Empty response
      gzerr() << _logMsg << "The " << recorder_name << " responded to the "
              << provant::camera_plugins::request_names::ENABLE_ON
              << " request with an empty data packet. The response of this request must contain a serialized message "
                 "of type int containing the step number in which the recorder must be enabled. Due to this error "
                 "the "
              << recorder_name << " recorder will not be enabled.\n";
    }
  }
}

void SyncCameraManager::OnDisableRecorderRequest(const ConstRequestPtr& req)
{
  if (req->has_data() && !req->data().empty())
  {
    std::scoped_lock<std::mutex> lock(_activeRecordersMutex);

    const auto recorder_name = req->data();

    _activeRecorders.erase(recorder_name);
    SendResponse(req, provant::camera_plugins::response_status::SUCCESS);
  }
  else
  {
    gzerr << _logMsg << "Received a " << provant::camera_plugins::request_names::DISABLE_RECORDER
          << " request call with an empty data field.\n";
    SendResponse(req, provant::camera_plugins::response_status::ERROR);
  }
}

void SyncCameraManager::SendResponse(const ConstRequestPtr& req, const std::string response)
{
  auto res_msg = gazebo::msgs::Response{};
  res_msg.set_id(req->id());
  res_msg.set_request(req->request());
  res_msg.set_response(response);

  _responsePublisher->Publish(res_msg);
}

void SyncCameraManager::OnWorldUpdateEnd()
{
  std::scoped_lock<std::mutex> lock(_activeRecordersMutex);

  for (const auto& [recorder_name, initial_step] : _activeRecorders)
  {
    if (_world->Iterations() >= static_cast<uint32_t>(initial_step))
    {
      const auto res =
          gazebo::transport::request(_world->Name(), provant::camera_plugins::request_names::GRAB_FRAME, recorder_name);
    }
  }
}
