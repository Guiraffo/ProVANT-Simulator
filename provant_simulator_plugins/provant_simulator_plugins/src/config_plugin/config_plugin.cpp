/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file config_plugin.cpp
 * @brief This file contains the implementation of the ConfigPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/config_plugin/config_plugin.h"

#include <cstring>
#include <fstream>
#include <streambuf>

#include <boost/filesystem.hpp>

#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/msgs/request.pb.h>
#include <gazebo/common/Events.hh>

#include <provant_simulator_xml_reader/config_reader.h>

using provant::plugins::ConfigPlugin;

GZ_REGISTER_SYSTEM_PLUGIN(provant::plugins::ConfigPlugin)

void ConfigPlugin::Load(int argc, char** argv)
{
  // Setup the logging
  _logMsg = "[ConfigPlugin: " + GetHandle() + "] ";
  gzdbg << _logMsg << "Starting plugin initialization.\n";

  const auto confName = std::string("--provant_config");
  const auto confStr = confName.c_str();

  // Loop trough the variables and look for the --provant_config option
  int pos = -1;
  for (int i = 0; i < argc; i++)
  {
    if (strncmp(confStr, argv[i], confName.length()) == 0)
    {
      pos = i;
      break;
    }
  }

  if (pos == -1)
  {
    gzwarn << _logMsg << "The " << confName << " was not found! The config plugin will not be loaded.\n";
    return;
  }

  // Get
  const auto fullOpt = std::string(pos[argv]);
  const auto configPath = ParseConfigPath(fullOpt);
  if (configPath.empty())
  {
    gzerr << "The --provant_config=path option was incorrectly specified. This option must contain the path of the "
             "config.xml file for the current simulation.\n";
    return;
  }
  gzdbg << _logMsg << "Found the " << confName << " with value " << configPath << " \n";

  // Verify the file
  if (!VerifyConfigPath(configPath))
  {
    // Error messages are handled by the VerifyConfigPath option.
    return;
  }

  // Read the contents of the file
  if (!ReadConfigFile(configPath))
  {
    gzerr << "Error while reading the config.xml file at \"" << configPath << "\"\n";
    return;
  }

  // Check if the ConfigReader can parse the file
  if (!VerifyConfigReader())
  {
    gzerr << "An error ocurred while parsing the config.xml file at \"" << configPath
          << "\". Please verify if the file is a valid config.xml file and try again.\n";
    return;
  }

  // Prepare the response message
  PrepareResponseMessage();

  // Connect to the topics
  ConnectToWorldCreated();
}

void ConfigPlugin::Init()
{
  gzdbg << _logMsg << "Finished the plugin loading process successfully.\n";
}

std::string ConfigPlugin::ParseConfigPath(const std::string& configOpt) const
{
  // Find the equal sign
  const std::string delimiter = "=";

  // Check if the delimiter was found
  const auto pos = configOpt.find(delimiter);
  if (pos == std::string::npos)
    return "";

  // Remove the first part from the resultstring
  return configOpt.substr(pos + delimiter.length());
}

bool ConfigPlugin::VerifyConfigPath(const std::string& path) const
{
  namespace fsystem = boost::filesystem;
  const auto bpath = fsystem::path(path);

  try
  {
    // Check that the path exists
    if (!fsystem::exists(bpath))
    {
      gzerr << "A file with the specified path \"" << path << "\" does not exists.\n";
      return false;
    }

    // Check that the path is a file
    if (!fsystem::is_regular_file(bpath))
    {
      gzerr << "The specified config.xml path \"" << path << "\" is not a file.\n";
      return false;
    }

    // Check that the path is readable
    if (!fsystem::status(bpath).permissions() & fsystem::perms::owner_read)
    {
      gzerr << "The specified config.xml path \"" << path << "\" is not readable.\n";
      return false;
    }
  }
  catch (const fsystem::filesystem_error& e)
  {
    gzerr << "An unhandled exception with the following message: \"" << e.what()
          << "\" was caught while verifying the provided config.xml file path \"" << path << "\"\n";
    return false;
  }

  return true;
}

bool ConfigPlugin::ReadConfigFile(const std::string& path)
{
  std::fstream file(path);
  if (!file.good())
    return false;

  _configFileContents.assign((std::istreambuf_iterator<char>(file)), (std::istreambuf_iterator<char>()));

  return !_configFileContents.empty();
}

bool ConfigPlugin::VerifyConfigReader() const
{
  ConfigReader reader;
  return reader.loadFromString(_configFileContents);
}

void ConfigPlugin::PrepareResponseMessage()
{
  using gazebo::msgs::GzString;

  // Encode the content of the config file as a GzString message
  GzString strMsg;
  strMsg.set_data(_configFileContents);

  // Set the status of the response
  _responseMessage.set_response("success");
  _responseMessage.set_type(strMsg.GetTypeName());

  auto serialized_data_ptr = _responseMessage.mutable_serialized_data();
  strMsg.SerializeToString(serialized_data_ptr);
}

bool ConfigPlugin::SetupGazeboTopic()
{
  using gazebo::msgs::Response;
  using gazebo::transport::Node;
  using gazebo::transport::NodePtr;

  // Config the topic
  gzdbg << _logMsg << "Configuring node for the plugin.\n";
  _gzNode = NodePtr(new gazebo::transport::Node());
  if (_gzNode.get() == nullptr)
  {
    gzerr << "Error while trying to create a gazebo topic for the plugin\n";
    return false;
  }
  _gzNode->Init();

  _responsePublisher = _gzNode->Advertise<Response>("~/response");
  if (!_responsePublisher)
  {
    gzerr << "Error while advertising to the ~/response topic.\n";
    return false;
  }

  _requestSubscriber = _gzNode->Subscribe("~/request", &provant::plugins::ConfigPlugin::OnRequest, this, true);
  if (!_requestSubscriber)
  {
    gzerr << "Error while subscribing to the ~/request topic.\n";
    return false;
  }

  gzdbg << _logMsg << "Connected to the request and response topics.\n";
  return true;
}

void ConfigPlugin::ConnectToWorldCreated()
{
  gzdbg << _logMsg << "Connecting to the world created signal.\n";
  _worldCreatedPtr = gazebo::event::Events::ConnectWorldCreated([this](std::string arg) {
    (void)arg;  // Silence unused variable warning
    this->OnWorldCreated();
  });
}

void ConfigPlugin::OnRequest(ConstRequestPtr& req)
{
  using gazebo::msgs::Response;

  // If the request is equal to provant_config, we must respond
  if (req->request() == "provant_config")
  {
    Response res;
    res.CopyFrom(_responseMessage);

    // Prepare a new response message
    res.set_id(req->id());
    res.set_request(req->request());

    // Publish the message
    _responsePublisher->Publish(res);
  }
}

void ConfigPlugin::OnWorldCreated()
{
  if (SetupGazeboTopic())
  {
    _worldCreatedPtr.reset();
  }
  else
  {
    gzerr << "Error while setting the node for the plugin, the plugin is not loaded.\n";
  }
}
