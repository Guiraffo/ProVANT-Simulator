/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gzconfig_reader.h
 * @brief This file contains the implementation of the GazeboConfigReader class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_gzconfig_reader/gzconfig_reader.h"

#include <gazebo/msgs/gz_string.pb.h>
#include <gazebo/transport/transport.hh>

#include <memory>

GazeboConfigReader::GazeboConfigReader(const std::string& worldName) : _worldName(worldName)
{

}

const std::string& GazeboConfigReader::GetWorldName() const
{
  return _worldName;
}

ConfigReader *GazeboConfigReader::GetConfigReader() const
{
  // Send a request for the provant_config service
  auto res = gazebo::transport::request(_worldName, "provant_config");

  auto reader = std::make_unique<ConfigReader>();

  // Get the data from the message
  if(res->response() != "success")
  {
    return nullptr;
  }

  // Parse the content of the file
  auto strMsg = gazebo::msgs::GzString();
  strMsg.ParseFromString(res->serialized_data());

  // Read the content to the config reader
  if(!reader->loadFromString(strMsg.data()))
  {
    return nullptr;
  }

  return reader.release();
}
