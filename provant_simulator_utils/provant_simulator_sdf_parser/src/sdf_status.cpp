/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_status.cpp
 * @brief This file contains the implementation of the SDFStatus classes.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_sdf_parser/sdf_status.h"

#include <sstream>

SDFStatus::SDFStatus(bool isError, const std::string msg)
  : std::runtime_error(msg.c_str()), _isError(isError), _msg(msg)
{
}

SDFStatus::~SDFStatus()
{
}

const char* SDFStatus::what() const noexcept
{
  return _msg.c_str();
}

bool SDFStatus::isError() const
{
  return _isError;
}

const std::string& SDFStatus::errorMessage() const
{
  return _msg;
}

OkStatus::OkStatus() : SDFStatus(false, ""){};

NotFoundError::NotFoundError(const std::string& name, const std::string& type) : SDFStatus(true)
{
  std::stringstream str;
  str << "The " << type << " named \"" << name
      << "\" was not found in this plugin SDF description. Please verify that if this " << type
      << " is correctly defined and try again.";
  _msg = str.str();
}

ElementNotFoundError::ElementNotFoundError(const std::string& name) : NotFoundError(name, "element")
{
}

AttributeNotFoundError::AttributeNotFoundError(const std::string& name) : NotFoundError(name, "attribute")
{
}

XMLSyntaxError::XMLSyntaxError(const std::string& msg) : SDFStatus(true, msg)
{
}

ConversionError::ConversionError(const std::string& element, const std::string& originalContent,
                                 const std::string& type, const std::string& extraMsg)
  : SDFStatus(true)
{
  std::stringstream str;
  str << "An error ocurred while trying to convert the value (" << originalContent << ") of the " << element
      << " element to a " << type << ".";
  if (!extraMsg.empty())
  {
    str << " " << extraMsg;
  }
  _msg = str.str();
}

NullPointerError::NullPointerError(const std::string& method) : SDFStatus(true)
{
  std::stringstream str;
  str << "A nullpointer was passed as the output parameter to the " << method << " method.";
  _msg = str.str();
}
