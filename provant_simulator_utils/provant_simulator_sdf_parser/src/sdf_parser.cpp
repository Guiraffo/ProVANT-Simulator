/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_parser.cpp
 * @brief This file contains the implementation of the SDFParser class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_sdf_parser/sdf_parser.h"

#include <provant_simulator_parser_utils/type_conversion.h>

SDFParser::SDFParser(sdf::ElementPtr sdf) : _sdf(sdf)
{
}

SDFParser::~SDFParser()
{
}

sdf::ElementPtr SDFParser::GetElementPtr() const
{
  return _sdf;
}

bool SDFParser::HasAttribute(const std::string& name) const
{
  return _sdf->HasAttribute(name);
}

bool SDFParser::HasElement(const std::string& name) const
{
  return _sdf->HasElement(name);
}

SDFStatus SDFParser::GetAttributeValue(const std::string& name, gsl::not_null<std::string*> value) const noexcept
{
  value->clear();
  if (HasAttribute(name))
  {
    (*value) = _sdf->GetAttribute(name)->GetAsString();
    return OkStatus();
  }
  else
  {
    return AttributeNotFoundError(name);
  }
}

std::string SDFParser::GetAttributeValue(const std::string& name) const
{
  if (!HasAttribute(name))
  {
    throw AttributeNotFoundError(name);
  }
  return _sdf->GetAttribute(name)->GetAsString();
}

SDFStatus SDFParser::GetElementText(const std::string& name, gsl::not_null<std::string*> value) const noexcept
{
  value->clear();
  if (HasElement(name))
  {
    (*value) = _sdf->GetElement(name)->GetValue()->GetAsString();
    return OkStatus();
  }
  else
  {
    return ElementNotFoundError(name);
  }
}

std::string SDFParser::GetElementText(const std::string& name) const
{
  if (!HasElement(name))
  {
    throw ElementNotFoundError(name);
  }
  return _sdf->GetElement(name)->GetValue()->GetAsString();
}

bool SDFParser::GetElementBool(const std::string& name) const
{
  try
  {
    std::string strValue = GetElementText(name);
    try
    {
      return ParseBool(strValue);
    }
    catch (const std::exception& e)
    {
      throw ConversionError(name, strValue, "bool");
    }
  }
  catch (const SDFStatus& e)
  {
    throw e;
  }
}

int SDFParser::GetElementInt(const std::string& name) const
{
  try
  {
    std::string strValue = GetElementText(name);
    try
    {
      return ParseInt(strValue);
    }
    catch (const std::exception& e)
    {
      throw ConversionError(name, strValue, "int");
    }
  }
  catch (const SDFStatus& e)
  {
    throw e;
  }
}

unsigned int SDFParser::GetElementUnsignedInt(const std::string& name) const
{
  try
  {
    std::string strValue = GetElementText(name);
    try
    {
      return ParseUnsignedInt(strValue);
    }
    catch (const std::exception& e)
    {
      throw ConversionError(name, strValue, "unsigned int");
    }
  }
  catch (const SDFStatus& e)
  {
    throw e;
  }
}

float SDFParser::GetElementFloat(const std::string& name) const
{
  try
  {
    std::string strValue = GetElementText(name);
    try
    {
      return ParseFloat(strValue);
    }
    catch (const std::exception& e)
    {
      throw ConversionError(name, strValue, "float");
    }
  }
  catch (const SDFStatus& e)
  {
    throw e;
  }
}

double SDFParser::GetElementDouble(const std::string& name) const
{
  try
  {
    std::string strValue = GetElementText(name);
    try
    {
      return ParseDouble(strValue);
    }
    catch (const std::exception& e)
    {
      throw ConversionError(name, strValue, "double");
    }
  }
  catch (const SDFStatus& e)
  {
    throw e;
  }
}

SDFStatus SDFParser::GetElementBool(const std::string& name, gsl::not_null<bool*> value) const noexcept
{
  try
  {
    *value = GetElementBool(name);
    return OkStatus();
  }
  catch (const SDFStatus& e)
  {
    return e;
  }
}

SDFStatus SDFParser::GetElementInt(const std::string& name, gsl::not_null<int*> value) const noexcept
{
  try
  {
    *value = GetElementInt(name);
    return OkStatus();
  }
  catch (const SDFStatus& e)
  {
    return e;
  }
}

SDFStatus SDFParser::GetElementUnsignedInt(const std::string& name, gsl::not_null<unsigned int*> value) const noexcept
{
  try
  {
    *value = GetElementUnsignedInt(name);
    return OkStatus();
  }
  catch (const SDFStatus& e)
  {
    return e;
  }
}

SDFStatus SDFParser::GetElementFloat(const std::string& name, gsl::not_null<float*> value) const noexcept
{
  try
  {
    *value = GetElementFloat(name);
    return OkStatus();
  }
  catch (const SDFStatus& e)
  {
    return e;
  }
}

SDFStatus SDFParser::GetElementDouble(const std::string& name, gsl::not_null<double*> value) const noexcept
{
  try
  {
    *value = GetElementDouble(name);
    return OkStatus();
  }
  catch (const SDFStatus& e)
  {
    return e;
  }
}
