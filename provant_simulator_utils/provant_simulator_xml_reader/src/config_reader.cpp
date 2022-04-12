/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the implementation of the ConfigReader class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_xml_reader/config_reader.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>

ConfigReader::ConfigReader() : _filepath("")
{
}

ConfigReader::ConfigReader(const std::string& path) : _filepath(path)
{
}

ConfigReader::~ConfigReader()
{
}

bool ConfigReader::open()
{
  if (_filepath.empty())
    return false;

  tinyxml2::XMLError res;
  res = doc.LoadFile(_filepath.c_str());
  return res == tinyxml2::XML_SUCCESS;
}

bool ConfigReader::open(const std::string& path)
{
  _filepath = path;
  return open();
}

bool ConfigReader::loadFromString(const std::string& content)
{
  const auto res = doc.Parse(content.c_str());
  return res == tinyxml2::XML_SUCCESS;
}

bool ConfigReader::checkError() const
{
  return doc.Error();
}

std::string ConfigReader::getErrorMsg() const
{
  if (checkError())
  {
    return toStdString(doc.ErrorStr());
  }

  return std::string("");
}

const tinyxml2::XMLElement* ConfigReader::getChildElement(const std::string& name, bool* found) const
{
  const tinyxml2::XMLElement* root = doc.RootElement();
  if (root == NULL)
  {
    setFoundRes(false, found);
    return root;
  }

  const tinyxml2::XMLElement* child = root->FirstChildElement(name.c_str());
  if (child == NULL)
  {
    setFoundRes(false, found);
    return nullptr;
  }

  setFoundRes(true, found);
  return child;
}

std::string ConfigReader::getElementText(const std::string& name, bool* found) const
{
  bool res = false;
  const tinyxml2::XMLElement* data = getChildElement(name, &res);

  if (res)
  {
    const char* str = data->GetText();
    setFoundRes(true, found);
    return toStdString(str);
  }

  setFoundRes(false, found);
  return std::string("");
}

std::string ConfigReader::getDataTopic(bool* found) const
{
  return getElementText("topicdata", found);
}

std::string ConfigReader::getStepTopic(bool* found) const
{
  return getElementText("TopicoStep", found);
}

std::string ConfigReader::getControlStrategy(bool* found) const
{
  return getElementText("Strategy", found);
}

std::string ConfigReader::getTurbulenceModel(bool* found) const
{
  return getElementText("Turbulance", found);
}

std::string ConfigReader::getReferenceFilePath(bool* found) const
{
  return getElementText("RefPath", found);
}

std::string ConfigReader::getOutputFilePath(bool* found) const
{
  return getElementText("Outputfile", found);
}

std::string ConfigReader::getInputFilePath(bool* found) const
{
  return getElementText("InputPath", found);
}

std::string ConfigReader::getErrorFilePath(bool* found) const
{
  return getElementText("ErroPath", found);
}

ConfigReader::stringlist ConfigReader::getChildElementValues(const std::string& name, const std::string& childName,
                                                             bool* found) const
{
  // Get the element with the specified name
  bool res = false;
  const tinyxml2::XMLElement* child = getChildElement(name, &res);

  ConfigReader::stringlist values;

  if (res)
  {
    // Get first sibling element
    const tinyxml2::XMLElement* sibling = child->FirstChildElement(childName.c_str());
    while (sibling != NULL)
    {
      // Get the value of the current element
      const char* str = sibling->GetText();
      values.push_back(toStdString(str));

      // Go to next sibling element
      sibling = sibling->NextSiblingElement(childName.c_str());
    }
    setFoundRes(true, found);
  }
  else
  {
    setFoundRes(false, found);
  }

  return values;
}

ConfigReader::stringlist ConfigReader::getSensors(bool* found) const
{
  return getChildElementValues("Sensors", "Device", found);
}

ConfigReader::stringlist ConfigReader::getActuators(bool* found) const
{
  return getChildElementValues("Actuators", "Device", found);
}

int ConfigReader::getElementInt(const std::string& name, bool* found) const
{
  bool res = false;
  std::string strValue = getElementText(name, &res);

  int value = 0;

  if (res)
  {
    // Try to convert the value to a integer
    try
    {
      value = std::stoi(strValue, 0, 10);
      // Verify if the conversion from the value to a string is the same in order to catch errors such as double value
      // truncation.
      std::string reversedStr = std::to_string(value);
      if (trim(strValue) != reversedStr)
      {
        setFoundRes(false, found);
        return 0;
      }
      setFoundRes(true, found);
    }
    catch (const std::invalid_argument& e)
    {
      setFoundRes(false, found);
    }
    catch (const std::out_of_range& e)
    {
      setFoundRes(false, found);
    }
  }
  else
  {
    setFoundRes(false, found);
  }

  return value;
}

uint64_t ConfigReader::getElementUint64(const std::string& name, bool* found) const
{
  bool res = false;
  std::string strValue = getElementText(name, &res);

  uint64_t value = 0;

  if (res)
  {
    // Try to convert the value to a integer
    try
    {
      value = std::stoull(strValue, 0, 10);
      // Verify if the conversion from the value to a string is the same in order to catch errors such as double value
      // truncation and negative value buffer overflow.
      std::string reversedStr = std::to_string(value);
      if (trim(strValue) != reversedStr)
      {
        setFoundRes(false, found);
        return 0;
      }
      setFoundRes(true, found);
    }
    catch (const std::invalid_argument& e)
    {
      setFoundRes(false, found);
    }
    catch (const std::out_of_range& e)
    {
      setFoundRes(false, found);
    }
  }
  else
  {
    setFoundRes(false, found);
  }

  return value;
}

bool ConfigReader::getElementBool(const std::string& name, bool* found) const
{
  bool res = false;
  std::string strValue = getElementText(name, &res);
  // Convert the string to lower case
  strValue = toLower(strValue);
  // Trim white space from the string
  strValue = trim(strValue);

  bool value = false;

  if (res)
  {
    if (strValue == "true" || strValue == "1")
    {
      setFoundRes(true, found);
      return true;
    }
    else if (strValue == "false" || strValue == "0")
    {
      setFoundRes(true, found);
      return false;
    }
  }
  else
  {
    setFoundRes(false, found);
  }

  return value;
}

int ConfigReader::getSampleTime(bool* found) const
{
  return getElementInt("Sampletime", found);
}

uint64_t ConfigReader::getSimulationDuration(bool* found) const
{
  return getElementUint64("Duration", found);
}

bool ConfigReader::getShutdownWhenFinished(bool* found) const
{
  // return getElementBool("ShutdownWhenFinished", found);
  return getElementBool("ShutdownWhenFinished", found);
}

bool ConfigReader::getHilFlagSynchronous(bool* found) const
{
  return getElementBool("HilFlagSynchronous", found);
}

bool ConfigReader::getHilFlagAsynchronous(bool* found) const
{
  return getElementBool("HilFlagAsynchronous", found);
}

int ConfigReader::getBaudRate(bool* found) const
{
  return getElementInt("baudRate", found);
}

std::string ConfigReader::getUsart1(bool* found) const
{
  return getElementText("usart1", found);
}

std::string ConfigReader::getUsart2(bool* found) const
{
  return getElementText("usart2", found);
}

bool ConfigReader::getStartPaused(bool* found) const
{
  bool innerFound = false;
  bool res = getElementBool("StartPaused", &innerFound);
  if (innerFound)
  {
    setFoundRes(true, found);
    return res;
  }
  else
  {
    setFoundRes(false, found);
    return true;
  }
}

void ConfigReader::setFoundRes(bool res, bool* found) const
{
  if (found != nullptr && found != NULL)
    *found = res;
}

std::string ConfigReader::toStdString(const char* str) const
{
  if (str != nullptr && str != NULL)
    return std::string(str);
  return std::string("");
}

ConfigReader::stringvector ConfigReader::toStringvector(const ConfigReader::stringlist& list) const
{
  ConfigReader::stringvector vec;
  vec.reserve(list.size());

  for (ConfigReader::stringlist::const_iterator i = list.cbegin(); i != list.cend(); ++i)
  {
    vec.push_back(*i);
  }

  return vec;
}

std::string ConfigReader::getFilePath() const
{
  return _filepath;
}

std::string ConfigReader::getItem(const std::string& name, bool* found)
{
  return getElementText(name, found);
}

ConfigReader::stringlist ConfigReader::getItens(const std::string& name, const std::string& childName, bool* found)
{
  return getChildElementValues(name, childName, found);
}

ConfigReader::stringvector ConfigReader::getItensVector(const std::string& name, const std::string& childName,
                                                        bool* found)
{
  return toStringvector(getItens(name, childName, found));
}

std::string ConfigReader::trim(const std::string& str) const
{
  std::string newStr = str;
  newStr.erase(newStr.begin(),
               std::find_if(newStr.begin(), newStr.end(), [](unsigned char c) { return !std::isspace(c); }));
  newStr.erase(std::find_if(newStr.rbegin(), newStr.rend(), [](unsigned char c) { return !std::isspace(c); }).base(),
               newStr.end());
  return newStr;
}

std::string ConfigReader::toLower(const std::string& str) const
{
  std::string newStr = str;
  std::transform(newStr.begin(), newStr.end(), newStr.begin(), [](unsigned char c) { return std::tolower(c); });
  return newStr;
}
