/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file type_conversion.cpp
 * @brief This files contains the implementation of the functions defined in the
 * type_conversion.h header.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_parser_utils/type_conversion.h"

#include <boost/algorithm/string.hpp>

#include <limits>
#include <sstream>

bool ParseBool(const std::string& str)
{
  std::string strValue = boost::algorithm::trim_copy(str);
  boost::algorithm::to_lower(strValue);

  // Check if the string is equal to a valid form of a true value "true" or "1"
  if (strValue == "true" || strValue == "1")
  {
    return true;
  }
  // Check if the string is equal to a valid form of a false value "false" or "0"
  else if (strValue == "false" || strValue == "0")
  {
    return false;
  }
  // In case of a conversion error
  else
  {
    throw std::invalid_argument("The received value is not a valid option for the boolean type. Valid options are any "
                                "combination of lower and upper case letters that form the strings \"true\" and "
                                "\"false\", and \"0\" or \"1\"");
  }
}

int ParseInt(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  // Convert the string to an integer
  try
  {
    std::size_t nCharsReaded;
    int value = std::stoi(strValue, &nCharsReaded, 0);
    if (nCharsReaded != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into an int.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

long int ParseLongInt(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  // Convert the string to an integer
  try
  {
    std::size_t nCharsReaded;
    long int value = std::stol(strValue, &nCharsReaded, 0);
    if (nCharsReaded != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into a long int.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

unsigned int ParseUnsignedInt(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  if (strValue.find("-") != std::string::npos)
  {
    throw std::invalid_argument("Unsigned integers cannot be signed.");
  }

  // Convert the string to an integer
  try
  {
    std::size_t lastChar;
    unsigned long int value = std::stoul(strValue, &lastChar, 0);

    if (value > static_cast<unsigned long int>(std::numeric_limits<unsigned int>::max()))
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" is out of the range of an unsinged int, which is ["
         << std::numeric_limits<unsigned int>::min() << ", " << std::numeric_limits<unsigned int>::max() << "].";
      throw std::out_of_range(ss.str());
    }

    if (lastChar != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into an unsigned integer.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

float ParseFloat(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  // Convert the string to an integer
  try
  {
    std::size_t lastChar;
    float value = std::stof(strValue, &lastChar);
    if (lastChar != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into a float.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

double ParseDouble(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  // Convert the string to an integer
  try
  {
    std::size_t lastChar;
    double value = std::stod(strValue, &lastChar);
    if (lastChar != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into a double.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

long double ParseLongDouble(const std::string& str)
{
  // Trim the string
  std::string strValue = boost::algorithm::trim_copy(str);

  // Convert the string to an integer
  try
  {
    std::size_t lastChar;
    long double value = std::stold(strValue, &lastChar);
    if (lastChar != strValue.size())
    {
      std::stringstream ss;
      ss << "The received string \"" << str << "\" could not be fully parsed into a double.";
      throw std::invalid_argument(ss.str());
    }
    return value;
  }
  catch (const std::logic_error& e)
  {
    throw e;
  }
}

bool ParseBool(const std::string& str, bool& value, std::exception& e) noexcept
{
  try
  {
    value = ParseBool(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = false;
    return false;
  }
}

bool ParseInt(const std::string& str, int& value, std::exception& e) noexcept
{
  try
  {
    value = ParseInt(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<int>::max();
    return false;
  }
}

bool ParseLongInt(const std::string& str, long int& value, std::exception& e) noexcept
{
  try
  {
    value = ParseLongInt(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<unsigned int>::max();
    return false;
  }
}

bool ParseUnsignedInt(const std::string& str, unsigned int& value, std::exception& e) noexcept
{
  try
  {
    value = ParseUnsignedInt(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<unsigned int>::max();
    return false;
  }
}

bool ParseFloat(const std::string& str, float& value, std::exception& e) noexcept
{
  try
  {
    value = ParseFloat(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<float>::quiet_NaN();
    return false;
  }
}

bool ParseDouble(const std::string& str, double& value, std::exception& e) noexcept
{
  try
  {
    value = ParseDouble(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<double>::quiet_NaN();
    return false;
  }
}

bool ParseLongDouble(const std::string& str, long double& value, std::exception& e) noexcept
{
  try
  {
    value = ParseLongDouble(str);
    return true;
  }
  catch (const std::exception& ex)
  {
    e = ex;
    value = std::numeric_limits<long double>::quiet_NaN();
    return false;
  }
}
