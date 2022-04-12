/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file type_conversion.h
 * @brief This file contains the declaration of functions used in the conversion
 * of strings to other data types, such as booleans, integers, floats or doubles.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SIMULATOR_PARSER_UTILS_TYPE_CONVERSION_H
#define PROVANT_SIMULATOR_PARSER_UTILS_TYPE_CONVERSION_H

#include <exception>
#include <string>

/**
 * @brief Convert the value of a string to a boolean.
 *
 * Valid options are:
 * For true values: true in any case combination, or 1.
 * For false values: false in any case combination, or 0.
 *
 * @sa ParseBool(const std::string&, bool&, std::exception&)
 *
 * @param str String to convert.
 * @return Value of the parsed string.
 * @throws std::invalid_argument if the string cannot be parsed into a bool.
 */
bool ParseBool(const std::string& str);
/**
 * @brief Convert the value of a string to a boolean.
 *
 * Valid options are:
 * For true values: true in any case combination, or 1.
 * For false values: false in any case combination, or 0.
 *
 * @sa ParseBool(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseBool(const std::string& str, bool& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to an integer.
 *
 * This function converts a string to an integer, and checks that the converted
 * value is inside the bounds of an integer value for the current platform.
 *
 * @sa ParseInt(const std::string&, int&, std::excpetion &e)
 *
 * @param str String to convert.
 * @return int Value of the parsed string.
 * @throw std::invalid_argument In case the string cannot be converted to an integer.
 * @throw std::logic_error In case the string can be converted to an integer, but
 * it is outstide the bounds of an integer value.
 */
int ParseInt(const std::string& str);
/**
 * @brief Convert the value of a string to an integer.
 *
 * This function converts a string to an integer, and checks that the converted
 * value is inside the bounds of an integer value for the current platform.
 *
 * @sa ParseInt(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseInt(const std::string& str, int& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to a long integer.
 *
 * This function converts a string to an integer, and checks that the converted
 * value is inside the bounds of a long integer value for the current platform.
 *
 * @sa ParseLongInt(const std::string&, long int&, std::exception&)
 *
 * @param str String to convert.
 * @return long int Value of the parsed string.
 */
long int ParseLongInt(const std::string& str);
/**
 * @brief Convert the value of a string to a long integer.
 *
 * This function converts a string to an integer, and checks that the converted
 * value is inside the bounds of a long integer value for the current platform.
 *
 * @sa ParseLongInt(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseLongInt(const std::string& str, long int& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to an unsigned integer.
 *
 * This function converts a string to unsigned integer, and checks that the
 * converted value is inside the bounds an unsigned integer for the current
 * platform.
 *
 * @sa ParseUnsignedInt(const std::string&, unsigned int&, std::exception&)
 *
 * @param str String to convert.
 * @return unsigned int Value of the parsed string.
 * @throws std::invalid_argument If the string cannot be converted into an
 * unsigned int.
 * @throws std::out_of_range If the string can be converted into an unsigned int,
 * but is outside the valid range of an unsigned int for the current platform.
 */
unsigned int ParseUnsignedInt(const std::string& str);
/**
 * @brief Convert the value of a string to an unsigned integer.
 *
 * This function converts a string to unsigned integer, and checks that the
 * converted value is inside the bounds an unsigned integer for the current
 * platform.
 *
 * @sa ParseUnsignedInt(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseUnsignedInt(const std::string& str, unsigned int& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to a single precision floating point number (float).
 *
 * @sa ParseFloat(const std::string&, float&, std::exception&)
 *
 * @param str String to convert.
 * @return float Value of the parsed string.
 * @throws std::invalid_argument If the string cannot be parsed into a floating
 * point value.
 * @throws std::logic_error If the string can be parsed into a float, but it is
 * outside the valid range of a floating pointer number for the current platform.
 */
float ParseFloat(const std::string& str);
/**
 * @brief Convert the value of a string to a single precision floating point number (float).
 *
 * @sa ParseFloat(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseFloat(const std::string& str, float& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to a double precision floating point number (double).
 *
 * @sa ParseDouble(const std::string&, double&, std::exception&)
 *
 * @param str String to convert.
 * @return double Value of the parsed string.
 * @throws std::invalid_argument If the string cannot be parsed into a floating
 * point value.
 * @throws std::logic_error If the string can be parsed into a double, but it is
 * outside the valid range of a floating pointer number for the current platform.
 */
double ParseDouble(const std::string& str);
/**
 * @brief Convert the value of a string to a double precision floating point number (double).
 *
 * @sa ParseDouble(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseDouble(const std::string& str, double& value, std::exception& e) noexcept;

/**
 * @brief Convert the value of a string to an extend precision floating point number (long double).
 *
 * @sa ParseLongDouble(const std::string&, long double&, std::exception&)
 *
 * @param str String to convert.
 * @return long double Value of the parsed string.
 * @throws std::invalid_argument If the string cannot be parsed into a floating
 * point value.
 * @throws std::logic_error If the string can be parsed into a long double, but it is
 * outside the valid range of a floating pointer number for the current platform.
 */
long double ParseLongDouble(const std::string& str);
/**
 * @brief Convert the value of a string to an extend precision floating point number (long double).
 *
 * @sa ParseLongDouble(const std::string&)
 *
 * @param str String to convert.
 * @param value Value of the parsed string.
 * @param e In case an error occurs, this exception contains an error message
 * identifying the error cause.
 * @return true If the conversion was successfull and false otherwise.
 */
bool ParseLongDouble(const std::string& str, long double& value, std::exception& e) noexcept;

#endif  // PROVANT_SIMULATOR_PARSER_UTILS_TYPE_CONVERSION_H
