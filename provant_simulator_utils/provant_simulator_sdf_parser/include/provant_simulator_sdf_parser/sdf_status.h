/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_status.h
 * @brief This file contains the declaration of the SDFStatus class.
 *
 * This file contains several definitions used to report the status of
 * operations performed by the SDFStatus class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SDF_STATUS_H
#define PROVANT_SDF_STATUS_H

#include <stdexcept>
#include <string>

/**
 * @brief The SDFStatus is a return type of the methods used by the SDFReader.
 *
 * This status is used to indicate if an error ocurred or not, and also to
 * contain an error message, indicating what exactly was the error, and what
 * caused it.
 *
 * This class derives from std::runtime_error, and thus can be used as an
 * exception.
 *
 * It is also possible to use this a parameter or a return value, if exceptions
 * are not desireable.
 */
class SDFStatus : public std::runtime_error
{
public:
  /**
   * @brief Construct a new SDFStatus object.
   *
   * @param isError Indicates if this is an error status or not.
   * @param msg Error message.
   */
  SDFStatus(bool isError = true, const std::string msg = "");
  /**
   * @brief Destroy the SDFStatus object.
   * Cleans the string buffer.
   */
  virtual ~SDFStatus();

  /**
   * @brief Indicates if the status is an error status or not.
   *
   * @return true If this is an error status.
   * @return false Otherwise.
   */
  virtual bool isError() const;
  /**
   * @brief Return the error message of this status.
   *
   * Note: A status that is not an error, should have an empty error message.
   *
   * @return const std::string&
   */
  virtual const std::string& errorMessage() const;

  /**
   * @brief Returns a C string (null terminated) version of the error message.
   *
   * Overrides the what() method of std::runtime_error in order to return
   * the error message.
   *
   * @return const char*
   */
  const char* what() const noexcept override;

protected:
  //! Indicates if this status is an error status or not.
  bool _isError;
  //! Error message indicating what exactly was the error, and what caused it.
  std::string _msg;
};

/**
 * @brief The OkStatus indicates that no error was found during the requested
 * operation.
 *
 * The specified element or attribute was found in the provided SDF element
 * pointer, and the conversion to the request type was successfull.
 */
class OkStatus : public SDFStatus
{
public:
  /**
   * @brief Construct a new Ok Status object.
   */
  OkStatus();
};

/**
 * @brief Helper class to construct a not found error.
 *
 * This class is a shortcut for the ElementNotFoundError and
 * AttributeNotFoundError to build upon.
 *
 * For more details on these erros, see the documentation of the derived
 * classes.
 *
 * @sa AttributeNotFoundError
 * @sa ElementNotFoundError
 */
class NotFoundError : public SDFStatus
{
public:
  /**
   * @brief Construct a new Not Found Error object.
   *
   * @param name Name of the element or attribute that was not found.
   * @param type Type of the element that was not found.
   */
  NotFoundError(const std::string& name, const std::string& type);
};

/**
 * @brief The ElementNotFoundError is a status that indicates that an element
 * with a given name was not found as a child of the SDF element.
 *
 * The error message indicates this to user that the element was not found,
 * specific instructions for correction, and the contents of the element must
 * be specified by the developer in the log message.
 */
class ElementNotFoundError : public NotFoundError
{
public:
  /**
   * @brief Construct a new Element Not Found Error object
   *
   * @param name The name of the element that was not found.
   */
  ElementNotFoundError(const std::string& name);
};

/**
 * @brief The AttributeNotFoundError is a status that indicates that an
 * attribute with a given name was not found in the SDF element.
 *
 * The error message indicates this to user that the attribute was not found,
 * specific instructions for correction, and the contents of the element must
 * be specified by the developer in the log message.
 */
class AttributeNotFoundError : public NotFoundError
{
public:
  /**
   * @brief Construct a new Attribute Not Found Error object
   *
   * @param name Name of the attribute that was not found.
   */
  AttributeNotFoundError(const std::string& name);
};

/**
 * @brief The XMLSyntaxError is a status that indicates the the received SDF
 * element is not valid because a XML error ocurred in its parsing.
 *
 * Generally this error should not occur, because the parser will fail if it
 * encounters an error, before the SDF pointer is passed to the plugin, but
 * this class was included here in order to provide a complete solution to
 * error reporting.
 */
class XMLSyntaxError : public SDFStatus
{
public:
  /**
   * @brief Construct a new XMLSyntaxError object.
   *
   * @param msg XML error message.
   */
  XMLSyntaxError(const std::string& msg);
};

/**
 * @brief The ConversionError is a status that indicates that the element was
 * found in the SDF, but the string could not be converted to the requested
 * type.
 */
class ConversionError : public SDFStatus
{
public:
  /**
   * @brief Construct a new Conversion Error object.
   *
   * @param element The name of the element that failed conversion.
   * @param originalContent The string that failed conversion.
   * @param type The type the string was going to be converted to.
   * @param extraMsg An optional message to print at the end of the error reporting.
   * May be usefull for reporting valid options.
   */
  ConversionError(const std::string& element, const std::string& originalContent, const std::string& type,
                  const std::string& extraMsg = "");
};

/**
 * @brief The NullPointerError is a status that indicates that a null pointer
 * was passsed as an output parameter to a method.
 */
class NullPointerError : public SDFStatus
{
public:
  /**
   * @brief Construct a new Null Pointer Error object.
   *
   * @param method The name of the method were the error ocurred.
   */
  NullPointerError(const std::string& method);
};

#endif  // PROVANT_SDF_STATUS_H
