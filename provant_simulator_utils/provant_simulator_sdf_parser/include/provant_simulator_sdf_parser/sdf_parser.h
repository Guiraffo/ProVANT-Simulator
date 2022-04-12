/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sdf_parser.h
 * @brief This file contains the declaration of the SDFParser class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SDF_READER_H
#define PROVANT_SDF_READER_H

#include <sdf/sdf.hh>

#include <gsl/gsl>

#include "sdf_status.h"

/**
 * @brief The SDFParser class is a helper class to read properties from a SDF
 * Element object.
 *
 * This class is specially usefull during the loading of Gazebo plugins where
 * is necessary to read parameters from the plugin SDF description.
 *
 * A SDF (Simulation Description Format) Element, is a specialized XML element
 * that may contain attributes and child elements, as any other XML element
 * can.
 *
 * Gazebo provides a SDF Reader, but many times it has a confusing syntax and
 * very lax documentation, which is very hard to understand and discourages the
 * usage of the SDF class to read its own elements.
 *
 * As there were many instances in the ProVANT Simulator source code that
 * users and developers were using the now depreacted XMLRead class to read the
 * values of SDF elements, this class was designed as a substitution for the
 * XMLRead class, that wraps the methods provided by gazebo into well documented
 * and tested methods.
 *
 * So far, this class provides methods to check the existence, and returns the
 * string values of attributes and elements. In the future, it is very likely
 * that this class will also contain methods to read the values of elements in
 * other types, checking for adequate conversion and validity.
 *
 * @todo It turns out that is really hard to create SDF elements in memory
 * in order to do the testing of this class, so by now THIS CLASS IS UNTESTED!
 * It is imperative that we develop tests for this class. The tests are not
 * hard to create, the problem is that it creating SDF descriptions in memory
 * throws a series of undocumented errors and leads the test executable to a
 * segmentation fault, rendering the test procedure unfeasible.
 */
class SDFParser
{
public:
  /**
   * @brief Construct a new SDFParser object and initializes the SDF pointer.
   *
   * @param sdf SDF element to read data from.
   */
  SDFParser(sdf::ElementPtr sdf);
  /**
   * @brief Destroy the SDFParser object.
   * Releases the shared pointer to the SDF element.
   */
  virtual ~SDFParser();

  /**
   * @brief Get the SDF ptr from which this class reads data from.
   * @return sdf::ElementPtr
   */
  virtual sdf::ElementPtr GetElementPtr() const;

  /**
   * @brief Checks if the SDF contain an attribute with the specified name.
   *
   * Attributes are data specific to a element, that are usually strings.
   * For example, the following XML element:
   * @code{.xml}
   * <test type="example">
   *  <other_data/>
   * </test>
   * @endcode
   *
   * Contains an attribute named type and with value example.
   *
   * Usage example:
   * Suppose a plugin SDF description as follows:
   * @code{.xml}
   * <plugin name="example" filename="provant_simulator_step_plugin">
   *  <plugin_data/>
   * </plugin>
   * @endcode
   *
   * Assuming this plugin was read, the following code has res=true, because
   * the plugin contains an attribute "name".
   * @code{.cpp}
   * SDFParser reader(_sdf);
   * bool res = reader.HasAttribute("name");
   * @endcode
   *
   * And the following code results in res=false, because the plugin does not
   * contain an attribute "type".
   * @code{.cpp}
   * SDFParser reader(_sdf);
   * bool res = reader.HasAttribute("type");
   * @endcode
   *
   * @param name
   * @return true If the SDF element contains an attribute with the specified
   * name.
   * @return false Otherwise.
   */
  virtual bool HasAttribute(const std::string& name) const;
  /**
   * @brief Checks if the SDF has an element with the specified name.
   *
   * Usage example:
   * Suppose a plugin SDF description as follows:
   * @code{.xml}
   * <plugin name="example" filename="provant_simulator_step_plugin">
   *  <plugin_data/>
   * </plugin>
   * @endcode
   *
   * Assuming this plugin was read, the following code has res=true, because
   * the plugin contains an element "plugin_data".
   * @code{.cpp}
   * SDFParser reader(_sdf);
   * bool res = reader.HasElement("plugin_data");
   * @endcode
   *
   * And the following code results in res=false, because the plugin does not
   * contain an attribute "type".
   * @code{.cpp}
   * SDFParser reader(_sdf);
   * bool res = reader.HasElement("type");
   * @endcode
   *
   * @param name Name of the element to look for.
   * @return true If the SDF contains an element with the specified name.
   * @return false Otherwise.
   */
  virtual bool HasElement(const std::string& name) const;

  /**
   * @brief Get an attribute value.
   *
   * This checks if an attribute with the specified value exists, and if it
   * exists, copies its value to the value parameter.
   *
   * If the attribute does not exists, or any other occurs, an SDFStatus with an
   * error and a message describing the problem is returned, and the value is
   * left emtpy.
   *
   * @param name Name of the attribute to return.
   * @param value Buffer string that will contain the value of the attribute with the specified name.
   * @return SDFStatus that informs the status of the operation. If the
   * attribute is found and returned, a OkStatus object is returned, in the case
   * the attribute does not exist an AttributeNotFoundError object is returned,
   * containing an error status and a message informing the problem.
   */
  virtual SDFStatus GetAttributeValue(const std::string& name, gsl::not_null<std::string*> value) const noexcept;
  /**
   * @brief Get an attribute value.
   *
   * This method checks if an attribute with the specified name exists, and if
   * it exists, returns its value.
   *
   * @param name Name of the attribute to locate.
   * @return std::string Value of the attribute.
   * @throw AttributeNotFoundError If an attribute with the specifed name does
   * not exist.
   */
  virtual std::string GetAttributeValue(const std::string& name) const;
  /**
   * @brief Get the text value of a SDF element.
   *
   * Check if an element with the specified name exists, and if it exists,
   * copies its value to the value parameter.
   *
   * If an element with the specified name does not exist, or any other
   * error occurs, the value string is left empty, and a SDFStatus error is
   * returned with a message identifying the problem.
   *
   * @param name Name of the element to read the value.
   * @param value Buffer string that will contain the value of the element
   * @return SDFStatus that informs the status of the operation. If at least one
   * element with the specified name exists, a OkStatus object is returned.
   * In case an error occurs, an object identifying the problem is returned
   * with an errors status and a message describing the problem.
   */
  virtual SDFStatus GetElementText(const std::string& name, gsl::not_null<std::string*> value) const noexcept;
  /**
   * @brief Get the text value of a SDF element.
   *
   * Check if an element with the specified name exists, and return is value.
   *
   * @sa GetElementText(const std::string& name, std::string* value)
   *
   * @param name Name of the element to read the value.
   * @return std::string Content of the element.
   * @throw ElementNotFoundError if the element cannot be found as a child of
   * the SDF element this class reads from.
   */
  virtual std::string GetElementText(const std::string& name) const;

  /**
   * @brief Get the value of a boolean SDF element.
   *
   * The valid options are:
   * For true elements: true in any case combination, or 1.
   * For false elements: false in any case combinatation, or 0.
   *
   * @sa GetElementBool(const std::string&)
   *
   * @param name Name of the element to read.
   * @param value Content of the element.
   * @return SDFStatus Status of the parse operation. If the element was found
   * and had a valid value, an OKStatus is returned, otherwise an object
   * containing and error status and error message is returned.
   */
  virtual SDFStatus GetElementBool(const std::string& name, gsl::not_null<bool*> value) const noexcept;
  /**
   * @brief Get the value of a boolean SDF element.
   *
   * The valid options are:
   * For true elements: true in any case combination, or 1.
   * For false elements: false in any case combinatation, or 0.
   *
   * @sa GetElementBool(const std::string&, gsl::not_null<int*>)
   *
   * @param name Name of the element to read.
   * @return Value of the element.
   * @throw SDFStatus exception in case the element is not found or has
   * invalid content, ie. content that cannot be parsed as a boolean.
   */
  virtual bool GetElementBool(const std::string& name) const;

  /**
   * @brief Get the value of a int SDF element.
   *
   * This method can read integer values in the binary (0b), octal (0), decimal
   * (no prefix), or hexadecimal (0x) formats, identified by their prefixes.
   * It can also parse signed values with either a - or + signal.
   *
   * @sa GetElementInt(const std::string&)
   *
   * @param name Name of the element to read.
   * @param value Value of the element.
   * @return SDFStatus Status of the parse operation. If the element was found
   * and had a valid value, an OKStatus is returned, otherwise an object
   * containing and error status and error message is returned.
   */
  virtual SDFStatus GetElementInt(const std::string& name, gsl::not_null<int*> value) const noexcept;
  /**
   * @brief Get the value of a int SDF element.
   *
   * This method can read integer values in the binary (0b), octal (0), decimal
   * (no prefix), or hexadecimal (0x) bases, identified by their prefixes.
   * It can also parse signed values with either a - or + signal.
   *
   * @sa GetElementInt(const std::string&, gsl::not_null<int*>)
   *
   * @param name Name of the element to read.
   * @return int Value of the element.
   * @throw SDFStatus exception in case the element is not found or has
   * invalid content, ie. content that cannot be parsed as an integer such as
   * string, or if the value of the element is beyond the current platform limits
   * for an integer value.
   */
  virtual int GetElementInt(const std::string& name) const;

  /**
   * @brief Get the value of an unsigned int SDF element.
   *
   * This method can read unsigned integer values in the binary (0b), octal(0),
   * decimal (no prefix) or hexadecimal (0x) bases.
   * In case this methods find a signed integer, it returns an error.
   *
   * @sa GetElementUnsignedInt(const std::string&)
   *
   * @param name Name of the element to read.
   * @param value Value of the element.
   * @return SDFStatus Status of the parse operation. If the element was found
   * and had a valid value, an OKStatus is returned, otherwise an object
   * containing and error status and error message is returned.
   */
  virtual SDFStatus GetElementUnsignedInt(const std::string& name, gsl::not_null<unsigned int*> value) const noexcept;
  /**
   * @brief Get the value of an unsigned int SDF element.
   *
   * This method can read unsigned integer values in the binary (0b), octal(0),
   * decimal (no prefix) or hexadecimal (0x) bases.
   * In case this methods find a signed integer, it throws an exception.
   *
   * @sa GetElementUnsignedInt(const std::string&, gsl::not_null<unsigned int*>)
   *
   * @param name Name of the element to read.
   * @return unsigned int Value of the element.
   * @throws SDFStatus exception in case the element is not found or has
   * invalid content, ie. content that cannot be parsed as an integer such as
   * string, if the value is signed, or if the value is beyond the limits of an
   * unsigned integer for the current platform.
   */
  virtual unsigned int GetElementUnsignedInt(const std::string& name) const;

  /**
   * @brief Get the value of a single precision (float) SDF Element.
   *
   * This method can read real values as single precision floating point values
   * (float). It is also possible to read floats specified in the hexadecimal
   * format with the 0x prefix.
   *
   * @sa GetElementFloat(const std::string&)
   *
   * @param name Name of the element to read.
   * @param value Value of the element.
   * @return SDFStatus Status of the parse operation. If the element was found
   * and had a valid value, an OKStatus is returned, otherwise an object
   * containing and error status and error message is returned.
   */
  virtual SDFStatus GetElementFloat(const std::string& name, gsl::not_null<float*> value) const noexcept;
  /**
   * @brief Get the value of a single precision (float) SDF Element.
   *
   * This method can read real values as single precision floating point values
   * (float). It is also possible to read floats specified in the hexadecimal
   * format with the 0x prefix.
   *
   * @sa GetElementFloat(const std::string&, gsl::not_null<float*>)
   *
   * @param name Name of the element to read.
   * @return float Value of the element.
   * @throws SDFStatus exception in case the element is not found or has
   * invalid content, ie. content that cannot be parsed as a float or is beyond
   * the limits of a float for the current platform.
   */
  virtual float GetElementFloat(const std::string& name) const;

  /**
   * @brief Get the value of a double precision floating point (double) SDF Element.
   *
   * This method can read real values as double precision floating point values
   * (double). It is also possible to read doubles specified in the hexadecimal
   * format with the 0x prefix.
   *
   * @sa GetElementDouble(const std::string&)
   *
   * @param name Name of the element to read.
   * @param value Value of the element.
   * @return SDFStatus Status of the parse operation. If the element was found
   * and had a valid value, an OKStatus is returned, otherwise an object
   * containing and error status and error message is returned.
   */
  virtual SDFStatus GetElementDouble(const std::string& name, gsl::not_null<double*> value) const noexcept;
  /**
   * @brief Get the value of a double precision floating point (double) SDF Element.
   *
   * This method can read real values as double precision floating point values
   * (double). It is also possible to read doubles specified in the hexadecimal
   * format with the 0x prefix.
   *
   * @sa GetElementDouble(const std::string&, gsl::not_null<double*>)
   *
   * @param name Name of the element to read.
   * @return double Value of the element.
   * @throws SDFStatus exception in case the element is not found or has
   * invalid content, ie. content that cannot be parsed as a double or is beyond
   * the limits of a double for the current platform.
   */
  virtual double GetElementDouble(const std::string& name) const;

private:
  //! Stores the pointer of the SDF from which this class reads the values of attributes and elements.
  sdf::ElementPtr _sdf;
};

#endif  // PROVANT_SDF_READER_H
