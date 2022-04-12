/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the ConfigReader class.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CONFIG_READER_H
#define PROVANT_CONFIG_READER_H

#include <list>
#include <string>
#include <vector>

#include <tinyxml2.h>

/**
 * @brief The ConfigReader class is XML parser to read the contents of the
 * config.xml file.
 *
 * The config.xml file is used by the ProVANT simulator to set the properties
 * for a given simulation, including the control strategy to use, the name
 * of the log files that will be generated during simulation, the sensors and
 * actuators of a given model and etc.
 */
class ConfigReader
{
public:
  //! @brief List of std strings.
  typedef std::list<std::string> stringlist;
  //! @brief Vector of std strings.
  typedef std::vector<std::string> stringvector;

  /**
   * @brief Construct a new Config Reader object with an empty file path.
   *
   * Note. In order to correctly use this class when using the default
   * constructor, a file path must be specified in the open method.
   *
   * @sa open()
   */
  ConfigReader();
  /**
   * @brief Construct a new Config Reader object with the specified file path.
   *
   * @param path Path to the config.xml file.
   */
  ConfigReader(const std::string& path);
  /**
   * @brief Destroy the Config Reader object.
   */
  virtual ~ConfigReader();

  /**
   * @brief Opens the file with the value specified in the constructor.
   *
   * @sa open(const std::string& path), checkError(), getErrorMsg()
   *
   * @return true if the file was opened successfully and false otherwise.
   */
  bool open();
  /**
   * @brief Opens the file with the specified path.
   *
   * This method receives the path to the config.xml file, and tries to parse
   * the XML from the specified file.
   *
   * If an error occurs, this method will return false, and the error message
   * can be obtained using the getErrorMsg() method.
   *
   * @sa open(), checkError(), getErrorMsg()
   *
   * @param path File path to the congig.xml file.
   * @return true if the file was opened successfully and false otherwise.
   */
  bool open(const std::string& path);

  /**
   * @brief Loads the XML content of a string.
   * 
   * @param content XML string to load.
   * @return true if the loading process succeeds and false otherwise.
   */
  bool loadFromString(const std::string& content);

  // Generic readers
  /**
   * @brief Read the text content of a specified XML element.
   *
   * @param name Name of the XML element to read.
   * @param found Indicates a child XML element of the root node with the
   * specified name was found or not.
   * @return std::string Text content of the XML element, or an empty string if
   * the element is not found.
   */
  std::string getItem(const std::string& name, bool* found = nullptr);
  /**
   * @brief Read the text values from the child nodes of a child element of the
   * root XML node with the specified name.
   *
   * @param name Name of the parent element.
   * @param childName Name of the child elements to read.
   * @param found Indicates if the parent element was found or not.
   * @return stringlist List of values, or an empty list if the parent element
   * is not found.
   */
  stringlist getItens(const std::string& name, const std::string& childName, bool* found = nullptr);
  /**
   * @brief Read the text values from the child nodes of a child element of the
   * root XML node with the specified name, and returns a vector of the child
   * nodes values.
   *
   * Note: This method only exists as a convenience to help the users from the
   * deprecated XMLRead class. Please use getItens() instead of this method.
   *
   * @sa getItens()
   *
   * @param name Name of the parent element.
   * @param childName Name of the child elements to read.
   * @param found Indicates if the parent element was found or not.
   * @return stringvector Vector of values, or an empty list if the parent element
   * is not found.
   */
  stringvector getItensVector(const std::string& name, const std::string& childName, bool* found = nullptr)
      __attribute__((deprecated));

  // Error checking and reporting
  /**
   * @brief Verify if an error ocurred during the parsing of the config.xml file.
   *
   * @sa getErrorMsg().
   *
   * @return true if an error was found, and false otherwise.
   */
  bool checkError() const;
  /**
   * @brief Get an error message identifying the line number, node element and
   * further details about an error that occurs during parsing of the config.xml
   * file.
   *
   * @sa checkError().
   *
   * @return std::string Eror message, or an empty string if no error ocurred.
   */
  std::string getErrorMsg() const;

  // Readers
  /**
   * @brief Get the name of the data topic.
   *
   * @todo I have no idea what this parameters does, I highly believe it is not
   * used anywhere. If it is, this method must be deprecated.
   *
   * @param found Indicates if the topicdata element was found or not.
   * @return std::string Name of the data topic.
   */
  std::string getDataTopic(bool* found = nullptr) const;
  /**
   * @brief Get the name of the step topic.
   *
   * The step topic is used by the controller node to advance the simulation
   * step.
   *
   * @param found Indicates if the TopicoStep element was found or not.
   * @return std::string Name of the step topic.
   */
  std::string getStepTopic(bool* found = nullptr) const;
  /**
   * @brief Get the name of the desired control strategy dynamic library.
   *
   * @param found Indicates if the Strategy element was found or not.
   * @return std::string Name of the dynamic library containg the control
   * strategy name.
   */
  std::string getControlStrategy(bool* found = nullptr) const;
  /**
   * @brief Get the name of the turbulence model used in the simulation.
   *
   * @param found Indicates if the Turbulence element was found or not.
   * @return std::string Turbulence model name.
   */
  std::string getTurbulenceModel(bool* found = nullptr) const;
  /**
   * @brief Get the name of the reference log file.
   *
   * The reference log file contains the references for the control strategy
   * at each simulation step.
   *
   * @param found Indicates if the RefPath element was found or not.
   * @return std::string Name of the reference log file.
   */
  std::string getReferenceFilePath(bool* found = nullptr) const;
  /**
   * @brief Get the name of the output log file.
   *
   * The output log file contains the control inputs (control law outputs) at
   * each simulation step.
   *
   * @param found Indicates if the Outputfile element was found or not.
   * @return std::string Name of the output log file.
   */
  std::string getOutputFilePath(bool* found = nullptr) const;
  /**
   * @brief Get the name of the input log file.
   *
   * The input log file contains the state values reported by the simulator
   * at each simulation step.
   *
   * @param found Indicates if InputPath element was found or not.
   * @return std::string Name of the input log file.
   */
  std::string getInputFilePath(bool* found = nullptr) const;
  /**
   * @brief Get the name of the error log file.
   *
   * The error log file contains the error values for each simulation step.
   *
   * @param found Indicates if the ErroPath element was found or not.
   * @return std::string Name of the error log file.
   */
  std::string getErrorFilePath(bool* found = nullptr) const;
  /**
   * @brief Get a list of the sensors of the model.
   *
   * Returns the text content of the Device nodes inside of the Sensors Element.
   *
   * @param found Indicates if the Sensors XML element was found or not.
   * @return stringlist List of sensor names.
   */
  stringlist getSensors(bool* found = nullptr) const;
  /**
   * @brief Get a list of the actuators of the model.
   *
   * Returns the text content of Device node inside of the Actuators Element.
   *
   * @param found Indicates if the Actuators XML element was found or not.
   * @return stringlist List of actuator names.
   */
  stringlist getActuators(bool* found = nullptr) const;
  /**
   * @brief Get the sample time for the control strategy.
   *
   * Sample time is not an adequate name. This parameter identify how simulations
   * steps must occur for one update of the control inputs by the control law.
   *
   * For example, if the Sampletime parameter has a value of 3, the control law
   * will only be updated once every three steps.
   *
   * @todo Deprecate this method and insert a replacement with a more adequate
   * name.
   *
   * @param found Indicates if the Sampletime element was found or not.
   * @return int Sample time of the control strategy.
   */
  int getSampleTime(bool* found = nullptr) const;

  /**
   * @brief Get the Simulation duration.
   *
   * Get the number of steps the simulation must be executed for.
   * A zero value indicates an user controlled simulation (with infinite
   * duration).
   *
   * @param found Indicates if the Duration element was found or not.
   * @return uint64_t Number of steps the simulation must be executed for.
   */
  uint64_t getSimulationDuration(bool* found = nullptr) const;

  /**
   * @brief Indicates if the controller node must automatically close when the
   * simulation is finished or not.
   *
   * @param found Indicates if the ShutdownWhenFinished element was found or not.
   * @return true If the controller node must automatically shutdown when the
   * simulation finished.
   * @return false If the controller node must keep running after the simulation
   * was finished, or the ShutdownWhenFinished element was not found.
   */
  bool getShutdownWhenFinished(bool* found = nullptr) const;

  /**
   * @brief Indicates whether a simulation is in synchronous hardware in the loop (HIL) mode.
   *
   * @param found found Indicates whether the HilFlagSynchronous element was found or not.
   * @return true If the simulation is in synchronous HIL mode and false otherwise.
   */
  bool getHilFlagSynchronous(bool* found = nullptr) const;

  /**
   * @brief Indicates whether a simulation is in asynchronous hardware in the loop (HIL) mode.
   *
   * @param found found Indicates whether the HilFlagAsynchronous element was found or not.
   * @return true If the simulation is in asynchronous HIL mode and false otherwise.
   */
  bool getHilFlagAsynchronous(bool* found = nullptr) const;

  /**
   * @brief Indicates the baudrate in bits per second (second) used in the communication with the embedded
   * hardware when the simulation is in hardware in the loop (HIL) mode.
   *
   * @param found found Indicates if the baudRate element was found in the config file.
   * @return Integer containing the baudrate for communication in bits per second (bps).
   */
  int getBaudRate(bool* found = nullptr) const;

  /**
   * @brief Get the name of the first port used in communication when the simulation is in HIL mode.
   *
   * @param found Indicates if the Usart1 element was found in the config file.
   * @return std::string Path for the port to use.
   */
  std::string getUsart1(bool* found = nullptr) const;

  /**
   * @brief Get the name of the second port used in communication when the simulation is in HIL mode.
   *
   * @param found Indicates if the Usart2 element was found in the config file.
   * @return std::string Path for the port to use.
   */
  std::string getUsart2(bool* found = nullptr) const;

  /**
   * @brief Indicates if the simulation must start in a paused state or not.
   *
   * If the simulation is started in paused mode, the user must click either
   * the start or the step button in the Gazebo GUI to start the simulation.
   *
   * If the simulation is started in unpaused mode, the controller node must
   * start the simulation.
   *
   * @param found Indicates if the StartPaused element was found or not.
   * @return true If the user must start the simulation manually.
   * @return false If the controller node must start the simulation.
   */
  bool getStartPaused(bool* found = nullptr) const;

  /**
   * @brief Getter for the path to the config.xml file.
   *
   * @return std::string Filepath.
   */
  std::string getFilePath() const;

protected:
  //! @brief Path the config.xml file
  std::string _filepath;
  //! @brief XML document for the config.xml file
  tinyxml2::XMLDocument doc;

  /**
   * @brief Converts a null terminated C string to a std string object.
   *
   * If a null ptr is received, this method returns an empty string.
   *
   * @param str String to convert.
   * @return std::string Converted string.
   */
  std::string toStdString(const char* str) const;
  /**
   * @brief Converts a string to all lower case characters.
   *
   * @param str String to convert.
   * @return std::string The equivalent of the str parameter with all lower case
   * characters.
   */
  std::string toLower(const std::string& str) const;
  /**
   * @brief Removes trailing white space from a string.
   *
   * @param str String to remove the trailing white space.
   * @return std::string String without trailing whitespace.
   */
  std::string trim(const std::string& str) const;
  /**
   * @brief Helper method to set the result of the found boolean pointer.
   *
   * Verify if the found pointer is different from a nullptr and if it is, set
   * the result to the value of the res parameter.
   *
   * @param res Indicate if a given XML Element was found or not.
   * @param found Pointer to indicate the result.
   */
  void setFoundRes(bool res, bool* found) const;
  /**
   * @brief Helper method to convert a null terminated c string to a std::string
   * object.
   *
   * @param str Null terminated C string to convert.Reference File Path object
   * @return stringvector Vector containing all of the elements contained in the list.
   */
  stringvector toStringvector(const stringlist& list) const;
  /**
   * @brief Helper method to get the pointer to a child XML element.
   *
   * Verify if there exists a child element of the root node with the specified
   * name, if there is the found variable is set to true and the pointer to this
   * element is returned.
   *
   * If the element is not found, the found parameter is set to false, and a
   * null pointer is returned.
   *
   * @param name Name of the child element to locate.
   * @param found Indicates a child element of the root node with the specified
   * name was found or not.
   * @return const tinyxml2::XMLElement* Pointer to the child XML element, or
   * a null ptr if the object is not found.
   */
  const tinyxml2::XMLElement* getChildElement(const std::string& name, bool* found) const;
  /**
   * @brief Helper method to get the content of a XML element.
   *
   * This method verifies if exists a child element of the document root node
   * with the specified name. If this element exists, the found boolean value
   * is set to trough and the value of the element is returned as a string.
   *
   * If the element is not found an empty string is returned, and the found
   * value is set to false.
   *
   * @param name Name of the XML element to read.
   * @param found Indicates if a XML element with the specified name was found or not.
   * @return std::string Content of the XML element. If the element is not found
   * a empty string is returned.
   */
  std::string getElementText(const std::string& name, bool* found) const;
  /**
   * @brief Helper method to return the values of the children with a specified
   * name with the parent from the specified parent.
   *
   * This method first verifies if there exists a child node of the document
   * root XML node with the specified name.
   *
   * If this element exists, the method will iterate trough trough the child
   * nodes of this element, verify if these nodes have the specified childName
   * and return a list with all of the values from the leaf nodes.
   *
   * @param name Name of the parent element.
   * @param childName Name of the child elements.
   * @param found Indicates if the parent element was found or not.
   * @return stringlist List containing the text values of the child elements of
   * the name element that have the specified childName.
   */
  stringlist getChildElementValues(const std::string& name, const std::string& childName, bool* found) const;
  /**
   * @brief Helper method to return the integer value of a XML element with the
   * specified name.
   *
   * @param name Name of the node to return the value.
   * @param found Indicates if a node with the specified name was found or not.
   * @return int Integer value of the node.
   */
  int getElementInt(const std::string& name, bool* found) const;
  /**
   * @brief Helper method to return the unsigned integer with 64 bits value of
   * a XML element with the specified name.
   *
   * @param name Name of the element to return the value.
   * @param found Indicates if a node with the specified name was found or not.
   * @return uint64_t Unsigned 64 bits integer value of the element.
   */
  uint64_t getElementUint64(const std::string& name, bool* found) const;
  /**
   * @brief Helper method to return the bool value of a XML element with the
   * specified name.
   *
   * Recognizes TRUE, true, and any combination of upper and lower case letters
   * in the word true, and the integer 1 as true. Everything else is returned
   * as false.
   *
   * @param name Name of the element to return the value.
   * @param found Indicates if an elment with the specified name was found or not.
   * @return bool Value of the element.
   */
  bool getElementBool(const std::string& name, bool* found) const;
};

#endif  // PROVANT_CONFIG_READER_H
