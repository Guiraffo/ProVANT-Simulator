/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gzconfig_reader.h
 * @brief This file contains the declaration of the GazeboConfigReader class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_GZCONFIG_READER_H
#define PROVANT_GZCONFIG_READER_H

#include <string>

#include <provant_simulator_xml_reader/config_reader.h>

/**
 * @brief The GazeboConfigReader class allows the user to obtain an instance
 * of the ConfigReader class inside a Gazebo plugin.
 *
 * During the refactoring of the Hardware in The Loop (HIL) server plugins, the
 * need to read the ConfigReader object arose.
 *
 * Usually, the plugins are expected to read the path of the file from a ROS
 * parameter and create its own instance of the ConfigReader object with the
 * file path, however, the HIL plugins cannot rely on ROS, because we plan on
 * removing the depency on ROS completely to allow the use of the synchronous
 * HIL mode in low spec hardware.
 *
 * To fullfill this issue, a new plugin, the ConfigReader, was developed.
 * This is a system plugin launched at the start of every simulation that
 * provides a service using the Gazebo tranport layer that can be quired for
 * the content of the ConfigReader file.
 *
 * This class is a helper that allows users to obtain an instance of the
 * ConfigReader object inside Gazebo plugins, without any run time depency
 * on ROS. When a new instance is requested, a message is sent on the ~/request
 * topic and wait for a response message in the ~/response topic.
 *
 * The response message contains the content of the config.xml file encoded
 * as a GzString message, this content will be parsed to a std::string object
 * and fed to the ConfigReader parser. If the process succeeds a pointer to
 * an instance of the ConfigReader class is returned.
 *
 * Note that the user must take ownership and delete this instance after it is
 * no longer used.
 *
 * This class only works if the ConfigReader plugin is loaded successfully, what
 * is expected during normal simulator operation.
 */
class GazeboConfigReader
{
public:
  /**
   * @brief Construct a new Gazebo Config Reader object.
   *
   * @param worldName Name of the current simulation world.
   */
  GazeboConfigReader(const std::string& worldName);

  /**
   * @brief Get the name of the current simulation world.
   *
   * @return const std::string&
   */
  const std::string& GetWorldName() const;

  /**
   * @brief Create an instance of a config reader.
   *
   * This method will send a request for the contents of the config.xml file
   * for the current simulation, and wait for a response of the ConfigPlugin.
   *
   * If the response is valid, and the parsing of the file content succeeds
   * this methods returns a pointer to a ConfigReader object with the value of
   * the simulation settings populated.
   *
   * Note that you must take ownership of the config reader pointer and delete
   * it after it is no longer necessary.
   *
   * @return ConfigReader* Pointer to a config reader object if parsing succeeds
   * and a nullptr otherwise.
   */
  ConfigReader* GetConfigReader() const;

private:
  //! Name of the current simulation world
  std::string _worldName;
};

#endif  // PROVANT_GZCONFIG_READER_H
