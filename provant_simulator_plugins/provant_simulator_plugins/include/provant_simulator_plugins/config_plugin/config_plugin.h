/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file config_plugin.h
 * @brief This file contains the declaration for the ConfigPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CONFIG_PLUGIN_H
#define PROVANT_CONFIG_PLUGIN_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/response.pb.h>
#include <gazebo/transport/transport.hh>

#include <string>

namespace provant
{
namespace plugins
{
/**
 * @brief The ConfigPlugin is a Gazebo system plugin that receives an option
 * containing the path the config.xml file for the current simulation and
 * provides a service that allows recovery of this content.
 *
 * This plugin is a crucial part of the infrastructure developed to allow other
 * plugins, especially the plugins developed for the synchronous hardware in the
 * loop (HIL) server plugins that cannot rely on ROS for their functionality.
 *
 * This plugin uses the Gazebo response and request mechanism to provide the
 * contents of the config.xml file that contains the settings for the current
 * simulation such as the control strategy name and simulation mode.
 *
 * When a message is received in the Gazebo ~/request topic, the service
 * providers must check the request field of the request protobuf message. This
 * field identifies the service that is being requested. If the service provides
 * the requested service, it must publish a response protobuf message in the
 * ~/response gazebo topic.
 *
 * The reponse message must contain the same ID and request as the received
 * request message. The field response is used to identify if the process
 * succeed or failed and any other status about the service. Optionally a
 * response message can contain a byte encoded protobuf message.
 *
 * To use the optional response fields, the type field must contain the type
 * name of the encoded message, and the data field must contain the serialized
 * message.
 */
class ConfigPlugin : public gazebo::SystemPlugin
{
public:
  /**
   * @brief Construct a new Config Plugin object.
   */
  ConfigPlugin() = default;
  /**
   * @brief Destroy the Config Plugin object.
   */
  virtual ~ConfigPlugin() = default;

  /**
   * @brief Method called when the plugin is loaded by Gazebo.
   *
   * This method calls the other methods used in the plugin configuration and
   * initialization, including configuring the log message that identifies
   * this plugin instance.
   *
   * This method will loop trough the received parameters looking for the
   * --provant_config parameter, if this parameter is found, this string is
   * parsed obtaining the path to the config.xml file.
   *
   * The path to the file is then verified to check if it exists, is a valid
   * file and is readable to the current user. After verification the contents
   * of the file are read, and fed to the ConfigReader class to ensure that
   * the contents are a valid XML file and can be correctly parsed.
   *
   * After that, the method connects to the WorldCreated signal, and exits.
   * The remainder of the plugin configuration is executed by the
   * SetupGazeboTopic() method called after the Gazebo transport layer is
   * fully initialized.
   *
   * @param argc Number of command line arguments passed to Gazebo.
   * @param argv Pointer to char arrays containing the command line arguments
   * received by Gazebo.
   */
  void Load(int argc, char** argv) override;
  /**
   * @brief Method called once after the Load method.
   *
   * Used to emit a log message informing that the plugin finished loading.
   */
  void Init() override;

protected:
  /**
   * @brief Receives the full content of the --provant_config option, and parses
   * this string for the path of the config.xml file path.
   *
   * @param configOpt Full content of the --provant_config option.
   * @return std::string Path to the config.xml file.
   */
  std::string ParseConfigPath(const std::string& configOpt) const;
  bool VerifyConfigPath(const std::string& path) const;
  /**
   * @brief Read the contents of the provided config.xml file.
   *
   * @param path Path to the config.xml file.
   * @return true If reading the file succeeds and false otherwise.
   */
  bool ReadConfigFile(const std::string& path);
  /**
   * @brief Verifies if the contents of the provided config.xml file are a
   * valid XML string and can be parsed by the ConfigReader class.
   *
   * @return true If the contents of the file can be read and parsed into a
   * valid ConfigReader object and false otherwise.
   */
  bool VerifyConfigReader() const;
  /**
   * @brief Constructs a response message with the constant fields filled.
   *
   * Some fields on the response message sent by this plugin when a valid
   * request is received never change, such as the response, that indicates
   * the success or failure of the operation, the type field which indicates
   * the kind of the message encoded in the data field, and the data field
   * that in the case of this plugin contains a encoded GzString message with
   * the contents of the config.xml file.
   *
   * As these fields don't change, we create a pre-built message that can be
   * used as a base to construct new responses.
   */
  void PrepareResponseMessage();
  /**
   * @brief Connect to WorldCreated signal.
   *
   * The WorldCreated signal is used to notify this plugin that the Gazebo
   * tranport layer is initialized and that the plugin loading may proceed.
   *
   * This method will connect the OnWorldCreated method as a handler for the
   * WorldCreated signal, and that method must then finish the plugin
   * configuration.
   *
   * @sa OnWorldCreated()
   */
  void ConnectToWorldCreated();
  /**
   * @brief Configures the communication with the request/response engine
   * using the Gazebo transport layer.
   *
   * Please note that this method cannot be executed during loading, as the
   * Gazebo initialization process will not have finished, therefore we call
   * this function at the WorldCreated event.
   *
   * This method will create and initialize a Gazebo node handler, and use it
   * to advertise the ~/response topic, and subscribe to the ~/request topic.
   *
   * @sa OnRequest(ConstRequestPtr&)
   */
  bool SetupGazeboTopic();

  /**
   * @brief Method called when a new message is received on the ~/request topic.
   *
   * If this method is called with a message containing a request ot the
   * provant_config service, it will create a copy of the response message
   * that contains the contents of the config.xml file for the current
   * simulation, fill in the message id and request type and publish it in the
   * ~/response topic.
   *
   * If this method receives any other service request, it does nothing.
   *
   * @param req Request message.
   */
  void OnRequest(ConstRequestPtr& req);
  /**
   * @brief Method called when the WorldCreated signal is received.
   *
   * As it is not possible to initialize the communication on the ~/response
   * and ~/request topics during the Load phase, because Gazebo hasn't yet
   * fully initialized its transport layer, we defer the connection to these
   * topics to after the WorldCreated signal is received.
   *
   * After running for the first time, moment when this functions finishes
   * configuration of the plugin, it must release the connection pointer
   * to the WorldCreated signal, ensuring this function won't unnecessarily
   * be called again in the future.
   */
  void OnWorldCreated();

  //! Connection pointer to the WorldCreated signal
  gazebo::event::ConnectionPtr _worldCreatedPtr;
  //! Gazebo Node handler for the current plugin.
  gazebo::transport::NodePtr _gzNode;
  //! Subscriber to the ~/request gazebo topic.
  gazebo::transport::SubscriberPtr _requestSubscriber;
  //! Publisher to the ~/response gazebo topic.
  gazebo::transport::PublisherPtr _responsePublisher;
  //! Pre filled response message containing the response, type, and data fields already filled.
  gazebo::msgs::Response _responseMessage;

private:
  //! Message used to identify the specific plugin instance at the log output
  std::string _logMsg;

  //! Contents of the config.xml file for the current simulation
  std::string _configFileContents;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_CONFIG_PLUGIN_H
