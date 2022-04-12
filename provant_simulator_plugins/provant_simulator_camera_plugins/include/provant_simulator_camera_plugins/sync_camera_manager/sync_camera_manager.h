/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file sync_camera_manager.h
 * @brief This file contains the declaration of the SyncCameraManager class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_SYNC_CAMERA_MANAGER_H
#define PROVANT_SYNC_CAMERA_MANAGER_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/request.pb.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include <map>
#include <mutex>
#include <vector>
#include <thread>

namespace provant::camera_plugins
{
/**
 * @brief The Sync Camera Manager is a Gazebo World plugin created to manage synchronize the capture of user camera
 * frames according to the simulation steps.
 *
 * Due to the Gazebo architecture which is divided into client and server applications, the plugins running on those
 * components do not have access to all features. Crucially for the case of this plugin, plugins running on the
 * Gazebo server, such as system, world, model, sensor, and visual plugins do not have access to the user camera.
 *
 * The plugins running on the client application, such as system and GUI plugins have access to the user camera, but
 * don't have access to the WorldUpdateBegin and WorldUpdateEnd signals, nor any other way that provides synchronized
 * access to the gazebo step iteration process.
 *
 * Due to the constraints of each of these plugins, the SyncCameraManager was designed to provide a solution. This
 * plugin runs on the Gazebo server, and keeps a list of active camera recorders, on the end of each step update, the
 * plugin will send a request using the Gazebo transport system, and wait for the response from all of the currently
 * active synchronized camera recorders. This ensures that the step is only advanced after each of those recorders
 * captured and saved a new video frame.
 *
 * Note that this action is only possible because the Gazebo request and response cycle is synchronous, providing
 * functionality similar to ROS services. In this case synchronous means that when a request is made, the execution
 * of the program blocks until a response is received or a timeout ocurred.
 *
 * When a synchronized camera recorder is loaded, it must send a request to the Sync Camera Manager plugin informing
 * that a recorder is enabled. The manager plugin will then send a request to the recorder, querying the step number
 * in which the recorder must be enabled.
 *
 * After the initialization, the manager plugin will pause the simulation on each step time, and wait for a response
 * from the recorder, thus allowing correct synchronization between the plugins running on the client and server
 * applications.
 */
class SyncCameraManager : public gazebo::WorldPlugin
{
public:
  SyncCameraManager();
  virtual ~SyncCameraManager() = default;
  /**
   * @brief Loads the plugin and configures the gazebo transport node and signal connections.
   *
   * @param _world Pointer to the simulation world that contains this plugin.
   * @param _sdf Pointer to the SDF element that contains this plugin options. Unused.
   */
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Create and initialize the gazebo tranport node, advertise the response topic, and subscribe to the request
   * topic.
   *
   * @return true If the configuration succeeds.
   * @return false Otherwise.
   */
  virtual bool SetupTransportNode();
  /**
   * @brief Handler for messages received on the ~/request topic.
   *
   * Checks if the received message is for one of the requests provided by this plugin, and if so, forwards the call
   * to appropriate handler method.
   *
   * @sa OnEnableRecorderRequest(const ConstRequestPtr&)
   * @sa OnDisableRecorderRequest(const ConstRequestPtr&)
   *
   * @param req Received request message.
   */
  virtual void OnRequest(const ConstRequestPtr& req);
  /**
   * @brief Handler for requests to enable synchronized recorders.
   *
   * This method will check if a valid request was sent, i.e., a request that contains the name of the synchronized
   * recorder plugin in its data field and sends a response message.
   *
   * If a valid request was received, the method will then send a new request to the recorder plugin in order to obtain
   * the step number in which recording must be initiated.
   *
   * @param req Received request message for the enable service.
   */
  virtual void OnEnableRecorderRequest(const ConstRequestPtr& req);
  /**
   * @brief Handler for requests to disable synchronized recorders.
   *
   * Checks if a valid request was sent, and if so, removes the desired recorder from the active recorders list.
   *
   * @param req
   */
  virtual void OnDisableRecorderRequest(const ConstRequestPtr& req);

  /**
   * @brief Helper method to publish response messages.
   *
   * This method will fill in the appropriate fields in a response message and publish it with the desired response
   * in the appropriate topic.
   *
   * @param req Request message that originated the response.
   * @param response Content of the response field. Must contain the status of the request execution, i.e., success or
   * error.
   */
  virtual void SendResponse(const ConstRequestPtr& req, const std::string response);

  /**
   * @brief Handler for the World Update End signal.
   *
   * Loops trough the active recorders list, and sends requests for each of the enabled camera recorders to grab a new
   * frame.
   */
  virtual void OnWorldUpdateEnd();

  /**
   * @brief Method that sends a request to a recorder in order to obtain the initial step when a recorder must be
   * enabled.
   *
   * @param recorder_name Name of the recorder to send the request.
   */
  virtual void RequestInitialStep(std::string recorder_name);

private:
  //! Pointer to the simulation world.
  gazebo::physics::WorldPtr _world;

  //! Message used to uniquely identify this plugin in the log output.
  std::string _logMsg;

  //! Gazebo node to advertise and subscribe to the request and response topics.
  gazebo::transport::NodePtr _gzNode;
  //! Publisher to the ~/response topic. Used to respond to request messages.
  gazebo::transport::PublisherPtr _responsePublisher;
  //! Subscriber connection to the ~/request topic. Used to receive request messages.
  gazebo::transport::SubscriberPtr _requestSubscriber;

  //! Mutex to synchronize access to the _activeRecorders map.
  std::mutex _activeRecordersMutex;
  //! Map containing the name of the currently active recorders (the map key), and the step number in which the recorder
  //! must be enabled (the map value)
  std::map<std::string, int32_t> _activeRecorders;

  //! Mutex to synchronize access to the _enableOnThreads vector.
  std::mutex _enableOnThreadsMutex;
  //! Keeps the thread objects used to query the recorders about the initial step when they should be enabled.
  std::vector<std::thread> _enableOnThreads;

  //! Connection to the Gazebo WorldUpdateEnd signal. See the OnWorldUpdateEnd() method.
  gazebo::event::ConnectionPtr _worldUpdateEndConn;
};
}  // namespace provant::camera_plugins

#endif  // PROVANT_SYNC_CAMERA_MANAGER_H
