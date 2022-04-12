/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file This file contains the declaration of the AutoSteppingGUIPlugin class.
 *
 * This class implements a plugin that allows starting, pausing, and single
 * stepping a controller in the ProVANT Simulator.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef AUTO_STEPPING_H
#define AUTO_STEPPING_H

#include <mutex>
#include <string>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <QHBoxLayout>
#include <QIcon>
#include <QPushButton>

namespace gazebo
{
/**
 * @brief The Auto Stepping GUI Plugin is a Gazebo GUI plugin that shows a
 * GUI overlay widget that allows controlling the simulation mode in the
 * ProVANT Simulator controller.
 *
 * This class contains two buttons, one that allows starting and pausing the
 * simulation, and one that allows the user to advance the simulation by a
 * single step.
 *
 * This plugin was created in order to facilitate the debugging process of
 * control strategies and plugins other plugins for the simulator, and to
 * allow the user greater control of a simulation. For example, with the use
 * of this plugin, the user can finish a simulation without having to kill the
 * Gazebo GUI and Server when an experiment is finished.
 */
class GAZEBO_VISIBLE AutoSteppingGUIPlugin : public GUIPlugin
{
  Q_OBJECT

public:
  /**
   * @brief Construct a new Auto Stepping GUI Plugin object.
   */
  AutoSteppingGUIPlugin();

  /**
   * @brief Destroy the Auto Stepping GUI Plugin object and shutdown the
   * execution of every ROS service and node advertised or subscribed by this
   * plugin.
   */
  virtual ~AutoSteppingGUIPlugin();

  /**
   * @brief Method called once when the plugin is loaded.
   *
   * Verify that a ROS Node for gazebo is running, setups the user interface
   * objects and enable the buttons in the UI.
   *
   * @param sdf Pointer to the SDF plugin element that added this plugin to the
   * GUI. If the plugin is loaded via the command line and not trough the SDF
   * configuration, this parameter is NULL.
   */
  virtual void Load(sdf::ElementPtr sdf) override;

  /**
   * @brief Indicates if the simulation if paused or not.
   * @sa IsRunning()
   *
   * @return true if the simulation is paused and false otherwise.
   */
  bool IsPaused() const;
  /**
   * @brief Indicates if the simulation is running or not.
   * @sa IsPaused()
   *
   * @return true if the simulation is running and false otherwise.
   */
  bool IsRunning() const;

protected:
  //! ROS Node Handle to manage the services and topics of this plugin.
  ros::NodeHandle nodeHandle;

  /**
   * @brief Publishes a message on the Step topic in order to advance the
   * simulation by one step.
   */
  void AdvanceStep();

public slots:
  /**
   * @brief Defines the state of the UI elements of the plugin.
   *
   * @param enabled Indicates if the elements of the user interface should be
   * enabled (true) or disabled (false).
   */
  void SetEnabled(bool enabled = true);
  /**
   * @brief Helper method to enable the elements in the UI for user interation.
   * @sa SetEnabled()
   */
  void Enable();
  /**
   * @brief Helper method to disable the elements in the UI.
   * @sa SetEnabled()
   */
  void Disable();

protected slots:
  /**
   * @brief Method called when the play/pause pushbutton is clicked.
   *
   * Updates the state of the _paused variable, and make appropriate calls to
   * the enable_autostepping service.
   *
   * The enable_autostepping service is used to indicate if the controller node
   * should automatically advance the simulation step at the end of the control
   * law calculation. If this is enabled, the simulation will run continuously
   * until the pause pushbutton is clicked.
   */
  void OnPlayPausePushbuttonClick();
  /**
   * @brief Method called when the step pushbutton is clicked.
   *
   * Check if the simulation is paused, if it is, sends a message on the Step
   * topic in order to advance the simulation by one step.
   *
   * Note: This method checks if the simulation is paused before proceeding
   * to avoid advancing the simulation step during the middle of a control law
   * calculation.
   */
  void OnStepPushbuttonClick();

private:
  /**
   * @brief Layout that holds every element in the plugin UI.
   */
  QHBoxLayout* _mainLayout = nullptr;
  /**
   * @brief Icon used on the play/pause pushbutton when the simulation is
   * paused. Used to indicate that clicking the button will start the simulation
   * is continuous mode.
   */
  QIcon _playIcon = QIcon(":/images/start.svg");
  /**
   * @brief Icon used on the play/pause pushbutton when the simulation is
   * running. Used to indicate that clicking the button will pause the
   * simulation.
   */
  QIcon _pauseIcon = QIcon(":/images/pause.svg");
  /**
   * @brief Play/Pause pushbutton. Used to set the running mode of the
   * simulation. If the simulation is paused, clicking this button will put the
   * simulation in continuos run mode, and if the simulation is running, it will
   * pause the simulation.
   */
  QPushButton* _playPausePushbutton = nullptr;
  /**
   * @brief Step pushbutton. Used to advance a single step in the simulation.
   */
  QPushButton* _stepPushbutton = nullptr;

  /**
   * @brief Name of the child logger this plugin uses to publish its messages.
   */
  const std::string PLUGIN_ID = "auto_stepping";

  /**
   * @brief Indicates if the simulation is paused or not.
   */
  bool _paused = true;

  /**
   * @brief Mutex used to control the updating of the _paused variable.
   */
  std::mutex _pausedMutex;

  /**
   * @brief Message used to identify this plugin instance in log messages.
   */
  std::string _logMsg;

  /**
   * @brief Previous state of the simulation, used in the updating of the
   * simulation state.
   */
  std::string _prevState = "";

  /**
   * @brief ROS publisher handler for the Step topic.
   * Used in order to advance the simulation steps.
   */
  ros::Publisher _stepPublisher;
  /**
   * @brief ROS service client for the enable_autostepping service.
   * Used in order the control the simulation mode in the controller node.
   */
  ros::ServiceClient _playPauseClient;
  /**
   * @brief ROS subscriber to the provant_simulator/simulation_state topic.
   */
  ros::Subscriber _simulationStateSubscriber;

  /**
   * @brief Helper method that constructs and setups the widgets in the user
   * interface.
   */
  void SetupUI();
  /**
   * @brief Helper method that initializes the publisher for the Step topic.
   * @sa _stepPublisher
   */
  void SetupStepTopic();
  /**
   * @brief Helper method that initializes the subscriber for the 
   * /simulation_state topic.
   */
  void SetupSimulationStateTopic();
  /**
   * @brief Helper method that initializes the enable_autostepping service
   * client.
   * @sa _playPauseClient
   */
  void SetupPlayPauseService();
  /**
   * @brief Method called when new messages are received in the 
   * /simulation_step topic.
   * 
   * The simulation_step topic is used to bradcast strings indicating in which
   * state the simulation is, the valid options are:
   *  * starting -> The simulation is still opening and initializing its nodes,
   * and therefore it still is not possible to start, pause or step the 
   * simulation. At this state the GUI must be disabled.
   *  * ready -> All of the nodes used in the simulation are started and fully
   * initialized, but the simulation is in the start paused mode, and the user
   * has not started the simulation yet. At this state the GUI must be enabled.
   *  * running -> All of the nodes used in the simulation are started, and the
   * user or the controller node has already started the simulation. At this
   * state the GUI must be enabled.
   *  * paused -> The simulation was already started, but was paused by the
   * user. At this state the GUI must be enabled.
   *  * finished -> The simulation has reached the specified number of steps and
   * is finished, but waiting user action. At this state the GUI must be 
   * enabled.
   *  * shuttingdown -> The simulation has finished, and the controller node is
   * taking the steps necessary to close the nodes used in the simulation. At
   * this state the GUI must be disabled.
   * 
   * @param msg Pointer to the message received in the topic.
   */
  void SimulationStatusCallback(const std_msgs::String::ConstPtr& msg);
  /**
   * @brief Helper method used to update the icon showing in the Play/Pause
   * pushbutton icon, if the simulation is paused, the play icon is shown in
   * the button, if the simulation is not paused, the pause pushbutton is shown
   * in the screen.
   * 
   * @param paused Indicates if the simulation is paused or not.
   */
  void UpdatePlayPausePushbuttonIcon(bool paused);
};
}  // namespace gazebo

#endif  // AUTO_STEPPING_H
