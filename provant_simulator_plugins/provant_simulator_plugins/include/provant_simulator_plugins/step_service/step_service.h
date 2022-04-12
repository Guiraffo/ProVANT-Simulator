/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file step_service.h
 * @brief This file contains the declaration of the StepServicePlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_STEP_SERVICE_H
#define PROVANT_STEP_SERVICE_H

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

#include <ros/ros.h>

#include <std_srvs/Trigger.h>

namespace provant
{
namespace plugins
{
/**
 * @brief The StepServicePlugin exposes a ROS service that allows other components of the ProVANT Simulator to request
 * simulator steps in a synchronous manner.
 *
 * The /provant_simulator/step service will only return after a step was finished by the Gazebo simulator or in case of
 * a timeout after 120 seconds, thus allowing the service caller to ensure a step was taken by the simulator.
 */
class StepServicePlugin : public gazebo::WorldPlugin
{
public:
  /**
   * @brief Construct a new Step Service Plugin object.
   */
  StepServicePlugin() = default;

  /**
   * @brief Method called once when the plugin is loaded by the Gazebo simulator.
   *
   * This method will configure the logging message for this plugin instance, check if ROS is initialized, and in case
   * of success advertise the /provant_simulator/step service.
   *
   * @param _world World which contains this plugin.
   * @param _sdf Unused paramter, pointer to the plugin SDF configuration.
   */
  void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Request handler for the /provant_simulator/step service.
   *
   * When a new request for a world step is received this method will trigger a new world step and wait until a
   * WorldUpdateEnd event is finished in order to confirm that the step was taken by the simulator.
   *
   * If the simulator does not respond in 120 seconds, the request fails and an appropriate error message is emitted.
   *
   * @param req Unused. Request data for the service.
   * @param res Response data for the service. Has a flag indicating if the operation was successfull and a message for
   * use in case of error.
   * @return true If the step request succeeds.
   * @return false Otherwise.
   */
  virtual bool StepRequestHandler(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);
  /**
   * @brief Get the log message used to identify this plugin.
   *
   * Available for testing purposes.
   *
   * @return const std::string&
   */
  virtual const std::string& getLogMessage() const;

private:
  /// Pointer to the simulation world. Used to request steps.
  gazebo::physics::WorldPtr _world;
  /// Log message used to identify this plugin instance in the log output.
  std::string _logMsg;
  /// Name of the named log used for publication of the plugin log messages.
  const std::string PLUGIN_ID = "step_service_plugin";
  /// ROS Node Handle that manages the /provant_simulator/step service.
  ros::NodeHandle _rosNodeHandle;
  /// ROS Service server for the /provant_simulator/step service.
  ros::ServiceServer _server;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_STEP_SERVICE_H
