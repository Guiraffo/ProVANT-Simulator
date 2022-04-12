/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file lidar_closets_obj_ros.h
 * @brief This file contains the definition of the LIDARClosestObjectPlugin class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_CLOSEST_OBJ_ROS_H
#define PROVANT_CLOSEST_OBJ_ROS_H

#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <gazebo/plugins/GpuRayPlugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>

#include <ros/ros.h>

#include <memory>
#include <thread>

namespace provant
{
namespace plugins
{
class ClosestPointFinder;

/**
 * @brief The LIDARClosestObjectPlugin is a Gazebo sensor plugin that receives
 * readings from a LIDAR sensor, finds the closest detected point and publishes
 * a message in a ROS topic with data relative to this point.
 */
class LIDARClosestObjectPlugin : public gazebo::GpuRayPlugin
{
public:
  LIDARClosestObjectPlugin() = default;
  /**
   * @brief Loads the LIDARClosestObject plugin.
   *
   * @param _sensor Pointer to the sensor managed by this plugin.
   * @param _sdf Pointer to the SDF element containing the plugin options.
   */
  void Load(gazebo::sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

protected:
  /**
   * @brief Finishes the loading process.
   *
   * Initialize the ROS and Gazebo nodes, subscribes and advertises the necessary topics.
   * This method also initializes the ClosestPointFinder object.
   */
  void SetupNodes();
  /**
   * @brief Receives readings from the LIDAR sensor, process the data and publishes a message with the data of the
   * closest detected point.
   *
   * @param msg Message with the data of the closest point, in the following order:
   *
   * - Distance of the closest object in meters;
   * - Azimuthal angle of the closest object in radians;
   * - Polar angle of the closest object in radians;
   * - Position in meters of the x axis of the closest point, relative to the sensor reference frame, expressed in the
   * sensor reference frame;
   * - Position in meters of the y axis of the closest point, relative to the sensor reference frame, expressed in the
   * sensor reference frame;
   * - Position in meters of the z axis of the closest point, relative to the sensor reference frame, expressed in the
   * sensor reference frame.
   */
  void OnNewScan(ConstLaserScanStampedPtr& msg);

private:
  /// Pointer to the Ray Sensor managed by this plugin.
  gazebo::sensors::GpuRaySensorPtr _raySensor;
  /// Object that analyzes the sensor readings and creates messages with the data.
  std::unique_ptr<ClosestPointFinder> _finder;

  /// Stores a thread used to finish the initialization process.
  std::thread _initializationThread;

  /// Name of the ROS topic used to publish the messages with the sensor data.
  std::string _topicName;
  /// ROS node handle used to advertise the sensor readings topic.
  ros::NodeHandle _rosNodeHandle;
  /// Publisher used to advertise the sensor readings topic.
  ros::Publisher _publisher;

  /// Gazebo node used to receive the sensor readings.
  gazebo::transport::NodePtr _gzNode;
  /// Gazebo subscriber to receive the sensor readings.
  gazebo::transport::SubscriberPtr _raySensorSub;

  /// Message that uniquely identify this plugin instance log messages in the log output.
  std::string _logMsg;
  /// Name of the ROS child logger used to output he log messages of this plugin.
  const std::string PLUGIN_ID = "lidar_closest_obj_plugin";
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_CLOSEST_OBJ_ROS_H
