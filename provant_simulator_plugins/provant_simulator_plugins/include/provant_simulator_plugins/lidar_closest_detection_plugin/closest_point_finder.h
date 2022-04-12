/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file lidar_closest_obj.h
 * @brief This file contains the declaration of the ClosestPointFinder class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#ifndef PROVANT_LIDAR_CLOSEST_OBJ_H
#define PROVANT_LIDAR_CLOSEST_OBJ_H

#include <gazebo/msgs/laserscan_stamped.pb.h>
#include <gazebo/sensors/GpuRaySensor.hh>

#include <simulator_msgs/Sensor.h>

namespace provant
{
namespace plugins
{
/**
 * @brief The Closest Point Finder class receives a pointer to a GPU Ray Sensor and provides a method that analyzes
 * the readings of the sensor fiding the closest detected point and returing a message with this point data.
 *
 * For more details on the data, format and measure units of the returned message, see the documentation of the
 * @sa OnNewScan(ConstLaserScanStampedPtr&) method.
 *
 */
class ClosestPointFinder
{
public:
  /**
   * @brief Construct a new Closest Point Finder object
   *
   * @param sensor Pointer to the GPU Ray Sensor that makes the readings analyzed by this object.
   * @param logMsg Message that uniquely identifies the messages of this sensor in the log output.
   * @param pluginId Name of the ROS child logger that outputs the log messages of this sensor.
   *
   * @throw std::invalid_argument If the provided sensor pointer is a null.
   */
  ClosestPointFinder(gazebo::sensors::GpuRaySensorPtr sensor, const std::string& logMsg, const std::string& pluginId);

  /**
   * @brief Iterates over the readings of a LIDAR Sensor, and returns a Sensor message with the data of the closest
   * point.
   *
   * The returned message has the following data:
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
   *
   * @param msg Message containing the data of the closest detected point.
   * @return simulator_msgs::Sensor
   */
  simulator_msgs::Sensor OnNewScan(ConstLaserScanStampedPtr& msg);

private:
  /// Pointer to the GPU Ray Sensor that makes the readings.
  gazebo::sensors::GpuRaySensorPtr _sensor;
  /// Log message used to uniquely identify the messages of this sensor in the log output.
  const std::string _logMsg;
  /// Name of the ROS child logger used to output the log messages of this sensor.
  const std::string PLUGIN_ID;
};
}  // namespace plugins
}  // namespace provant

#endif  // PROVANT_LIDAR_CLOSEST_OBJ_H
