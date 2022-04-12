/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file lidar_closest_obj.cpp
 * @brief This file contains the implementation of the ClosestPointFinder class.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include "provant_simulator_plugins/lidar_closest_detection_plugin/closest_point_finder.h"

#include <ros/ros.h>

#include <numeric>
#include <optional>

using provant::plugins::ClosestPointFinder;

ClosestPointFinder::ClosestPointFinder(gazebo::sensors::GpuRaySensorPtr sensor, const std::string& logMsg,
                                       const std::string& pluginId)
  : _sensor(std::move(sensor)), _logMsg(logMsg), PLUGIN_ID(pluginId)
{
  if (!this->_sensor)
  {
    throw std::invalid_argument("The received sensor pointer is null.");
  }

  // Initialize the sensor
  this->_sensor->SetActive(true);
}

simulator_msgs::Sensor ClosestPointFinder::OnNewScan(ConstLaserScanStampedPtr& msg)
{
  if (!_sensor)
  {
    ROS_ERROR_STREAM_NAMED(PLUGIN_ID, _logMsg << "Trying to execute scan with a null sensor provided.");
    return simulator_msgs::Sensor{};
  }

  const auto rangeCount = _sensor->RangeCount();
  const auto verticalRangeCount = _sensor->VerticalRangeCount();

  // If it finds a minimum distance it has value and is empty otherwise
  std::optional<double> minDist;
  int minDistRow = 0;
  int minDistCol = 0;

  // Iterate and find closest distance point
  for (int i = 0; i < rangeCount; i++)
  {
    for (int j = 0; j < verticalRangeCount; j++)
    {
      const auto r = msg->scan().ranges(i + j * rangeCount);

      // Ignore invalid numbers (-/+inf and NaN, or outside the range of the sensor)
      if (!isfinite(r) || r < _sensor->RangeMin() || r > _sensor->RangeMax())
        continue;

      if (r < minDist.value_or(std::numeric_limits<double>::infinity()))
      {
        minDist = r;
        minDistRow = i;
        minDistCol = j;
      }
    }
  }

  if (minDist.has_value())
  {
    // Calculate angles of the minimum distance point
    const auto minAngle = _sensor->AngleMin().Radian();
    const auto maxAngle = _sensor->AngleMax().Radian();
    const auto yDiff = maxAngle - minAngle;

    const auto verticalMinAngle = _sensor->VerticalAngleMin().Radian();
    const auto verticalMaxAngle = _sensor->VerticalAngleMax().Radian();
    const auto pDiff = verticalMaxAngle - verticalMinAngle;

    const auto yAngle = rangeCount > 1 ? minDistRow * yDiff / (rangeCount - 1) + minAngle : minAngle;
    const auto pAngle =
        verticalRangeCount > 1 ? minDistCol * pDiff / (verticalRangeCount - 1) + verticalMinAngle : verticalMinAngle;

    // Calculate coordinates of the obstacle in the sensor reference frame
    const auto x = minDist.value() * cos(pAngle) * cos(yAngle);
    const auto y = minDist.value() * cos(pAngle) * sin(yAngle);
    const auto z = minDist.value() * sin(pAngle);

    // Assemble message
    simulator_msgs::Sensor sensorMsg{};
    sensorMsg.header.stamp = ros::Time::now();
    sensorMsg.values = { minDist.value(), yAngle, pAngle, x, y, z, ros::Time::now().toSec() };
    return sensorMsg;
  }
  else
  {
    // If no object in range was found, publish a message reporting an object at an infinite distance
    simulator_msgs::Sensor sensorMsg{};
    const auto inf = std::numeric_limits<double>::infinity();
    sensorMsg.header.stamp = ros::Time::now();
    sensorMsg.values = { inf, 0, 0, 0, 0, inf, ros::Time::now().toSec() };
    return sensorMsg;
  }
}
