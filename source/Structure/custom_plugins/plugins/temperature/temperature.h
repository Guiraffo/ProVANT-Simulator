/*
 * File: temperature.h.
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description:  This library is responsable to implement the temperature sensor
 */

#ifndef PROVANT_TEMPERATURE_PLUGIN_H
#define PROVANT_TEMPERATURE_PLUGIN_H

#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "ros/ros.h"
#include <random>
#include "quantization.h"
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"

namespace gazebo
{
class temp_press : public SensorPlugin
{
  std::default_random_engine generator;               // random generator of numbers
  std::normal_distribution<double> distributionTEMP;  // noise of temperature has normal distribution
  std::normal_distribution<double> distributionBARO;  // noise of barometerS has normal distribution

public:
  temp_press() = default;
  virtual ~temp_press() = default;
  // initial setup
 void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) override;

private:
// for each step time
  virtual void OnUpdate();
  // pointer to the sensor
  sensors::AltimeterSensorPtr parentSensor;
  // connection to the simulation time
  event::ConnectionPtr updateConnection;
  // ROS node handle
  ros::NodeHandle n;
  // publisher
  ros::Publisher temp_pub;
  // name of topic to publish data
  std::string Topic_;
  double TempOffset;  // offset oh temperature
  double TempStandardDeviation;  // Stardard deviation of temperature
  double BaroOffset;  // offset of barometer
  double BaroStandardDeviation;  // standard deviation of barometer
  double maxtemp;  // max limit
  double mintemp;  // min limit
  double maxbaro;  // max limit
  double minbaro;  // min limit
  double Nbits;  // number of bit to quantize continous signal
};
}  // namespace gazebo

#endif  // PROVANT_TEMPERATURE_PLUGIN_H
