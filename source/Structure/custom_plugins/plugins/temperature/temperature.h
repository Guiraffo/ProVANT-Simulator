/*
* File: temperature.h.
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement the temperature sensor
*/

#ifndef _GAZEBO_TEMP_PLUGIN_HH_
#define _GAZEBO_TEMP_PLUGIN_HH_


#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include "ros/ros.h"
#include <random>
#include "quantization.h"
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"


namespace gazebo
{
  /// \brief An example plugin for a contact sensor.
  class temp_press : public SensorPlugin
  {
    std::default_random_engine generator; // random generator of numbers
    std::normal_distribution<double> distributionTEMP; // noise of temperature has normal distribution
    std::normal_distribution<double> distributionBARO; // noise of barometerS has normal distribution

    // contructor
    public: temp_press();
    // destructor
    public: virtual ~temp_press();
    // initial setup
    public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    // for each step time
    private: virtual void OnUpdate();
    // pointer to the sensor
    private: sensors::AltimeterSensorPtr parentSensor;
    // connection to the simulation time
    private: event::ConnectionPtr updateConnection;
    // ROS node handle
    private: ros::NodeHandle n;
    // publisher
    private: ros::Publisher temp_pub;
    // name of topic to publish data
    private: std::string Topic_;  
    private: double TempOffset; // offset oh temperature
    private: double TempStandardDeviation; // Stardard deviation of temperature
    private: double BaroOffset; // offset of barometer
    private: double BaroStandardDeviation; // standard deviation of barometer
    private: double maxtemp; // max limit
    private: double mintemp; // min limit
    private: double maxbaro; // max limit
    private: double minbaro; // min limit
    private: double Nbits; // number of bit to quantize continous signal
  };
}

#endif
