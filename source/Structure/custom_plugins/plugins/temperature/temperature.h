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
    std::default_random_engine generator;
    std::normal_distribution<double> distributionTEMP;
    std::normal_distribution<double> distributionBARO;

    public: temp_press();
    public: virtual ~temp_press();
public: virtual void Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf);
    private: virtual void OnUpdate();
    private: sensors::AltimeterSensorPtr parentSensor;
    private: event::ConnectionPtr updateConnection;
    private: ros::NodeHandle n;
    private: ros::Publisher temp_pub;

    private: std::string Topic_;  
    private: double TempOffset;
    private: double TempStandardDeviation;
    private: double BaroOffset;
    private: double BaroStandardDeviation;
    private: double maxtemp;
    private: double mintemp;
    private: double maxbaro;
    private: double minbaro;
    private: double Nbits;


  };
}

#endif
