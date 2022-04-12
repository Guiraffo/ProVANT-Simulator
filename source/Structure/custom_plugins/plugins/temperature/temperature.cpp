/*
 * File: temperature.cpp.
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description:  This library is responsable to implement the temperature sensor
 */

#include "temperature.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;

// initial setup
void temp_press::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
  this->parentSensor = std::dynamic_pointer_cast<sensors::AltimeterSensor>(_sensor);
  if (!this->parentSensor)
  {
    gzerr << "temperature_and_pressure Plugin requires a temperature_and_pressure.\n";
    return;
  }

  Topic_ = XMLRead::ReadXMLString("Topic", _sdf);           // get name of topic to publish data
  TempOffset = XMLRead::ReadXMLDouble("TempOffset", _sdf);  // get value of Temperature's offset
  TempStandardDeviation =
      XMLRead::ReadXMLDouble("TempStandardDeviation", _sdf);  // get value of Temperature's standard deviation
  BaroOffset = XMLRead::ReadXMLDouble("BaroOffset", _sdf);    // get value of Barometer's offset
  BaroStandardDeviation =
      XMLRead::ReadXMLDouble("BaroStandardDeviation", _sdf);  // get value of Barometer's Standard deviation
  maxtemp = XMLRead::ReadXMLDouble("maxtemp", _sdf);          // get max temp
  mintemp = XMLRead::ReadXMLDouble("mintemp", _sdf);          // get min temp
  maxbaro = XMLRead::ReadXMLDouble("maxbaro", _sdf);          // get max baro
  minbaro = XMLRead::ReadXMLDouble("minbaro", _sdf);          // get min baro
  Nbits = XMLRead::ReadXMLDouble("Nbits", _sdf);              // number of bits

  // publisher
  temp_pub = n.advertise<simulator_msgs::Sensor>(Topic_, 1);

  // randon numbers generator
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  std::default_random_engine customgenerator(seed);
  generator = customgenerator;
  std::normal_distribution<double> customdistributionTEMP(TempOffset, TempStandardDeviation);
  std::normal_distribution<double> customdistributionBARO(BaroOffset, BaroStandardDeviation);
  distributionTEMP = customdistributionTEMP;
  distributionBARO = customdistributionBARO;

  // connection to the simulation time
  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&temp_press::OnUpdate, this));
  this->parentSensor->SetActive(true);
}

// for each step time
void temp_press::OnUpdate()
{
  quantization converter;

  // constants
  const double To = 288.15;  // Kelvins
  const double Po = 101325;  // N/m^2
  const double g = 9.801;    // m/s^2
  const double c = -0.0065;  // K/m
  const double R = 287.04;

  double h = this->parentSensor->Altitude();  // get altitude

  simulator_msgs::Sensor newmsg;
  newmsg.name = Topic_;
  newmsg.header.stamp = ros::Time::now();  // time stamp
  newmsg.header.frame_id = "1";

  double temperature = To + c * h;
  double pressure = Po * (std::pow(temperature / To, (-g / (c * R))));

  // calculation of temperature and pressure based on altitude
  temperature = temperature + distributionTEMP(generator);
  pressure = pressure + distributionBARO(generator);
  if (temperature > maxtemp)
    temperature = maxtemp;
  if (temperature < mintemp)
    temperature = mintemp;
  temperature = converter.action(temperature, maxtemp, mintemp, Nbits);
  newmsg.values.push_back(temperature);
  if (pressure > maxbaro)
    pressure = maxbaro;
  if (pressure < minbaro)
    pressure = minbaro;
  pressure = converter.action(pressure, maxbaro, minbaro, Nbits);
  newmsg.values.push_back(pressure);

  // publish data
  temp_pub.publish(newmsg);
}

GZ_REGISTER_SENSOR_PLUGIN(temp_press)
