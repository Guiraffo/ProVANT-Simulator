#include "temperature.h"

#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;


/////////////////////////////////////////////////
temp_press::temp_press() 
{

}

/////////////////////////////////////////////////
temp_press::~temp_press()
{

}

/////////////////////////////////////////////////
void temp_press::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf)
{
	  this->parentSensor = std::dynamic_pointer_cast<sensors::AltimeterSensor>(_sensor);
	  if (!this->parentSensor)
	  {
	    gzerr << "temperature_and_pressure Plugin requires a temperature_and_pressure.\n";
	    return;
	  }

	  Topic_ = XMLRead::ReadXMLString("Topic",_sdf);
	  TempOffset = XMLRead::ReadXMLDouble("TempOffset",_sdf);
	  TempStandardDeviation = XMLRead::ReadXMLDouble("TempStandardDeviation",_sdf);
	  BaroOffset = XMLRead::ReadXMLDouble("BaroOffset",_sdf);
	  BaroStandardDeviation = XMLRead::ReadXMLDouble("BaroStandardDeviation",_sdf);
	  maxtemp = XMLRead::ReadXMLDouble("maxtemp",_sdf);
	  mintemp = XMLRead::ReadXMLDouble("mintemp",_sdf);
	  maxbaro = XMLRead::ReadXMLDouble("maxbaro",_sdf);
	  minbaro = XMLRead::ReadXMLDouble("minbaro",_sdf);
	  Nbits = XMLRead::ReadXMLDouble("Nbits",_sdf);

	  temp_pub = n.advertise<simulator_msgs::Sensor>(Topic_, 1);

          unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
          std::default_random_engine customgenerator (seed);
	  generator = customgenerator;

          std::normal_distribution<double> customdistributionTEMP (TempOffset,TempStandardDeviation);
	  std::normal_distribution<double> customdistributionBARO (BaroOffset,BaroStandardDeviation);
	  distributionTEMP = customdistributionTEMP;
          distributionBARO = customdistributionBARO;
	  

	  this->updateConnection = this->parentSensor->ConnectUpdated(std::bind(&temp_press::OnUpdate, this));
	  this->parentSensor->SetActive(true);
	  
}

/////////////////////////////////////////////////
void temp_press::OnUpdate()
{

	  quantization converter;		

	  const double To = 288.15; // Kelvins
    	  const double Po = 101325; // N/m^2
          const double g = 9.801; // m/s^2
          const double c = -0.0065; // K/m
          const double R = 287.04;

	  double h = this->parentSensor->Altitude();

	  simulator_msgs::Sensor newmsg;
	  newmsg.name = Topic_;
	  newmsg.header.stamp = ros::Time::now();
	  newmsg.header.frame_id = "1";

	  double temperature = To + c*h;
          double pressure = Po*(std::pow(temperature/To,(-g/(c*R))));

	  temperature = temperature + distributionTEMP(generator);
	  pressure = pressure + distributionBARO(generator)  ;        

          if( temperature > maxtemp) temperature = maxtemp;
          if( temperature < mintemp) temperature = mintemp;
          temperature = converter.action(temperature,maxtemp,mintemp,Nbits);
	  newmsg.values.push_back(temperature);
	
          if( pressure > maxbaro) pressure = maxbaro;
          if( pressure < minbaro) pressure = minbaro;
          pressure = converter.action(pressure,maxbaro,minbaro,Nbits);
	  newmsg.values.push_back(pressure);	  

	  temp_pub.publish(newmsg);
  
}

GZ_REGISTER_SENSOR_PLUGIN(temp_press)
