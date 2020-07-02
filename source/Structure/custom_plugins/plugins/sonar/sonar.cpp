/*
* File: sonar.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a servo motor. It works in Torque mode or Position mode and returns values of angular position and angular velocity
*/

#include "sonar.h"
#include "std_msgs/String.h"
#include <sstream>

namespace gazebo
{
	// constructor
	sonar::sonar() 
	{
	}

	// destructor
	sonar::~sonar()
	{

	}

	// initial setup
	void sonar::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
	{	  
		gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf); // get Gazebo's topic
		rostopic = XMLRead::ReadXMLString("rostopic",_sdf); // get Ros's topic
	  	std::string link = XMLRead::ReadXMLString("link",_sdf);; // get link's name
	  
      		this->model = _model; // save model pointer
		// starts Gazebo's node
      		this->node = transport::NodePtr(new transport::Node());
      		this->node->Init(this->model->GetWorld()->Name());
		// subscriber      		
		std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
      		this->sub = this->node->Subscribe(topic, &sonar::OnUpdate, this);
		// publisher
      		publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);

	}

	// Callback that receives the sonar's update signal.
	void sonar::OnUpdate(ConstSonarPtr &msg)
	{	 	
		simulator_msgs::Sensor newmsg;
		newmsg.name = rostopic;
		newmsg.header.stamp = ros::Time::now(); // time stamp
		newmsg.header.frame_id = "1"; // any value
		newmsg.values.push_back(msg->range()); // value of signal
		publisher_.publish(newmsg); // publish value
	}
	GZ_REGISTER_MODEL_PLUGIN(sonar)
}
