/*
* File: gps.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement GPS. It gets information from Gazebo topics and publishes in Ros topic.
*/

#include "gps.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(gps)

//constructor
gps::gps() 
{

}

//destructor
gps::~gps()
{

}

// initial setup
void gps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf); // gazebo's topic for reading data
	rostopic = XMLRead::ReadXMLString("rostopic",_sdf); // ROS's topic for publishing data
	std::string link = XMLRead::ReadXMLString("link",_sdf); // link where GPS will stay
        
	// Gazebo subscriber
	this->model = _model;
        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(this->model->GetWorld()->Name());
        std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
        this->sub = this->node->Subscribe(topic, &gps::OnUpdate, this);

	// ROS publisher
	publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);
}

// for each updatetime, this function is called
void gps::OnUpdate(ConstGPSPtr &_msg)
{
	// publishing data on Gazebo
	simulator_msgs::Sensor newmsg;
	newmsg.name = rostopic;
	newmsg.header.stamp = ros::Time::now();
	newmsg.header.frame_id = "1";
	newmsg.values.push_back(_msg->latitude_deg());
	newmsg.values.push_back(_msg->longitude_deg());
	newmsg.values.push_back(_msg->altitude());
	newmsg.values.push_back(_msg->velocity_east());
	newmsg.values.push_back(_msg->velocity_north());
	newmsg.values.push_back(_msg->velocity_up());
	publisher_.publish(newmsg);

}
