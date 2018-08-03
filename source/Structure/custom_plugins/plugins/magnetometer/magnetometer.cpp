/*
* File: magnetometer.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement magnetometer. It gets information from Gazebo and quantizes the data.
*/


#include "magnetometer.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(magnetometer)

// constructor
magnetometer::magnetometer() 
{

}

// destructor
magnetometer::~magnetometer()
{
	
}

// initial setup
void magnetometer::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
	
      gazebotopic = XMLRead::ReadXMLString("gazebotopic",_sdf); //get Gazebo's topic
      rostopic = XMLRead::ReadXMLString("rostopic",_sdf); // get ROS's topic
      std::string link = XMLRead::ReadXMLString("link",_sdf); // get link's name

      this->model = _model;  // save model's pointer
      // starts Gazebo's node
      this->node = transport::NodePtr(new transport::Node()); 
      this->node->Init(this->model->GetWorld()->GetName());
      // subscriber to get magnetometers's data
      std::string topic = "/gazebo/default/" + this->model->GetName() + "/" + link + "/" + gazebotopic;
      this->sub = this->node->Subscribe(topic, &magnetometer::OnUpdate, this);
      // publisher data to the ROS's topics
      publisher_ = n.advertise<simulator_msgs::Sensor>(rostopic, 1);

}

// Callback that receives the magnetometer sensor's update signal.
void magnetometer::OnUpdate(ConstMagnetometerPtr &_msg)
{
	simulator_msgs::Sensor newmsg;
	newmsg.name = rostopic;
	newmsg.header.stamp = ros::Time::now(); // timestamp
	newmsg.header.frame_id = "1";
	// data
	newmsg.values.push_back(_msg->field_tesla().x());
	newmsg.values.push_back(_msg->field_tesla().y());
	newmsg.values.push_back(_msg->field_tesla().z());
	// publish
	publisher_.publish(newmsg);
	
}
