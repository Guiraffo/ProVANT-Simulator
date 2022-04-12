/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file gps.cpp
 * @brief This file contains the implementation of a Gazebo model plugin that
 * reads GPS data from Gazebo and publishs it in a ROS topic as a sensor message.
 *
 * @author Arthur Viana Lara
 * @author Júnio Eduardo de Morais Aquino
 */

#include "gps.h"

#include <sstream>

#include <simulator_msgs/Sensor.h>

#include "XMLRead.h"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(gps)

// initial setup
void gps::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  gazebotopic = XMLRead::ReadXMLString("gazebotopic", _sdf);  // gazebo's topic for reading data
  rostopic = XMLRead::ReadXMLString("rostopic", _sdf);        // ROS's topic for publishing data
  std::string link = XMLRead::ReadXMLString("link", _sdf);    // link where GPS will stay

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
void gps::OnUpdate(const ConstGPSPtr& _msg)
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
