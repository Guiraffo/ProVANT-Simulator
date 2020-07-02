/*
* File: force.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a force in a speficific link of UAV
*/

//#ifndef AERO_H
//#define AERO_H

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include "std_msgs/Float64.h"
// #include <gazebo/math/Vector3.hh>
#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>
#include "XMLRead.h"

namespace gazebo
{

	class force : public ModelPlugin
	{
		
		// constructor
		public: force();
		// destructor 
  		public:virtual ~force(); 
		// initial setup		
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset  		
		public: virtual void Reset();
		// callback for applying forces  
		public: void CallbackX(std_msgs::Float64);
		public: void CallbackY(std_msgs::Float64);
		public: void CallbackZ(std_msgs::Float64);		

		private: 
			ros::NodeHandle node_handle_; // node handle
			physics::WorldPtr world; // pointer to world
			physics::LinkPtr link; // pointer to link
			std::string topicX; // name of topic
			std::string topicY; // name of topic
			std::string topicZ; // name of topic
			std::string NameOfNode_; // name of node
			// ROS subscribers
			ros::Subscriber pertubation_subscriberX; 
			ros::Subscriber pertubation_subscriberY;
			ros::Subscriber pertubation_subscriberZ;
			std::string NameOfLink;	// name of link
			double Fx, Fy,Fz; // value of forces
	};
}

//#endif
