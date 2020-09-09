/*
* File: aerodinamica.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement aerodynamics forces in a UAV
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

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{

	class Aerodinamica : public ModelPlugin
	{
		
		public: Aerodinamica(); // constructor
  		public:virtual ~Aerodinamica(); // destructor
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // initial setup
  		public: virtual void Reset();  // reset
		public: void CallbackFR(std_msgs::Float64); // callback to apply forces at right brushless
		public: void CallbackFL(std_msgs::Float64); // callback to apply forces at left brushless		

		private: 
			ros::NodeHandle node_handle_; // node handle of ROS
			physics::WorldPtr world; // pointer to the simulation world
			physics::LinkPtr linkR; // pointer to the right brushless's link
			physics::LinkPtr linkL; // pointer to the left brushless's link
			std::string topic_FR; // name of topic of right brushless
			std::string topic_FL; // name of topic of left brushless
			// ROS subscribers
			ros::Subscriber motor_subscriberFL_; 
			ros::Subscriber motor_subscriberFR_;
			std::string NameOfLinkDir_; // name of right brushless's link
			std::string NameOfLinkEsq_; // name of left brushless's link	
			double Fr, Fl; // Lift Forces
	};
}

//#endif
