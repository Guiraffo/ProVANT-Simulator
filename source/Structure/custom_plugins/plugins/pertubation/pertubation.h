/*
* File: pertubation.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a plugin to simulate a force pertubation.
*/

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

	class pertubation : public ModelPlugin
	{
		// constructor
		public: pertubation();
		// destructor 
  		public:virtual ~pertubation(); 
		// inital setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset();  
		// Callback to provide a pertubation in x direction
		public: void CallbackX(std_msgs::Float64);
		// Callback to provide a pertubation in y direction
		public: void CallbackY(std_msgs::Float64);
		// Callback to provide a pertubation in z direction
		public: void CallbackZ(std_msgs::Float64);		

		private: 
			ros::NodeHandle node_handle_; // ROS node handle
			physics::WorldPtr world; // world's pointer
			physics::LinkPtr link; // link's world
			// topics
			std::string topicX;
			std::string topicY;
			std::string topicZ;
			// name of node handle
			std::string NameOfNode_;
			// subscriber
			ros::Subscriber pertubation_subscriberX;
			ros::Subscriber pertubation_subscriberY;
			ros::Subscriber pertubation_subscriberZ;
			// name of link			
			std::string NameOfLink;
			// values of pertubation	
			double Fx, Fy,Fz; 
	};
}

//#endif
