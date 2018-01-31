/*
* File: UniversalJointSensor.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor the returns all kind of data enabled in the simulation of a specific joint
*/

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include <iostream>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <random>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"

namespace gazebo
{

	
	class UniversalJointSensor : public ModelPlugin
	{
		// constructor
		public: UniversalJointSensor(); 
		// destructor
  		public:virtual ~UniversalJointSensor(); 
		// initial setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset();  
		// for each step time
  		protected: virtual void Update(); 
		

		private:  
			std::string axis; // kind of axis
			std::string NameOfJoint_; // name of joint
			std::string NameOfNode_; // name of node
			physics::WorldPtr world; // pointer to the world
			physics::JointPtr junta; // pointer the joint  
			UpdateTimer updateTimer; 
  			event::ConnectionPtr updateConnection; 
			ros::NodeHandle node_handle_; // ROS's node handle
			boost::mutex lock; // mutex
			ros::Publisher publisher_; // publisher	
	};
}
