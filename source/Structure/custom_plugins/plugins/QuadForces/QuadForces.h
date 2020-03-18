/*
* File: QuadForces.h
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
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
#include <gazebo/math/Vector3.hh>
#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>
#include "XMLRead.h"
#include <eigen3/Eigen/Eigen>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{

	class QuadForces : public ModelPlugin
	{
		
		public: QuadForces(); // constructor
  		public: virtual ~QuadForces(); // destructor
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // initial setup
  		public: virtual void Reset();  // reset
		public: void CallbackF1(std_msgs::Float64);// callback to get the force at brushless 1
		public: void CallbackF2(std_msgs::Float64);// callback to get the force at brushless 2
		public: void CallbackF3(std_msgs::Float64);// callback to get the force at brushless 3
		public: void CallbackF4(std_msgs::Float64);// callback to get the force at brushless 4
		public: void Update();		

		private: 
			ros::NodeHandle node_handle_; // node handle of ROS
			physics::WorldPtr world; // pointer to the simulation world
			physics::LinkPtr link; // pointer to the main body link
			
			std::string topic_F1; // name of topic of brushless 1
			std::string topic_F2; // name of topic of brushless 2
			std::string topic_F3; // name of topic of brushless 3
			std::string topic_F4; // name of topic of brushless 4
			
			// ROS subscribers
			ros::Subscriber motor_subscriberF1_; 
			ros::Subscriber motor_subscriberF2_;
			ros::Subscriber motor_subscriberF3_;
			ros::Subscriber motor_subscriberF4_;
					
			std::string NameOfLink_; // name of body

			boost::mutex lock;
			UpdateTimer updateTimer;
 			event::ConnectionPtr updateConnection;
			
			double alpha;
			double length;
			Eigen::VectorXd ForceBody;
			Eigen::VectorXd TorqueBody;
			double F1, F2, F3, F4; // Lift Forces
			double DragConst;                                                    
	};
}

//#endif
