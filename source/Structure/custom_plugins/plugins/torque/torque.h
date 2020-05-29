/*
* File: torque.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement torque actuator with one step time duration
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
	
	class torque : public ModelPlugin
	{
		// constructor
		public: torque();
		// destructor 
  		public:virtual ~torque(); 
		// initial setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset  		
		public: virtual void Reset();  
		// callback for receiving data of torque in x
		public: void CallbackX(std_msgs::Float64);
		// callback for receiving data of torque in y
		public: void CallbackY(std_msgs::Float64);
		// callback for receiving data of torque in z
		public: void CallbackZ(std_msgs::Float64);		

		private: 
			ros::NodeHandle node_handle_; // ROS's node handle
			physics::WorldPtr world; // pointer to the world
			physics::LinkPtr link; // pointer to the link
			std::string topicX; // topic of torque's value to be applied in x
			std::string topicY; // topic of torque's value to be applied in y
			std::string topicZ; // topic of torque's value to be applied in z
			// subscribers
			ros::Subscriber pertubation_subscriberX;
			ros::Subscriber pertubation_subscriberY;
			ros::Subscriber pertubation_subscriberZ;
			std::string NameOfLink;	 // name o link where the torque will be applied
			double Fx, Fy,Fz; // values o torque's components
	};
}

//#endif
