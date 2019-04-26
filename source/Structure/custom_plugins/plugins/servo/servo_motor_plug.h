/*
* File: servo_motor_plug.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a servo motor. It works in Torque mode or Position mode and returns values of angular position and angular velocity
*/

#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include "XMLRead.h"
#include "simulator_msgs/Sensor.h"


namespace gazebo
{
	class ServoMotorPlugin : public ModelPlugin
	{
		// constructor
		public: ServoMotorPlugin();
		// destructor 
  		public:virtual ~ServoMotorPlugin();
		// initial setup 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset(); 
		// update fpr each step time 
  		protected: virtual void Update(); 
		// calback for receiving references
		public: void CallbackReferencias(std_msgs::Float64);		

		private:  
			std::string NameOfJoint_; // name of joint
			std::string TopicSubscriber_; // name of topic for receiving references
			std::string TopicPublisher_; // name of topic for sending sensor data
			std::string Modo_; // mode of servo working
			double Saturation_;
			physics::WorldPtr world; // world's pointer
			physics::JointPtr junta;  // joint's pointer
			UpdateTimer updateTimer; // pointer for notifying new step time
  			event::ConnectionPtr updateConnection; // connection pointer
			ros::NodeHandle node_handle_; // ROS's node handle
			ros::Publisher motor_publisher_; // ROS publisher
			ros::Subscriber motor_subscriber_; // ROS subscriber
			boost::mutex lock; // mutex

	};
}
