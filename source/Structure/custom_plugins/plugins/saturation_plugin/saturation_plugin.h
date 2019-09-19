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
#include "teste.h"
#include "testedois.h"
//#include "/home/macro/catkin_ws/src/ProVANT-Simulator_Developer/source/Structure/Controller/include/controller/XMLRead.h"
#include "simulator_msgs/Sensor.h"
//#include "/home/macro/catkin_ws/src/ProVANT-Simulator_Developer/source/Structure/Controller/include/controller/MatlabData.h"
#include <string>
#include <fstream>
#include "std_msgs/String.h"






namespace gazebo
{
	class SaturationPlugin : public ModelPlugin
	{
		// constructor
		public: SaturationPlugin();
		// destructor 
  		public:virtual ~SaturationPlugin();
		// initial setup 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset
  		public: virtual void Reset(); 
		// update fpr each step time 
  		protected: virtual void Update(); 
		// calback for receiving references
		public: void CallbackReferencias1(std_msgs::Float64);
		public: void CallbackReferencias2(std_msgs::Float64);
		public: void CallbackReferencias3(std_msgs::Float64);
		public: void CallbackReferencias4(std_msgs::Float64);
		public: void CallbackReferencias5(std_msgs::Float64);
		public: void CallbackReferencias6(std_msgs::Float64);		

		private:  
			
			std::string TopicSubscriber1_; // name of topic for receiving references
			std::string TopicSubscriber2_; // name of topic for receiving references
			std::string TopicSubscriber3_; // name of topic for receiving references
			std::string TopicSubscriber4_; // name of topic for receiving references
			std::string TopicSubscriber5_; // name of topic for receiving references
			std::string TopicSubscriber6_; // name of topic for receiving references
			
			
			
			UpdateTimer updateTimer; // pointer for notifying new step time
  			event::ConnectionPtr updateConnection; // connection pointer
			
			ros::NodeHandle node_handle_; // ROS's node handle
			
			ros::Subscriber subscriber1_; // ROS subscriber
			ros::Subscriber subscriber2_; // ROS subscriber
			ros::Subscriber subscriber3_; // ROS subscriber
			ros::Subscriber subscriber4_; // ROS subscriber
			ros::Subscriber subscriber5_; // ROS subscriber
			ros::Subscriber subscriber6_; // ROS subscriber
			
			physics::WorldPtr world; // world's pointer
			teste docme;
			
			testedois outsfileRotR;
			testedois outsfileRotL;
			testedois outsfileAilR;
			testedois outsfileAilL;
			testedois outsfileRudR;
			testedois outsfileRudL;
			
			boost::mutex lock; // mutex
			

	};
}
