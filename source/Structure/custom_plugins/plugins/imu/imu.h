/*
* File: imu.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement IMU. It gets information from Gazebo and quantizes the data.
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
#include "simulator_msgs/Sensor.h"
#include <random>
#include "quantization.h"
#include <stdlib.h>
#include "XMLRead.h"

// testes
#include <boost/date_time.hpp>
#include "std_msgs/String.h"

#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>


namespace gazebo
{
	class imu : public ModelPlugin
	{
		// constructor
		public: imu();
		// destructor 
  		public:virtual ~imu();
		// initial setup 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset  		
		public: virtual void Reset();  
		// callback for each time step  		
		protected: virtual void Update(); 
		private:  
			std::string link_name_; // link name
			physics::LinkPtr link; // link
			physics::WorldPtr world; // world pointer
			UpdateTimer updateTimer; // pointer to update time
  			event::ConnectionPtr updateConnection; // pointer to update connection
			private: ros::NodeHandle n; // ROS node handle
    			private: ros::Publisher imu_pub; // ROS publisher
		        private: std::string Topic_;  // ROS topic
		        /*private: double rpyOffset;
		    	private: double rpyStandardDeviation;
		    	private: double accelOffset;
		    	private: double accelStandardDeviation;
		    	private: double angvelOffset;
		    	private: double angvelStandardDeviation;
		    	private: double maxrpy;
		    	private: double minrpy;
		    	private: double maxaccel;
		    	private: double minaccel;
		    	private: double maxangvel;
		    	private: double minangvel;
		    	private: double Nbits;	
			std::default_random_engine generator;
    			std::normal_distribution<double> distributionRPY;
    			std::normal_distribution<double> distributionACCEL;
    			std::normal_distribution<double> distributionANGVEL;*/
	};
}
