/*
* File: AllData2.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor that return all data of the folowing state space:

- x,y,z,roll,pitch,yaw,aR,aL,gammax,gammay,dx,dy,dz,droll,dpitch,dyaw,daR,daL,dgammax,dgammay

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
	class AllData2 : public ModelPlugin
	{
		// costructor
		public: AllData2();
		// destructor 
  		public:virtual ~AllData2(); 
		// initial setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
		// reset  		
		public: virtual void Reset();  
		// for each step time  		
		protected: virtual void Update(); 
		

		private:  
			std::string NameOfJointLoad_X_; // name of joint gammax
			std::string NameOfJointLoad_Y_; // name of joint gammay
			std::string NameOfJointR_; // name of right joint
			std::string NameOfJointL_; // name of left joint
			std::string NameOfNode_; // name of ROS node
			std::string link_name_; // name of main body
			physics::LinkPtr link; // main body's link
			physics::WorldPtr world; // pointer to world
			physics::JointPtr juntaR; // pointer to right joint
			physics::JointPtr juntaL; // pointer to left joint
			physics::JointPtr juntaLoadX; // pointer to joint gammax
			physics::JointPtr juntaLoadY; // pointer to right gammay  
			UpdateTimer updateTimer; // update time
  			event::ConnectionPtr updateConnection; // update connection
			ros::NodeHandle node_handle_; // ROS's node handle
			boost::mutex lock; // mutex
			ros::Publisher publisher_; // publisher	
	};
}
