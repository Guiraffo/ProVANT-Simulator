/*
* File: AllData.h
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description:  This library is responsable to implement a sensor that return all data of the folowing state space:

- x,y,z,roll,pitch,yaw,aR,aL,dx,dy,dz,droll,dpitch,dyaw,daR,daL

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
	class AllData : public ModelPlugin
	{
		// constructor
		public: AllData();
		// destructor 
  		public:virtual ~AllData(); 
		// initial setup
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
		// reset 
  		public: virtual void Reset();
		// for each steo time  
  		protected: virtual void Update(); 
		
		private:  
			std::string NameOfJointR_; // name of right joint
			std::string NameOfJointL_; // name od left joint
			std::string NameOfNode_; // nme of node
			std::string link_name_; // name of link
			physics::LinkPtr link; // pointer to the link
			physics::WorldPtr world; // pointer to the world
			physics::JointPtr juntaR; //poiter to the right joint
			physics::JointPtr juntaL; // pointer to the left joint  
			UpdateTimer updateTimer;  // update time
  			event::ConnectionPtr updateConnection; // update connection
			ros::NodeHandle node_handle_; // ROS's node handle
			boost::mutex lock; // mutex
			ros::Publisher publisher_;  // ROS publisher	
	};
}
