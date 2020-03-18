/*
* File: QuadForces.h
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 10/12/19
* Description:  This library is responsable to satured and save the control inputs for vant 4.0
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
#include <MatlabData.h>
#include <MatlabData2.h> 

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{

	class DataSaveTiltRotor : public ModelPlugin
	{
		
		public: DataSaveTiltRotor(); // constructor
  		public:virtual ~DataSaveTiltRotor(); // destructor
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // initial setup
  		public: virtual void Reset();  // reset
		public: void CallbackFr(std_msgs::Float64); // callback to saturate and save forces at right brushless
		public: void CallbackFl(std_msgs::Float64); // callback to saturate and save forces at left brushless
		public: void CallbackDAr(std_msgs::Float64);// callback to saturate and save deflection at right aileron
		public: void CallbackDAl(std_msgs::Float64);// callback to saturate and save deflection at left aileron		
		public: void CallbackDRr(std_msgs::Float64);// callback to saturate and save deflection at right ruddervator
		public: void CallbackDRl(std_msgs::Float64);// callback to saturate and save deflection at left ruddervator
		public: void Update();
		private: 
			ros::NodeHandle node_handle_; // node handle of ROS
			physics::WorldPtr world; // pointer to the simulation world

			std::string topic_Fr; // name of topic of right brushless
			std::string topic_Fl; // name of topic of left brushless
			std::string topic_DAr;// name of topic of right aileron
			std::string topic_DAl; // name of topic of left aileron
			std::string topic_DRr; // name of topic of right ruddervator
			std::string topic_DRl; // name of topic of left ruddervator
			
			// ROS subscribers
			ros::Subscriber subscriberFr_; 
			ros::Subscriber subscriberFl_;
			ros::Subscriber subscriberDAr_;
			ros::Subscriber subscriberDAl_;
			ros::Subscriber subscriberDRr_;
			ros::Subscriber subscriberDRl_;
					
			double Fr, Fl, DAr, DAl, DRr, DRl; //Values to be satureted and saved
			double Fr_sat,Fl_sat,DAr_sat,DAl_sat,DRr_sat,DRl_sat; //Saturation values
			

			//Files for saving data
			MatlabData2 FrFile; 
			MatlabData2 FlFile;
			MatlabData2 DArFile;
			MatlabData2 DAlFile;
			MatlabData2 DRrFile;
			MatlabData2 DRlFile;

	};
}

//#endif
