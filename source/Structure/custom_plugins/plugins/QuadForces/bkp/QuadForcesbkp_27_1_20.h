/*
* File: QuadForces.h
* Author: Jonatan Mota Campos
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
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
// #include <gazebo/math/Vector3.hh>
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
  		public:virtual ~QuadForces(); // destructor
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // initial setup
  		public: virtual void Reset();  // reset
		public: void CallbackF1(std_msgs::Float64); // callback to apply forces at right brushless
		public: void CallbackF2(std_msgs::Float64);
		public: void CallbackF3(std_msgs::Float64);
		public: void CallbackF4(std_msgs::Float64); // callback to apply forces at left brushless		

		private: 
			ros::NodeHandle node_handle_; // node handle of ROS
			physics::WorldPtr world; // pointer to the simulation world
			physics::LinkPtr link; // pointer to the right brushless's link
			physics::LinkPtr Motor1; 
			physics::LinkPtr Motor2; 
			physics::LinkPtr Motor3; 
			physics::LinkPtr Motor4;
			
			physics::JointPtr M1;		// mexer helice
			physics::JointPtr M2;
			physics::JointPtr M3;
			physics::JointPtr M4;
			
			 
			std::string topic_F1; // name of topic of right brushless
			std::string topic_F2;
			std::string topic_F3;
			std::string topic_F4; // name of topic of left brushless
			// ROS subscribers
			ros::Subscriber motor_subscriberF1_; 
			ros::Subscriber motor_subscriberF2_;
				ros::Subscriber motor_subscriberF3_;
					ros::Subscriber motor_subscriberF4_;
					
			std::string NameOfLink_; // name of right brushless's link
				std::string NameOfLink1_;
				std::string NameOfLink2_;
				std::string NameOfLink3_;
				std::string NameOfLink4_;
				
				std::string NameOfJoint1_; //para mexer helice
				std::string NameOfJoint2_;
				std::string NameOfJoint3_;
				std::string NameOfJoint4_;
				
				
			double F1, F2, F3, 	F4; // Lift Forces
			double DragConst;                                                    //TESTE QUAD
		//	Eigen::MatrixXd RIB, W_n, B1, B2,W_n_transpose, Bx;
			Eigen::MatrixXd F1Body, F2Body, F3Body, F4Body;
		//	Eigen::MatrixXd F1Inertial, F2Inertial, F3Inertial, F4Inertial;
		//	void RotationMatrixInertialBody();
	};
}

//#endif
