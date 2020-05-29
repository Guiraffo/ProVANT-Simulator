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


using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{

	class VisualPropellers : public ModelPlugin
	{
		
		public: VisualPropellers(); // constructor
  	public:virtual ~VisualPropellers(); // destructor
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); // initial setup
  	public: virtual void Reset();  // reset	

		private: 
		std::string Propeller1_;
		std::string Propeller2_;
		physics::JointPtr Propeller_1;
		physics::JointPtr Propeller_2;
		double Velocity_;
		
	};
}

//#endif
