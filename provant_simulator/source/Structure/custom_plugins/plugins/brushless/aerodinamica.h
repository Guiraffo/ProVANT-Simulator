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

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class Aerodinamica : public ModelPlugin
	{
		
		public: Aerodinamica(); 
  		public:virtual ~Aerodinamica(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
		public: void CallbackFR(std_msgs::Float64);
		public: void CallbackFL(std_msgs::Float64);		

		private: 
			std::string path;
			ros::NodeHandle node_handle_;
			physics::WorldPtr world;
			physics::LinkPtr linkR;
			physics::LinkPtr linkL;
			std::string topic_FR;
			std::string topic_FL;
			std::string NameOfNode_;
			ros::Subscriber motor_subscriberFL_;
			ros::Subscriber motor_subscriberFR_;
			std::string NameOfLinkDir_;
			std::string NameOfLinkEsq_;	
			double Fr, Fl; 
	};
}

//#endif
