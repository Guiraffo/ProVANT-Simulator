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

	class pertubation : public ModelPlugin
	{
		
		public: pertubation(); 
  		public:virtual ~pertubation(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
		public: void CallbackX(std_msgs::Float64);
		public: void CallbackY(std_msgs::Float64);
		public: void CallbackZ(std_msgs::Float64);		

		private: 
			std::string path;
			ros::NodeHandle node_handle_;
			physics::WorldPtr world;
			physics::LinkPtr link;
			std::string topicX;
			std::string topicY;
			std::string topicZ;
			std::string NameOfNode_;
			ros::Subscriber pertubation_subscriberX;
			ros::Subscriber pertubation_subscriberY;
			ros::Subscriber pertubation_subscriberZ;
			std::string NameOfLink;	
			double Fx, Fy,Fz; 
	};
}

//#endif
