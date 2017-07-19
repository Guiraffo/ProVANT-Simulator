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

// testes
#include <boost/date_time.hpp>
#include "std_msgs/String.h"

#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{

	
	class UniversalJointSensor : public ModelPlugin
	{

		std::fstream out;
		public: UniversalJointSensor(); 
  		public:virtual ~UniversalJointSensor(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		

		private:  
			std::string axis;
			std::string NameOfJoint_;
			std::string NameOfNode_;
			physics::WorldPtr world; 
			physics::JointPtr junta;   
			UpdateTimer updateTimer;
  			event::ConnectionPtr updateConnection;
			ros::NodeHandle node_handle_;
			boost::mutex lock;
			ros::Publisher publisher_;	
			

	};
}
