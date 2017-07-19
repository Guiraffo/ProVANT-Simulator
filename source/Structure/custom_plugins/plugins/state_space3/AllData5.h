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
	class AllData5 : public ModelPlugin
	{

		std::fstream out;
		time_t  timev;
		std::default_random_engine generator;
		std::normal_distribution<double> distributionX;
		std::normal_distribution<double> distributionY;
		std::normal_distribution<double> distributionZ;

		public: AllData5(); 
	  	public: virtual ~AllData5(); 
		public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		
		private:  
		
			// variáveis para armazenar os nomes:	
			std::string NameOfJointR_;
			std::string NameOfJointL_;
			std::string NameOfNode_;
			std::string link_name_;

			// variáveis físicas
			physics::LinkPtr link;
			physics::WorldPtr world; 
			physics::JointPtr juntaR;
			physics::JointPtr juntaL;    

			UpdateTimer updateTimer;
  			event::ConnectionPtr updateConnection;
			ros::NodeHandle node_handle_;
			boost::mutex lock;
			ros::Publisher publisher_;	
	};
}
