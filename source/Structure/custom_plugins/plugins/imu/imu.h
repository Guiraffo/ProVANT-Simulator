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

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	class imu : public ModelPlugin
	{
		std::fstream out;
		time_t  timev;
		public: imu(); 
  		public:virtual ~imu(); 
		public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
  		public: virtual void Reset();  
  		protected: virtual void Update(); 
		private:  
			std::string link_name_;
			physics::LinkPtr link;
			physics::WorldPtr world; 
			UpdateTimer updateTimer;
  			event::ConnectionPtr updateConnection;
			private: ros::NodeHandle n;
    			private: ros::Publisher imu_pub;
		        private: std::string Topic_;  
		        private: double rpyOffset;
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
    			std::normal_distribution<double> distributionANGVEL;
			boost::mutex lock;
	};
}
