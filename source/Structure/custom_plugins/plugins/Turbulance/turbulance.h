#include <ros/ros.h>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <update_timer.h>
#include "std_msgs/Float64.h"
//#include <gazebo/math/Vector3.hh>
#include <ros/package.h>
#include <log4cxx/logger.h>
#include <log4cxx/xml/domconfigurator.h>
#include "XMLRead.h"
#include "/usr/include/eigen3/Eigen/Eigen"
#include "/usr/include/eigen3/Eigen/Dense"
#include <vector>
#include <MatlabData.h>
#include "simulator_msgs/Sensor.h"
#include <XMLRead2.h>

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class turbulance : public ModelPlugin{
        public: turbulance();
  		// destructor
    		public:virtual ~turbulance();
  		// initial setup
  		  public:virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  		// reset
    		public: virtual void Reset();
  		// for each step time
    		protected: virtual void Update();
        public:
        			std::string TurbTag;

        private:
             std::string NameOfTopic_; // nme of node
             physics::WorldPtr world; // pointer to the world
			       UpdateTimer updateTimer;  // update time
  			     event::ConnectionPtr updateConnection; // update connection
			       ros::NodeHandle node_handle_; // ROS's node handle
			       boost::mutex lock; // mutex
			       ros::Publisher publisher_;  // ROS publisher
             double delT;
             //Turbulance Model Matrices - Von Karman
             Eigen::VectorXd v;
             Eigen::MatrixXd Ad;
           	 Eigen::MatrixXd Bd;
             Eigen::MatrixXd Cd;
             Eigen::VectorXd dp;
             Eigen::VectorXd EnvWind;
  };
}
