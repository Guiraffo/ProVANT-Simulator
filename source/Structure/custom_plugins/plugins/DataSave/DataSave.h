
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
#include "/usr/include/eigen3/Eigen/Eigen"
#include "/usr/include/eigen3/Eigen/Dense"
#include <vector>
#include <MatlabData.h>
#include "simulator_msgs/Sensor.h"


using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class DataSave : public ModelPlugin
	{
		
		public: 
			DataSave(); 
	  		~DataSave(); 
				void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
	  	 	void Reset();  
		  	void OnUpdate();

		private: 
			physics::ModelPtr model;
			event::ConnectionPtr updateConnection;
			UpdateTimer updateTimer;
			physics::WorldPtr world;
			physics::LinkPtr link; // pointer to the link
			std::string NameOfJointR_; // name of right joint
			std::string NameOfJointL_; // name od left joint
			std::string NameOfNode_; // nme of node
			std::string link_name_; // name of link
			physics::JointPtr juntaR; //poiter to the right joint
			physics::JointPtr juntaL; // pointer to the left joint  
			Eigen::MatrixXd RIB;
			Eigen::MatrixXd W_n;
			Eigen::MatrixXd WIIB;
			Eigen::MatrixXd PhipThetapPsip;
			Eigen::MatrixXd XpYpZp;
			Eigen::MatrixXd UVW;
			Eigen::MatrixXd PQR;
			MatlabData GeneralizedCoordinatesNVelocities;
			MatlabData GeneralizedVelocitiesExpressedOnbodyFrame;
			Eigen::VectorXd q;
			Eigen::VectorXd qp; // x y z phi theta psi ar al
			Eigen::VectorXd qpBody; // x y z phi theta psi ar al
			Eigen::VectorXd u;
			double Phi;
			double Theta;
			double Psi;
	};
}
