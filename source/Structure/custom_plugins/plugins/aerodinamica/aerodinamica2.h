/*! \brief Brief description.
 *         Brief description continued.
 *
 *  Detailed description starts here.
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
#include "/usr/include/eigen3/Eigen/Eigen"
#include "/usr/include/eigen3/Eigen/Dense"

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class Aerodinamica2 : public ModelPlugin
	{
		
		public: 
			Aerodinamica2(); 
	  		virtual ~Aerodinamica2(); 
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
	  	 	virtual void Reset();  
			void CallbackFR(std_msgs::Float64);
			void CallbackFL(std_msgs::Float64);		
			void CallbackFElev(std_msgs::Float64);		
			void CallbackFRud(std_msgs::Float64);	
					
		protected: 
			//adicionada função callback para atualizar a força da fuselagem:
			virtual void Update(); 

		private: 
			//adiciona funções auxiliares para atualizar a aerodinâmica:
			double getAlpha(double wb, double wa, double ub, double ua);
			double getBeta(double vb, double va, double ub, double ua);
			double getAirxy(double wb, double wa, double ub, double ua);
			double getAirxz(double vb, double va, double ub, double ua);
			double cd_fxz(double alpha);
			double cl_fxz(double alpha);
			double cd_fxy(double beta);
			double cl_fxy(double beta);
			double cd_vxy(double beta);
			double cl_vxy(double beta);	
			double cd_hxz(double alpha);
			double cl_hxz(double alpha);
			double c_r(double Dr);
			double c_e(double De);
		
			std::string path;
			ros::NodeHandle node_handle_;

			physics::WorldPtr world;
			physics::LinkPtr linkFr;
			physics::LinkPtr linkFl;
			physics::LinkPtr linkE;
			physics::LinkPtr linkR;
			physics::LinkPtr link;
			physics::JointPtr JointFl;
			physics::JointPtr JointFr;
			
			std::string topic_Fr;
			std::string topic_Fl;
			std::string topic_Elev;
			std::string topic_Rud;
			
			ros::Subscriber motor_subscriberFL_;
			ros::Subscriber motor_subscriberFR_;
			ros::Subscriber motor_subscriberFElev_;
			ros::Subscriber motor_subscriberFRud_;
			
			std::string NameOfLinkFr_;
			std::string NameOfLinkFl_;	
			std::string NameOfLinkElev_;
			std::string NameOfLinkRud_;	
			std::string NameOfLinkBody_;
			std::string NameOfJointFl_;	
			std::string NameOfJointFr_;		
						
			boost::mutex lock;
			UpdateTimer updateTimer;
 			event::ConnectionPtr updateConnection;

						
			// velocidades lineares decompostas do vento e corpo
			double ua, va, wa, ub, vb, wb, phi, theta, psi;		
			// ângulo de ataque e derrapagem
			double alpha, beta, rho, sf, sh, sv, kt, b;
			double air_xy, air_xz, v_xz, v_xy, Fr, Fl;			
			// forças aerodinâmicas
			math::Vector3 Ff, Fh, Fv;
			math::Pose pose;
			Eigen::MatrixXd R_IB, wind_I, wind_B;
	};
}

//#endif
