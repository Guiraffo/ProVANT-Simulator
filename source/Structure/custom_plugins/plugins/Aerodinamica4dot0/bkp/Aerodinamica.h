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
#include <vector>
#include <MatlabData.h>
#include "simulator_msgs/Sensor.h"

using namespace log4cxx;
using namespace log4cxx::xml;
using namespace log4cxx::helpers;

namespace gazebo
{
	LoggerPtr loggerMyMain(Logger::getLogger( "main"));

	class Aerodinamica : public ModelPlugin
	{
		
		public: 
			Aerodinamica(); 
	  		virtual ~Aerodinamica(); 
			virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf); 
	  	 	virtual void Reset();  
			void CallbackFR(std_msgs::Float64);
			void CallbackFL(std_msgs::Float64);		
			void CallbackFAileronR(std_msgs::Float64);
			void CallbackFAileronL(std_msgs::Float64);
			void CallbackRudderR(std_msgs::Float64);
			void CallbackRudderL(std_msgs::Float64);			
		
			MatlabData Wind;
		protected: 
			//adicionada função callback para atualizar a força da fuselagem:
			virtual void Update(); 

		private: 
			//adiciona funções auxiliares para atualizar a aerodinâmica:
			std::vector<double> getAlpha(double wa, double ua, std::vector<double> ha, std::vector<double> ia,std::vector<double> ja);
			std::vector<double> getBeta(double va, double ua, std::vector<double> hb);
			std::vector<double> getGama(double, double, double, double, std::vector<double> q, std::vector<double> r);
			std::vector<double> getAirxy(double va, double ua,std::vector<double> n,std::vector<double> o,std::vector<double> p);
			std::vector<double> getAirxz(double wa, double ua, std::vector<double> k,std::vector<double> l,std::vector<double> m);
			std::vector<double> getAirTail(double, double, double, double, std::vector<double> qq, std::vector<double> rr); 
			std::vector<double> GetVel_IS(double a, double b, double c);
	  		std::vector<double> GetRelVel(double d, double e, double f,int g);
			double cd_fxz(double alpha);
			double cl_fxz(double alpha);
			double cd_fxy(double beta);
			double cl_fxy(double beta);
			double cd_vxy(double beta);
			double cl_vxy(double beta);	
			double cd_hxz(double alpha);
			double cl_hxz(double alpha);
			double c_AileronR(double DaR);
			double c_AileronL(double DaL);
			double c_RudR(double DrR);
			double c_RudL(double DrL);
			double u_angle;
			math::Vector3 Np;
			math::Vector3 Ep;
			std::vector<double> alpha_Vec;
			std::vector<double> beta_Vec;
			std::vector<double> air_xz_Vec;
			std::vector<double> air_xy_Vec;
			std::vector<double> air_Tail_Vec;
		
//			Eigen::VectorXd SaveDataWind;
		
		
			std::string path;
			ros::NodeHandle node_handle_;

			physics::WorldPtr world;
			physics::LinkPtr linkFr;
			physics::LinkPtr linkFl;	
			physics::LinkPtr linkWr;
			physics::LinkPtr linkWl;
			physics::LinkPtr link;
			physics::LinkPtr linkAileronR;
			physics::LinkPtr linkAileronL;
			physics::LinkPtr linkRudderR;
			physics::LinkPtr linkRudderL;
			physics::LinkPtr linkRudR;
			physics::LinkPtr linkRudL;
			
			
			std::string topic_Fr;
			std::string topic_Fl;	
			std::string topic_AileronR;
			std::string topic_AileronL;
			std::string topic_RudderR;
			std::string topic_RudderL;
			
			
			ros::Subscriber motor_subscriberFL_;
			ros::Subscriber motor_subscriberFR_;
			ros::Subscriber motor_subscriberFAileronR_;
			ros::Subscriber motor_subscriberFAileronL_;
			ros::Subscriber motor_subscriberRudderR_;
			ros::Subscriber motor_subscriberRudderL_;
		
			
			std::string NameOfLinkFr_;
			std::string NameOfLinkFl_;		
			std::string NameOfLinkMainBody_;
			std::string NameOfLinkCentroAerodWr_;	
			std::string NameOfLinkCentroAerodWl_;
			std::string NameOfLinkAileronR_;
			std::string NameOfLinkAileronL_;
			std::string NameOfLinkRudderR_;
			std::string NameOfLinkRudderL_;
			std::string NameOfLinkCentroAerodRudR_;
			std::string NameOfLinkCentroAerodRudL_;	
		//	std::string NameOfJointAileronL_;		
						
			boost::mutex lock;
			UpdateTimer updateTimer;
 			event::ConnectionPtr updateConnection;
			//Tempo Aerodinamica %Passo integracao Gazebo
			double T;
						
			// velocidades lineares decompostas do vento e corpo
			double ua, va, wa, ua_TailR, va_TailR, wa_TailR, ua_TailL, va_TailL, wa_TailL, phi, theta, psi, x, y, z;			
			
			
			
		/*	double RudderDeflection;		*/
			double ElevatorDeflectionR;
			double ElevatorDeflectionL;
			double RudderDeflectionR;
			double RudderDeflectionL;
		//	double AileronDeflec;
			
			// ângulo de ataque e derrapagem
			double alpha, beta, rho, sf,swr, sh, sv, sTailR, sTailL, kt, b;
			double alpha_F,alpha_Wr,alpha_Wl,beta_F,beta_Wr,beta_Wl, gama_TailR, gama_TailL;
			std::vector<double> gama_Tail;
			double air_xy, air_xz, v_xz, v_xy, Fr, Fl;		
			
				
			double VetCDv[63] = 
 					 {0.0071,  -0.11395,  0.10658,   0.45137,     0.711,   0.85029,  0.98929,
                      1.1324,    1.2382,   1.3224,    1.3995,    1.4579,    1.5009,   1.5443,
                       1.581,    1.5978,   1.5993,    1.5894,    1.5642,    1.5276,   1.4894,
                      1.4474,    1.3927,   1.3111,     1.222,    1.1889,    1.0564,  0.67454,
                     0.30154,   0.11121, 0.024927,  0.010853,  0.012969,  0.035073,  0.13043,
                     0.35371,   0.74531,   1.0998,    1.1925,    1.2344,     1.327,   1.4034,
                      1.4551,    1.4959,   1.5341,    1.5695,     1.592,    1.5998,   1.5963,
                      1.5761,     1.537,    1.494,    1.4497,    1.3875,    1.3087,   1.2227,
                      1.1104,   0.96413,  0.82972,   0.67893,   0.39398,  0.054892,  -0.1234};

			double VetCLv[63] = {0,0.3994,   0.55274,  0.58755,   0.62718,   0.70302,   0.76193,   0.7903,
                     0.79189,   0.75553,  0.68102,   0.59354,   0.49502,   0.36827,  0.23541,
                       0.114, -0.061171, -0.30075,  -0.49385,  -0.61787,  -0.73228, -0.84611,
                    -0.94327,   -1.0036,  -1.0128,  -0.97063,  -0.92622,  -0.99894,  -1.0372,
                    -0.74074,  -0.43079, -0.12844,   0.18012,   0.48016,   0.79568,   1.0575,
                     0.97331,   0.92997,  0.98023,    1.0149,   0.99685,   0.92894,  0.82764,
                     0.71268,   0.59908,  0.46781,   0.26059,  0.024942,  -0.13593, -0.25643,
                    -0.39112,  -0.51329, -0.60848,  -0.69516,  -0.76483,  -0.79388, -0.78733,
                    -0.75437,  -0.69013, -0.61666,  -0.58454,  -0.53872,  -0.35292};

 			double VetCDf[63] = 
					{     0.025,  0.065452,  0.16654,   0.26951,   0.37366,   0.52852,  0.73279,
                        0.92854,    1.1099,   1.2652,    1.4051,    1.5216,    1.6162,    1.694,
 					     1.7517,    1.7846,   1.8024,    1.7931,    1.7549,    1.6829,   1.5829,
                         1.4625,     1.318,   1.1666,   0.99974,   0.80772,   0.60564,  0.42784,
                         0.2875,  0.023346, 0.012045, 0.0068935, 0.0072707,  0.013511, 0.017643,
                        0.30916,   0.45295,  0.63915,    0.8418,    1.0291,     1.193,   1.3428,
                         1.4847,    1.6012,   1.6972,    1.7637,    1.7964,    1.8013,   1.7799,
                         1.7438,    1.6823,    1.601,    1.5045,    1.3823,    1.2406,   1.0812,
                         0.8966,   0.69802,  0.49702,   0.35513,   0.25209,   0.14907, 0.051889};

      		double VetCLf[63] = 
					{0,0.74461,   0.81047,  0.64392,   0.70064,    0.8229,   0.90449,  0.94542,
                       0.94146,   0.89219,  0.79445,    0.6682,    0.5406,    0.3852,  0.21348,
                      0.043238,  -0.11908, -0.29265,   -0.4615,  -0.62107,  -0.75584, -0.88146,
                      -0.98195,   -1.0618,   -1.083,   -1.0583,   -0.8989,  -0.92432, -0.65444,
                      -0.91363,  -0.89073, -0.26214,   0.36812,    0.9726,   0.74113,  0.69509,
                       0.97789,   0.91181,   1.0676,    1.0854,    1.0505,   0.96643,  0.86174,
                        0.7343,   0.59556,  0.43358,   0.26364,  0.090584, -0.070567, -0.24276,
                      -0.41334,  -0.56309, -0.68936,  -0.81442,  -0.90355,  -0.94545, -0.94099,
                      -0.89544,  -0.80124, -0.68545,  -0.65111,  -0.83993, -0.67012};

			double  VetCDh[63] = 
					{   0.01, -0.014686, 0.039606,   0.14179,   0.26115,   0.37536,   0.4718,
                     0.55138,   0.62136,  0.68835,   0.75304,   0.80588,   0.84351,  0.86966,
                     0.89079,   0.91094,  0.92141,   0.91636,   0.89982,   0.87552,  0.84549,
                      0.8076,   0.75908,  0.70106,   0.63335,   0.55213,   0.45149,  0.33068,
                     0.18957,  0.086203, 0.035775,  0.010921,  0.013072,  0.042734, 0.098476,
                     0.21631,   0.42069,  0.60983,   0.75792,   0.85157,   0.92049,  0.98952,
                      1.0572,    1.1133,   1.1518,    1.1738,    1.1892,    1.1993,   1.1997,
                      1.1876,    1.1619,   1.1241,    1.0777,    1.0276,   0.96749,  0.88554,
                      0.7838,   0.66984,  0.55251,   0.41773,   0.22878,  0.033737,-0.062168};

			double  VetCLh[63] = 
				    {0,0.59164,   0.69735,  0.61673,   0.63814,   0.80205,   0.92648,  0.98066,
                      1.0012,   0.97829,  0.90569,   0.80394,   0.68694,   0.55683,   0.3929,
                     0.17536, -0.074495, -0.32313,  -0.53312,  -0.69313,  -0.81462, -0.90425,
                    -0.96773,    -1.016,  -1.0475,   -1.0422,   -1.0261,   -0.9924, -0.92609,
                    -0.98447,  -0.58049, -0.17426,   0.23549,    0.6446,   0.99882,   0.8849,
                     0.97275,    1.0499,   1.0893,    1.1125,    1.0981,    1.0053,  0.86342,
                     0.71528,   0.56377,  0.40394,   0.22927,  0.027647,  -0.20726, -0.42502,
                     -0.5818,  -0.70695, -0.82204,  -0.92071,  -0.98601,   -1.0002, -0.97439,
                    -0.91178,  -0.77285,  -0.6198,  -0.63052,  -0.70076,   -0.5343};							

			// forças aerodinâmicas
			math::Vector3 Ff, FWr, FWl, FTailR, FTailL;
			math::Pose pose;
			Eigen::MatrixXd R_IB, wind_I, wind_B, wind_TailR,wind_TailL, Rxu1, Rxu2;
		
	};
}

//#endif
