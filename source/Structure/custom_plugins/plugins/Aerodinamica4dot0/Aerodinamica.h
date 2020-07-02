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
			double c_AileronR(double DeR);
			double c_AileronL(double DeL);
			double c_RudR(double DrR);
			double c_RudL(double DrL);	
		protected: 
			//adicionada função callback para atualizar a força da fuselagem:
			virtual void Update(); 

		private: 
			Eigen::MatrixXd SkewSymmetricMatrix(Eigen::VectorXd Vetor);
			double T;
		
//			Eigen::VectorXd SaveDataWind;
		
			std::string path;
			ros::NodeHandle node_handle_;

			physics::WorldPtr world;
			physics::LinkPtr MainBody;
			physics::LinkPtr linkFr;
			physics::LinkPtr linkFl;
			physics::LinkPtr linkF;	
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
		
			std::string NameOfLinkCentroAerodF_;
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
			
			//incluído para melhor desempenho de aplicação das forças propellers
			std::string NameOfJointL_;
			std::string NameOfJointR_;
			physics::JointPtr juntaL;
			physics::JointPtr juntaR;
		//	std::string NameOfJointAileronL_;		
						
			boost::mutex lock;
			UpdateTimer updateTimer;
 			event::ConnectionPtr updateConnection;
 			
 			
 			double Phi;
			double Theta;
			double Psi;
			
			//Distances from B to aerodynamic center
			// f  -> fuselage 
			// wr -> wing right
			// wl -> wing left
			// tr -> tail right
			// wl -> tail left
			Eigen::VectorXd DBf;
			Eigen::VectorXd DBwr;
			Eigen::VectorXd DBwl;
			Eigen::VectorXd DBtr;
			Eigen::VectorXd DBtl;
			
			//Velocities Aerodinamic centers
			Eigen::VectorXd dPI_f;
			Eigen::VectorXd dPI_wr;
			Eigen::VectorXd dPI_wl;
			Eigen::VectorXd dPI_tr;
			Eigen::VectorXd dPI_tl;
			
			//Transformation matrices and configuration variables
			Eigen::MatrixXd RI_B;
			Eigen::MatrixXd Wn;
			Eigen::VectorXd WI_IB;
			Eigen::VectorXd PhipThetapPsip;
			Eigen::VectorXd XpYpZp;
			Eigen::VectorXd EnvironmentWind;   
			Eigen::MatrixXd RBmiL;
			Eigen::MatrixXd RBmiR;
			Eigen::MatrixXd RBwdL;
			Eigen::MatrixXd RBwdR;
			
			//aerodynamic center velocities and wind velocities
			Eigen::VectorXd UVWf;
			Eigen::VectorXd UVWfa;
			Eigen::VectorXd UVWwr;
			Eigen::VectorXd UVWwra;
			Eigen::VectorXd UVWwl;
			Eigen::VectorXd UVWwla;
			Eigen::VectorXd UVWtr;
			Eigen::VectorXd UVWtra;
			Eigen::VectorXd UVWtl;
			Eigen::VectorXd UVWtla;
			
			
			//--------aerodynamic forces------------
			
			//fuselagem
			Eigen::MatrixXd RAlphaf;
			Eigen::MatrixXd RBetaf;
			Eigen::VectorXd Fxz;
			Eigen::VectorXd Fxy;
			Eigen::VectorXd Forca_F;
			
			//wings
			Eigen::VectorXd FWrxz;
			Eigen::VectorXd FWlxz;
			Eigen::VectorXd Forca_Wr;
			Eigen::VectorXd Forca_Wl;
			Eigen::MatrixXd RAlphawr;
			Eigen::MatrixXd RAlphawl;
			
		
			//tail	
			Eigen::MatrixXd RGammatr;
			Eigen::MatrixXd RGammatl;
			Eigen::VectorXd Forca_Tr;
			Eigen::VectorXd Forca_Tl;
			Eigen::VectorXd F_TailR;
			Eigen::VectorXd F_TailL;
 			

			double AileronRDeflection;
			double AileronLDeflection;
			double RudderRDeflection;
			double RudderLDeflection;
			
			// ângulo de ataque e derrapagem
			double rho, sw, st, kt, b, mi, wd, SfLateral, SfFrontal;
			ignition::math::Vector3d Ff, FWr, FWl, FTr, FTl;
			
			double VetCDf[63] = {0.025,0.065452,0.16654,0.26951,0.37366,0.52852,0.73279,0.92854,1.1099,1.2652,1.4051,1.5216,1.6162,1.694,1.7517,1.7846,1.8024,1.7931,1.7549,1.6829,1.5829,1.4625,1.318,1.1666,0.99974,0.80772,0.60564,0.42784,0.2875,0.023346,0.012045,0.0068935,0.0072707,0.013511,0.017643,0.30916,0.45295,0.63915,0.8418,1.0291,1.193,1.3428,1.4847,1.6012,1.6972,1.7637,1.7964,1.8013,1.7799,1.7438,1.6823,1.601,1.5045,1.3823,1.2406,1.0812,0.8966,0.69802,0.49702,0.35513,0.25209,0.14907,0.051889};
			double VetCLf[63] = {0,0.74461,0.81047,0.64392,0.70064,0.8229,0.90449,0.94542,0.94146,0.89219,0.79445,0.6682,0.5406,0.3852,0.21348,0.043238,-0.11908,-0.29265,-0.4615,-0.62107,-0.75584,-0.88146,-0.98183,-1.0626,-1.0791,-1.082,-0.90438,-0.92431,-0.65444,-0.91363,-0.89073,-0.26214,0.3679999,0.9726,0.74113,0.69509,0.97793,0.92464,1.0841,1.0828,1.0509,0.96642,0.86173,0.73431,0.59556,0.43358,0.26364,0.090584,-0.070567,-0.24276,-0.41334,-0.56309,-0.68936,-0.81442,-0.90355,-0.94545,-0.94099,-0.89544,-0.80124,-0.68545,-0.65111,-0.83993,-0.67012};
			double VetCLW[63] = {-0.1,-0.11168,0.046158,0.19012,0.14949,0.032696,0.16352,0.40322,0.47168,0.43987,0.41031,0.39008,0.36076,0.29406,0.19604,0.084877,-0.036643,-0.17593,-0.34932,-0.52255,-0.62463,-0.64979,-0.64757,-0.75134,-0.92226,-0.8552,-0.84943,-0.95235,-1.1505,-0.80517,-0.33892,0.12202,0.66824,1.1202,1.5215,1.3454,1.1965,1.2698,1.4431,1.3737,1.1108,0.97621,0.88426,0.68382,0.48775,0.35297,0.19672,0.019917,-0.11727,-0.2102,-0.27283,-0.31364,-0.34131,-0.36589,-0.39616,-0.43407,-0.44923,-0.42199,-0.38794,-0.36149,-0.31994,-0.25637,-0.17593};
			double VetCDW[63] = {0.3,0.34268,0.42225,0.51124,0.58331,0.63736,0.69401,0.75098,0.79637,0.82679,0.84797,0.87748,0.9134,0.93713,0.95045,0.96032,0.9569,0.93842,0.92269,0.91245,0.89517,0.85707,0.80044,0.78136,0.75541,0.53444,0.40511,0.25973,0.14268,0.060593,0.025275,0.01631,0.03762,0.08312,0.1536,0.25923,0.43233,0.5999,0.69536,0.81398,0.89075,0.95251,1.0054,1.0477,1.0817,1.1101,1.135,1.1496,1.144,1.1279,1.1121,1.0905,1.0612,1.0305,1.0027,0.97769,0.94374,0.89018,0.81981,0.73368,0.62861,0.51045,0.39123};
			double VetCDt[63] = {0.0071,-0.11395,0.10658,0.45137,0.711,0.85029,0.98929,1.1324,1.2382,1.3224,1.3995,1.4579,1.5009,1.5443,1.581,1.5978,1.5993,1.5894,1.5642,1.5276,1.4894,1.4474,1.3927,1.3111,1.222,1.1889,1.0564,0.67454,0.30154,0.11121,0.024927,0.010853,0.012969,0.035073,0.13043,0.35371,0.74531,1.0998,1.1925,1.2344,1.327,1.4034,1.4551,1.4959,1.5341,1.5695,1.592,1.5998,1.5963,1.5761,1.537,1.494,1.4497,1.3875,1.3087,1.2227,1.1104,0.96413,0.82972,0.67893,0.39398,0.054892,-0.1234};
			double VetCLt[63] = {0,0.3994,0.55274,0.58755,0.62718,0.70302,0.76193,0.7903,0.79189,0.75553,0.68102,0.59354,0.49502,0.36827,0.23541,0.114,-0.061171,-0.30075,-0.49385,-0.61787,-0.73228,-0.84611,-0.94327,-1.0036,-1.0128,-0.97063,-0.92622,-0.99894,-1.0372,-0.74074,-0.43079,-0.12844,0.18012,0.48016,0.79568,1.0575,0.97331,0.92997,0.98023,1.0149,0.99685,0.92894,0.82764,0.71268,0.59908,0.46781,0.26059,0.024942,-0.13593,-0.25643,-0.39112,-0.51329,-0.60848,-0.69516,-0.76483,-0.79388,-0.78733,-0.75437,-0.69013,-0.61666,-0.58454,-0.53872,-0.35292};
		
	};
}

//#endif
