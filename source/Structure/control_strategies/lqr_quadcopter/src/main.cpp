/*
* File: quad_lqr.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This file is responsable to implement LQR control law to VANT2.0
*/

#include "Icontroller.hpp"
#include<iostream>
#include <Eigen/Eigen>
#include "simulator_msgs/Sensor.h"

#define NINPUTS 4
#define NSTATES 12

class teste : public Icontroller
{
	private: int Iterations;
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: double T; // Sampled timr
	private: Eigen::MatrixXd RIB, W_n,W_n_transpose, B1, B2, Bx;

	// constructor
	public: teste(): Xref(NSTATES), K(NINPUTS,NSTATES), X(NSTATES), Erro(NSTATES), Input(NINPUTS)
	{
		T = 0.012;
		Iterations = 0;
		config();
	}

	// destructor
	public: ~teste()
	{

	}

	// initial configuration
	public: void config()
	{



/*K << -1.5247,      0, 0.86603,      0,-6.793, 3.9544,-3.0489,      0, 3.3129,      0,  -3.8461, 3.3997,
           0,-1.5247, 0.86603, 10.793,     0,-3.9544,      0,-3.0489, 3.3129, 3.8461,        0,-3.3997,
      1.5247,      0, 0.86603,      0, 6.793, 3.9544, 3.0489,      0, 3.3129,      0,   3.8461, 3.3997,
           0, 1.5247, 0.86603,-10.793,     0,-3.9544,      0, 3.0489, 3.3129,-3.8461,        0,-3.3997; 	*/
           
           
//     K << -0.6247,      0, 5.89603,      0,-6.793, 3.9544,0      ,-3.0489, 3.3129,      0,  -0.8461, 0.3997,
//           0     ,-1.5247, 5.89603, 5.793,     0,-3.9544, 3.0489,   0   , 3.3129, 4.8461,        0,-0.3997,
//          0.6247,      0, 5.89603,      0, 6.793, 3.9544, 0     , 3.0489, 3.3129,      0,   0.8461, 0.3997,
//           0     , 1.5247, 5.89603,-5.793,     0,-3.9544,-3.0489,   0   , 3.3129,-4.8461,        0,-0.3997; 
           
     
 
/*		K << -0.500000, -0.027819, 0.376289, 0.227345, -2.832945, 0.328074, -0.560425, -0.045626, 0.751956, 0.055273, -0.758507, 0.141713,
 -0.000000, -0.776039, 0.380894, 5.093023, -0.000000, -0.502676, -0.000000, -1.187413, 0.755924, 1.120645, -0.000000, -0.203890,
 0.500000, -0.027819, 0.376289, 0.227345, 2.832945, 0.328074, 0.560425, -0.045626, 0.751956, 0.055273, 0.758507,0.141713,
 0.000000, 0.444223, 0.379833, -3.034218, 0.000000, -0.397986, 0.000000, 0.686894, 0.755430, -0.709841, 0.000000, -0.171307;*/

// K << -0.500000, -0.044131, 0.376336, 0.429545, -3.663118, 0.326227, -0.631496, -0.076794, 0.751972, 0.115936, -1.187384, 0.141084,
// 0.000000, -0.749051, 0.381617, 6.184544, 0.000000, -0.541563, 0.000000, -1.228190, 0.756178, 1.650836, 0.000000, -0.216706,
// 0.500000, -0.044131, 0.376336, 0.429545, 3.663118, 0.326227, 0.631496, -0.076794, 0.751972, 0.115936, 1.187384, 0.141084,
 //-0.000000, 0.464291, 0.379377, -4.042288, -0.000000, -0.374843, -0.000000, 0.772946, 0.755271, -1.164992, -0.000000, -0.163641;

//	K <<   -0.500000,	0.000457,	0.346885,	-0.015094,	-3.663118,	0.360098,	-0.631496,	0.001276,	0.712324,	-0.009086,	-1.187384,	0.798145, //MATLAB
// 					0.000000,	-0.815947	,0.416881,	6.810335,	0.000000,	-0.400550,	0.000000,	-1.342818,	0.802771,	1.818925,	0.000000,	-0.837662,
// 					0.500000	,0.000457,	0.346885,	-0.015094,	3.663118,	0.360098,	0.631496,	0.001276,	0.712324,	-0.009086,	1.187384,	0.798145,
 //				 -0.000000,	0.408797,	0.415266	,-3.476348,	-0.000000	,-0.400548,	-0.000000	,0.674887,	0.802608,	-1.005851,	-0.000000,	-0.838914;
 				 
/* 	K << -5,4.4974e-13,2.2361,-2.3747e-11,-18.147,1.5811,-5.3402,7.1856e-13,2.2392,-3.8214e-13,-3.2273,2.2863,
-4.9778e-12,-5,2.2361,18.147,2.6346e-11,-1.5811,-5.7495e-12,-5.3402,2.2392,3.2273,5.511e-12,-2.2863,
5,-4.7627e-13,2.2361,2.5202e-11,18.147,1.5811,5.3402,-8.8021e-13,2.2392,8.6709e-13,3.2273,2.2863,
-4.0721e-12,5,2.2361,-18.147,2.3207e-11,-1.5811,-5.3948e-12,5.3402,2.2392,-3.2273,4.2373e-12,-2.2863;	*/

/* K << -3.873,5.3135e-13,2.7386,-1.4315e-11,-72.983,2.1851,-18.913,2.6172e-12,9.8404,-1.527e-12,-14.364,9.9634,
-9.0759e-13,-3.873,2.7386,72.983,-8.2265e-12,-2.1851,-4.1884e-12,-18.913,9.8404,14.364,-1.8867e-12,-9.9634,
3.873,2.1302e-13,2.7386,2.2722e-13,72.983,2.1851,18.913,1.0026e-12,9.8404,-1.4884e-12,14.364,9.9634,
-5.3002e-13,3.873,2.7386,-72.983,-5.7694e-12,-2.1851,-2.7691e-12,18.913,9.8404,-14.364,-1.9708e-12,-9.9634;  */ //Daniel
 
K << -70.711,2.1302e-11,7.0711,-2.3813e-12,-156.46,5,-165.1,4.7388e-11,11.53,8.2065e-13,-7.5435,5.7987,
1.3204e-11,-70.711,7.0711,156.46,8.257e-11,-5,3.1756e-11,-165.1,11.53,7.5435,8.1385e-12,-5.7987,
70.711,3.9104e-11,7.0711,-4.7265e-11,156.46,5,165.1,8.9701e-11,11.53,9.2039e-13,7.5435,5.7987,
-5.6384e-12,70.711,7.0711,-156.46,6.4636e-11,-5,-8.3103e-12,165.1,11.53,-7.5435,8.4109e-12,-5.7987;





	}

	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		
		int i = 0;
		simulator_msgs::Sensor msg;
		while(true)
		{
			if (arraymsg.values.at(i).name == "Estados"){
				msg = arraymsg.values.at(i);			 	
				break;
			}
			i++;
		}

// reference (Helicoidal Trajectory)
ros::Time time = ros::Time::now();
double tempo =time.toSec();

Xref << sin(tempo/2),cos(tempo/2),tempo/10,0,0,0,0,0,0,0,0,0;
		
		
	//Up and Down Trajectory
/*		Iterations++;
		if(Iterations%500==0){
			static bool p=0;
			if(p==0){
				Xref << 0,0,2.0,0,0,0,0,0,0,0,0,0;	
				p=true;}
			else{
				Xref << 0,0,1.0,0,0,0,0,0,0,0,0,0;
				p=false;
			}
		}																								*/
		
		// state vector
		X << msg.values.at(0), // x
		    msg.values.at(1), // y
			  msg.values.at(2), // z
		    msg.values.at(3), // roll
		    msg.values.at(4), // pitch
		    msg.values.at(5), // yaw
		    msg.values.at(6), // dx
		    msg.values.at(7), // dy
		    msg.values.at(8), // dz
		    msg.values.at(9), // droll
		    msg.values.at(10),// dpitch
		    msg.values.at(11);// dyaw

		// control law
		
		Erro = X-Xref;
		Input = -K*Erro;



	
	Input(0) =  Input(0) +  5.5;	 
	Input(1) =  Input(1) +  5.5;
	Input(2) =  Input(2) +  5.5;
	Input(3) =  Input(3) +  5.5;
	

		std::vector<double> out(Input.data(), Input.data() + Input.size());
		std::cout << out[0] <<  out[1] <<  out[2] <<  out[3] << std::endl;

		// std::cout << "saida 2 do controlador" << std::endl;
		return out;
	}

	// reference data
	public: std::vector<double> Reference()
	{
		std::vector<double> out(Xref.data(), Xref.data() + Xref.rows() * Xref.cols());
		return out;
	}

	// error data
	public: std::vector<double> Error()
	{
		std::vector<double> out(Erro.data(), Erro.data() + Erro.rows() * Erro.cols());
		return out;
	}

	// state data
	public: std::vector<double> State()
	{
		std::vector<double> out(X.data(), X.data() + X.rows() * X.cols());
		return out;
	}

};

// to implement plugin
extern "C"
{
	Icontroller *create(void) {
		return new teste;
	}
	void destroy(Icontroller *p) {
		delete p;
	}
}
