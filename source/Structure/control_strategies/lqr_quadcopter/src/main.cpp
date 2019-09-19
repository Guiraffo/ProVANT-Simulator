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
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: double T; // Sampled timr

	// constructor
	public: teste(): Xref(NSTATES), K(NINPUTS,NSTATES), X(NSTATES), Erro(NSTATES), Input(NINPUTS)
	{
		T = 0.012;
	}

	// destructor
	public: ~teste()
	{

	}

	// initial configuration
	public: void config()
	{
		// std::cout << "config do controlador";
		// control matrix


		K << -0.062902,-2.2683e-15,0.052116,1.5825e-17,-0.47661,0.016686,-0.14812,-6.1847e-16,0.12042,1.6099e-16,-0.079017,0.05158,
		-8.0276e-16,-0.055987,0.052116,0.40623,3.1753e-16,-0.016686,6.5625e-16,-0.13105,0.12042,0.064584,-1.922e-16,-0.05158,
		0.062902,-1.721e-15,0.052116,5.4183e-15,0.47661,0.016686,0.14812,-1.0727e-15,0.12042,-5.1813e-18,0.079017,0.05158,
		1.1908e-15,0.055987,0.052116,-0.40623,7.3559e-15,-0.016686,1.8827e-16,0.13105,0.12042,-0.064584,2.2216e-16,-0.05158;

		// reference

		Xref << 0,0,2,0,0,0,0,0,0,0,0,0;
	}

	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
	    // std::cout << "Funciona teste";
		// integrators variables
		static double xint, x_ant = 0;
		static double yint, y_ant = 0;
		static double zint, z_ant = 0;
		static double yawint, yaw_ant = 0;

		// get data
		//int i = 0;

		// std::cout << "entrada do controlador" << std::endl;

		//simulator_msgs::Sensor msg;
		//while(true)
		//{
		//	if (arraymsg.values.at(i).name == "States"){
		//		msg = arraymsg.values.at(0);
		//q/		break;
		//	}
		//	i++;
		//}
		
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

		// Trapezoidal Integrator
		// double x_atual = msg.values.at(0) - Xref(0);
		// xint = xint + (T/2)*(x_atual + x_ant);
		// x_ant = x_atual;
		// double y_atual = msg.values.at(1) - Xref(1);
		// yint = yint + (T/2)*(y_atual + y_ant);
		// y_ant = y_atual;
		// double z_atual = msg.values.at(2) - Xref(2);
		// zint = zint + (T/2)*(z_atual + z_ant);
		// z_ant = z_atual;
		// double yaw_atual = msg.values.at(5) - Xref(5);
		// yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
		// yaw_ant = yaw_atual;

		// std::cout << "saida -1 do controlador" << std::endl;
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
		    // xint, // x integrator
		    // yint, // y integrator
		    // zint, // z integrator
		    // yawint; // yaw integrator

		// control law
		Erro = X-Xref;
		Input = -K*Erro;

		// double droll = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
		// double dpitch =  0 + q*cos(phi) - r*sin(phi);
		// double dyaw = 0 + q*sin(phi)*(1/cos(theta)) + r*cos(phi)*(1/cos(theta));

		// std::cout << "saida 0 do controlador" << std::endl;
		// Feedforward
	//	Input(0) = Input(0) + 0.34237;
	//	Input(1) = Input(1) + 0.34237;
	//	Input(2) = Input(2) + 0.34237;
	//	Input(3) = Input(3) + 0.34237;
	//	Input(0) =  0.34237;
	//	Input(1) =  0.34237;
	//	Input(2) =  0.34237;
	//	Input(3) =  0.34237;
	
	Input(0) =  0.19480;	//valores achados empiracamente
	Input(1) =  0.19480;
	Input(2) =  0.19480;
	Input(3) =  0.19480;

		// std::cout << "saida 1 do controlador" << std::endl;
		// output
		// std::vector<double> out(std::begin(Input).data(), Input.data() + Input.rows() * Input.cols());
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
