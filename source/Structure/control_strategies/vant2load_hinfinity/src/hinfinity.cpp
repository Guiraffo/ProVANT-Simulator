/*
* File: hinfinity.cpp
* Author: Arthur Viana Lara
* Project: ProVANT
* Company: Federal University of Minas Gerais
* Version: 1.0
* Date: 29/01/18
* Description: This file is responsable to implement H2/Hinifinity control law to VANT2.0 with load
*/

#include <control_strategies_base/icontroller.hpp>
#include <iostream>
#include <Eigen/Eigen>
#include "simulator_msgs/Sensor.h"
#include "math.h"
#include "vant2load_hinfinity/feedfoward.h"


class hinfinity : public Icontroller 
{
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: double T; // Sampled time

	// constructor
	public: hinfinity(): Xref(24), K(4,24), X(24), Erro(24), Input(4)
	{ 
		T = 0.012; // in ms
	}
	// destructor
	public: ~hinfinity()
	{
		
	}
	// initial configuration
	public: void config()
	{
	K<< 0.7101244468428165,6.141471260155515,7.765440795725267,-18.97385920336097,1.826066960066656,-0.09338611048322061,17.57791369983566,-0.7164683413615529,-0.03350735071807359,0.4235128078145839,0.2708588223207514,4.311055003650324,4.170146360015496,-5.072227698361692,0.3809539101204998,-0.06650607548831029,2.122392130887405,-0.02473442956879733,0.0006404185518007683,0.008513558472367464,0.4662177448319676,3.523707063368016,5.320429316782071,-0.1472247707396855,-1.571274680558446,-8.596271330525751,6.985745062499001,21.91480875426527,-1.94921538690414,0.09535891280953998,-18.42904004957552,0.4434387926540275,0.1072202681687932,-0.317249054464631,-0.9548973567439005,-5.910676814456982,4.55703288130231,5.454379896200787,-0.2518284755625079,0.06360770401465654,-2.133681658482248,-0.008445306788653999,0.001718825918710477,-0.005954099368496411,-1.2250793041699,-5.503330766582297,3.925043415306752,0.09204438852464802,0.1952075590979948,-0.03032542746108816,-0.003380038380425396,0.06292464197787898,0.3808490158409268,0.04098134842638689,-0.0437294922542443,-0.1815063858105274,0.1672299855993115,-0.004950772796723103,0.1160491338776613,-0.01901526180028523,-0.01046202265880625,0.01457474072728251,0.07364843512499895,0.02819675274652457,-0.003764578805403473,-0.008102716027547696,0.004792989032152177,0.000274744109690024,0.1320526541764119,-0.01982603318640202,0.008140109577766043,0.01296630640217303,0.1858289717940038,-0.004531750410384653,-0.001691334240977282,-0.01099980803116952,0.3650350924346544,-0.03429654457475473,0.02114487202233904,-0.1765601446988155,-0.0087900241089497,0.1676412256268605,0.1107587378532268,-0.0008316297740917086,-0.0101361709665762,-0.00430220334492462,0.07112118136129204,-0.02734833615631714,0.002775618184624419,-0.007952050965466266,0.000294409460551884,0.004683773810332413,0.1255028830715101,-0.004335263292000928,0.009748974116588649,-0.01431313055730361;
	
	}

	// control law
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		static float count = 0; // counter for save time
		
		// static variable to implement integrators                
		static float xint, x_ant = 0; 
                static float yint, y_ant = 0;
                static float zint, z_ant = 0;
                static float yawint, yaw_ant = 0;
	
		// get sensor data 
		int i = 0;		
		simulator_msgs::Sensor msgstates;
		msgstates = arraymsg.values.at(0);		

		// Reference
		float trajectoryRadius = 2;
		float trajectoryHeight = 4*trajectoryRadius;
		float trajTime = 80;
		float pi = 3.14;

		float x = trajectoryRadius*cos((count*T)*2*pi/trajTime);
		float xdot = -trajectoryRadius*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);
		float xddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);

		float y = trajectoryRadius*sin((count*T)*2*pi/trajTime);
		float ydot = trajectoryRadius*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);
		float yddot = -trajectoryRadius*(2*pi/trajTime)*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);

		float z = trajectoryHeight+1 - trajectoryHeight*cos((count*T)*2*pi/trajTime);
		float zdot = trajectoryHeight*(2*pi/trajTime)*sin((count*T)*2*pi/trajTime);
		float zddot = trajectoryHeight*(2*pi/trajTime)*(2*pi/trajTime)*cos((count*T)*2*pi/trajTime);
		
		Xref << x,y,z,0,0,0,0.00002965,0.004885,0.004893,0.00484,xdot,ydot,zdot,0,0,0,0,0,0,0,0,0,0,0;

		//Converting to angular velocity
                std::vector<double> etadot = pqr2EtaDot(msgstates.values.at(13),
                                                         msgstates.values.at(14),
                                                         msgstates.values.at(15),
                                                         msgstates.values.at(3),
                                                         msgstates.values.at(4),
                                                         msgstates.values.at(5));

                // Trapezoidal Integrator
		float x_atual = msgstates.values.at(0) - Xref(0);
                xint = xint + (T/2)*(x_atual + x_ant);
                x_ant = x_atual;
                float y_atual = msgstates.values.at(1) - Xref(1);
                yint = yint + (T/2)*(y_atual + y_ant);
                y_ant = y_atual;
                float z_atual = msgstates.values.at(2) - Xref(2);
                zint = zint + (T/2)*(z_atual + z_ant);
                z_ant = z_atual;
                float yaw_atual = msgstates.values.at(5) - Xref(5);
                yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
                yaw_ant = yaw_atual;

		// State vector with integrators
                X << msgstates.values.at(0),//x
                     msgstates.values.at(1),//y
                     msgstates.values.at(2),//z
                     msgstates.values.at(3),//roll
                     msgstates.values.at(4),//pitch
                     msgstates.values.at(5),//yaw
                     msgstates.values.at(8),//g1 x - load
                     msgstates.values.at(9),//g2 y - load
                     msgstates.values.at(6),//aR
                     msgstates.values.at(7),//aL
                     msgstates.values.at(10),//vx
                     msgstates.values.at(11),//vy
                     msgstates.values.at(12),//vz
                     etadot.at(0),//droll
                     etadot.at(1),//dpitch
                     etadot.at(2),//dyaw
                     msgstates.values.at(18), // dg1 load
                     msgstates.values.at(19), // dg2 load
                     msgstates.values.at(16), // daR
                     msgstates.values.at(17), // daL
                     xint, // x integrator
                     yint, // y integrator
                     zint, // z integrator
                     yawint; // yaw integrator


		// control law
		Erro = X-Xref;
		Input = -K*Erro;

		Eigen::MatrixXd qref(10,1);
		qref << x,y,z,0,0,0,0.00002965,0.004885,0.004893,0.00484;
		Eigen::MatrixXd qrefdot(10,1);
		qrefdot << xdot,ydot,zdot,0,0,0,0,0,0,0;
		Eigen::MatrixXd qrefddot(10,1);
		qrefddot << xddot,yddot,zddot,0,0,0,0,0,0,0;
		Eigen::MatrixXd  varfeedforward  = feedforward::compute(qref,qrefdot,qrefddot);

		// Feedforward
                //Input(0) = Input(0) + 12.6005;
                //Input(1) = Input(1) + 12.609;
		Input(0) = Input(0) + varfeedforward(0,0);
		Input(1) = Input(1) + varfeedforward(1,0);
		Input(2) = Input(2) + varfeedforward(2,0);
		Input(3) = Input(3) + varfeedforward(3,0);
		count++; 


		// output
		std::vector<float> out2(Input.data(), Input.data() + Input.rows() * Input.cols());
		std::vector<double> out(out2.size());
		for(int i=0; i<out2.size();i++) out.at(i) = out2.at(i);
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

	// converting angular velocity
        private: std::vector<double> pqr2EtaDot(double in_a, double in_b, double in_c, double phi, double theta, double psii)
        {
            std::vector<double> out;
            out.push_back(in_a + in_c*cos(phi)*tan(theta) + in_b*sin(phi)*tan(theta));
            out.push_back(in_b*cos(phi) - in_c*sin(phi));
            out.push_back((in_c*cos(phi))/cos(theta) + (in_b*sin(phi))/cos(theta));
            return out;
        }
};


// to implement plugin
extern "C"
{ 
	Icontroller *create(void) {
		return new hinfinity;
	}
	void destroy(Icontroller *p) {
		delete p;
	}
}
