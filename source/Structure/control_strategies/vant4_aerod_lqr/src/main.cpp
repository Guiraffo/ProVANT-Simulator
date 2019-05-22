#include "Icontroller.hpp"
#include <Eigen/Eigen>

class vant4_aerod_lqr : public Icontroller
{
private: int Iterations;	
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: Eigen::VectorXd Input2;
	private: double T; // Sampled time
	
	public: vant4_aerod_lqr():Input(4), Xref(16), K(4,16), X(16), Erro(16),Input2(8)
	{
		T = 0.012;
		Iterations = 0;
	}
	
	public: ~vant4_aerod_lqr(){}
	public: void config(){
	
	K << -0.708576,	0.664812,	-14.986776,	-0.776799,	0.118369,	-0.268399,	6.965978,	7.071208,	-0.001229,	-0.002073,	-2.713855,	-0.003877,	0.147773,	-0.242210,	5.515657,	7.664070,
0.710318,	-0.662794,	14.986949,	1.058387,	-0.118370,	0.273579,	-6.966062,	7.070926,	0.001243,	0.002089,	2.713870,	0.239238,	-0.147776,	0.261981,	-5.515728,	7.664346,
3.610315,	0.517505,	0.371945,	2.092520,	0.076866,	0.630798,	-0.106075,	-0.000231,	0.134907,	0.008762,	0.094175,	0.444221,	0.107453,	0.553006,	-0.104833,	0.000427,
0.516506,	2.857548,	-0.370013,	2.434956,	-0.061736,	0.775000,	0.135053,	-0.000283,	0.009139,	0.129625,	-0.081029,	0.453630,	-0.085691,	0.669647,	0.119584,	0.000532;
	
	
	Xref << 0.577,0.577,0,-0.578,0,0,0,2,0,0,0,0,0,0,0,0;
	
}


	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
		std::cout << "macro" << std::endl;
		
	/*	// integrators variables
		static double zint, z_ant = 0;
		static double uint, u_ant = 0;
		static double vint, v_ant = 0;
		static double yawint, yaw_ant = 0;     */
	
		// get data
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
		
		
		//Movimentar para cima e para baixo
		Iterations++;
		if(Iterations%500==0){
			static bool p=0;
			if(p==0){
				Xref << 0.577,0.577,0,-0.578,0,0,0,2,0,0,0,0,0,0,0,0;
				p=true;}
			else{
				Xref << 0.577,0.577,0,-0.578,0,0,0,3,0,0,0,0,0,0,0,0;
				p=false;
			}
		}
		
		//Movimentar para um lado e para o outro
	/*	Iterations++;
		if(Iterations%500==0){
			static bool p=0;
			if(p==0){
				Xref << 0.577,0.577,0,-0.578,0,0,0,2,0,0,0,0,0,0,0,0;
				p=true;}
			else{
				Xref << 0.577,0.577,0,-0.578,0,0,0.5,2,0,0,0,0,0,0,0,0;
				p=false;
			}
		} */
		
		
	
		
	
		// state vector
		X << msg.values.at(6), //aR
			 msg.values.at(7), //aL
	         msg.values.at(3), //roll
	         msg.values.at(4), //pitch
	         msg.values.at(5), //yaw
	         msg.values.at(0), //x
	         msg.values.at(1),// y
	         msg.values.at(2),// z
	         msg.values.at(14), // daR
	         msg.values.at(15), // daL
		     msg.values.at(11), // droll	
		     msg.values.at(12), // dpitch
		     msg.values.at(13), // dyaw
		     msg.values.at(8), // dx
		     msg.values.at(9), // dy
		     msg.values.at(10); //dz
		    	
		
		
                    
		
		// control law
		Erro = X-Xref;
		Input = -K*Erro;

		// Feedforward
		Input(0) = Input(0) + 37.3; 
		Input(1) = Input(1) + 37.3;
		
		// output
		Input2(0) = Input(0);
		Input2(1) = Input(1);
		//Input2(0) = 37.3;
		//Input2(1) = 37.3;
		Input2(2) = Input(2);
		Input2(3) = Input(3);
		Input2(4) = 0; 
		Input2(5) = 0; 
		Input2(6) = 0;
		Input2(7) = 0;
		
	
		
		
		std::vector<double> out(Input2.data(), Input2.data() + Input2.rows() * Input2.cols());
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



extern "C"
{
	Icontroller *create(void) {return new vant4_aerod_lqr;}
	void destroy(Icontroller *p) {delete p;}
}
