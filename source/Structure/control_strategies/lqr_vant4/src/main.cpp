#include "Icontroller.hpp"
#include <Eigen/Eigen>


class lqr_vant4 : public Icontroller
{
	private: int Iterations;	
	private: Eigen::VectorXd Xref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: Eigen::VectorXd Input2;
	private: double T; // Sampled time
	
	public: lqr_vant4():Input(4), Xref(16), K(4,16), X(16), Erro(16),Input2(4)
	{
		T = 0.012;
		Iterations = 0;
	}
	public: ~lqr_vant4(){}
	public: void config()
	{
	


K << -0.708576,	0.664812,	-14.986776,	-0.776799,	0.118369,	-0.268399,	6.965978,	7.071208,	-0.001229,	-0.002073,	-2.713855,	-0.003877,	0.147773,	-0.242210,	5.515657,	7.664070,
0.710318,	-0.662794,	14.986949,	1.058387,	-0.118370,	0.273579,	-6.966062,	7.070926,	0.001243,	0.002089,	2.713870,	0.239238,	-0.147776,	0.261981,	-5.515728,	7.664346,
3.610315,	0.517505,	0.371945,	2.092520,	0.076866,	0.630798,	-0.106075,	-0.000231,	0.134907,	0.008762,	0.094175,	0.444221,	0.107453,	0.553006,	-0.104833,	0.000427,
0.516506,	2.857548,	-0.370013,	2.434956,	-0.061736,	0.775000,	0.135053,	-0.000283,	0.009139,	0.129625,	-0.081029,	0.453630,	-0.085691,	0.669647,	0.119584,	0.000532;



/*K << 4.621014,6.688256,6.824401,-5.532729,0.068677,3.630270,0.083248,-0.082395,7.193535,-24.703775,0.418022,0.076686,-1.233172,1.494011,2.434822,0.267600,2.448889,0.054564,
4.621016,-6.688215,6.824433,5.532730,0.068666,-3.630271,-0.082396,0.083247,7.193574,24.703814,0.417960,-0.076687,1.493982,-1.233203,2.434836,0.267592,-2.448875,-0.054564,
0.564635,-0.028454,-0.354853,0.353667,1.672791,0.678208,1.449973,0.009278,-0.168131,0.360028,4.553543,0.857851,14.693958,0.366264,-0.034546,0.314335,-0.007045,0.316149,
0.564635,0.028451,-0.354854,-0.353667,1.672791,-0.678208,0.009278,1.449973,-0.168131,-0.360027,4.553542,-0.857851,0.366264,14.693957,-0.034547,0.314335,0.007044,-0.316149;
*/

/*K << 5.250209,6.576787,7.832233,-5.000166,0.064598,4.615510,2.998176,-3.011000,7.608406,-24.014735,0.355390,1.174004,17.076327,-16.918721,2.439157,0.224682,2.411234,0.431259,
5.250197,-6.576746,7.832283,5.000168,0.064563,-4.615510,-3.011001,2.998175,7.608450,24.014787,0.355226,-1.174005,-16.918827,17.076212,2.439171,0.224669,-2.411220,-0.431259,
0.079544,-0.016135,-0.051431,0.051118,0.311450,0.074598,0.454039,0.006846,-0.017693,0.086401,0.738131,0.090045,2.981379,0.573285,-0.002901,0.031489,-0.005568,0.031129,
0.079544,0.016134,-0.051431,-0.051118,0.311450,-0.074598,0.006846,0.454039,-0.017693,-0.086400,0.738131,-0.090045,0.573285,2.981379,-0.002901,0.031489,0.005567,-0.031129;

*/
		// reference
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
		
		
	/*		//Mapeando UVW e PQR
		Eigen::MatrixXd RIB(3,3);
		Eigen::MatrixXd W_n(3,3);
		Eigen::VectorXd UVW(3);
		Eigen::VectorXd PQR(3);
		Eigen::VectorXd XpYpZp(3);
		Eigen::VectorXd PhipThetapPsip(3);
		
		double Phi;
		double Psi;
		double Theta;
		
		XpYpZp << msg.values.at(8), msg.values.at(9), msg.values.at(10);
		PhipThetapPsip << msg.values.at(11), msg.values.at(12), msg.values.at(13);
		
		Phi = msg.values.at(3); // roll
		Theta = msg.values.at(4);// pitch
		Psi = msg.values.at(5); // yaw
		
		
	
	RIB <<  (cos(Psi)*cos(Theta)), (cos(Psi)*sin(Phi)*sin(Theta) - cos(Phi)*sin(Psi)), (sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
				(cos(Theta)*sin(Psi)), (cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)), (cos(Phi)*sin(Psi)*sin(Theta) - cos(Psi)*sin(Phi)), 
                        (-sin(Theta)),                              (cos(Theta)*sin(Phi)),                              (cos(Phi)*cos(Theta));
                        
W_n << 1.0,         0.0,          -sin(Theta), 
		       0.0,  cos(Phi),  cos(Theta)*sin(Phi),
               0.0, -sin(Phi),  cos(Phi)*cos(Theta);
               
               
               
UVW=RIB.transpose()*XpYpZp;

PQR=W_n*PhipThetapPsip;


	
		// Trapezoidal Integrator
		double z_atual = msg.values.at(2) - Xref(8);
		zint = zint + (T/2)*(z_atual + z_ant);
		z_ant = z_atual;
		double u_atual = UVW(0) - Xref(0);
		uint = uint + (T/2)*(u_atual + u_ant);
		u_ant = u_atual;
		double v_atual = UVW(1) - Xref(1);
		vint = vint + (T/2)*(v_atual + v_ant);
		v_ant = v_atual;
		double yaw_atual = msg.values.at(5) - Xref(11);
		yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
		yaw_ant = yaw_atual;                                       */
		
		
	
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
		
		
		std::cout << "Ta tudo bem entrada de controle" << std::endl;
		
		
		
		
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
	Icontroller *create(void) {return new lqr_vant4;}
	void destroy(Icontroller *p) {delete p;}
}
