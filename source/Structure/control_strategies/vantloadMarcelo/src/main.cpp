#include "Icontroller.hpp"
#include <Eigen/Eigen>


class vantloadMarcelo : public Icontroller
{

	private: Eigen::VectorXd Ref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd Input;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd X;
	private: double T;


	public: vantloadMarcelo():X(24),Ref(12),Erro(24),Input(4),K(4,24)
	{

	}
	public: ~vantloadMarcelo(){}
	public: void config()
	{
		T = 0.012;

		K << 0.022251392000988, 3.610826372848198, 3.943587093610300, -9.034518066592437, 0.039653987198220, -0.241757162319362, 0.061306672458228, -0.044821776616793, 0.309374456114332, 0.001748088655740, 0.014542375733504, 2.703032939140247, 3.125621405238989, -1.684209819593526, 0.007876061884759, -0.078095879721028, 0.000281393541496, -0.000276991508491, -0.077015595065443, 0.000856503366214, 0.010084993403776, 1.537439810050385, 1.633800433868023,-0.157157443888174, 

-0.020813121711187, -3.614092170663940, 3.947085771680709, 9.043200746822759, -0.020082443304359, 0.241968260346650, -0.061344467152279, 0.044878472679226, -0.309648022307062, -0.001757351313423, -0.012340611073574, -2.705513729255204, 3.128405764711232, 1.686079591040529, 0.003660858664638, 0.078162056978699, -0.000284233999991, 0.000274660145251, 0.077084577482598, -0.000881434768221, -0.009686229162019, -1.538823509215560, 1.635245164541128, 0.157295459552833, 

0.200889565833321, 0.026515797035201, -0.000011682858462, -0.045504782961918, 0.295963232371242, 0.115422324926127, 0.262318418618355, 0.059563107792473, 0.000892614162504, 0.010498701106612, 0.125706400293278, 0.017686168835410, 0.000005056981452, -0.004437388623971, 0.031574381469096, 0.029213890367875, 0.010015532413157, 0.000594026279097, -0.001077209467075, 0.005487913164878, 0.092481784252386, 0.011844536186301, -0.000011495532807, 0.080014517913616,

0.201722146581317, -0.029885817596187, -0.000012135266896, 0.053463401580167, 0.297540118492152, -0.115702318885957, 0.059818345062158, 0.263706738056797, -0.001255975458065, 0.010480355551767, 0.126278440143691, -0.020181017766001, 0.000005014602256, 0.005689682400861, 0.031764759414179, -0.029301610562947, 0.000595800675554, 0.010063321920969, 0.001183256863290, 0.005523771833418, 0.092849108902727, -0.013282881948145, -0.000011811007142, -0.080201187968573;
	}
	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{

		static double xint, x_ant = 0;
		static double yint, y_ant = 0;
		static double zint, z_ant = 0;
		static double yawint, yaw_ant = 0;

		simulator_msgs::Sensor msgstates;
		int i = 0;		
		while(true)
		{
			if (arraymsg.values.at(i).name == "loadperspective_states"){
				msgstates = arraymsg.values.at(i);			 	
				break;
			}
			i++;			
		}

		// Integrador Trapezoidal
		double x_atual = msgstates.values.at(0) - Ref(0);
		xint = xint + (T/2)*(x_atual + x_ant);
		x_ant = x_atual;
		double y_atual = msgstates.values.at(1) - Ref(1);
		yint = yint + (T/2)*(y_atual + y_ant);
		y_ant = y_atual;
		double z_atual = msgstates.values.at(2) - Ref(2);
		zint = zint + (T/2)*(z_atual + z_ant);
		z_ant = z_atual;
		double yaw_atual = msgstates.values.at(5) - Ref(3);
		yawint = yawint + (T/2)*(yaw_atual + yaw_ant);
		yaw_ant = yaw_atual;

		

		std::vector<double> etadot = wIIL2EtaDot(msgstates.values.at(13),
							msgstates.values.at(14),
                                                         msgstates.values.at(15),
                                                         msgstates.values.at(3),
                                                         msgstates.values.at(4),
                                                         msgstates.values.at(5));

		
		X << msgstates.values.at(0),//x
                     msgstates.values.at(1),//y
                     msgstates.values.at(2),//z
                     msgstates.values.at(3),//roll
                     msgstates.values.at(4),//pitch
                     msgstates.values.at(5),//yaw
		     msgstates.values.at(6),//aR
                     msgstates.values.at(7),//aL
                     msgstates.values.at(8),//g1 - x
                     msgstates.values.at(9),//g2 - y
                     msgstates.values.at(10),//vx
                     msgstates.values.at(11),//vy
                     msgstates.values.at(12),//vz
                     etadot.at(0),//droll
                     etadot.at(1),//pitch
                     etadot.at(2),//yaw
		     msgstates.values.at(16),
                     msgstates.values.at(17),
                     msgstates.values.at(19),
                     msgstates.values.at(18),
                     xint,
                     yint,
                     zint,
                     yawint;

		
		Ref << 0,0,1,0,0,0,0.00002965,0.004885,0.004893,
			0.00484,0,0; 
		
		/*Equilibrium values*/
		double phi = -0.000039033, theta =-0.0335494, aR = 0.0334508, aL = 0.033393, g1 = 0.000039011, g2 = 0.0335494;

		Erro(0)  = X(0)  - Ref(0); // x
		Erro(1)  = X(1)  - Ref(1); // y
		Erro(2)  = X(2)  - Ref(2); // z
		Erro(3)  = X(3)  - phi;    // phi
		Erro(4)  = X(4)  - theta;  // theta
		Erro(5)  = X(5)  - Ref(3); // psi
		Erro(6)  = X(6)  - aR; 	   // alphaR
		Erro(7)  = X(7)  - aL;     // alphaL
		Erro(8)  = X(8)  - g1; 	   // gamma1
		Erro(9)  = X(9)  - g2; 	   // gamma2
		Erro(10) = X(10) - Ref(4); // dx
		Erro(11) = X(11) - Ref(5); // dy
		Erro(12) = X(12) - Ref(6); // dz
		Erro(13) = X(13) - 0; 	   // dphi
		Erro(14) = X(14) - 0;	   // dtheta
		Erro(15) = X(15) - Ref(7); // dpsi
		Erro(16) = X(16) - 0;	   // dalphaR
		Erro(17) = X(17) - 0;	   // dalphaL
		Erro(18) = X(18) - 0;	   // dgamma1
		Erro(19) = X(19) - 0;	   // dgamma2
		Erro(20) = X(20) - 0;	   // intx
		Erro(21) = X(21) - 0;	   // inty
		Erro(22) = X(22) - 0;	   // intz
		Erro(23) = X(23) - 0;	   // intpsi

		Input = -K*Erro;
		
 		Input(0) = Input(0) + 12.6005;
                Input(1) = Input(1) + 12.609;
		
                std::vector<double> out(4);
		out.at(0) = Input(0);
		out.at(1) = Input(1);
		out.at(2) = Input(2);
		out.at(3) = Input(3);
		return out;
	}
	public: std::vector<double> Reference()
	{
		std::vector<double> out(1);
		out.at(0);
		return out;
	}
	public: std::vector<double> Error()
	{
		std::vector<double> out(1);
		out.at(0);		
		return out;
	}
	public: std::vector<double> State()

	{
		std::vector<double> out(1);
		out.at(0);		
		return out;
	}
	
	private: std::vector<double> wIIL2EtaDot(double in_a, double in_b, double in_c, double phi, double theta, double psii)
        {
            std::vector<double> out;
            out.push_back((in_a*cos(psii) + in_b*sin(psii))/cos(theta));
            out.push_back(in_b*cos(psii) - in_a*sin(psii));
            out.push_back(in_c + in_a*cos(psii)*tan(theta) + in_b*sin(psii)*tan(theta));
            return out;
        }

        private: std::vector<double> pqr2EtaDot(double in_a, double in_b, double in_c, double phi, double theta, double psii)
        {
            std::vector<double> out;
            out.push_back(in_a + in_c*cos(phi)*tan(theta) + in_b*sin(phi)*tan(theta));
            out.push_back(in_b*cos(phi) - in_c*sin(phi));
            out.push_back((in_c*cos(phi))/cos(theta) + (in_b*sin(phi))/cos(theta));
            return out;
        }

};


extern "C"
{
	Icontroller *create(void) {return new vantloadMarcelo;}
	void destroy(Icontroller *p) {delete p;}
}
