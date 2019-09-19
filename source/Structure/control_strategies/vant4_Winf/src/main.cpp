#include "Icontroller.hpp"
#include <Eigen/Eigen>
#include "InertiaMatrix.h"
#include "CoriolisMatrix.h"
#include "GravitationalVector.h"
#include "InputCoupling.h"


class vant4_Winf : public Icontroller
{
        
        //Variáveis
        private: int Iterations;
        private: double T;
        
        //vetores de dados
        private: Eigen::VectorXd qref;
	private: Eigen::VectorXd Erro;
	private: Eigen::VectorXd x;
	private: Eigen::VectorXd Xref;

        //Vetores para controle
        private: Eigen::VectorXd q;
        private: Eigen::VectorXd qp;
        private: Eigen::VectorXd qrp;
        private: Eigen::VectorXd qrpp;
        private: Eigen::VectorXd intqctil;
        private: Eigen::VectorXd qc;
        private: Eigen::VectorXd qcr;
        private: Eigen::VectorXd qctil;
        private: Eigen::VectorXd qptil;   
	private: Eigen::VectorXd Input;
	private: Eigen::VectorXd Input2;
        
        //Variáveis de Configuração
        private: double AlphaR;
        private: double AlphaL;
        private: double Phi;
        private: double Theta;
        private: double Psi;
        private: double X;
        private: double Y;
        private: double Z;

        //Derivada Variáveis de Configuração
        private: double AlphaRp;
        private: double AlphaLp;
        private: double Phip;
        private: double Thetap;
        private: double Psip;
        private: double Xp;
        private: double Yp;
        private: double Zp;

	//Variáveis do controlador
	private: Eigen::MatrixXd Uc;
	private: Eigen::MatrixXd Qc;
	private: Eigen::MatrixXd Kc;
	private: Eigen::MatrixXd Tc;
	private: Eigen::MatrixXd invT3c;
	private: Eigen::MatrixXd invY1c;
	private: Eigen::VectorXd dc;  //FF
	private: Eigen::VectorXd V1;
	private: Eigen::VectorXd V2;
	private: Eigen::MatrixXd K1;
	private: Eigen::MatrixXd K2;
	private: Eigen::MatrixXd K;
	private: Eigen::VectorXd Trajectory;
	private: Eigen::VectorXd Uopt;
	
	
	//Matrizes TIlt-rotor
	private: Eigen::MatrixXd M; // Inertia
	private: Eigen::MatrixXd C; // Coriolis
        private: Eigen::VectorXd G;   // Gravitation
	private: Eigen::MatrixXd Bi; // Auxiliar InputcouplingMatrix
	private: Eigen::MatrixXd B; // InputcouplingMatrix
        
        public: vant4_Winf():q(8), qp(8), qrp(8), qrpp(8), intqctil(4), qc(4),  qcr(4),  qctil(4),  qptil(8), x(16), Input(8), Input2(4), Uc(4,4), Qc(4,4), Kc(4,4), Tc(4,4), 
        invT3c(4,4), invY1c(4,4), dc(8), V1(8),  V2(8), K1(4,16), K2(4,16), K(8,16), M(8,8), C(8,8), G(8), Bi(8,4), B(8,4), Trajectory(20), Uopt(8), Erro(8), qref(8), Xref(8)
	{
		T = 0.012;
		Iterations = 0;
		Uc << 0.2000,         0,         0,         0,
			   0,    0.2000,         0,         0,
			   0,         0,    0.9487,         0,
			   0,         0,         0,    0.2000;

		Qc << 37.3252,    0.0000,   -0.0000,   -0.0000,
		       0.0000,    4.9231,   -0.0000,    0.0000,
		      -0.0000,   -0.0000,    4.9231,   -0.0000,
		      -0.0000,    0.0000,   -0.0000,    6.5937;

		Kc <<  20.7195,    0.0000,   -0.0000,   -0.0000,
		        0.0000,    4.5592,   -0.0000,   -0.0000,
		       -0.0000,   -0.0000,    4.5592,   -0.0000,
		       -0.0000,   -0.0000,   -0.0000,    5.8691;

		Tc <<   1.7321,       0,         0,         0,
			  0   ,  1.0954,         0,         0,
			  0   ,       0,    1.0954,         0,
			  0   ,       0,         0,    1.0954;


		invT3c <<    0.0333,         0,         0,         0,
			  	  0,    0.5000,         0,         0,
				  0,         0,    0.5000,         0,
				  0,         0,         0,    0.5000;

		invY1c <<   100,     0,     0,     0,
			      0,   100,     0,     0,
			      0,     0,    10,     0,
			      0,     0,     0,   100;

	}
	
	public: ~vant4_Winf(){}
	
        public: void config(){
	
        }
        
        private: Eigen::VectorXd TrajetoriaReferenciaCompleta(double Tempo)
	{
		Eigen::VectorXd Traj(20); //[psi x y z Arp Alp phip thetap psip xp yp zp Arpp Alpp phipp thetapp psipp xpp ypp zpp]
                Traj << 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
		return Traj;
	}


	public: std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
	{
	std::cout << "macro" <<std::endl;
	
		//read data from sensors
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
		
		static double Tempo = 0;
		Trajectory = TrajetoriaReferenciaCompleta(Tempo);
		Tempo = Tempo + T;
		
		// integrators variables
        	static double yawint = 0, yaw_ant = 0;
        	static double xint = 0, x_ant = 0;
		static double yint = 0, y_ant = 0;
		static double zint = 0, z_ant = 0;
                
        	double yaw_error = msg.values.at(5)-Trajectory(0);
		yawint = yawint + (T/2)*(yaw_error + yaw_ant);
		yaw_ant = yaw_error;

		double x_error = msg.values.at(0)-Trajectory(1);
		xint = xint + (T/2)*(x_error + x_ant);
		x_ant = x_error;

		double y_error = msg.values.at(1)-Trajectory(2);
		yint = yint + (T/2)*(y_error + y_ant);
		y_ant = y_error;
                
        	double z_error = msg.values.at(2)-Trajectory(3);
		zint = zint + (T/2)*(z_error + z_ant);
		z_ant = z_error;
		
                //Variáveis de Configuração
                double AlphaR = msg.values.at(6);
                double AlphaL = msg.values.at(7); 
                double Phi    = msg.values.at(3);
                double Theta  = msg.values.at(4);
                double Psi    = msg.values.at(5);
                double X      = msg.values.at(0);
                double Y      = msg.values.at(1);
                double Z      = msg.values.at(2);

                //Derivada Variáveis de Configuração
                double AlphaRp = msg.values.at(14);
                double AlphaLp = msg.values.at(15); 
                double Phip = msg.values.at(11);
                double Thetap = msg.values.at(12);
                double Psip = msg.values.at(13);
                double Xp = msg.values.at(8);
                double Yp = msg.values.at(9);
                double Zp = msg.values.at(10);
                
                intqctil << yawint, xint, yint, zint;
                q << AlphaR, AlphaL, Phi, Theta, Psi, X, Y, Z;
                qc << Psi, X, Y, Z;
                qcr << Trajectory(0), Trajectory(1), Trajectory(2), Trajectory(3); //[psi x y z]
                qp << AlphaRp, AlphaLp, Phip, Thetap, Psip, Xp, Yp, Zp;
                qrp << Trajectory(4), Trajectory(5), Trajectory(6), Trajectory(7), Trajectory(8), Trajectory(9), Trajectory(10), Trajectory(11); //[Arp Alp phip thetap psip xp yp zp]
                qrpp << Trajectory(12), Trajectory(13), Trajectory(14), Trajectory(15), Trajectory(16), Trajectory(17), Trajectory(18), Trajectory(19); //[Arpp Alpp phipp thetapp psipp xpp ypp zpp]
                qctil = qc - qcr;
                qptil = qp - qrp;
		
		//Compute Euler-Lagrange matrices
		M = InertiaMatrix(q); 
		C = coriolisMatrix(q, qp);
		G = GravitationVector(q);
		Bi = InputCouplingMatrix(q);
		
		//State vector
		x << qptil, qctil, intqctil;
		Erro = Eigen::VectorXd::Zero(8);
		Xref = Eigen::VectorXd::Zero(8);
		//FF		
		V1 << Eigen::MatrixXd::Zero(4,1),  qrp(4),  qrp(5),  qrp(6),  qrp(7); 
		V2 << Eigen::MatrixXd::Zero(4,1), qrpp(4), qrpp(5), qrpp(6), qrpp(7);
		
		dc =  G + M*V2 + C*V1;
		
		K1 << invY1c*Uc, Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4);
		K2 << Eigen::MatrixXd::Zero(4,4), invT3c*Qc, invT3c*Kc, invT3c*Tc;
		
		K << K1, K2;
		
		Uopt = -M*K*x + C*(qp-qrp) + dc;
				      
		B = ((Bi.transpose()*Bi).inverse())*Bi.transpose();
                Input2 = B*Uopt;
		
		// output
		Input(0) = Input2(0);
		Input(1) = Input2(1);
		Input(2) = Input2(2);
		Input(3) = Input2(3);
		Input(4) = 0; 
		Input(5) = 0; 
		Input(6) = 0;
		Input(7) = 0;		
		
		std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
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
		std::vector<double> out(x.data(), x.data() + x.rows() * x.cols());
		return out;
	}	
};



extern "C"
{
	Icontroller *create(void) {return new vant4_Winf;}
	void destroy(Icontroller *p) {delete p;}
}
