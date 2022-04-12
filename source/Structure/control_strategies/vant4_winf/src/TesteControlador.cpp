#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <math.h>
#include "InertiaMatrix.h"
#include "CoriolisMatrix.h"
#include "GravitationalVector.h"
#include "InputCoupling.h"


using namespace std;

int main() {


        
        //Variáveis
        int Iterations;
        double T;
        
        //vetores de dados
        Eigen::VectorXd qref(8);
	Eigen::VectorXd Erro(12);
	Eigen::VectorXd x(16);

        //Vetores para controle
        Eigen::VectorXd q(8);
        Eigen::VectorXd qp(8);
        Eigen::VectorXd qrp(8);
        Eigen::VectorXd qrpp(8);
        Eigen::VectorXd intqctil(4);
        Eigen::VectorXd qc(4);
        Eigen::VectorXd qcr(4);
        Eigen::VectorXd qctil(4);
        Eigen::VectorXd qptil(8);   
	Eigen::VectorXd Input(8);
	Eigen::VectorXd Input2(4);
       

	//Variáveis do controlador
	Eigen::MatrixXd Uc(4,4);
	Eigen::MatrixXd Qc(4,4);
	Eigen::MatrixXd Kc(4,4);
	Eigen::MatrixXd Tc(4,4);
	Eigen::MatrixXd invT3c(4,4);
	Eigen::MatrixXd invY1c(4,4);
	Eigen::VectorXd dc(8);  //FF
	Eigen::VectorXd V1(8);
	Eigen::VectorXd V2(8);
	Eigen::MatrixXd K1(4,16);
	Eigen::MatrixXd K2(4,16);
	Eigen::MatrixXd K(8,16);
	Eigen::VectorXd Trajectory(20);
	Eigen::VectorXd Uopt(8);
	
	
	//Matrizes TIlt-rotor
	Eigen::MatrixXd M(8,8); // Inertia
	Eigen::MatrixXd C(8,8); // Coriolis
        Eigen::VectorXd G(8);   // Gravitation
	Eigen::MatrixXd Bi(8,4); // Auxiliar InputcouplingMatrix
	Eigen::MatrixXd B(8,4); // InputcouplingMatrix

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
		
		T = 0.012;
		Iterations = 0;

		static double Tempo = 0;
		Trajectory << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
		Tempo = Tempo + T;
		
		// integrators variables
                static double yawint, yaw_ant = 0;
            	static double xint, x_ant = 0;
		static double yint, y_ant = 0;
		static double zint, z_ant = 0;
                
                double yaw_error = Trajectory(0)-0;
		yawint = yawint + (T/2)*(yaw_error + yaw_ant);
		yaw_ant = yaw_error;

		double x_error = Trajectory(1)-0;
		xint = xint + (T/2)*(x_error + x_ant);
		x_ant = x_error;

		double y_error = Trajectory(2)-0;
		yint = yint + (T/2)*(y_error + y_ant);
		y_ant = y_error;
                
                double z_error = Trajectory(3)-0;
		zint = zint + (T/2)*(z_error + z_ant);
		z_ant = z_error;
                

		
                //Variáveis de Configuração
                double AlphaR = 0; 
                double AlphaL = 0; 
                double Phi    = 0; 
                double Theta  = 0; 
                double Psi    = 0; 
                double X      = 0; 
                double Y      = 0; 
                double Z      = 0; 

                //Derivada Variáveis de Configuração
                double AlphaRp = 0;
                double AlphaLp = 0; 
                double Phip = 0; 
                double Thetap = 0; 
                double Psip = 0; 
                double Xp = 0; 
                double Yp = 0; 
                double Zp = 0; 
                
                
                
                q << AlphaR, AlphaL, Phi, Theta, Psi, X, Y, Z;
                qc << Psi, X, Y, Z;
                qcr << Trajectory(0), Trajectory(1), Trajectory(2), Trajectory(3); //[psi x y z]
                qp << AlphaR, AlphaL, Phi, Theta, Psi, X, Y, Z;
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
		Erro << qptil, qctil;

		//FF		
		V1 << Eigen::MatrixXd::Zero(4,1),  qrp(4),  qrp(5),  qrp(6),  qrp(7); 
		V2 << Eigen::MatrixXd::Zero(4,1), qrpp(4), qrpp(5), qrpp(6), qrpp(7);
		
		dc =  G + M*V2 + C*V1;
		
		
		K1 << invY1c*Uc, Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4);
		K2 << Eigen::MatrixXd::Zero(4,4), invT3c*Qc, invT3c*Kc, invT3c*Tc;
		
		K << K1, K2;
		
		Uopt = -M*(K*x) + C*(qp-qrp) + dc;  
				      
				      
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
		std::cout << Input2;
		
		std::cout << "Passei Aqui!" << std::endl;
		
	return 0;
}
