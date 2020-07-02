#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <math.h>
#include "InertiaMatrix.h"
#include "CoriolisMatrix.h"
#include "GravitationalVector.h"
#include "InputCoupling.h"


using namespace std;

//int main() {
//		//Eigen::VectorXd X1(8);
//                //Eigen::VectorXd X2(8);
//		Eigen::MatrixXd X3(2,8);
//		Eigen::MatrixXd Xr(11,2);
//		//X1 << 1, 0, 0, 0, 0, 0, 0, 0;
//		//X2 << 2, 0, 5, 0, 0, 0, 0, 0;
//		X3 << 1, 0, 0, 0, 0, 0, 0, 0,
//		      2, 0, 5, 0, 0, 0, 0, 0;
//		//Xr << X1, X2, 1, 2;
//		Xr << X3.transpose(), 1, 2, Eigen::MatrixXd::Identity(2);
//		std::cout << "Resulting matrix" << std::endl;
//		std::cout << Xr << std::endl << std::endl;
//	return 0;
//}

int main() {
        
        Eigen::VectorXd q(8);
        Eigen::VectorXd qp(8);
        const double pi = 3.141592653589793;

		q << pi/2, -pi/2, 0, 0, 0, 0, 0, 0;
		qp << 0, 0, 0, 0, 0, 0, 0, 0;

		Eigen::MatrixXd M(8,8);
		Eigen::MatrixXd C(8,8);
		Eigen::VectorXd G(8);
		Eigen::MatrixXd B(8,4);
		Eigen::MatrixXd A1(3,2);
		Eigen::MatrixXd APinv(2,3);
		
		
		
		//Teste1
		M = InertiaMatrix(q);
		C = coriolisMatrix(q,qp);
		G = GravitationVector(q);
		B = InputCouplingMatrix(q);
		
		 
		
		std::cout << "Inertia Matrix" << std::endl;
		std::cout << M << std::endl << std::endl;
		
		std::cout << "Coriolis Matrix" << std::endl;
		std::cout << C << std::endl << std::endl;
		
		std::cout << "Gravitation Vector" << std::endl;
		std::cout << G << std::endl << std::endl;
		
		std::cout << "Input Coupling Matrix" << std::endl;
		std::cout << B << std::endl << std::endl;
		
		//Teste Pseudo-inverse
		
		A1 << 1, 2,
		      3, 4,
		      5, 6;
		std::cout << "A1" << std::endl;
		std::cout << A1 << std::endl << std::endl;

		APinv << (A1.transpose()*A1).inverse()*A1.transpose();
		std::cout << "Test Pseudo-inverse" << std::endl;
		std::cout << APinv << std::endl << std::endl;
		
		
		//teste 3
		
		Eigen::VectorXd V1(8);
		Eigen::VectorXd V2(8);
		Eigen::VectorXd qrp(8);
		Eigen::VectorXd qrpp(8);
		
		qrp << 0, 0, 0, 0, 0, 0, 0, 0;
		qrpp << 0, 0, 0, 0, 0, 0, 0, 0;
		
		V1 = Eigen::MatrixXd::Zero(4,1), qrp(2), qrp(3), qrp(4), qrp(5); 
		V2 = Eigen::MatrixXd::Zero(4,1), qrpp(2), qrpp(3), qrpp(4), qrpp(5);
		
		//teste 4
		//Variáveis do controlador
		Eigen::MatrixXd Uc(4,4);
		Eigen::MatrixXd Qc(4,4);
		Eigen::MatrixXd Kc(4,4);
		Eigen::MatrixXd Tc(4,4);
		Eigen::MatrixXd invT3c(4,4);
		Eigen::MatrixXd invY1c(4,4);
		Eigen::VectorXd dc(8);  //FF
	
	
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
			     
			     
		Eigen::MatrixXd K1(4,16);
		Eigen::MatrixXd K2(4,16);
		Eigen::MatrixXd K(8,16);
		
		K1 << invY1c*Uc, Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4), Eigen::MatrixXd::Zero(4,4);
		K2 << Eigen::MatrixXd::Zero(4,4), invT3c*Qc, invT3c*Kc, invT3c*Tc;
		
		K << K1, K2;
		
		std::cout << "K" << std::endl;
		std::cout << K << std::endl << std::endl;
		
		
	return 0;
}
