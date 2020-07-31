#include <iostream>
#include <eigen3/Eigen/Eigen>
#include <stdio.h>
#include <math.h>
#include "InertiaMatrix.h"
#include "CoriolisMatrix.h"
#include "GravitationalVector.h"
#include "InputCoupling.h"
#include "InputCouplingAero.h"


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
	Eigen::MatrixXd G(8,1);
	Eigen::MatrixXd BIC(8,5);
	Eigen::MatrixXd B(8,8);
	Eigen::MatrixXd M(8,8);
	Eigen::MatrixXd C(8,8);
	double ub;
		q << pi/2, pi/2, 1, -1, 1, 0, 0, 0;
		qp << -40, -50, 100, 100, 200, 150, 80, -150;
		
		//Teste1
		M = InertiaMatrix(q);
		C = coriolisMatrix(q,qp);
		G = GravitationVector(q);
		B = InputCouplingMatrix(q);	
		BIC = InputCouplingMatrixAero(q, qp, &ub);
		std::cout << "InertiaMatrix:" << std::endl << M << std::endl;
		std::cout << "CoriolisMatrix:" << std::endl << C << std::endl;
		std::cout << "GravitationVector:" << std::endl << G << std::endl;
		std::cout << "BAero:" << std::endl << BIC << std::endl;
		std::cout << "B:" << std::endl << B << std::endl;
//		std::cout << BIC << std::endl;
//		Eigen::VectorXd FUa(8);
//		FUa = BaeroFUa.col(4);
//		std::cout << FUa << std::endl;
		
	return 0;
}
