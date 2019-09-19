
#include <iostream>
//#include <Eigen/Eigen>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::MatrixXd InputCouplingMatrix(Eigen::VectorXd q)
{
    
    //Variáveis de Configuração
    double AlphaR = q(0);
    double AlphaL = q(1); 
    double Phi    = q(2);
    double Theta  = q(3);
    double Psi    = q(4);
    double X      = q(5);
    double Y      = q(6);
    double Z      = q(7);

/* works well - first version
    const double Ixx1 = 0.1489;
    const double Iyy1 = 0.1789;
    const double Izz1 = 0.3011;
    const double Ixz1 = -0.0189;
    const double Ixx2 = 0.0007103;
    const double Iyy2 = 0.00071045;
    const double Izz2 = 0.00021337;
    const double Ixx3 = 0.0007103;
    const double Iyy3 = 0.00071045;
    const double Izz3 = 0.00021337;
    const double ds = 0.03;
    const double B = 5;
    const double g = 9.8;
    const double b = 9.5e-6;
    const double Kt = 1.7e-7;
    const double M1 = 7.0;
    const double M2 = 0.3;
    const double M3 = 0.3;
    const double XB1 = -0.0609;
    const double YB1 = 0;
    const double ZB1 = -0.0634;
    const double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628;
    const double XB2 = 0.0083;
    const double YB2 = -0.6073;
    const double ZB2 = 0.0406;
    const double XB3 = 0.0083;
    const double YB3 = 0.6073;
    const double ZB3 = 0.0406;
    */
    const double Ixx1 = 0.1489;
    const double Iyy1 = 0.1789;
    const double Izz1 = 0.3011;
    const double Ixz1 = -0.0189;
    const double Ixx2 = 0.0007103;
    const double Iyy2 = 0.00071045;
    const double Izz2 = 0.00021337;
    const double Ixx3 = 0.0007103;
    const double Iyy3 = 0.00071045;
    const double Izz3 = 0.00021337;
    const double ds = 0.03;
    const double B = 5; //deg
    const double g = 9.8;
    const double b = 9.5e-6;
    const double Kt = 1.7e-7;
    const double M1 = 7.0;
    const double M2 = 0.3;
    const double M3 = 0.3;
    const double XB1 = 0.06684;
    const double YB1 = 0;
    const double ZB1 = 0.005392;
    const double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628;
    const double XB2 = 0.078;
    const double YB2 = -0.6073;
    const double ZB2 = 0.1235;
    const double XB3 = 0.078;
    const double YB3 = 0.6073;
    const double ZB3 = 0.1235;

    Eigen::MatrixXd B1(8,2);
    Eigen::MatrixXd B2(8,2);
    Eigen::MatrixXd Baux(8,4);
    Eigen::MatrixXd BInputCoupling(8,4);
    Eigen::MatrixXd JvL(3,8);
    Eigen::MatrixXd JvR(3,8);
    Eigen::MatrixXd JwR(3,8);
    Eigen::MatrixXd JwL(3,8);
    Eigen::VectorXd az(3);
    Eigen::MatrixXd RIC2(3,3);
    Eigen::MatrixXd RIC3(3,3);
    Eigen::MatrixXd MatrizRotacaoPropulsorL(3,3);
    Eigen::MatrixXd MatrizRotacaoPropulsorR(3,3);

JvL << 1, 0, 0, YB3*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + ZB3*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + ds*(cos((B*pi)/180)*cos(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - sin((B*pi)/180)*cos(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))), -cos(Psi)*(XB3*sin(Theta) - ZB3*cos(Phi)*cos(Theta) - YB3*cos(Theta)*sin(Phi) + ds*sin(AlphaL)*sin(Theta) - ds*cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*cos(Theta) + ds*sin((B*pi)/180)*cos(AlphaL)*cos(Theta)*sin(Phi)), ds*(cos((B*pi)/180)*cos(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - sin(AlphaL)*cos(Theta)*sin(Psi)) - YB3*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + ZB3*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - XB3*cos(Theta)*sin(Psi), 0, ds*cos(AlphaL)*cos(Psi)*cos(Theta) - ds*sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - ds*cos((B*pi)/180)*sin(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)),
0, 1, 0, - YB3*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - ZB3*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - ds*(cos((B*pi)/180)*cos(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - sin((B*pi)/180)*cos(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))), -sin(Psi)*(XB3*sin(Theta) - ZB3*cos(Phi)*cos(Theta) - YB3*cos(Theta)*sin(Phi) + ds*sin(AlphaL)*sin(Theta) - ds*cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*cos(Theta) + ds*sin((B*pi)/180)*cos(AlphaL)*cos(Theta)*sin(Phi)), ds*(cos((B*pi)/180)*cos(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaL)*cos(Theta)) - YB3*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + ZB3*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + XB3*cos(Psi)*cos(Theta), 0, ds*cos((B*pi)/180)*sin(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + ds*sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + ds*cos(AlphaL)*cos(Theta)*sin(Psi),
0, 0, 1, -cos(Theta)*(ZB3*sin(Phi) - YB3*cos(Phi) + ds*cos((B*pi)/180)*cos(AlphaL)*sin(Phi) + ds*sin((B*pi)/180)*cos(AlphaL)*cos(Phi)), ds*sin((B*pi)/180)*cos(AlphaL)*sin(Phi)*sin(Theta) - ds*sin(AlphaL)*cos(Theta) - ZB3*cos(Phi)*sin(Theta) - YB3*sin(Phi)*sin(Theta) - ds*cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*sin(Theta) - XB3*cos(Theta), 0, 0, ds*sin((B*pi)/180)*sin(AlphaL)*cos(Theta)*sin(Phi) - ds*cos((B*pi)/180)*cos(Phi)*sin(AlphaL)*cos(Theta) - ds*cos(AlphaL)*sin(Theta);

JvR  << 1, 0, 0, YB2*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + ZB2*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + ds*(cos((B*pi)/180)*cos(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta))), cos(Psi)*(ZB2*cos(Phi)*cos(Theta) - XB2*sin(Theta) + YB2*cos(Theta)*sin(Phi) - ds*sin(AlphaR)*sin(Theta) + ds*cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*cos(Theta) + ds*sin((B*pi)/180)*cos(AlphaR)*cos(Theta)*sin(Phi)), ZB2*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - YB2*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - ds*(sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos((B*pi)/180)*cos(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin(AlphaR)*cos(Theta)*sin(Psi)) - XB2*cos(Theta)*sin(Psi), ds*sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - ds*cos((B*pi)/180)*sin(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + ds*cos(AlphaR)*cos(Psi)*cos(Theta), 0,
0, 1, 0, - YB2*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - ZB2*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - ds*(cos((B*pi)/180)*cos(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta))), sin(Psi)*(ZB2*cos(Phi)*cos(Theta) - XB2*sin(Theta) + YB2*cos(Theta)*sin(Phi) - ds*sin(AlphaR)*sin(Theta) + ds*cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*cos(Theta) + ds*sin((B*pi)/180)*cos(AlphaR)*cos(Theta)*sin(Phi)), ds*(cos((B*pi)/180)*cos(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaR)*cos(Theta)) - YB2*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + ZB2*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + XB2*cos(Psi)*cos(Theta), ds*cos((B*pi)/180)*sin(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - ds*sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + ds*cos(AlphaR)*cos(Theta)*sin(Psi), 0,
0, 0, 1, cos(Theta)*(YB2*cos(Phi) - ZB2*sin(Phi) - ds*cos((B*pi)/180)*cos(AlphaR)*sin(Phi) + ds*sin((B*pi)/180)*cos(AlphaR)*cos(Phi)), - XB2*cos(Theta) - ds*sin(AlphaR)*cos(Theta) - ZB2*cos(Phi)*sin(Theta) - YB2*sin(Phi)*sin(Theta) - ds*cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*sin(Theta) - ds*sin((B*pi)/180)*cos(AlphaR)*sin(Phi)*sin(Theta), 0, - ds*cos(AlphaR)*sin(Theta) - ds*cos((B*pi)/180)*cos(Phi)*sin(AlphaR)*cos(Theta) - ds*sin((B*pi)/180)*sin(AlphaR)*cos(Theta)*sin(Phi), 0;

MatrizRotacaoPropulsorR << sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - cos((B*pi)/180)*sin(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + cos(AlphaR)*cos(Psi)*cos(Theta), - cos((B*pi)/180)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - sin((B*pi)/180)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)), cos((B*pi)/180)*cos(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaR)*cos(Theta),
cos((B*pi)/180)*sin(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + cos(AlphaR)*cos(Theta)*sin(Psi), cos((B*pi)/180)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)), sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos((B*pi)/180)*cos(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin(AlphaR)*cos(Theta)*sin(Psi),
- cos(AlphaR)*sin(Theta) - cos((B*pi)/180)*cos(Phi)*sin(AlphaR)*cos(Theta) - sin((B*pi)/180)*sin(AlphaR)*cos(Theta)*sin(Phi), sin(Phi - (B*pi)/180)*cos(Theta), cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*cos(Theta) - sin(AlphaR)*sin(Theta) + sin((B*pi)/180)*cos(AlphaR)*cos(Theta)*sin(Phi);

MatrizRotacaoPropulsorL << cos(AlphaL)*cos(Psi)*cos(Theta) - sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - cos((B*pi)/180)*sin(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)), sin((B*pi)/180)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - cos((B*pi)/180)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)), cos((B*pi)/180)*cos(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaL)*cos(Theta),
cos((B*pi)/180)*sin(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + cos(AlphaL)*cos(Theta)*sin(Psi), cos((B*pi)/180)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - sin((B*pi)/180)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)), sin(AlphaL)*cos(Theta)*sin(Psi) - sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos((B*pi)/180)*cos(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)),
sin((B*pi)/180)*sin(AlphaL)*cos(Theta)*sin(Phi) - cos((B*pi)/180)*cos(Phi)*sin(AlphaL)*cos(Theta) - cos(AlphaL)*sin(Theta), sin(Phi + (B*pi)/180)*cos(Theta), cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*cos(Theta) - sin(AlphaL)*sin(Theta) - sin((B*pi)/180)*cos(AlphaL)*cos(Theta)*sin(Phi);

az << 0, 0, 1;

JwR << 0, 0, 0, cos(Psi)*cos(Theta), -sin(Psi), 0, cos((B*pi)/180)*cos(Psi)*sin(Phi)*sin(Theta) - sin((B*pi)/180)*sin(Phi)*sin(Psi) - cos((B*pi)/180)*cos(Phi)*sin(Psi) - sin((B*pi)/180)*cos(Phi)*cos(Psi)*sin(Theta), 0,
0, 0, 0, cos(Theta)*sin(Psi), cos(Psi), 0, cos((B*pi)/180)*cos(Phi)*cos(Psi) + sin((B*pi)/180)*cos(Psi)*sin(Phi) + cos((B*pi)/180)*sin(Phi)*sin(Psi)*sin(Theta) - sin((B*pi)/180)*cos(Phi)*sin(Psi)*sin(Theta), 0,
0, 0, 0, -sin(Theta), 0, 1, sin(Phi - (B*pi)/180)*cos(Theta), 0;

JwL << 0, 0, 0, cos(Psi)*cos(Theta), -sin(Psi), 0, 0, sin((B*pi)/180)*sin(Phi)*sin(Psi) - cos((B*pi)/180)*cos(Phi)*sin(Psi) + cos((B*pi)/180)*cos(Psi)*sin(Phi)*sin(Theta) + sin((B*pi)/180)*cos(Phi)*cos(Psi)*sin(Theta),
0, 0, 0, cos(Theta)*sin(Psi), cos(Psi), 0, 0, cos((B*pi)/180)*cos(Phi)*cos(Psi) - sin((B*pi)/180)*cos(Psi)*sin(Phi) + cos((B*pi)/180)*sin(Phi)*sin(Psi)*sin(Theta) + sin((B*pi)/180)*cos(Phi)*sin(Psi)*sin(Theta),
0, 0, 0, -sin(Theta), 0, 1, 0, sin(Phi + (B*pi)/180)*cos(Theta);

RIC2 << sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - cos((B*pi)/180)*sin(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + cos(AlphaR)*cos(Psi)*cos(Theta), - cos((B*pi)/180)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - sin((B*pi)/180)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)), cos((B*pi)/180)*cos(AlphaR)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaR)*cos(Theta),
cos((B*pi)/180)*sin(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) - sin((B*pi)/180)*sin(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + cos(AlphaR)*cos(Theta)*sin(Psi), cos((B*pi)/180)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)), sin((B*pi)/180)*cos(AlphaR)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos((B*pi)/180)*cos(AlphaR)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin(AlphaR)*cos(Theta)*sin(Psi),
- cos(AlphaR)*sin(Theta) - cos((B*pi)/180)*cos(Phi)*sin(AlphaR)*cos(Theta) - sin((B*pi)/180)*sin(AlphaR)*cos(Theta)*sin(Phi), sin(Phi - (B*pi)/180)*cos(Theta), cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*cos(Theta) - sin(AlphaR)*sin(Theta) + sin((B*pi)/180)*cos(AlphaR)*cos(Theta)*sin(Phi);

RIC3 << cos(AlphaL)*cos(Psi)*cos(Theta) - sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) - cos((B*pi)/180)*sin(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)), sin((B*pi)/180)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) - cos((B*pi)/180)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)), cos((B*pi)/180)*cos(AlphaL)*(sin(Phi)*sin(Psi) + cos(Phi)*cos(Psi)*sin(Theta)) + sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*sin(Psi) - cos(Psi)*sin(Phi)*sin(Theta)) + cos(Psi)*sin(AlphaL)*cos(Theta),
cos((B*pi)/180)*sin(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)) + sin((B*pi)/180)*sin(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) + cos(AlphaL)*cos(Theta)*sin(Psi), cos((B*pi)/180)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - sin((B*pi)/180)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)), sin(AlphaL)*cos(Theta)*sin(Psi) - sin((B*pi)/180)*cos(AlphaL)*(cos(Phi)*cos(Psi) + sin(Phi)*sin(Psi)*sin(Theta)) - cos((B*pi)/180)*cos(AlphaL)*(cos(Psi)*sin(Phi) - cos(Phi)*sin(Psi)*sin(Theta)),
sin((B*pi)/180)*sin(AlphaL)*cos(Theta)*sin(Phi) - cos((B*pi)/180)*cos(Phi)*sin(AlphaL)*cos(Theta) - cos(AlphaL)*sin(Theta), sin(Phi + (B*pi)/180)*cos(Theta), cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*cos(Theta) - sin(AlphaL)*sin(Theta) - sin((B*pi)/180)*cos(AlphaL)*cos(Theta)*sin(Phi);

B2 << 0, 0,
      0, 0,
      0, 0,
      0, 0,
      0, 0,
      0, 0,
      1, 0,
      0, 1;

B1 << JvR.transpose()* MatrizRotacaoPropulsorR * az + JwR.transpose()*RIC2*az*(Kt/b), JvL.transpose()* MatrizRotacaoPropulsorL * az + JwL.transpose()*RIC3*az*(-Kt/b);

//Adjust the Input coupling matrix considering q = [AlphaR AlphaL Phi Theta Psi X Y Z];

Baux << B1, B2;

BInputCoupling  <<	Baux(6,0), Baux(6,1), Baux(6,2), Baux(6,3),
	     		Baux(7,0), Baux(7,1), Baux(7,2), Baux(7,3),
			Baux(3,0), Baux(3,1), Baux(3,2), Baux(3,3),
			Baux(4,0), Baux(4,1), Baux(4,2), Baux(4,3),
			Baux(5,0), Baux(5,1), Baux(5,2), Baux(5,3),
			Baux(0,0), Baux(0,1), Baux(0,2), Baux(0,3),
			Baux(1,0), Baux(1,1), Baux(1,2), Baux(1,3),
			Baux(2,0), Baux(2,1), Baux(2,2), Baux(2,3);
	
return BInputCoupling ;

}

