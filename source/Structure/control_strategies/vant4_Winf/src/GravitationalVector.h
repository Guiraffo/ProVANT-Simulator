
#include <iostream>
//#include <Eigen/Eigen>
#include <eigen3/Eigen/Eigen>
#include <math.h>

Eigen::VectorXd GravitationVector(Eigen::VectorXd q)
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
    
    Eigen::VectorXd G(8);

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
    const double ds = 0.02;
    const double B = 0.0524; //rad
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

    double cosPhi = cos(Phi);
    double sinPhi = sin(Phi);
    double cosTheta = cos(Theta);
    double sinTheta = sin(Theta);
    double cosPsi = cos(Psi);
    double sinPsi = sin(Psi);
    double sinAlphaR = sin(AlphaR);
    double cosAlphaR = cos(AlphaR);
    double sinAlphaL = sin(AlphaL);
    double cosAlphaL = cos(AlphaL);
    double sinB = sin(B);
    double cosB = cos(B);
    
    //vetor gravidade
    G << -M2*ds*g*(cosAlphaR*cosB*sinTheta + cosB*cosPhi*sinAlphaR*cosTheta),
    -M3*ds*g*(cosAlphaL*cosB*sinTheta + cosB*cosPhi*sinAlphaL*cosTheta),
    M2*g*(ds*(cosPhi*sinB*cosTheta - cosAlphaR*cosB*cosTheta*sinPhi) + YB2*cosPhi*cosTheta - ZB2*cosTheta*sinPhi) - M3*g*(ds*(cosPhi*sinB*cosTheta + cosAlphaL*cosB*cosTheta*sinPhi) - YB3*cosPhi*cosTheta + ZB3*cosTheta*sinPhi) + M1*g*(YB1*cosPhi*cosTheta - ZB1*cosTheta*sinPhi),
    - M1*g*(XB1*cosTheta + ZB1*cosPhi*sinTheta + YB1*sinPhi*sinTheta) - M3*g*(ds*(cosB*sinAlphaL*cosTheta - sinB*sinPhi*sinTheta + cosAlphaL*cosB*cosPhi*sinTheta) + XB3*cosTheta + ZB3*cosPhi*sinTheta + YB3*sinPhi*sinTheta) - M2*g*(ds*(cosB*sinAlphaR*cosTheta + sinB*sinPhi*sinTheta + cosAlphaR*cosB*cosPhi*sinTheta) + XB2*cosTheta + ZB2*cosPhi*sinTheta + YB2*sinPhi*sinTheta),
    0,
    0,
    0,
    M1*g + M2*g + M3*g;
          
    return G;
      
}
