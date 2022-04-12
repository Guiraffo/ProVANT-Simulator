
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
    
    //vetor gravidade
    G << - M2*ds*g*cos(AlphaR)*sin(Theta) - M2*ds*g*sin((B*pi)/180)*sin(AlphaR)*cos(Theta)*sin(Phi) - M2*ds*g*cos((B*pi)/180)*cos(Phi)*sin(AlphaR)*cos(Theta),
           M3*ds*g*sin((B*pi)/180)*sin(AlphaL)*cos(Theta)*sin(Phi) - M3*ds*g*cos(AlphaL)*sin(Theta) - M3*ds*g*cos((B*pi)/180)*cos(Phi)*sin(AlphaL)*cos(Theta),
           M1*YB1*g*cos(Phi)*cos(Theta) + M2*YB2*g*cos(Phi)*cos(Theta) + M3*YB3*g*cos(Phi)*cos(Theta) - M1*ZB1*g*cos(Theta)*sin(Phi) - M2*ZB2*g*cos(Theta)*sin(Phi) - M3*ZB3*g*cos(Theta)*sin(Phi) - M3*ds*g*cos((B*pi)/180)*cos(AlphaL)*cos(Theta)*sin(Phi) - M3*ds*g*sin((B*pi)/180)*cos(AlphaL)*cos(Phi)*cos(Theta) - M2*ds*g*cos((B*pi)/180)*cos(AlphaR)*cos(Theta)*sin(Phi) + M2*ds*g*sin((B*pi)/180)*cos(AlphaR)*cos(Phi)*cos(Theta),
           M3*ds*g*sin((B*pi)/180)*cos(AlphaL)*sin(Phi)*sin(Theta) - M2*XB2*g*cos(Theta) - M3*XB3*g*cos(Theta) - M3*ds*g*sin(AlphaL)*cos(Theta) - M2*ds*g*sin(AlphaR)*cos(Theta) - M1*ZB1*g*cos(Phi)*sin(Theta) - M2*ZB2*g*cos(Phi)*sin(Theta) - M3*ZB3*g*cos(Phi)*sin(Theta) - M1*YB1*g*sin(Phi)*sin(Theta) - M2*YB2*g*sin(Phi)*sin(Theta) - M3*YB3*g*sin(Phi)*sin(Theta) - M1*XB1*g*cos(Theta) - M2*ds*g*sin((B*pi)/180)*cos(AlphaR)*sin(Phi)*sin(Theta) - M3*ds*g*cos((B*pi)/180)*cos(AlphaL)*cos(Phi)*sin(Theta) - M2*ds*g*cos((B*pi)/180)*cos(AlphaR)*cos(Phi)*sin(Theta),
           0,
           0,
           0,
          M1*g + M2*g + M3*g;
          
    return G;
      
}