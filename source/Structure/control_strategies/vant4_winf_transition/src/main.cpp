#include <limits>

#include "Icontroller.hpp"
#include "vant4_winf_transition/inertia_matrix.h"
#include "vant4_winf_transition/CoriolisMatrix.h"
#include "vant4_winf_transition/gravitational_vector.h"
#include "vant4_winf_transition/input_coupling.h"
#include "vant4_winf_transition/input_coupling_aero.h"
#include "vant4_winf_transition/cplexutils.h"

class vant4_WinfTransition4 : public Icontroller
{
  // Variáveis
private:
  int Iterations;

private:
  double T;

private:
  const double pi = 3.14;

  // vetores de dados
private:
  Eigen::VectorXd qref;

private:
  Eigen::VectorXd Erro;

private:
  Eigen::VectorXd x;

private:
  Eigen::VectorXd Xref;

private:
  Eigen::VectorXd States;

  // Vetores para controle
private:
  Eigen::VectorXd q;

private:
  Eigen::VectorXd qp;

private:
  Eigen::VectorXd qpr;

private:
  Eigen::VectorXd qppr;

private:
  Eigen::VectorXd intqctil;

private:
  Eigen::VectorXd qc;

private:
  Eigen::VectorXd qcr;

private:
  Eigen::VectorXd qctil;

private:
  Eigen::VectorXd qptil;

private:
  Eigen::VectorXd Input;

private:
  Eigen::VectorXd Input2;

  // Variáveis de Configuração
private:
  double AlphaR;

private:
  double AlphaL;

private:
  double Phi;

private:
  double Theta;

private:
  double Psi;

private:
  double X;

private:
  double Y;

private:
  double Z;

  // Derivada Variáveis de Configuração
private:
  double AlphaRp;

private:
  double AlphaLp;

private:
  double Phip;

private:
  double Thetap;

private:
  double Psip;

private:
  double Xp;

private:
  double Yp;

private:
  double Zp;

  // Variáveis do controlador
private:
  Eigen::MatrixXd V;

private:
  Eigen::MatrixXd invE;

private:
  Eigen::MatrixXd K;

private:
  Eigen::VectorXd Trajectory;

private:
  Eigen::VectorXd Uopt;

  // Matrizes TIlt-rotor
private:
  Eigen::MatrixXd M;  // Inertia
private:
  Eigen::MatrixXd C;  // Coriolis
private:
  Eigen::VectorXd G;  // Gravitation
private:
  Eigen::MatrixXd Bp;  // Auxiliar InputcouplingMatrix
private:
  Eigen::MatrixXd BAero;  // InputcouplingMatrix
private:
  Eigen::MatrixXd B;  // InputcouplingMatrix

public:
  vant4_WinfTransition4()
    : q(8)
    , qp(8)
    , qpr(8)
    , qppr(8)
    , intqctil(3)
    , qc(3)
    , qcr(3)
    , qctil(3)
    , qptil(8)
    , x(17)
    , Input(8)
    , Input2(4)
    , V(17, 17)
    , K(17, 8)
    , M(8, 8)
    , C(8, 8)
    , G(8)
    , Bp(8, 4)
    , B(8, 8)
    , invE(8, 8)
    , Trajectory(22)
    , Uopt(8)
    , Erro(8)
    , qref(8)
    , Xref(8)
    , BAero(8, 4)
    , States(16)
  {
    T = 0.012;
    Iterations = 0;

    // Best 1
    //		V << 	0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    // 0,0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    // 0,0,0.11603,0,0,0,0,0,0.17321,0,0,0,0,0,0,0,0,
    // 0,0,0,0.14761,0,0,0,0,0,0.044721,0,0,0,0,0,0,0,
    // 0,0,0,0,0.45776,0,0,0,0,0,0.54772,0,0,0,0,0,0,
    // 0,0,0,0,0,4.9231,3.393e-43,6.1655e-60,0,0,0,4.5592,7.9819e-43,-6.4177e-59,1.0954,0,0,
    // 0,0,0,0,0,3.393e-43,5.7711,-1.0541e-16,0,0,0,7.9819e-43,5.161,-6.0644e-16,0,1.2247,0,
    // 0,0,0,0,0,6.1655e-60,-1.0541e-16,6.5937,0,0,0,-6.4177e-59,-6.0644e-16,5.8691,0,0,1.0954,
    // 0,0,0.17321,0,0,0,0,0,2.0098,0,0,0,0,0,0,0,0,
    // 0,0,0,0.044721,0,0,0,0,0,0.33007,0,0,0,0,0,0,0,
    // 0,0,0,0,0.54772,0,0,0,0,0,2.5073,0,0,0,0,0,0,
    // 0,0,0,0,0,4.5592,7.9819e-43,-6.4177e-59,0,0,0,10.127,2.616e-42,-2.3118e-58,2.6965,1.8584e-43,3.377e-60,
    // 0,0,0,0,0,7.9819e-43,5.161,-6.0644e-16,0,0,0,2.6652e-42,10.689,-2.217e-15,1.6622e-43,2.8272,-5.1642e-17,
    // 0,0,0,0,0,-6.4177e-59,-6.0644e-16,5.8691,0,0,0,-2.2219e-58,-1.7093e-15,18.254,3.377e-60,-5.7737e-17,3.6115,
    // 0,0,0,0,0,1.0954,0,0,0,0,0,2.6965,1.8584e-43,3.377e-60,2.4972,4.3719e-43,-3.5151e-59,
    // 0,0,0,0,0,0,1.2247,0,0,0,0,1.6622e-43,2.8272,-5.1642e-17,3.9103e-43,2.5284,-2.971e-16,
    // 0,0,0,0,0,0,0,1.0954,0,0,0,3.377e-60,-5.7737e-17,3.6115,-3.5151e-59,-3.3216e-16,3.2146;

    //
    //		invE << 100,0,0,0,0,0,0,-0,
    // 0,100,0,0,0,0,0,-0,
    // 0,0,100,0,0,0,0,-0,
    // 0,0,0,50,0,0,0,-0,
    // 0,0,0,0,10,0,0,-0,
    // 0,0,0,0,0,0.5,0,-0,
    // 0,0,0,0,0,0,0.4,-0,
    //-0,-0,-0,-0,-0,-0,-0,0.5;

    V << 0.094868, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.094868, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0.11603, 0, 0, 0, 0, 0, 0.17321, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.14761, 0, 0, 0, 0, 0, 0.044721,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.45776, 0, 0, 0, 0, 0, 0.54772, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4.9231,
        3.393e-43, 6.1655e-60, 0, 0, 0, 4.5592, 7.9819e-43, -6.4177e-59, 1.0954, 0, 0, 0, 0, 0, 0, 0, 3.393e-43, 5.7711,
        -1.0541e-16, 0, 0, 0, 7.9819e-43, 5.161, -6.0644e-16, 0, 1.2247, 0, 0, 0, 0, 0, 0, 6.1655e-60, -1.0541e-16,
        6.5937, 0, 0, 0, -6.4177e-59, -6.0644e-16, 5.8691, 0, 0, 1.0954, 0, 0, 0.17321, 0, 0, 0, 0, 0, 2.0098, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0.044721, 0, 0, 0, 0, 0, 0.33007, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.54772, 0, 0, 0, 0,
        0, 2.5073, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4.5592, 7.9819e-43, -6.4177e-59, 0, 0, 0, 10.127, 2.616e-42,
        -2.3118e-58, 2.6965, 1.8584e-43, 3.377e-60, 0, 0, 0, 0, 0, 7.9819e-43, 5.161, -6.0644e-16, 0, 0, 0, 2.6652e-42,
        10.689, -2.217e-15, 1.6622e-43, 2.8272, -5.1642e-17, 0, 0, 0, 0, 0, -6.4177e-59, -6.0644e-16, 5.8691, 0, 0, 0,
        -2.2219e-58, -1.7093e-15, 18.254, 3.377e-60, -5.7737e-17, 3.6115, 0, 0, 0, 0, 0, 1.0954, 0, 0, 0, 0, 0, 2.6965,
        1.8584e-43, 3.377e-60, 2.4972, 4.3719e-43, -3.5151e-59, 0, 0, 0, 0, 0, 0, 1.2247, 0, 0, 0, 0, 1.6622e-43,
        2.8272, -5.1642e-17, 3.9103e-43, 2.5284, -2.971e-16, 0, 0, 0, 0, 0, 0, 0, 1.0954, 0, 0, 0, 3.377e-60,
        -5.7737e-17, 3.6115, -3.5151e-59, -3.3216e-16, 3.2146;

    invE << 111.11, 0, 0, 0, 0, 0, 0, 0, 0, 111.11, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 50, 0, 0, 0, 0,
        0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0.4, 0, 0, 0, 0, 0, 0, 0, 0, 0.5;
  }

public:
  ~vant4_WinfTransition4()
  {
  }

public:
  void config()
  {
  }

private:
  Eigen::VectorXd TrajetoriaReferenciaCompleta(double Tempo)
  {
    Eigen::VectorXd Traj(22);

    double AccelFat = 2;
    const double MaxVel = 20.45;
    const double TMaxVel = MaxVel * (2.5 / AccelFat);
    double x = 0;
    double y = 0;
    double z = 0;
    double phi = 0;
    double theta = -0.03;
    double psi = 0;

    double xp = 0;
    double yp = 0;
    double zp = 0;
    double phip = 0;
    double thetap = 0;
    double psip = 0;

    double xpp = 0;
    double ypp = 0;
    double zpp = 0;
    double phipp = 0;
    double thetapp = 0;
    double psipp = 0;

    if (Tempo <= TMaxVel)
    {
      x = (AccelFat / 5.0) * (pow(Tempo, 2));
      y = 0.0;
      z = 2.0;
      phi = 0;
      theta = -0.03;
      psi = 0.0;

      xp = (AccelFat / 2.5) * Tempo;
      yp = 0;
      zp = 0;
      phip = 0;
      thetap = 0;
      psip = 0;

      xpp = AccelFat / 2.5;
      ypp = 0;
      zpp = 0;
      phipp = 0;
      thetapp = 0;
      psipp = 0;
    }
    else
    {
      x = (AccelFat / 2.5) * TMaxVel * (Tempo - TMaxVel) + (AccelFat / 5.0) * (pow(TMaxVel, 2));
      y = 0;
      z = 2;
      phi = 0;
      theta = -0.03;
      psi = 0;

      xp = (AccelFat / 2.5) * TMaxVel;
      yp = 0;
      zp = 0;
      phip = 0;
      thetap = 0;
      psip = 0;

      xpp = 0;
      ypp = 0;
      zpp = 0;
      phipp = 0;
      thetapp = 0;
      psipp = 0;
    }

    // Traj = [phi psi x y z Arp Alp thetap phip psip xp yp zp Arpp Alpp thetapp phipp psipp xpp ypp zpp]

    Traj << phi, theta, psi, x, y, z, 0, 0, phip, thetap, psip, xp, yp, zp, 0, 0, phipp, thetapp, psipp, xpp, ypp, zpp;

    //		double Magnitude = 0.1;
    //		Traj << 0, 0, 1, 1, 1,
    //                        0, 0, 0, 0, 0, 0, 0, 0,
    //                        0, 0, 0, 0, 0, 0, 0, 0;
    //
    //                Traj << 0, Magnitude*pow(Tempo,2), 0, 1,
    //                        0, 0, 0, 0, 0, 2*Magnitude*Tempo, 0, 0,
    //                        0, 0, 0, 0, 0, 2*Magnitude, 0, 0;
    return Traj;
  }

public:
  std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
  {
    // read data from sensors
    int i = 0;
    simulator_msgs::Sensor msg;
    while (true)
    {
      if (arraymsg.values.at(i).name == "Estados")
      {
        msg = arraymsg.values.at(i);
        break;
      }
      i++;
    }

    static double Tempo = 0;
    Trajectory = TrajetoriaReferenciaCompleta(Tempo);
    Tempo = Tempo + T;

    // integrators variables
    static double xint = 0, x_ant = 0;
    static double yint = 0, y_ant = 0;
    static double zint = 0, z_ant = 0;

    double x_error = msg.values.at(0) - Trajectory(3);
    xint = xint + (T / 2) * (x_error + x_ant);
    x_ant = x_error;

    double y_error = msg.values.at(1) - Trajectory(4);
    yint = yint + (T / 2) * (y_error + y_ant);
    y_ant = y_error;

    double z_error = msg.values.at(2) - Trajectory(5);
    zint = zint + (T / 2) * (z_error + z_ant);
    z_ant = z_error;

    // Variáveis de Configuração
    double AlphaR = msg.values.at(6);
    double AlphaL = msg.values.at(7);
    double Phi = msg.values.at(3);
    double Theta = msg.values.at(4);
    double Psi = msg.values.at(5);
    double X = msg.values.at(0);
    double Y = msg.values.at(1);
    double Z = msg.values.at(2);

    // Derivada Variáveis de Configuração
    double AlphaRp = msg.values.at(14);
    double AlphaLp = msg.values.at(15);
    double Phip = msg.values.at(11);
    double Thetap = msg.values.at(12);
    double Psip = msg.values.at(13);
    double Xp = msg.values.at(8);
    double Yp = msg.values.at(9);
    double Zp = msg.values.at(10);

    fprintf(stderr, "Arp: %f Alp: %f\n", AlphaRp, AlphaLp);

    // Deal's with Psi Discontinuity at 180 degree
    while (Psi - Trajectory(2) < -pi)
    {
      Psi = Psi + 2 * pi;
    }
    while (Psi - Trajectory(2) > pi)
    {
      Psi = Psi - 2 * pi;
    }

    q << AlphaR, AlphaL, Phi, Theta, Psi, X, Y, Z;
    qp << AlphaRp, AlphaLp, Phip, Thetap, Psip, Xp, Yp, Zp;
    // Compute Euler-Lagrange matrices
    //                Eigen::MatrixXd Mi(8,8);
    //                Eigen::MatrixXd Miaux(8,8);
    //                Eigen::MatrixXd Ci(8,8);
    //                Eigen::MatrixXd Ciaux(8,8);
    //                Eigen::MatrixXd Gi(8,1);
    //                Eigen::MatrixXd Bi(8,4);
    //                Eigen::MatrixXd BaeroFUai(8,5);
    Eigen::VectorXd FUa(8);
    Eigen::MatrixXd BaeroFUa(8, 5);
    double ub;

    M = InertiaMatrix(q);
    C = coriolisMatrix(q, qp);
    G = GravitationVector(q);
    Bp = InputCouplingMatrix(q);
    BaeroFUa = InputCouplingMatrixAero(q, qp, &ub);
    FUa = BaeroFUa.col(4);
    BAero << BaeroFUa.col(0), BaeroFUa.col(1), BaeroFUa.col(2), BaeroFUa.col(3);

    intqctil << xint, yint, zint;

    Eigen::VectorXd qs(2);
    Eigen::VectorXd qr(3);
    Eigen::VectorXd qrtil(3);
    Eigen::VectorXd qrr(3);
    qs << AlphaR, AlphaL;
    qr << Phi, Theta, Psi;
    qc << X, Y, Z;
    qrr << Trajectory(0), Trajectory(1), Trajectory(2);
    qcr << Trajectory(3), Trajectory(4), Trajectory(5);  //[x y z]
    qpr << Trajectory(6), Trajectory(7), Trajectory(8), Trajectory(9), Trajectory(10), Trajectory(11), Trajectory(12),
        Trajectory(13);  //[Arp Alp thetap phip psip xp yp zp]
    qppr << Trajectory(14), Trajectory(15), Trajectory(16), Trajectory(17), Trajectory(18), Trajectory(19),
        Trajectory(20), Trajectory(21);  //[Arpp Alpp thetapp phipp psipp xpp ypp zpp]
    qctil = qc - qcr;
    qrtil = qr - qrr;
    qptil = qp - qpr;
    fprintf(stderr, "Earp: %f Ealp: %f\n", qptil(0), qptil(1));

    // fprintf (stderr, "Eqc: %f  %f  %f Eqr: %f %f %f\n", qctil(0), qctil(1), qctil(2), qrtil(0), qrtil(1), qrtil(2));

    Eigen::VectorXd ForcesFriction(8);
    ForcesFriction << 0.005 * AlphaRp, 0.005 * AlphaLp, 0, 0, 0, 0, 0, 0;
    // fprintf (stderr, "FF0: %f FF1: %f\n", ForcesFriction(0), ForcesFriction(1));
    // State vector
    x << qptil, qrtil, qctil, intqctil;
    K << invE, Eigen::MatrixXd::Zero(9, 8);
    Uopt = -M * K.transpose() * V * x + G + M * qppr + C * qp - FUa + ForcesFriction;

    //-----------Pseudo-inversa 3--Transition------------

    // input coupling matrix
    B << Bp, BAero;

    Eigen::MatrixXd H(8, 8);
    Eigen::MatrixXd f(8, 1);
    Eigen::MatrixXd A(0, 8);
    Eigen::MatrixXd b(0, 1);
    Eigen::MatrixXd Aeq(0, 8);
    Eigen::MatrixXd beq(0, 1);
    Eigen::MatrixXd LB(8, 1);
    Eigen::MatrixXd UB(8, 1);

    H = B.transpose() * B;
    f = -B.transpose() * Uopt;

    double AngleDeflectionA = 0.3142;
    double AngleDeflectionR = 0.3142;
    double Transition = 17;
    UB << 50, 50, 1, 1, AngleDeflectionA * sigmf(ub, 1.5, Transition), AngleDeflectionA * sigmf(ub, 1.5, Transition),
        AngleDeflectionR * sigmf(ub, 1.5, Transition),
        AngleDeflectionR * sigmf(ub, 1.5, Transition);  // 0.52,  0.52,  0.52,  0.52;
    LB << 0, 0, -1, -1, -AngleDeflectionA * sigmf(ub, 1.5, Transition), -AngleDeflectionA * sigmf(ub, 1.5, Transition),
        -AngleDeflectionR * sigmf(ub, 1.5, Transition),
        -AngleDeflectionR * sigmf(ub, 1.5, Transition);  //-0.52, -0.52, -0.52, -0.52;

    double objvalue;
    Eigen::MatrixXd xstar(8, 1);
    int exitflag;

    cplexutils::qp(H, f, A, b, Aeq, beq, LB, UB, objvalue, xstar, exitflag);

    Input(0) = xstar(0);
    Input(1) = xstar(1);
    Input(2) = xstar(2);
    Input(3) = xstar(3);
    Input(4) = xstar(4);
    Input(5) = xstar(5);
    Input(6) = xstar(6);
    Input(7) = xstar(7);
    //		// output
    //		Input(0) = 0;//xstar(0);
    //		Input(1) = 0;//xstar(1);
    //		Input(2) = 0;//xstar(2);
    //		Input(3) = 0;//xstar(3);
    //		Input(4) = 1;//xstar(4);
    //		Input(5) = 2;//xstar(5);
    //		Input(6) = 3;//xstar(6);
    //		Input(7) = 4;//xstar(7);

    fprintf(stderr, "Inputs: %f  %f  %f  %f %f %f %f %f\n", Input(0), Input(1), Input(2), Input(3), Input(4), Input(5),
            Input(6), Input(7));
    fprintf(stderr, "Ar: %f Al: %f Theta:%f Phi: %f Ub: %f\n", AlphaR, AlphaL, Theta, Phi, ub);
    //
    States << q, qp;
    Erro = Eigen::VectorXd::Zero(8);
    Xref = Eigen::VectorXd::Zero(8);

    std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
    return out;
  }

private:
  double sigmf(double Val, double Rate, double center)
  {
    return (1 / (pow(2.718281828459046, (-Rate * (Val - center))) + 1));
  }

  // reference data
public:
  std::vector<double> Reference()
  {
    std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
    return out;
  }

  // error data
public:
  std::vector<double> Error()
  {
    std::vector<double> out(x.data(), x.data() + x.rows() * x.cols());
    return out;
  }

  // state data
public:
  std::vector<double> State()
  {
    std::vector<double> out(States.data(), States.data() + States.rows() * States.cols());
    return out;
  }

public:
  Eigen::VectorXd TrajetoriaReferenciaCompleta2(double Tempo)
  {
    Eigen::VectorXd Traj(22);  //[psi x y z Arp Alp phip thetap psip xp yp zp Arpp Alpp phipp thetapp psipp xpp ypp zpp]

    double T = 12;
    double f = 1 / T;
    double w = 2 * pi * f;
    double a = 6;
    double az = 0.3;
    double azdown = 0.7;

    if (Tempo <= 24)
    {
      double x = a * sin(w * Tempo);
      double y = a * cos(w * Tempo);
      double z = az * Tempo + 0.1;  //%a*sin(w*Tempo)+2;
      double phi = 0;
      double theta = -0.04;
      double psi = -2 * pi * Tempo / T;

      double xp = a * w * cos(Tempo * w);
      double yp = -a * w * sin(Tempo * w);
      double zp = az;
      double phip = 0;
      double thetap = 0;
      double psip = -2 * pi / T;

      double xpp = -a * pow(w, 2) * sin(Tempo * w);
      double ypp = -a * pow(w, 2) * cos(Tempo * w);
      double zpp = 0;
      double phipp = 0;
      double thetapp = 0;
      double psipp = 0;

      Traj << phi, theta, psi, x, y, z, 0, 0, 0, 0, psip, xp, yp, zp, 0, 0, 0, 0, psipp, xpp, ypp, zpp;
    }
    if (Tempo > 24 && Tempo <= 26)
    {
      Traj << 0, -0.04, 0, -(a * w * cos(24 * w) * (Tempo - 24) * (Tempo - 28)) / 4, a, az * 24 + 0.1, 0, 0, 0, 0, 0,
          a * w * cos(24 * w) - (Tempo - 24) * (a * w * cos(24 * w) / 2), 0, 0, 0, 0, 0, 0, 0,
          -(a * w * cos(24 * w)) / 2, 0, 0;
    }
    else if (Tempo > 26 && Tempo <= 28)
    {
      Traj << 0, -0.04, 0, -(a * w * cos(24 * w) * (26 - 24) * (26 - 28)) / 4, a, az * 24 + 0.1, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0;
    }
    if (Tempo > 28 && Tempo <= 37)
    {
      Traj << 0, -0.04, 0, -(a * w * cos(24 * w) * (26 - 24) * (26 - 28)) / 4, a,
          (az * 24 + 0.1) - azdown * (Tempo - 28), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    }
    if (Tempo > 37)
    {
      Traj << 0, -0.04, 0, -(a * w * cos(24 * w) * (26 - 24) * (26 - 28)) / 4, a, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0;
    }

    return Traj;
  }
};

extern "C" {
Icontroller* create(void)
{
  return new vant4_WinfTransition4;
}
void destroy(Icontroller* p)
{
  delete p;
}
}
