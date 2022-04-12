/*
 * File: teste.cpp
 * Author: Arthur Viana Lara
 * Project: ProVANT
 * Company: Federal University of Minas Gerais
 * Version: 1.0
 * Date: 29/01/18
 * Description: This file is responsable to implement LQR control law to VANT2.0
 */

#include <control_strategies_base/icontroller.hpp>
#include <Eigen/Eigen>
#include "simulator_msgs/Sensor.h"

class teste : public Icontroller
{
private:
  Eigen::VectorXd Xref;

private:
  Eigen::VectorXd Erro;

private:
  Eigen::VectorXd Input;

private:
  Eigen::MatrixXd K;

private:
  Eigen::VectorXd X;

private:
  double T;  // Sampled timr

  // constructor
public:
  teste() : Xref(20), K(4, 20), X(20), Erro(20), Input(4)
  {
    T = 0.012;
  }

  // destructor
public:
  ~teste()
  {
  }

  // initial configuration
public:
  void config()
  {
    // control matrix
    K << -0.000509474023994, 1.381002006541810, 2.044930990723325, -4.098388643419657, 0.002544968427177,
        0.065243786421189, -0.011997162152724, 0.012231188237446, -0.000191109567030, 0.977046060078436,
        2.067519443836474, -1.069820095142832, 0.003981280957280, 0.048837474399233, -0.000157117358773,
        0.000160465562387, -0.000324184653490, 0.932396154093819, 0.987708499654750, 0.029590812630684,

        0.000485844262540, -1.379777079391250, 2.046778453514912, 4.095526259295323, 0.006483505671600,
        -0.065184427882726, 0.011988462007403, -0.012218371367099, 0.000647666101473, -0.976219155907380,
        2.069372953111319, 1.069647999496566, 0.005073107886213, -0.048792319825525, 0.000156410506717,
        -0.000160891309965, 0.000193352263628, -0.931559941080183, 0.988594878555003, -0.029564069576714,

        0.147511637570957, -0.044700625197703, 0.000027359219549, 0.100038920576059, 0.186220549413359,
        0.044874312445416, 0.218427852703492, 0.036876304589717, 0.070402232740872, -0.028416542202667,
        0.000007473706141, 0.017792742949581, 0.025919243351085, 0.030976534992245, 0.007155474366137,
        0.000723596619702, 0.140517760855087, -0.032497295109449, 0.000009086030883, 0.021343249034188,

        0.147480373299421, 0.044731365984124, 0.000026937597506, -0.099883806663005, 0.186155931716280,
        -0.044878136021767, 0.036878957916608, 0.218443336829840, 0.070383903601564, 0.028402976825487,
        0.000007312089313, -0.017740572220967, 0.025908596148404, -0.030981560936545, 0.000723658016145,
        0.007155720063258, 0.140489293081162, 0.032562443693621, 0.000009316184096, -0.021344301330953;

    // reference
    Xref << 2, 0, 1.7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
  }

public:
  std::vector<double> execute(simulator_msgs::SensorArray arraymsg)
  {
    // integrators variables
    static double xint, x_ant = 0;
    static double yint, y_ant = 0;
    static double zint, z_ant = 0;
    static double yawint, yaw_ant = 0;

    // get data
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

    // Trapezoidal Integrator
    double x_atual = msg.values.at(0) - Xref(0);
    xint = xint + (T / 2) * (x_atual + x_ant);
    x_ant = x_atual;
    double y_atual = msg.values.at(1) - Xref(1);
    yint = yint + (T / 2) * (y_atual + y_ant);
    y_ant = y_atual;
    double z_atual = msg.values.at(2) - Xref(2);
    zint = zint + (T / 2) * (z_atual + z_ant);
    z_ant = z_atual;
    double yaw_atual = msg.values.at(5) - Xref(5);
    yawint = yawint + (T / 2) * (yaw_atual + yaw_ant);
    yaw_ant = yaw_atual;

    // state vector
    X << msg.values.at(0),  // x
        msg.values.at(1),   // y
        msg.values.at(2),   // z
        msg.values.at(3),   // roll
        msg.values.at(4),   // pitch
        msg.values.at(5),   // yaw
        msg.values.at(6),   // aR
        msg.values.at(7),   // aL
        msg.values.at(8),   // dx
        msg.values.at(9),   // dy
        msg.values.at(10),  // dz
        msg.values.at(11),  // droll
        msg.values.at(12),  // dpitch
        msg.values.at(13),  // dyaw
        msg.values.at(14),  // daR
        msg.values.at(15),  // daL
        xint,               // x integrator
        yint,               // y integrator
        zint,               // z integrator
        yawint;             // yaw integrator

    // control law
    Erro = X - Xref;
    Input = -K * Erro;

    // Feedforward
    Input(0) = Input(0) + 10.2751;
    Input(1) = Input(1) + 10.2799;

    // output
    std::vector<double> out(Input.data(), Input.data() + Input.rows() * Input.cols());
    return out;
  }

  // reference data
public:
  std::vector<double> Reference()
  {
    std::vector<double> out(Xref.data(), Xref.data() + Xref.rows() * Xref.cols());
    return out;
  }

  // error data
public:
  std::vector<double> Error()
  {
    std::vector<double> out(Erro.data(), Erro.data() + Erro.rows() * Erro.cols());
    return out;
  }

  // state data
public:
  std::vector<double> State()
  {
    std::vector<double> out(X.data(), X.data() + X.rows() * X.cols());
    return out;
  }
};

// to implement plugin
extern "C" {
Icontroller* create(void)
{
  return new teste;
}
void destroy(Icontroller* p)
{
  delete p;
}
}
