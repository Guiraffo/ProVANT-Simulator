/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the implementation of the function that returns the
 * Coriollis foces matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#include "vant4_winf_transition/coriolis_matrix.h"

#include "vant4_winf_transition/uav4_parameters.h"

#include <cmath>

Eigen::MatrixXd coriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd qp)
{
  // Variáveis de Configuração
  double AlphaR = q(0);
  double AlphaL = q(1);
  double Phi = q(2);
  double Theta = q(3);
  double Psi = q(4);
  double X = q(5);
  double Y = q(6);
  double Z = q(7);

  // Derivada Variáveis de Configuração
  double AlphaRp = qp(0);
  double AlphaLp = qp(1);
  double Phip = qp(2);
  double Thetap = qp(3);
  double Psip = qp(4);
  double Xp = qp(5);
  double Yp = qp(6);
  double Zp = qp(7);

  Eigen::MatrixXd C(8, 8);

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

  C << 0, 0,
      0.5 * Ixx2 * Thetap * sinPhi - 0.5 * Iyy2 * Thetap * sinPhi - 0.5 * Izz2 * Thetap * sinPhi +
          Ixx2 * Phip * cosAlphaR * sinAlphaR - 1.0 * Iyy2 * Phip * cosAlphaR * sinAlphaR -
          0.5 * Ixx2 * Psip * cosPhi * cosTheta + 0.5 * Iyy2 * Psip * cosPhi * cosTheta +
          0.5 * Izz2 * Psip * cosPhi * cosTheta - 1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * sinPhi +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * sinPhi - 1.0 * Ixx2 * Psip * cosAlphaR * sinAlphaR * sinTheta +
          Iyy2 * Psip * cosAlphaR * sinAlphaR * sinTheta + Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi + M2 * Phip * ZB2 * ds * cosB * sinAlphaR +
          M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB - 1.0 * Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Iyy2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * Izz2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB +
          M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi -
          1.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * sinPhi -
          1.0 * M2 * Psip * ZB2 * ds * cosB * sinAlphaR * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi +
          M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta +
          M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi,
      0.5 * Ixx2 * Phip * sinPhi - 0.5 * Iyy2 * Phip * sinPhi - 0.5 * Izz2 * Phip * sinPhi -
          1.0 * Ixx2 * Thetap * cosAlphaR * sinAlphaR + Iyy2 * Thetap * cosAlphaR * sinAlphaR -
          0.5 * Ixx2 * Psip * sinPhi * sinTheta + 0.5 * Iyy2 * Psip * sinPhi * sinTheta -
          0.5 * Izz2 * Psip * sinPhi * sinTheta - 1.0 * Ixx2 * Phip * pow(cosAlphaR, 2) * sinPhi +
          Iyy2 * Phip * pow(cosAlphaR, 2) * sinPhi - 1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Ixx2 * Thetap * cosAlphaR * pow(cosPhi, 2) * sinAlphaR -
          1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosPhi, 2) * sinAlphaR +
          Ixx2 * Psip * pow(cosAlphaR, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * sinPhi * sinTheta + Izz2 * Psip * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi - 1.0 * M2 * Thetap * XB2 * ds * cosAlphaR * cosB -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR -
          1.0 * Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Phip * cosAlphaR * cosB * cosPhi * sinB - 1.0 * Izz2 * Phip * cosAlphaR * cosB * cosPhi * sinB +
          Iyy2 * Psip * cosB * sinAlphaR * sinB * cosTheta - 1.0 * Izz2 * Psip * cosB * sinAlphaR * sinB * cosTheta +
          M2 * Thetap * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB +
          Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          Iyy2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          1.0 * Izz2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * sinPhi +
          M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * cosTheta +
          M2 * Phip * YB2 * ds * cosAlphaR * cosB * cosPhi - 1.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * sinPhi +
          M2 * Psip * YB2 * ds * cosB * sinAlphaR * cosTheta -
          1.0 * Iyy2 * Psip * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          Izz2 * Psip * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          M2 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * sinPhi +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          M2 * Thetap * YB2 * ds * cosB * cosPhi * sinAlphaR * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Psip * XB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta -
          1.0 * M2 * Psip * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta +
          M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi,
      Ixx2 * Psip * cosAlphaR * sinAlphaR - 1.0 * Iyy2 * Psip * cosAlphaR * sinAlphaR -
          0.5 * Ixx2 * Phip * cosPhi * cosTheta + 0.5 * Iyy2 * Phip * cosPhi * cosTheta +
          0.5 * Izz2 * Phip * cosPhi * cosTheta - 0.5 * Ixx2 * Thetap * sinPhi * sinTheta +
          0.5 * Iyy2 * Thetap * sinPhi * sinTheta - 0.5 * Izz2 * Thetap * sinPhi * sinTheta -
          1.0 * Ixx2 * Phip * cosAlphaR * sinAlphaR * sinTheta + Iyy2 * Phip * cosAlphaR * sinAlphaR * sinTheta +
          Ixx2 * Psip * cosPhi * cosTheta * sinTheta - 1.0 * Iyy2 * Psip * cosPhi * cosTheta * sinTheta +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Ixx2 * Phip * pow(cosAlphaR, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * cosPhi * cosTheta -
          1.0 * Ixx2 * Psip * cosAlphaR * sinAlphaR * pow(cosTheta, 2) +
          Iyy2 * Psip * cosAlphaR * sinAlphaR * pow(cosTheta, 2) +
          Ixx2 * Thetap * pow(cosAlphaR, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosB, 2) * sinPhi * sinTheta + Izz2 * Thetap * pow(cosB, 2) * sinPhi * sinTheta +
          M2 * Psip * ZB2 * ds * cosB * sinAlphaR + M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          2.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta +
          Iyy2 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Ixx2 * Psip * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          Iyy2 * Psip * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Thetap * cosB * sinAlphaR * sinB * cosTheta -
          1.0 * Izz2 * Thetap * cosB * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) +
          M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          2.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Phip * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * Izz2 * Phip * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          Ixx2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          M2 * Thetap * pow(ds, 2) * cosB * sinAlphaR * sinB * cosTheta +
          M2 * Thetap * YB2 * ds * cosB * sinAlphaR * cosTheta -
          1.0 * Iyy2 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          Izz2 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Phip * ZB2 * ds * cosB * sinAlphaR * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * Iyy2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta +
          Izz2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          M2 * Phip * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta +
          M2 * Phip * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi +
          Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy2 * Psip * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          Izz2 * Psip * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Thetap * XB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta -
          1.0 * M2 * Thetap * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta -
          2.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinTheta +
          M2 * Psip * XB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta -
          1.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta +
          M2 * Thetap * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * YB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) * sinPhi,
      0, 0, 0, 0, 0,
      0.5 * Ixx3 * Thetap * sinPhi - 0.5 * Iyy3 * Thetap * sinPhi - 0.5 * Izz3 * Thetap * sinPhi +
          Ixx3 * Phip * cosAlphaL * sinAlphaL - 1.0 * Iyy3 * Phip * cosAlphaL * sinAlphaL -
          0.5 * Ixx3 * Psip * cosPhi * cosTheta + 0.5 * Iyy3 * Psip * cosPhi * cosTheta +
          0.5 * Izz3 * Psip * cosPhi * cosTheta - 1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * sinPhi +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * sinPhi - 1.0 * Ixx3 * Psip * cosAlphaL * sinAlphaL * sinTheta +
          Iyy3 * Psip * cosAlphaL * sinAlphaL * sinTheta + Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi + M3 * Phip * ZB3 * ds * cosB * sinAlphaL +
          M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB + Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          Izz3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB +
          M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi -
          1.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * sinPhi -
          1.0 * M3 * Psip * ZB3 * ds * cosB * sinAlphaL * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta +
          M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi,
      0.5 * Ixx3 * Phip * sinPhi - 0.5 * Iyy3 * Phip * sinPhi - 0.5 * Izz3 * Phip * sinPhi -
          1.0 * Ixx3 * Thetap * cosAlphaL * sinAlphaL + Iyy3 * Thetap * cosAlphaL * sinAlphaL -
          0.5 * Ixx3 * Psip * sinPhi * sinTheta + 0.5 * Iyy3 * Psip * sinPhi * sinTheta -
          0.5 * Izz3 * Psip * sinPhi * sinTheta - 1.0 * Ixx3 * Phip * pow(cosAlphaL, 2) * sinPhi +
          Iyy3 * Phip * pow(cosAlphaL, 2) * sinPhi - 1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Ixx3 * Thetap * cosAlphaL * pow(cosPhi, 2) * sinAlphaL -
          1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosPhi, 2) * sinAlphaL +
          Ixx3 * Psip * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosB, 2) * sinPhi * sinTheta + Izz3 * Psip * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi - 1.0 * M3 * Thetap * XB3 * ds * cosAlphaL * cosB -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL -
          1.0 * Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Phip * cosAlphaL * cosB * cosPhi * sinB + Izz3 * Phip * cosAlphaL * cosB * cosPhi * sinB -
          1.0 * Iyy3 * Psip * cosB * sinAlphaL * sinB * cosTheta + Izz3 * Psip * cosB * sinAlphaL * sinB * cosTheta +
          M3 * Thetap * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB +
          Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * Iyy3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * sinPhi +
          Izz3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * cosTheta +
          M3 * Phip * YB3 * ds * cosAlphaL * cosB * cosPhi - 1.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * sinPhi +
          M3 * Psip * YB3 * ds * cosB * sinAlphaL * cosTheta +
          Iyy3 * Psip * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * Izz3 * Psip * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * sinPhi +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          M3 * Thetap * YB3 * ds * cosB * cosPhi * sinAlphaL * sinPhi +
          M3 * Psip * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Psip * XB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta -
          1.0 * M3 * Psip * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta +
          M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi,
      Ixx3 * Psip * cosAlphaL * sinAlphaL - 1.0 * Iyy3 * Psip * cosAlphaL * sinAlphaL -
          0.5 * Ixx3 * Phip * cosPhi * cosTheta + 0.5 * Iyy3 * Phip * cosPhi * cosTheta +
          0.5 * Izz3 * Phip * cosPhi * cosTheta - 0.5 * Ixx3 * Thetap * sinPhi * sinTheta +
          0.5 * Iyy3 * Thetap * sinPhi * sinTheta - 0.5 * Izz3 * Thetap * sinPhi * sinTheta -
          1.0 * Ixx3 * Phip * cosAlphaL * sinAlphaL * sinTheta + Iyy3 * Phip * cosAlphaL * sinAlphaL * sinTheta +
          Ixx3 * Psip * cosPhi * cosTheta * sinTheta - 1.0 * Iyy3 * Psip * cosPhi * cosTheta * sinTheta +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Ixx3 * Phip * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * Ixx3 * Psip * cosAlphaL * sinAlphaL * pow(cosTheta, 2) +
          Iyy3 * Psip * cosAlphaL * sinAlphaL * pow(cosTheta, 2) +
          Ixx3 * Thetap * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * pow(cosB, 2) * sinPhi * sinTheta + Izz3 * Thetap * pow(cosB, 2) * sinPhi * sinTheta +
          M3 * Psip * ZB3 * ds * cosB * sinAlphaL + M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          2.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta +
          Iyy3 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Ixx3 * Psip * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          Iyy3 * Psip * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * cosB * sinAlphaL * sinB * cosTheta +
          Izz3 * Thetap * cosB * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) +
          M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          2.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Phip * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          Izz3 * Phip * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          Ixx3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * Iyy3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosB * sinAlphaL * sinB * cosTheta +
          M3 * Thetap * YB3 * ds * cosB * sinAlphaL * cosTheta +
          Iyy3 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * Izz3 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Phip * ZB3 * ds * cosB * sinAlphaL * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          Iyy3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * Izz3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Psip * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          M3 * Phip * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta +
          M3 * Phip * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi +
          Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Iyy3 * Psip * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Psip * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          M3 * Thetap * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Thetap * XB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta -
          1.0 * M3 * Thetap * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta -
          2.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinTheta +
          M3 * Psip * XB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta -
          1.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta +
          M3 * Thetap * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          M3 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * YB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) * sinPhi,
      0, 0, 0,
      0.5 * Iyy2 * Thetap * sinPhi - 0.5 * Ixx2 * Thetap * sinPhi + 0.5 * Izz2 * Thetap * sinPhi -
          1.0 * Ixx2 * Phip * cosAlphaR * sinAlphaR + Iyy2 * Phip * cosAlphaR * sinAlphaR +
          0.5 * Ixx2 * Psip * cosPhi * cosTheta - 0.5 * Iyy2 * Psip * cosPhi * cosTheta -
          0.5 * Izz2 * Psip * cosPhi * cosTheta + Ixx2 * Thetap * pow(cosAlphaR, 2) * sinPhi -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * sinPhi + Ixx2 * Psip * cosAlphaR * sinAlphaR * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * sinAlphaR * sinTheta -
          1.0 * Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * cosB * sinB + AlphaRp * Izz2 * cosAlphaR * cosB * sinB -
          1.0 * M2 * Phip * ZB2 * ds * cosB * sinAlphaR -
          1.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB + Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * sinB - 1.0 * AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi +
          Izz2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB -
          1.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi +
          M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * sinPhi + M2 * Psip * ZB2 * ds * cosB * sinAlphaR * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta -
          1.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi,
      0.5 * Iyy3 * Thetap * sinPhi - 0.5 * Ixx3 * Thetap * sinPhi + 0.5 * Izz3 * Thetap * sinPhi -
          1.0 * Ixx3 * Phip * cosAlphaL * sinAlphaL + Iyy3 * Phip * cosAlphaL * sinAlphaL +
          0.5 * Ixx3 * Psip * cosPhi * cosTheta - 0.5 * Iyy3 * Psip * cosPhi * cosTheta -
          0.5 * Izz3 * Psip * cosPhi * cosTheta + Ixx3 * Thetap * pow(cosAlphaL, 2) * sinPhi -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * sinPhi + Ixx3 * Psip * cosAlphaL * sinAlphaL * sinTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * sinAlphaL * sinTheta -
          1.0 * Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi + AlphaLp * Iyy3 * cosAlphaL * cosB * sinB -
          1.0 * AlphaLp * Izz3 * cosAlphaL * cosB * sinB - 1.0 * M3 * Phip * ZB3 * ds * cosB * sinAlphaL -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB - 1.0 * Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * sinB - 1.0 * AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Iyy3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * Izz3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB -
          1.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi +
          M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * sinPhi + M3 * Psip * ZB3 * ds * cosB * sinAlphaL * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta -
          1.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi,
      -AlphaLp *
              (1.0 * Ixx3 * cosAlphaL * sinAlphaL - Iyy3 * cosAlphaL * sinAlphaL +
               1.0 * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL - Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL +
               1.0 * M3 * ZB3 * ds * cosB * sinAlphaL + 1.0 * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL) -
          AlphaRp *
              (1.0 * Ixx2 * cosAlphaR * sinAlphaR - Iyy2 * cosAlphaR * sinAlphaR +
               1.0 * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR - Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR +
               1.0 * M2 * ZB2 * ds * cosB * sinAlphaR + 1.0 * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR),
      0.5 * AlphaLp * Iyy3 * sinPhi - 0.5 * AlphaRp * Ixx2 * sinPhi - 0.5 * AlphaLp * Ixx3 * sinPhi +
          0.5 * AlphaRp * Iyy2 * sinPhi + 0.5 * AlphaLp * Izz3 * sinPhi + 0.5 * AlphaRp * Izz2 * sinPhi -
          0.5 * Ixx1 * Psip * cosTheta - 0.5 * Ixx2 * Psip * cosTheta - 0.5 * Ixx3 * Psip * cosTheta +
          0.5 * Iyy1 * Psip * cosTheta - 0.5 * Iyy2 * Psip * cosTheta - 0.5 * Iyy3 * Psip * cosTheta -
          0.5 * Izz1 * Psip * cosTheta + 0.5 * Izz2 * Psip * cosTheta + 0.5 * Izz3 * Psip * cosTheta -
          1.0 * M1 * Psip * pow(YB1, 2) * cosTheta - 1.0 * M2 * Psip * pow(YB2, 2) * cosTheta -
          1.0 * M3 * Psip * pow(YB3, 2) * cosTheta - 1.0 * Ixx2 * Thetap * cosPhi * sinPhi -
          1.0 * Ixx3 * Thetap * cosPhi * sinPhi + Iyy1 * Thetap * cosPhi * sinPhi -
          1.0 * Izz1 * Thetap * cosPhi * sinPhi + Izz2 * Thetap * cosPhi * sinPhi + Izz3 * Thetap * cosPhi * sinPhi -
          1.0 * Ixz1 * Psip * cosPhi * sinTheta - 1.0 * M2 * Psip * pow(ds, 2) * cosTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosTheta + AlphaLp * Ixx3 * pow(cosAlphaL, 2) * sinPhi +
          AlphaRp * Ixx2 * pow(cosAlphaR, 2) * sinPhi - 1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * sinPhi -
          1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * sinPhi + M1 * Thetap * YB1 * ZB1 + M2 * Thetap * YB2 * ZB2 +
          M3 * Thetap * YB3 * ZB3 + Iyy2 * Psip * pow(cosB, 2) * cosTheta + Iyy3 * Psip * pow(cosB, 2) * cosTheta -
          1.0 * Izz2 * Psip * pow(cosB, 2) * cosTheta - 1.0 * Izz3 * Psip * pow(cosB, 2) * cosTheta +
          Ixx2 * Psip * pow(cosPhi, 2) * cosTheta + Ixx3 * Psip * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy1 * Psip * pow(cosPhi, 2) * cosTheta + Izz1 * Psip * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Psip * pow(cosPhi, 2) * cosTheta - 1.0 * Izz3 * Psip * pow(cosPhi, 2) * cosTheta -
          1.0 * M1 * Thetap * pow(YB1, 2) * cosPhi * sinPhi - 1.0 * M2 * Thetap * pow(YB2, 2) * cosPhi * sinPhi -
          1.0 * M3 * Thetap * pow(YB3, 2) * cosPhi * sinPhi + M1 * Thetap * pow(ZB1, 2) * cosPhi * sinPhi +
          M2 * Thetap * pow(ZB2, 2) * cosPhi * sinPhi + M3 * Thetap * pow(ZB3, 2) * cosPhi * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosPhi * sinPhi - 1.0 * M3 * Thetap * pow(ds, 2) * cosPhi * sinPhi +
          M2 * Thetap * ZB2 * ds * sinB - 1.0 * M3 * Thetap * ZB3 * ds * sinB +
          Ixx3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi + Ixx2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi + Iyy2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi +
          Iyy3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi - 1.0 * Izz2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Izz3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi + M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta + M1 * Psip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Psip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta + M3 * Psip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M1 * Psip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Psip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Psip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta + M3 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          2.0 * M1 * Thetap * YB1 * ZB1 * pow(cosPhi, 2) - 2.0 * M2 * Thetap * YB2 * ZB2 * pow(cosPhi, 2) -
          2.0 * M3 * Thetap * YB3 * ZB3 * pow(cosPhi, 2) -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta - 1.0 * Iyy3 * Thetap * cosAlphaL * cosB * sinB +
          Iyy2 * Thetap * cosAlphaR * cosB * sinB + Izz3 * Thetap * cosAlphaL * cosB * sinB -
          1.0 * Izz2 * Thetap * cosAlphaR * cosB * sinB + M3 * Thetap * YB3 * ds * cosAlphaL * cosB +
          M2 * Thetap * YB2 * ds * cosAlphaR * cosB + 2.0 * Iyy3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB -
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB -
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB +
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB -
          2.0 * M2 * Psip * YB2 * ds * sinB * cosTheta + 2.0 * M3 * Psip * YB3 * ds * sinB * cosTheta +
          M1 * Psip * XB1 * ZB1 * cosPhi * sinTheta + M2 * Psip * XB2 * ZB2 * cosPhi * sinTheta +
          M3 * Psip * XB3 * ZB3 * cosPhi * sinTheta + M1 * Psip * XB1 * YB1 * sinPhi * sinTheta +
          M2 * Psip * XB2 * YB2 * sinPhi * sinTheta + M3 * Psip * XB3 * YB3 * sinPhi * sinTheta +
          M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          2.0 * M2 * Thetap * ZB2 * ds * pow(cosPhi, 2) * sinB + 2.0 * M3 * Thetap * ZB3 * ds * pow(cosPhi, 2) * sinB -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          AlphaLp * Iyy3 * cosAlphaL * cosB * cosPhi * sinB - 1.0 * AlphaRp * Iyy2 * cosAlphaR * cosB * cosPhi * sinB -
          1.0 * AlphaLp * Izz3 * cosAlphaL * cosB * cosPhi * sinB + AlphaRp * Izz2 * cosAlphaR * cosB * cosPhi * sinB -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * sinB +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * sinB -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * sinTheta +
          Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * sinTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * sinTheta -
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) -
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) +
          2.0 * M2 * Psip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          2.0 * M3 * Psip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB -
          1.0 * Iyy3 * Psip * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          Iyy2 * Psip * cosB * sinAlphaR * sinB * sinPhi * sinTheta +
          Izz3 * Psip * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * Izz2 * Psip * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB * cosPhi -
          1.0 * AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB * cosPhi +
          AlphaLp * M3 * ZB3 * ds * cosAlphaL * cosB * sinPhi + AlphaRp * M2 * ZB2 * ds * cosAlphaR * cosB * sinPhi +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB -
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB -
          2.0 * M1 * Psip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi -
          2.0 * M2 * Psip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi -
          2.0 * M3 * Psip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi -
          2.0 * M2 * Thetap * YB2 * ds * cosPhi * sinB * sinPhi +
          2.0 * M3 * Thetap * YB3 * ds * cosPhi * sinB * sinPhi + M2 * Psip * XB2 * ds * sinB * sinPhi * sinTheta -
          1.0 * M3 * Psip * XB3 * ds * sinB * sinPhi * sinTheta +
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi * sinTheta +
          M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta +
          M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta +
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * cosPhi * sinPhi +
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * cosPhi * sinPhi +
          M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * sinTheta +
          M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          M3 * Psip * YB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta +
          M2 * Psip * YB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta -
          2.0 * M2 * Psip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M3 * Psip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta -
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta -
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi -
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi +
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi,
      0.5 * Iyy1 * Thetap * cosTheta - 0.5 * Ixx2 * Thetap * cosTheta - 0.5 * Ixx3 * Thetap * cosTheta -
          0.5 * Ixx1 * Thetap * cosTheta - 0.5 * Iyy2 * Thetap * cosTheta - 0.5 * Iyy3 * Thetap * cosTheta -
          0.5 * Izz1 * Thetap * cosTheta + 0.5 * Izz2 * Thetap * cosTheta + 0.5 * Izz3 * Thetap * cosTheta +
          0.5 * AlphaLp * Ixx3 * cosPhi * cosTheta + 0.5 * AlphaRp * Ixx2 * cosPhi * cosTheta -
          0.5 * AlphaLp * Iyy3 * cosPhi * cosTheta - 0.5 * AlphaRp * Iyy2 * cosPhi * cosTheta -
          0.5 * AlphaLp * Izz3 * cosPhi * cosTheta - 0.5 * AlphaRp * Izz2 * cosPhi * cosTheta -
          1.0 * M1 * Thetap * pow(YB1, 2) * cosTheta - 1.0 * M2 * Thetap * pow(YB2, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(YB3, 2) * cosTheta - 1.0 * Ixz1 * Thetap * cosPhi * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * cosTheta - 1.0 * M3 * Thetap * pow(ds, 2) * cosTheta +
          Iyy2 * Thetap * pow(cosB, 2) * cosTheta + Iyy3 * Thetap * pow(cosB, 2) * cosTheta -
          1.0 * Izz2 * Thetap * pow(cosB, 2) * cosTheta - 1.0 * Izz3 * Thetap * pow(cosB, 2) * cosTheta +
          Ixx2 * Thetap * pow(cosPhi, 2) * cosTheta + Ixx3 * Thetap * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy1 * Thetap * pow(cosPhi, 2) * cosTheta + Izz1 * Thetap * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Thetap * pow(cosPhi, 2) * cosTheta - 1.0 * Izz3 * Thetap * pow(cosPhi, 2) * cosTheta +
          AlphaLp * Ixx3 * cosAlphaL * sinAlphaL * sinTheta + AlphaRp * Ixx2 * cosAlphaR * sinAlphaR * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * sinAlphaL * sinTheta -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * sinAlphaR * sinTheta - 1.0 * Ixz1 * Psip * cosTheta * sinPhi * sinTheta -
          1.0 * AlphaLp * Ixx3 * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * Ixx2 * pow(cosAlphaR, 2) * cosPhi * cosTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * cosPhi * cosTheta +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * cosPhi * cosTheta + M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosTheta + M1 * Thetap * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Thetap * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Thetap * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M1 * Thetap * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Thetap * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi + Ixx3 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy1 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi + Izz1 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta + M3 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M1 * Psip * YB1 * ZB1 * pow(cosTheta, 2) - 1.0 * M2 * Psip * YB2 * ZB2 * pow(cosTheta, 2) -
          1.0 * M3 * Psip * YB3 * ZB3 * pow(cosTheta, 2) -
          1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Psip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) -
          1.0 * Iyy2 * Psip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) -
          1.0 * Izz3 * Psip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) +
          Izz2 * Psip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          2.0 * M2 * Thetap * YB2 * ds * sinB * cosTheta + 2.0 * M3 * Thetap * YB3 * ds * sinB * cosTheta +
          M1 * Thetap * XB1 * ZB1 * cosPhi * sinTheta + M2 * Thetap * XB2 * ZB2 * cosPhi * sinTheta +
          M3 * Thetap * XB3 * ZB3 * cosPhi * sinTheta + M1 * Thetap * XB1 * YB1 * sinPhi * sinTheta +
          M2 * Thetap * XB2 * YB2 * sinPhi * sinTheta + M3 * Thetap * XB3 * YB3 * sinPhi * sinTheta +
          M1 * Psip * pow(YB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Psip * pow(YB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * pow(YB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M1 * Psip * pow(ZB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * pow(ZB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * pow(ZB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Psip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * M2 * Psip * ZB2 * ds * sinB * pow(cosTheta, 2) + M3 * Psip * ZB3 * ds * sinB * pow(cosTheta, 2) -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy3 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz2 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz3 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * sinTheta +
          Ixx2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * sinTheta -
          1.0 * Iyy3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * sinTheta -
          1.0 * Iyy2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * sinTheta +
          2.0 * M1 * Psip * YB1 * ZB1 * pow(cosPhi, 2) * pow(cosTheta, 2) +
          2.0 * M2 * Psip * YB2 * ZB2 * pow(cosPhi, 2) * pow(cosTheta, 2) +
          2.0 * M3 * Psip * YB3 * ZB3 * pow(cosPhi, 2) * pow(cosTheta, 2) -
          1.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) -
          1.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) -
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * Izz3 * Psip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * Izz2 * Psip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          2.0 * M2 * Thetap * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          2.0 * M3 * Thetap * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          AlphaLp * Iyy3 * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * AlphaLp * Izz3 * cosAlphaL * cosB * sinB * cosTheta * sinPhi +
          AlphaRp * Izz2 * cosAlphaR * cosB * sinB * cosTheta * sinPhi -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Ixx3 * Psip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Ixx2 * Psip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          Iyy2 * Thetap * cosB * sinAlphaR * sinB * sinPhi * sinTheta +
          Izz3 * Thetap * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * Izz2 * Thetap * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          2.0 * M2 * Psip * ZB2 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * M3 * Psip * ZB3 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          AlphaLp * M3 * ZB3 * ds * cosB * sinAlphaL * sinTheta +
          AlphaRp * M2 * ZB2 * ds * cosB * sinAlphaR * sinTheta +
          Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          1.0 * Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * pow(cosTheta, 2) -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * M1 * Psip * XB1 * YB1 * cosPhi * cosTheta * sinTheta -
          1.0 * M2 * Psip * XB2 * YB2 * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Psip * XB3 * YB3 * cosPhi * cosTheta * sinTheta -
          2.0 * M1 * Thetap * YB1 * ZB1 * cosPhi * cosTheta * sinPhi -
          2.0 * M2 * Thetap * YB2 * ZB2 * cosPhi * cosTheta * sinPhi -
          2.0 * M3 * Thetap * YB3 * ZB3 * cosPhi * cosTheta * sinPhi +
          M1 * Psip * XB1 * ZB1 * cosTheta * sinPhi * sinTheta + M2 * Psip * XB2 * ZB2 * cosTheta * sinPhi * sinTheta +
          M3 * Psip * XB3 * ZB3 * cosTheta * sinPhi * sinTheta + M2 * Thetap * XB2 * ds * sinB * sinPhi * sinTheta -
          1.0 * M3 * Thetap * XB3 * ds * sinB * sinPhi * sinTheta +
          2.0 * Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi +
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) +
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) +
          Iyy3 * Psip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta +
          Izz2 * Psip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          M2 * Thetap * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta -
          1.0 * AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi +
          M3 * Thetap * XB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta +
          M2 * Thetap * XB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta +
          M3 * Thetap * ZB3 * ds * cosB * cosPhi * sinAlphaL * sinTheta +
          M2 * Thetap * ZB2 * ds * cosB * cosPhi * sinAlphaR * sinTheta +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          1.0 * M2 * Psip * XB2 * ds * cosPhi * sinB * cosTheta * sinTheta +
          M3 * Psip * XB3 * ds * cosPhi * sinB * cosTheta * sinTheta +
          M3 * Thetap * YB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta +
          M2 * Thetap * YB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta -
          2.0 * M2 * Thetap * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M3 * Thetap * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta -
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta -
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * M2 * Psip * YB2 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Psip * YB3 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta -
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi -
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi +
          M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta +
          M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Psip * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta -
          1.0 * M2 * Psip * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta +
          M3 * Psip * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi * sinTheta +
          M2 * Psip * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi,
      0, 0, 0,
      0.5 * Iyy2 * Phip * sinPhi - 0.5 * Ixx2 * Phip * sinPhi - 0.5 * Izz2 * Phip * sinPhi +
          Ixx2 * Thetap * cosAlphaR * sinAlphaR - 1.0 * Iyy2 * Thetap * cosAlphaR * sinAlphaR +
          0.5 * Ixx2 * Psip * sinPhi * sinTheta - 0.5 * Iyy2 * Psip * sinPhi * sinTheta +
          0.5 * Izz2 * Psip * sinPhi * sinTheta + Ixx2 * Phip * pow(cosAlphaR, 2) * sinPhi -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * sinPhi - 1.0 * Iyy2 * Phip * pow(cosB, 2) * sinPhi +
          Izz2 * Phip * pow(cosB, 2) * sinPhi + Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Ixx2 * Thetap * cosAlphaR * pow(cosPhi, 2) * sinAlphaR +
          Iyy2 * Thetap * cosAlphaR * pow(cosPhi, 2) * sinAlphaR -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * sinPhi * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * sinPhi * sinTheta - 1.0 * M2 * Phip * pow(ds, 2) * pow(cosB, 2) * sinPhi +
          Iyy2 * Psip * pow(cosB, 2) * sinPhi * sinTheta - 1.0 * Izz2 * Psip * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi + M2 * Thetap * XB2 * ds * cosAlphaR * cosB +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          M2 * Psip * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR +
          Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR +
          M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Iyy2 * cosB * sinAlphaR * sinB * sinPhi + AlphaRp * Izz2 * cosB * sinAlphaR * sinB * sinPhi -
          1.0 * Iyy2 * Psip * cosB * sinAlphaR * sinB * cosTheta + Izz2 * Psip * cosB * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Thetap * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * sinPhi +
          Izz2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * cosTheta +
          AlphaRp * M2 * XB2 * ds * cosAlphaR * cosB * cosPhi -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * cosPhi * sinAlphaR -
          1.0 * AlphaRp * M2 * YB2 * ds * cosB * sinAlphaR * sinPhi -
          1.0 * M2 * Psip * YB2 * ds * cosB * sinAlphaR * cosTheta +
          Iyy2 * Psip * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * Izz2 * Psip * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * M2 * Phip * XB2 * ds * cosB * sinAlphaR * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M2 * Thetap * YB2 * ds * cosB * cosPhi * sinAlphaR * sinPhi +
          M2 * Psip * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          M2 * Psip * XB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta +
          M2 * Psip * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta -
          1.0 * M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi,
      0.5 * Iyy3 * Phip * sinPhi - 0.5 * Ixx3 * Phip * sinPhi - 0.5 * Izz3 * Phip * sinPhi +
          Ixx3 * Thetap * cosAlphaL * sinAlphaL - 1.0 * Iyy3 * Thetap * cosAlphaL * sinAlphaL +
          0.5 * Ixx3 * Psip * sinPhi * sinTheta - 0.5 * Iyy3 * Psip * sinPhi * sinTheta +
          0.5 * Izz3 * Psip * sinPhi * sinTheta + Ixx3 * Phip * pow(cosAlphaL, 2) * sinPhi -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * sinPhi - 1.0 * Iyy3 * Phip * pow(cosB, 2) * sinPhi +
          Izz3 * Phip * pow(cosB, 2) * sinPhi + Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Ixx3 * Thetap * cosAlphaL * pow(cosPhi, 2) * sinAlphaL +
          Iyy3 * Thetap * cosAlphaL * pow(cosPhi, 2) * sinAlphaL -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * sinPhi * sinTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * sinPhi * sinTheta - 1.0 * M3 * Phip * pow(ds, 2) * pow(cosB, 2) * sinPhi +
          Iyy3 * Psip * pow(cosB, 2) * sinPhi * sinTheta - 1.0 * Izz3 * Psip * pow(cosB, 2) * sinPhi * sinTheta +
          Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi + M3 * Thetap * XB3 * ds * cosAlphaL * cosB +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL +
          Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL +
          M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaLp * Iyy3 * cosB * sinAlphaL * sinB * sinPhi - 1.0 * AlphaLp * Izz3 * cosB * sinAlphaL * sinB * sinPhi +
          Iyy3 * Psip * cosB * sinAlphaL * sinB * cosTheta - 1.0 * Izz3 * Psip * cosB * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Thetap * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Iyy3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * sinPhi -
          1.0 * Izz3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi +
          M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * cosTheta +
          AlphaLp * M3 * XB3 * ds * cosAlphaL * cosB * cosPhi -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosB * cosPhi * sinAlphaL -
          1.0 * AlphaLp * M3 * YB3 * ds * cosB * sinAlphaL * sinPhi -
          1.0 * M3 * Psip * YB3 * ds * cosB * sinAlphaL * cosTheta -
          1.0 * Iyy3 * Psip * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          Izz3 * Psip * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * M3 * Phip * XB3 * ds * cosB * sinAlphaL * sinPhi +
          M3 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * sinPhi -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Thetap * YB3 * ds * cosB * cosPhi * sinAlphaL * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          M3 * Psip * XB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta +
          M3 * Psip * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta -
          1.0 * M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi,
      0.5 * AlphaLp * Iyy3 * sinPhi - 0.5 * AlphaLp * Ixx3 * sinPhi - 0.5 * AlphaRp * Ixx2 * sinPhi -
          1.0 * Ixz1 * Phip * cosPhi + 0.5 * AlphaRp * Iyy2 * sinPhi - 0.5 * AlphaLp * Izz3 * sinPhi -
          0.5 * AlphaRp * Izz2 * sinPhi + 0.5 * Ixx1 * Psip * cosTheta + 0.5 * Ixx2 * Psip * cosTheta +
          0.5 * Ixx3 * Psip * cosTheta - 0.5 * Iyy1 * Psip * cosTheta + 0.5 * Iyy2 * Psip * cosTheta +
          0.5 * Iyy3 * Psip * cosTheta + 0.5 * Izz1 * Psip * cosTheta - 0.5 * Izz2 * Psip * cosTheta -
          0.5 * Izz3 * Psip * cosTheta + M1 * Psip * pow(YB1, 2) * cosTheta + M2 * Psip * pow(YB2, 2) * cosTheta +
          M3 * Psip * pow(YB3, 2) * cosTheta + Ixx2 * Thetap * cosPhi * sinPhi + Ixx3 * Thetap * cosPhi * sinPhi -
          1.0 * Iyy1 * Thetap * cosPhi * sinPhi + Izz1 * Thetap * cosPhi * sinPhi -
          1.0 * Izz2 * Thetap * cosPhi * sinPhi - 1.0 * Izz3 * Thetap * cosPhi * sinPhi +
          Ixz1 * Psip * cosPhi * sinTheta + M2 * Psip * pow(ds, 2) * cosTheta + M3 * Psip * pow(ds, 2) * cosTheta +
          AlphaLp * Ixx3 * pow(cosAlphaL, 2) * sinPhi + AlphaRp * Ixx2 * pow(cosAlphaR, 2) * sinPhi -
          1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * sinPhi - 1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * sinPhi -
          1.0 * AlphaLp * Iyy3 * pow(cosB, 2) * sinPhi - 1.0 * AlphaRp * Iyy2 * pow(cosB, 2) * sinPhi +
          AlphaLp * Izz3 * pow(cosB, 2) * sinPhi + AlphaRp * Izz2 * pow(cosB, 2) * sinPhi -
          1.0 * M1 * Thetap * YB1 * ZB1 - 1.0 * M2 * Thetap * YB2 * ZB2 - 1.0 * M3 * Thetap * YB3 * ZB3 -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * cosTheta - 1.0 * Iyy3 * Psip * pow(cosB, 2) * cosTheta +
          Izz2 * Psip * pow(cosB, 2) * cosTheta + Izz3 * Psip * pow(cosB, 2) * cosTheta -
          1.0 * Ixx2 * Psip * pow(cosPhi, 2) * cosTheta - 1.0 * Ixx3 * Psip * pow(cosPhi, 2) * cosTheta +
          Iyy1 * Psip * pow(cosPhi, 2) * cosTheta - 1.0 * Izz1 * Psip * pow(cosPhi, 2) * cosTheta +
          Izz2 * Psip * pow(cosPhi, 2) * cosTheta + Izz3 * Psip * pow(cosPhi, 2) * cosTheta +
          Ixx3 * Phip * cosAlphaL * cosPhi * sinAlphaL + Ixx2 * Phip * cosAlphaR * cosPhi * sinAlphaR -
          1.0 * Iyy3 * Phip * cosAlphaL * cosPhi * sinAlphaL - 1.0 * Iyy2 * Phip * cosAlphaR * cosPhi * sinAlphaR +
          M1 * Thetap * pow(YB1, 2) * cosPhi * sinPhi + M2 * Thetap * pow(YB2, 2) * cosPhi * sinPhi +
          M3 * Thetap * pow(YB3, 2) * cosPhi * sinPhi - 1.0 * M1 * Thetap * pow(ZB1, 2) * cosPhi * sinPhi -
          1.0 * M2 * Thetap * pow(ZB2, 2) * cosPhi * sinPhi - 1.0 * M3 * Thetap * pow(ZB3, 2) * cosPhi * sinPhi +
          M2 * Thetap * pow(ds, 2) * cosPhi * sinPhi + M3 * Thetap * pow(ds, 2) * cosPhi * sinPhi +
          M1 * Phip * XB1 * ZB1 * cosPhi + M2 * Phip * XB2 * ZB2 * cosPhi + M3 * Phip * XB3 * ZB3 * cosPhi +
          M1 * Phip * XB1 * YB1 * sinPhi + M2 * Phip * XB2 * YB2 * sinPhi + M3 * Phip * XB3 * YB3 * sinPhi -
          1.0 * M2 * Thetap * ZB2 * ds * sinB + M3 * Thetap * ZB3 * ds * sinB -
          1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi -
          1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi + Iyy2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi -
          1.0 * Iyy2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi - 1.0 * Iyy3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi +
          Izz2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi + Izz3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosB, 2) * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta -
          1.0 * M1 * Psip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Psip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Psip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta +
          M1 * Psip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta + M2 * Psip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Psip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi -
          1.0 * AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          2.0 * M1 * Thetap * YB1 * ZB1 * pow(cosPhi, 2) + 2.0 * M2 * Thetap * YB2 * ZB2 * pow(cosPhi, 2) +
          2.0 * M3 * Thetap * YB3 * ZB3 * pow(cosPhi, 2) + Ixx3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta + Iyy3 * Thetap * cosAlphaL * cosB * sinB -
          1.0 * Iyy2 * Thetap * cosAlphaR * cosB * sinB - 1.0 * Izz3 * Thetap * cosAlphaL * cosB * sinB +
          Izz2 * Thetap * cosAlphaR * cosB * sinB - 1.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB -
          1.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB +
          Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Izz3 * Phip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * Izz2 * Phip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          2.0 * Iyy3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB +
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB -
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          2.0 * M2 * Psip * YB2 * ds * sinB * cosTheta - 2.0 * M3 * Psip * YB3 * ds * sinB * cosTheta -
          1.0 * M1 * Psip * XB1 * ZB1 * cosPhi * sinTheta - 1.0 * M2 * Psip * XB2 * ZB2 * cosPhi * sinTheta -
          1.0 * M3 * Psip * XB3 * ZB3 * cosPhi * sinTheta + M2 * Phip * XB2 * ds * sinB * sinPhi -
          1.0 * M3 * Phip * XB3 * ds * sinB * sinPhi - 1.0 * M1 * Psip * XB1 * YB1 * sinPhi * sinTheta -
          1.0 * M2 * Psip * XB2 * YB2 * sinPhi * sinTheta - 1.0 * M3 * Psip * XB3 * YB3 * sinPhi * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi +
          AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi +
          2.0 * M2 * Thetap * ZB2 * ds * pow(cosPhi, 2) * sinB - 2.0 * M3 * Thetap * ZB3 * ds * pow(cosPhi, 2) * sinB +
          M2 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * sinB -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * sinB -
          1.0 * Iyy3 * Phip * cosB * sinAlphaL * sinB * sinPhi + Iyy2 * Phip * cosB * sinAlphaR * sinB * sinPhi +
          Izz3 * Phip * cosB * sinAlphaL * sinB * sinPhi - 1.0 * Izz2 * Phip * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * sinTheta -
          1.0 * Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * sinTheta +
          Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * sinTheta +
          Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * sinTheta +
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) +
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) -
          2.0 * M2 * Psip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          2.0 * M3 * Psip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * M3 * Phip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi +
          M2 * Phip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Psip * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * Iyy2 * Psip * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * Izz3 * Psip * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          Izz2 * Psip * cosB * sinAlphaR * sinB * sinPhi * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Phip * XB3 * ds * cosAlphaL * cosB * cosPhi + M2 * Phip * XB2 * ds * cosAlphaR * cosB * cosPhi -
          1.0 * AlphaLp * M3 * XB3 * ds * cosB * sinAlphaL * sinPhi -
          1.0 * AlphaRp * M2 * XB2 * ds * cosB * sinAlphaR * sinPhi + M3 * Phip * ZB3 * ds * cosB * cosPhi * sinAlphaL +
          M2 * Phip * ZB2 * ds * cosB * cosPhi * sinAlphaR +
          M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB +
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          M3 * Phip * YB3 * ds * cosB * sinAlphaL * sinPhi + M2 * Phip * YB2 * ds * cosB * sinAlphaR * sinPhi +
          2.0 * M1 * Psip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Psip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi +
          2.0 * M3 * Psip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Thetap * YB2 * ds * cosPhi * sinB * sinPhi -
          2.0 * M3 * Thetap * YB3 * ds * cosPhi * sinB * sinPhi -
          1.0 * M2 * Psip * XB2 * ds * sinB * sinPhi * sinTheta + M3 * Psip * XB3 * ds * sinB * sinPhi * sinTheta -
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta -
          1.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta -
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * cosPhi * sinPhi -
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * cosPhi * sinPhi -
          1.0 * M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * sinTheta -
          1.0 * M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          1.0 * M3 * Psip * YB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta -
          1.0 * M2 * Psip * YB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta +
          2.0 * M2 * Psip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Psip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi -
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi,
      0.5 * AlphaLp * Ixx3 * sin(2.0 * AlphaL) + (AlphaRp * Ixx2 * sin(2 * AlphaR)) / 2 -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * sinAlphaL - 1.0 * AlphaRp * Iyy2 * cosAlphaR * sinAlphaR +
          Ixx2 * Phip * cosPhi * sinPhi + Ixx3 * Phip * cosPhi * sinPhi - 1.0 * Iyy1 * Phip * cosPhi * sinPhi +
          Izz1 * Phip * cosPhi * sinPhi - 1.0 * Izz2 * Phip * cosPhi * sinPhi - 1.0 * Izz3 * Phip * cosPhi * sinPhi -
          1.0 * M1 * Phip * YB1 * ZB1 - 1.0 * M2 * Phip * YB2 * ZB2 - 1.0 * M3 * Phip * YB3 * ZB3 +
          M1 * Phip * pow(YB1, 2) * cosPhi * sinPhi + M2 * Phip * pow(YB2, 2) * cosPhi * sinPhi +
          M3 * Phip * pow(YB3, 2) * cosPhi * sinPhi - 1.0 * M1 * Phip * pow(ZB1, 2) * cosPhi * sinPhi -
          1.0 * M2 * Phip * pow(ZB2, 2) * cosPhi * sinPhi - 1.0 * M3 * Phip * pow(ZB3, 2) * cosPhi * sinPhi +
          M2 * Phip * pow(ds, 2) * cosPhi * sinPhi + M3 * Phip * pow(ds, 2) * cosPhi * sinPhi +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * AlphaLp * Ixx3 * cosAlphaL * pow(cosPhi, 2) * sinAlphaL -
          1.0 * AlphaRp * Ixx2 * cosAlphaR * pow(cosPhi, 2) * sinAlphaR +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosPhi, 2) * sinAlphaL +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosPhi, 2) * sinAlphaR - 1.0 * M2 * Phip * ZB2 * ds * sinB +
          M3 * Phip * ZB3 * ds * sinB - 1.0 * Ixx3 * Phip * pow(cosAlphaL, 2) * cosPhi * sinPhi -
          1.0 * Ixx2 * Phip * pow(cosAlphaR, 2) * cosPhi * sinPhi + Iyy3 * Phip * pow(cosAlphaL, 2) * cosPhi * sinPhi +
          Iyy2 * Phip * pow(cosAlphaR, 2) * cosPhi * sinPhi - 1.0 * Iyy2 * Phip * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Iyy3 * Phip * pow(cosB, 2) * cosPhi * sinPhi + Izz2 * Phip * pow(cosB, 2) * cosPhi * sinPhi +
          Izz3 * Phip * pow(cosB, 2) * cosPhi * sinPhi + 2.0 * M1 * Phip * YB1 * ZB1 * pow(cosPhi, 2) +
          2.0 * M2 * Phip * YB2 * ZB2 * pow(cosPhi, 2) + 2.0 * M3 * Phip * YB3 * ZB3 * pow(cosPhi, 2) +
          Iyy3 * Phip * cosAlphaL * cosB * sinB - 1.0 * Iyy2 * Phip * cosAlphaR * cosB * sinB -
          1.0 * Izz3 * Phip * cosAlphaL * cosB * sinB + Izz2 * Phip * cosAlphaR * cosB * sinB -
          1.0 * M3 * Phip * YB3 * ds * cosAlphaL * cosB - 1.0 * M2 * Phip * YB2 * ds * cosAlphaR * cosB -
          2.0 * Iyy3 * Phip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB +
          2.0 * Iyy2 * Phip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          2.0 * Izz3 * Phip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB -
          2.0 * Izz2 * Phip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR +
          AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL +
          AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          2.0 * M2 * Phip * ZB2 * ds * pow(cosPhi, 2) * sinB - 2.0 * M3 * Phip * ZB3 * ds * pow(cosPhi, 2) * sinB +
          M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * sinB - 1.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * sinB +
          AlphaLp * M3 * XB3 * ds * cosAlphaL * cosB + AlphaRp * M2 * XB2 * ds * cosAlphaR * cosB +
          2.0 * M3 * Phip * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) +
          2.0 * M2 * Phip * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi +
          AlphaLp * Iyy3 * cosB * cosPhi * sinAlphaL * sinB * sinPhi -
          1.0 * AlphaRp * Iyy2 * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          1.0 * AlphaLp * Izz3 * cosB * cosPhi * sinAlphaL * sinB * sinPhi +
          AlphaRp * Izz2 * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          2.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB +
          2.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB +
          2.0 * M2 * Phip * YB2 * ds * cosPhi * sinB * sinPhi - 2.0 * M3 * Phip * YB3 * ds * cosPhi * sinB * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * sinPhi -
          1.0 * AlphaLp * M3 * YB3 * ds * cosB * cosPhi * sinAlphaL * sinPhi -
          1.0 * AlphaRp * M2 * YB2 * ds * cosB * cosPhi * sinAlphaR * sinPhi -
          2.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * cosPhi * sinPhi -
          2.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * cosPhi * sinPhi,
      0.5 * Ixx1 * Phip * cosTheta - 1.0 * Ixz1 * Psip * cosPhi + 0.5 * Ixx2 * Phip * cosTheta +
          0.5 * Ixx3 * Phip * cosTheta - 0.5 * Iyy1 * Phip * cosTheta + 0.5 * Iyy2 * Phip * cosTheta +
          0.5 * Iyy3 * Phip * cosTheta + 0.5 * Izz1 * Phip * cosTheta - 0.5 * Izz2 * Phip * cosTheta -
          0.5 * Izz3 * Phip * cosTheta + M1 * Phip * pow(YB1, 2) * cosTheta + M2 * Phip * pow(YB2, 2) * cosTheta +
          M3 * Phip * pow(YB3, 2) * cosTheta + Ixz1 * Phip * cosPhi * sinTheta +
          0.5 * AlphaLp * Ixx3 * sinPhi * sinTheta + 0.5 * AlphaRp * Ixx2 * sinPhi * sinTheta -
          0.5 * AlphaLp * Iyy3 * sinPhi * sinTheta - 0.5 * AlphaRp * Iyy2 * sinPhi * sinTheta +
          0.5 * AlphaLp * Izz3 * sinPhi * sinTheta + 0.5 * AlphaRp * Izz2 * sinPhi * sinTheta +
          M2 * Phip * pow(ds, 2) * cosTheta + M3 * Phip * pow(ds, 2) * cosTheta -
          1.0 * Ixx1 * Psip * cosTheta * sinTheta + Iyy1 * Psip * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * cosTheta * sinTheta - 1.0 * Iyy3 * Psip * cosTheta * sinTheta +
          Izz2 * Psip * cosTheta * sinTheta + Izz3 * Psip * cosTheta * sinTheta -
          1.0 * Iyy2 * Phip * pow(cosB, 2) * cosTheta - 1.0 * Iyy3 * Phip * pow(cosB, 2) * cosTheta +
          Izz2 * Phip * pow(cosB, 2) * cosTheta + Izz3 * Phip * pow(cosB, 2) * cosTheta -
          1.0 * Ixx2 * Phip * pow(cosPhi, 2) * cosTheta - 1.0 * Ixx3 * Phip * pow(cosPhi, 2) * cosTheta +
          Iyy1 * Phip * pow(cosPhi, 2) * cosTheta - 1.0 * Izz1 * Phip * pow(cosPhi, 2) * cosTheta +
          Izz2 * Phip * pow(cosPhi, 2) * cosTheta + Izz3 * Phip * pow(cosPhi, 2) * cosTheta +
          2.0 * Ixz1 * Psip * cosPhi * pow(cosTheta, 2) + Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL +
          Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR - 1.0 * Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL -
          1.0 * Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR + M1 * Psip * pow(XB1, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(XB2, 2) * cosTheta * sinTheta + M3 * Psip * pow(XB3, 2) * cosTheta * sinTheta -
          1.0 * M1 * Psip * pow(YB1, 2) * cosTheta * sinTheta - 1.0 * M2 * Psip * pow(YB2, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(YB3, 2) * cosTheta * sinTheta - 1.0 * M2 * Psip * pow(ds, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosTheta * sinTheta + M1 * Psip * XB1 * ZB1 * cosPhi +
          M2 * Psip * XB2 * ZB2 * cosPhi + M3 * Psip * XB3 * ZB3 * cosPhi + M1 * Psip * XB1 * YB1 * sinPhi +
          M2 * Psip * XB2 * YB2 * sinPhi + M3 * Psip * XB3 * YB3 * sinPhi -
          1.0 * AlphaLp * Ixx3 * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Ixx2 * pow(cosAlphaR, 2) * sinPhi * sinTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * sinPhi * sinTheta +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * sinPhi * sinTheta + AlphaLp * Iyy3 * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * Iyy2 * pow(cosB, 2) * sinPhi * sinTheta - 1.0 * AlphaLp * Izz3 * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Izz2 * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * cosTheta * sinTheta -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * cosTheta * sinTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * cosTheta * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * cosTheta * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosB, 2) * cosTheta -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosB, 2) * cosTheta +
          2.0 * Iyy2 * Psip * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * Iyy3 * Psip * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * Izz2 * Psip * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * Izz3 * Psip * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * M1 * Phip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Phip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Phip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta +
          M1 * Phip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta + M2 * Phip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Phip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta + Ixx3 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy1 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz1 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta +
          Ixx3 * Phip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Phip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          2.0 * Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M2 * Phip * YB2 * ds * sinB * cosTheta - 2.0 * M3 * Phip * YB3 * ds * sinB * cosTheta -
          1.0 * M1 * Phip * XB1 * ZB1 * cosPhi * sinTheta - 1.0 * M2 * Phip * XB2 * ZB2 * cosPhi * sinTheta -
          1.0 * M3 * Phip * XB3 * ZB3 * cosPhi * sinTheta + M2 * Psip * XB2 * ds * sinB * sinPhi -
          1.0 * M3 * Psip * XB3 * ds * sinB * sinPhi - 1.0 * M1 * Phip * XB1 * YB1 * sinPhi * sinTheta -
          1.0 * M2 * Phip * XB2 * YB2 * sinPhi * sinTheta - 1.0 * M3 * Phip * XB3 * YB3 * sinPhi * sinTheta +
          AlphaLp * M3 * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          2.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          M1 * Psip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Psip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M1 * Psip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * M1 * Psip * XB1 * ZB1 * cosPhi * pow(cosTheta, 2) -
          2.0 * M2 * Psip * XB2 * ZB2 * cosPhi * pow(cosTheta, 2) -
          2.0 * M3 * Psip * XB3 * ZB3 * cosPhi * pow(cosTheta, 2) -
          1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * M1 * Psip * XB1 * YB1 * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Psip * XB2 * YB2 * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Psip * XB3 * YB3 * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Phip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Phip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          AlphaLp * Iyy3 * cosB * sinAlphaL * sinB * cosTheta -
          1.0 * AlphaRp * Iyy2 * cosB * sinAlphaR * sinB * cosTheta -
          1.0 * AlphaLp * Izz3 * cosB * sinAlphaL * sinB * cosTheta +
          AlphaRp * Izz2 * cosB * sinAlphaR * sinB * cosTheta - 1.0 * Iyy3 * Psip * cosB * sinAlphaL * sinB * sinPhi +
          Iyy2 * Psip * cosB * sinAlphaR * sinB * sinPhi + Izz3 * Psip * cosB * sinAlphaL * sinB * sinPhi -
          1.0 * Izz2 * Psip * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx3 * Phip * cosAlphaL * cosPhi * sinAlphaL * sinTheta -
          1.0 * Ixx2 * Phip * cosAlphaR * cosPhi * sinAlphaR * sinTheta +
          Iyy3 * Phip * cosAlphaL * cosPhi * sinAlphaL * sinTheta +
          Iyy2 * Phip * cosAlphaR * cosPhi * sinAlphaR * sinTheta -
          2.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * M2 * Phip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          2.0 * M3 * Phip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * M2 * Psip * XB2 * ds * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Psip * XB3 * ds * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * AlphaLp * Ixx3 * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * Ixx2 * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaLp * Iyy3 * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          AlphaRp * Iyy2 * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaLp * M3 * pow(ds, 2) * cosB * sinAlphaL * sinB * cosTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosB * sinAlphaR * sinB * cosTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi +
          M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Phip * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * Iyy2 * Phip * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * Izz3 * Phip * cosB * sinAlphaL * sinB * sinPhi * sinTheta +
          Izz2 * Phip * cosB * sinAlphaR * sinB * sinPhi * sinTheta +
          M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi + M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi -
          1.0 * AlphaLp * M3 * YB3 * ds * cosB * sinAlphaL * cosTheta -
          1.0 * AlphaRp * M2 * YB2 * ds * cosB * sinAlphaR * cosTheta -
          1.0 * AlphaLp * Iyy3 * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          AlphaRp * Iyy2 * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          AlphaLp * Izz3 * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * AlphaRp * Izz2 * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL + M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          Izz3 * Phip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta +
          Izz2 * Phip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta +
          M3 * Psip * YB3 * ds * cosB * sinAlphaL * sinPhi + M2 * Psip * YB2 * ds * cosB * sinAlphaR * sinPhi +
          2.0 * M1 * Phip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Phip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi +
          2.0 * M3 * Phip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi +
          2.0 * Iyy3 * Psip * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * Iyy2 * Psip * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * Izz3 * Psip * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * Izz2 * Psip * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Psip * YB2 * ds * sinB * cosTheta * sinTheta +
          2.0 * M3 * Psip * YB3 * ds * sinB * cosTheta * sinTheta -
          1.0 * M2 * Phip * XB2 * ds * sinB * sinPhi * sinTheta + M3 * Phip * XB3 * ds * sinB * sinPhi * sinTheta +
          2.0 * M2 * Psip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta -
          2.0 * M3 * Psip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta -
          2.0 * Iyy3 * Phip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Iyy2 * Phip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz3 * Phip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz2 * Phip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          M3 * Phip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M3 * Phip * XB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta -
          1.0 * M2 * Phip * XB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          AlphaRp * M2 * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          AlphaLp * M3 * XB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta +
          AlphaRp * M2 * XB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta -
          1.0 * M3 * Phip * ZB3 * ds * cosB * cosPhi * sinAlphaL * sinTheta -
          1.0 * M2 * Phip * ZB2 * ds * cosB * cosPhi * sinAlphaR * sinTheta +
          2.0 * M3 * Psip * XB3 * ds * cosB * sinAlphaL * cosTheta * sinTheta +
          2.0 * M2 * Psip * XB2 * ds * cosB * sinAlphaR * cosTheta * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * sinTheta -
          1.0 * M3 * Phip * YB3 * ds * cosB * sinAlphaL * sinPhi * sinTheta -
          1.0 * M2 * Phip * YB2 * ds * cosB * sinAlphaR * sinPhi * sinTheta +
          2.0 * M2 * Phip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Phip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M1 * Psip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M2 * Psip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) -
          2.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) +
          AlphaLp * M3 * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta +
          AlphaRp * M2 * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta +
          2.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta -
          2.0 * M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * M3 * Psip * YB3 * ds * cosB * sinAlphaL * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Psip * YB2 * ds * cosB * sinAlphaR * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Phip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Phip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          2.0 * M2 * Psip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * Izz3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * Izz2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi * sinTheta,
      0, 0, 0,
      Iyy2 * Psip * cosAlphaR * sinAlphaR - 1.0 * Ixx2 * Psip * cosAlphaR * sinAlphaR +
          0.5 * Ixx2 * Phip * cosPhi * cosTheta - 0.5 * Iyy2 * Phip * cosPhi * cosTheta +
          0.5 * Izz2 * Phip * cosPhi * cosTheta + 0.5 * Ixx2 * Thetap * sinPhi * sinTheta -
          0.5 * Iyy2 * Thetap * sinPhi * sinTheta - 0.5 * Izz2 * Thetap * sinPhi * sinTheta +
          Ixx2 * Phip * cosAlphaR * sinAlphaR * sinTheta - 1.0 * Iyy2 * Phip * cosAlphaR * sinAlphaR * sinTheta -
          1.0 * Ixx2 * Psip * cosPhi * cosTheta * sinTheta + Iyy2 * Psip * cosPhi * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR -
          1.0 * Ixx2 * Phip * pow(cosAlphaR, 2) * cosPhi * cosTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * cosPhi * cosTheta + Iyy2 * Phip * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz2 * Phip * pow(cosB, 2) * cosPhi * cosTheta +
          Ixx2 * Psip * cosAlphaR * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Iyy2 * Psip * cosAlphaR * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * sinPhi * sinTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * sinPhi * sinTheta - 1.0 * M2 * Psip * ZB2 * ds * cosB * sinAlphaR -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta +
          2.0 * Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta +
          M2 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy2 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          Izz2 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) +
          Ixx2 * Psip * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * Iyy2 * cosAlphaR * cosB * sinB * sinTheta -
          1.0 * AlphaRp * Izz2 * cosAlphaR * cosB * sinB * sinTheta +
          M2 * Psip * XB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          2.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * Iyy2 * cosB * cosPhi * sinAlphaR * sinB * cosTheta -
          1.0 * AlphaRp * Izz2 * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * sinTheta -
          1.0 * Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * sinB * sinTheta +
          Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Ixx2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          Iyy2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB * sinTheta +
          Iyy2 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * Izz2 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          M2 * Phip * ZB2 * ds * cosB * sinAlphaR * sinTheta +
          M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          Iyy2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * Izz2 * Psip * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta +
          M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * sinTheta +
          M2 * Psip * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          AlphaRp * M2 * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi +
          AlphaRp * M2 * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta +
          M2 * Phip * XB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi +
          M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta -
          1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * sinPhi * sinTheta +
          Iyy2 * Psip * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Psip * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          M2 * Thetap * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          M2 * Thetap * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta +
          2.0 * M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta +
          M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinTheta -
          1.0 * M2 * Psip * XB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta +
          M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Thetap * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          M2 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          M2 * Psip * YB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) * sinPhi,
      Iyy3 * Psip * cosAlphaL * sinAlphaL - 1.0 * Ixx3 * Psip * cosAlphaL * sinAlphaL +
          0.5 * Ixx3 * Phip * cosPhi * cosTheta - 0.5 * Iyy3 * Phip * cosPhi * cosTheta +
          0.5 * Izz3 * Phip * cosPhi * cosTheta + 0.5 * Ixx3 * Thetap * sinPhi * sinTheta -
          0.5 * Iyy3 * Thetap * sinPhi * sinTheta - 0.5 * Izz3 * Thetap * sinPhi * sinTheta +
          Ixx3 * Phip * cosAlphaL * sinAlphaL * sinTheta - 1.0 * Iyy3 * Phip * cosAlphaL * sinAlphaL * sinTheta -
          1.0 * Ixx3 * Psip * cosPhi * cosTheta * sinTheta + Iyy3 * Psip * cosPhi * cosTheta * sinTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * Ixx3 * Phip * pow(cosAlphaL, 2) * cosPhi * cosTheta +
          Iyy3 * Phip * pow(cosAlphaL, 2) * cosPhi * cosTheta + Iyy3 * Phip * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Izz3 * Phip * pow(cosB, 2) * cosPhi * cosTheta +
          Ixx3 * Psip * cosAlphaL * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Iyy3 * Psip * cosAlphaL * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * sinPhi * sinTheta +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * sinPhi * sinTheta - 1.0 * M3 * Psip * ZB3 * ds * cosB * sinAlphaL -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          2.0 * Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta +
          M3 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * Iyy3 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          Izz3 * Psip * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) +
          Ixx3 * Psip * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * cosB * sinB * sinTheta +
          AlphaLp * Izz3 * cosAlphaL * cosB * sinB * sinTheta +
          M3 * Psip * XB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          2.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosB * cosPhi * sinAlphaL * sinB * cosTheta +
          AlphaLp * Izz3 * cosB * cosPhi * sinAlphaL * sinB * cosTheta -
          1.0 * Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * sinTheta +
          Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * sinB * sinTheta +
          Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * Ixx3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Iyy3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB * sinTheta -
          1.0 * Iyy3 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          Izz3 * Thetap * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          M3 * Phip * ZB3 * ds * cosB * sinAlphaL * sinTheta +
          M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta +
          Izz3 * Psip * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * sinTheta +
          M3 * Psip * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaLp * M3 * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi +
          AlphaLp * M3 * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta +
          M3 * Phip * XB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi +
          M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta -
          1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          Izz3 * Psip * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          M3 * Thetap * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta +
          2.0 * M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta +
          M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinTheta -
          1.0 * M3 * Psip * XB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta +
          M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Thetap * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * YB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) * sinPhi,
      0.5 * Ixx2 * Thetap * cosTheta - 0.5 * Ixx1 * Thetap * cosTheta + 0.5 * Ixx3 * Thetap * cosTheta -
          0.5 * Iyy1 * Thetap * cosTheta - 0.5 * Iyy2 * Thetap * cosTheta - 0.5 * Iyy3 * Thetap * cosTheta +
          0.5 * Izz1 * Thetap * cosTheta - 0.5 * Izz2 * Thetap * cosTheta - 0.5 * Izz3 * Thetap * cosTheta +
          0.5 * AlphaLp * Ixx3 * cosPhi * cosTheta + 0.5 * AlphaRp * Ixx2 * cosPhi * cosTheta -
          0.5 * AlphaLp * Iyy3 * cosPhi * cosTheta - 0.5 * AlphaRp * Iyy2 * cosPhi * cosTheta +
          0.5 * AlphaLp * Izz3 * cosPhi * cosTheta + 0.5 * AlphaRp * Izz2 * cosPhi * cosTheta -
          1.0 * M1 * Thetap * pow(ZB1, 2) * cosTheta - 1.0 * M2 * Thetap * pow(ZB2, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(ZB3, 2) * cosTheta - 1.0 * Ixz1 * Phip * cosTheta * sinPhi -
          1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * cosTheta - 1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * cosTheta +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * cosTheta + Iyy2 * Thetap * pow(cosAlphaR, 2) * cosTheta -
          1.0 * Ixx2 * Thetap * pow(cosPhi, 2) * cosTheta - 1.0 * Ixx3 * Thetap * pow(cosPhi, 2) * cosTheta +
          Iyy1 * Thetap * pow(cosPhi, 2) * cosTheta - 1.0 * Izz1 * Thetap * pow(cosPhi, 2) * cosTheta +
          Izz2 * Thetap * pow(cosPhi, 2) * cosTheta + Izz3 * Thetap * pow(cosPhi, 2) * cosTheta +
          AlphaLp * Ixx3 * cosAlphaL * sinAlphaL * sinTheta + AlphaRp * Ixx2 * cosAlphaR * sinAlphaR * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * sinAlphaL * sinTheta -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * sinAlphaR * sinTheta + Ixz1 * Psip * cosTheta * sinPhi * sinTheta -
          1.0 * AlphaLp * Ixx3 * pow(cosAlphaL, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * Ixx2 * pow(cosAlphaR, 2) * cosPhi * cosTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * cosPhi * cosTheta +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * cosPhi * cosTheta + AlphaLp * Iyy3 * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaRp * Iyy2 * pow(cosB, 2) * cosPhi * cosTheta - 1.0 * AlphaLp * Izz3 * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * Izz2 * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * M1 * Thetap * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Thetap * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta +
          M1 * Thetap * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Thetap * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Thetap * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx2 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx3 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi + Iyy1 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz1 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi + Izz2 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz3 * Psip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta + M1 * Psip * YB1 * ZB1 * pow(cosTheta, 2) +
          M2 * Psip * YB2 * ZB2 * pow(cosTheta, 2) + M3 * Psip * YB3 * ZB3 * pow(cosTheta, 2) +
          Ixx3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) +
          Iyy2 * Psip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          Izz3 * Psip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) -
          1.0 * Izz2 * Psip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta -
          1.0 * AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * M1 * Phip * XB1 * YB1 * cosPhi * cosTheta - 1.0 * M2 * Phip * XB2 * YB2 * cosPhi * cosTheta -
          1.0 * M3 * Phip * XB3 * YB3 * cosPhi * cosTheta +
          AlphaLp * M3 * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaRp * M2 * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta + M1 * Phip * XB1 * ZB1 * cosTheta * sinPhi +
          M2 * Phip * XB2 * ZB2 * cosTheta * sinPhi + M3 * Phip * XB3 * ZB3 * cosTheta * sinPhi -
          1.0 * M1 * Psip * pow(YB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * pow(YB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * pow(YB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M1 * Psip * pow(ZB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Psip * pow(ZB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * pow(ZB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta +
          AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta +
          M2 * Psip * ZB2 * ds * sinB * pow(cosTheta, 2) - 1.0 * M3 * Psip * ZB3 * ds * sinB * pow(cosTheta, 2) +
          Ixx3 * Psip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Ixx2 * Psip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy2 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy3 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Psip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx3 * Phip * cosAlphaL * sinAlphaL * cosTheta * sinPhi +
          Ixx2 * Phip * cosAlphaR * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy3 * Phip * cosAlphaL * sinAlphaL * cosTheta * sinPhi -
          1.0 * Iyy2 * Phip * cosAlphaR * sinAlphaR * cosTheta * sinPhi +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          2.0 * M1 * Psip * YB1 * ZB1 * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M2 * Psip * YB2 * ZB2 * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M3 * Psip * YB3 * ZB3 * pow(cosPhi, 2) * pow(cosTheta, 2) +
          M3 * Psip * YB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) +
          M2 * Psip * YB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) +
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * Izz3 * Psip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * Izz2 * Psip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta -
          2.0 * M2 * Thetap * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          2.0 * M3 * Thetap * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy3 * Phip * cosB * cosPhi * sinAlphaL * sinB * cosTheta -
          1.0 * Iyy2 * Phip * cosB * cosPhi * sinAlphaR * sinB * cosTheta -
          1.0 * Izz3 * Phip * cosB * cosPhi * sinAlphaL * sinB * cosTheta +
          Izz2 * Phip * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx3 * Psip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Ixx2 * Psip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta +
          Iyy3 * Psip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Iyy2 * Psip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          2.0 * M2 * Psip * ZB2 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * M3 * Psip * ZB3 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * cosTheta -
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * cosTheta +
          AlphaLp * M3 * ZB3 * ds * cosB * sinAlphaL * sinTheta +
          AlphaRp * M2 * ZB2 * ds * cosB * sinAlphaR * sinTheta +
          Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi +
          Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi -
          1.0 * Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi -
          1.0 * Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * sinB * pow(cosTheta, 2) +
          M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * sinTheta -
          1.0 * M2 * Phip * XB2 * ds * cosPhi * sinB * cosTheta + M3 * Phip * XB3 * ds * cosPhi * sinB * cosTheta +
          M1 * Psip * XB1 * YB1 * cosPhi * cosTheta * sinTheta + M2 * Psip * XB2 * YB2 * cosPhi * cosTheta * sinTheta +
          M3 * Psip * XB3 * YB3 * cosPhi * cosTheta * sinTheta +
          2.0 * M1 * Thetap * YB1 * ZB1 * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Thetap * YB2 * ZB2 * cosPhi * cosTheta * sinPhi +
          2.0 * M3 * Thetap * YB3 * ZB3 * cosPhi * cosTheta * sinPhi -
          1.0 * M1 * Psip * XB1 * ZB1 * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * XB2 * ZB2 * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Psip * XB3 * ZB3 * cosTheta * sinPhi * sinTheta -
          2.0 * Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) +
          M3 * Phip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta -
          1.0 * Iyy3 * Psip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta +
          Iyy2 * Psip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          Izz3 * Psip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          AlphaLp * M3 * XB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta +
          AlphaRp * M2 * XB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta +
          M3 * Phip * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi +
          M2 * Phip * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi -
          1.0 * M3 * Phip * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta -
          1.0 * M2 * Phip * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta +
          M3 * Phip * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi +
          M2 * Phip * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi +
          M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi +
          M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M2 * Psip * XB2 * ds * cosPhi * sinB * cosTheta * sinTheta -
          1.0 * M3 * Psip * XB3 * ds * cosPhi * sinB * cosTheta * sinTheta +
          2.0 * M2 * Thetap * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Thetap * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * M2 * Psip * YB2 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Psip * YB3 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi -
          1.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta +
          M3 * Psip * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta +
          M2 * Psip * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta -
          1.0 * M3 * Psip * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi,
      Ixz1 * Psip * cosPhi - 0.5 * Ixx1 * Phip * cosTheta + 0.5 * Ixx2 * Phip * cosTheta +
          0.5 * Ixx3 * Phip * cosTheta - 0.5 * Iyy1 * Phip * cosTheta - 0.5 * Iyy2 * Phip * cosTheta -
          0.5 * Iyy3 * Phip * cosTheta + 0.5 * Izz1 * Phip * cosTheta - 0.5 * Izz2 * Phip * cosTheta -
          0.5 * Izz3 * Phip * cosTheta - 1.0 * M1 * Phip * pow(ZB1, 2) * cosTheta -
          1.0 * M2 * Phip * pow(ZB2, 2) * cosTheta - 1.0 * M3 * Phip * pow(ZB3, 2) * cosTheta +
          0.5 * AlphaLp * Ixx3 * sinPhi * sinTheta + 0.5 * AlphaRp * Ixx2 * sinPhi * sinTheta -
          0.5 * AlphaLp * Iyy3 * sinPhi * sinTheta - 0.5 * AlphaRp * Iyy2 * sinPhi * sinTheta -
          0.5 * AlphaLp * Izz3 * sinPhi * sinTheta - 0.5 * AlphaRp * Izz2 * sinPhi * sinTheta +
          Ixz1 * Thetap * cosTheta * sinPhi + Ixx1 * Psip * cosTheta * sinTheta -
          1.0 * Iyy1 * Psip * cosTheta * sinTheta + Iyy2 * Psip * cosTheta * sinTheta +
          Iyy3 * Psip * cosTheta * sinTheta - 1.0 * Izz2 * Psip * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * cosTheta * sinTheta - 1.0 * Ixx3 * Phip * pow(cosAlphaL, 2) * cosTheta -
          1.0 * Ixx2 * Phip * pow(cosAlphaR, 2) * cosTheta + Iyy3 * Phip * pow(cosAlphaL, 2) * cosTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * cosTheta - 1.0 * Ixx2 * Phip * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx3 * Phip * pow(cosPhi, 2) * cosTheta + Iyy1 * Phip * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz1 * Phip * pow(cosPhi, 2) * cosTheta + Izz2 * Phip * pow(cosPhi, 2) * cosTheta +
          Izz3 * Phip * pow(cosPhi, 2) * cosTheta - 2.0 * Ixz1 * Psip * cosPhi * pow(cosTheta, 2) -
          1.0 * Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL - 1.0 * Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR +
          Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL + Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR -
          1.0 * M1 * Psip * pow(XB1, 2) * cosTheta * sinTheta - 1.0 * M2 * Psip * pow(XB2, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(XB3, 2) * cosTheta * sinTheta + M1 * Psip * pow(YB1, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(YB2, 2) * cosTheta * sinTheta + M3 * Psip * pow(YB3, 2) * cosTheta * sinTheta +
          Ixx2 * Thetap * cosPhi * sinPhi * sinTheta + Ixx3 * Thetap * cosPhi * sinPhi * sinTheta -
          1.0 * Iyy1 * Thetap * cosPhi * sinPhi * sinTheta + Izz1 * Thetap * cosPhi * sinPhi * sinTheta -
          1.0 * Izz2 * Thetap * cosPhi * sinPhi * sinTheta - 1.0 * Izz3 * Thetap * cosPhi * sinPhi * sinTheta +
          M2 * Psip * pow(ds, 2) * cosTheta * sinTheta + M3 * Psip * pow(ds, 2) * cosTheta * sinTheta -
          1.0 * M1 * Psip * XB1 * ZB1 * cosPhi - 1.0 * M2 * Psip * XB2 * ZB2 * cosPhi -
          1.0 * M3 * Psip * XB3 * ZB3 * cosPhi - 1.0 * M1 * Psip * XB1 * YB1 * sinPhi -
          1.0 * M2 * Psip * XB2 * YB2 * sinPhi - 1.0 * M3 * Psip * XB3 * YB3 * sinPhi -
          1.0 * AlphaLp * Ixx3 * pow(cosAlphaL, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Ixx2 * pow(cosAlphaR, 2) * sinPhi * sinTheta +
          AlphaLp * Iyy3 * pow(cosAlphaL, 2) * sinPhi * sinTheta +
          AlphaRp * Iyy2 * pow(cosAlphaR, 2) * sinPhi * sinTheta - 1.0 * M1 * Thetap * YB1 * ZB1 * sinTheta -
          1.0 * M2 * Thetap * YB2 * ZB2 * sinTheta - 1.0 * M3 * Thetap * YB3 * ZB3 * sinTheta +
          Ixx3 * Psip * pow(cosAlphaL, 2) * cosTheta * sinTheta +
          Ixx2 * Psip * pow(cosAlphaR, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * cosTheta * sinTheta -
          2.0 * Iyy2 * Psip * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * Iyy3 * Psip * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * Izz2 * Psip * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * Izz3 * Psip * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * M1 * Phip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Phip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Phip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta +
          M1 * Phip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta + M2 * Phip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Phip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx2 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Ixx3 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy1 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz1 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz2 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta + Izz3 * Psip * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta +
          Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta +
          Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta +
          Ixx3 * Phip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta +
          Ixx2 * Phip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Phip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Thetap * pow(ds, 2) * cosPhi * sinPhi * sinTheta +
          M3 * Thetap * pow(ds, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR +
          Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR + M1 * Thetap * XB1 * YB1 * cosPhi * cosTheta +
          M2 * Thetap * XB2 * YB2 * cosPhi * cosTheta + M3 * Thetap * XB3 * YB3 * cosPhi * cosTheta +
          2.0 * Ixx3 * Psip * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * Ixx2 * Psip * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * Iyy3 * Psip * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * Iyy2 * Psip * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          1.0 * M1 * Thetap * XB1 * ZB1 * cosTheta * sinPhi - 1.0 * M2 * Thetap * XB2 * ZB2 * cosTheta * sinPhi -
          1.0 * M3 * Thetap * XB3 * ZB3 * cosTheta * sinPhi - 1.0 * M2 * Psip * XB2 * ds * sinB * sinPhi +
          M3 * Psip * XB3 * ds * sinB * sinPhi - 1.0 * M2 * Thetap * ZB2 * ds * sinB * sinTheta +
          M3 * Thetap * ZB3 * ds * sinB * sinTheta -
          1.0 * Ixx3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * Ixx2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi * sinTheta +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * cosPhi * sinPhi * sinTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * Iyy3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi * sinTheta +
          Izz2 * Thetap * pow(cosB, 2) * cosPhi * sinPhi * sinTheta +
          Izz3 * Thetap * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          2.0 * M2 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * M3 * Psip * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * M1 * Psip * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M1 * Psip * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Psip * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          2.0 * M1 * Psip * XB1 * ZB1 * cosPhi * pow(cosTheta, 2) +
          2.0 * M2 * Psip * XB2 * ZB2 * cosPhi * pow(cosTheta, 2) +
          2.0 * M3 * Psip * XB3 * ZB3 * cosPhi * pow(cosTheta, 2) -
          1.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta -
          1.0 * M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * M1 * Psip * XB1 * YB1 * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Psip * XB2 * YB2 * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Psip * XB3 * YB3 * pow(cosTheta, 2) * sinPhi +
          2.0 * M1 * Thetap * YB1 * ZB1 * pow(cosPhi, 2) * sinTheta +
          2.0 * M2 * Thetap * YB2 * ZB2 * pow(cosPhi, 2) * sinTheta +
          2.0 * M3 * Thetap * YB3 * ZB3 * pow(cosPhi, 2) * sinTheta +
          Ixx3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Ixx2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Phip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M3 * Phip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy3 * Thetap * cosAlphaL * cosB * sinB * sinTheta -
          1.0 * Iyy2 * Thetap * cosAlphaR * cosB * sinB * sinTheta -
          1.0 * Izz3 * Thetap * cosAlphaL * cosB * sinB * sinTheta +
          Izz2 * Thetap * cosAlphaR * cosB * sinB * sinTheta + Iyy3 * Psip * cosB * sinAlphaL * sinB * sinPhi -
          1.0 * Iyy2 * Psip * cosB * sinAlphaR * sinB * sinPhi - 1.0 * Izz3 * Psip * cosB * sinAlphaL * sinB * sinPhi +
          Izz2 * Psip * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta -
          1.0 * Ixx3 * Thetap * cosAlphaL * sinAlphaL * cosTheta * sinPhi -
          1.0 * Ixx2 * Thetap * cosAlphaR * sinAlphaR * cosTheta * sinPhi +
          Iyy3 * Thetap * cosAlphaL * sinAlphaL * cosTheta * sinPhi +
          Iyy2 * Thetap * cosAlphaR * sinAlphaR * cosTheta * sinPhi +
          M1 * Thetap * pow(YB1, 2) * cosPhi * sinPhi * sinTheta +
          M2 * Thetap * pow(YB2, 2) * cosPhi * sinPhi * sinTheta +
          M3 * Thetap * pow(YB3, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M1 * Thetap * pow(ZB1, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M2 * Thetap * pow(ZB2, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M3 * Thetap * pow(ZB3, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta +
          2.0 * Iyy3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * Iyy2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * Izz3 * Psip * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * Izz2 * Psip * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * M2 * Phip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta +
          2.0 * M3 * Phip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta +
          Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta +
          Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * sinPhi * sinTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * sinPhi * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * M2 * Psip * XB2 * ds * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Psip * XB3 * ds * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Thetap * ZB2 * ds * pow(cosPhi, 2) * sinB * sinTheta -
          2.0 * M3 * Thetap * ZB3 * ds * pow(cosPhi, 2) * sinB * sinTheta +
          M2 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Psip * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * cosB * cosPhi * sinB * sinTheta +
          AlphaRp * Iyy2 * cosAlphaR * cosB * cosPhi * sinB * sinTheta +
          AlphaLp * Izz3 * cosAlphaL * cosB * cosPhi * sinB * sinTheta -
          1.0 * AlphaRp * Izz2 * cosAlphaR * cosB * cosPhi * sinB * sinTheta -
          1.0 * AlphaLp * Ixx3 * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * Ixx2 * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaLp * Iyy3 * cosAlphaL * cosPhi * sinAlphaL * cosTheta * sinPhi +
          AlphaRp * Iyy2 * cosAlphaR * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * Iyy3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * cosTheta +
          Iyy2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          Izz3 * Thetap * cosB * cosPhi * sinAlphaL * sinB * cosTheta -
          1.0 * Izz2 * Thetap * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * sinB * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * sinB * sinTheta +
          M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi -
          1.0 * M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi +
          M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta +
          Iyy3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Psip * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Psip * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi -
          1.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi -
          2.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * cosTheta -
          2.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * cosTheta -
          1.0 * AlphaLp * Iyy3 * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          AlphaRp * Iyy2 * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta +
          AlphaLp * Izz3 * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta -
          1.0 * AlphaRp * Izz2 * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          1.0 * M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL -
          1.0 * M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR -
          1.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * sinTheta -
          1.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * sinTheta -
          1.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi -
          1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi +
          Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi +
          Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi -
          2.0 * Iyy3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * sinTheta +
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * sinTheta +
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * sinTheta -
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * sinTheta -
          1.0 * M3 * Psip * YB3 * ds * cosB * sinAlphaL * sinPhi -
          1.0 * M2 * Psip * YB2 * ds * cosB * sinAlphaR * sinPhi + M2 * Thetap * XB2 * ds * cosPhi * sinB * cosTheta -
          1.0 * M3 * Thetap * XB3 * ds * cosPhi * sinB * cosTheta +
          2.0 * M1 * Phip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Phip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi +
          2.0 * M3 * Phip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi -
          2.0 * Iyy3 * Psip * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * Iyy2 * Psip * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * Izz3 * Psip * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * Izz2 * Psip * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Psip * YB2 * ds * sinB * cosTheta * sinTheta -
          2.0 * M3 * Psip * YB3 * ds * sinB * cosTheta * sinTheta -
          2.0 * M2 * Psip * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta +
          2.0 * M3 * Psip * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * sinPhi * sinTheta -
          2.0 * Iyy3 * Phip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Iyy2 * Phip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * Izz3 * Phip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Izz2 * Phip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta +
          M2 * Thetap * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta +
          M3 * Psip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Psip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB * cosPhi * sinTheta +
          AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB * cosPhi * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi +
          AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi +
          AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosAlphaL * cosB * sinPhi * sinTheta -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosAlphaR * cosB * sinPhi * sinTheta -
          1.0 * M3 * Thetap * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi -
          1.0 * M2 * Thetap * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi +
          M3 * Thetap * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta +
          M2 * Thetap * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaL * sinB * cosTheta +
          AlphaRp * M2 * pow(ds, 2) * cosB * pow(cosPhi, 2) * sinAlphaR * sinB * cosTheta -
          2.0 * M3 * Psip * XB3 * ds * cosB * sinAlphaL * cosTheta * sinTheta -
          2.0 * M2 * Psip * XB2 * ds * cosB * sinAlphaR * cosTheta * sinTheta -
          1.0 * M3 * Thetap * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi -
          1.0 * M2 * Thetap * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi -
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * sinTheta +
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * sinTheta +
          2.0 * M2 * Phip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * M3 * Phip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M1 * Psip * YB1 * ZB1 * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Psip * YB2 * ZB2 * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * YB3 * ZB3 * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * pow(ds, 2) * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Psip * pow(ds, 2) * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Thetap * YB2 * ds * cosPhi * sinB * sinPhi * sinTheta -
          2.0 * M3 * Thetap * YB3 * ds * cosPhi * sinB * sinPhi * sinTheta +
          2.0 * M3 * Psip * XB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) +
          2.0 * M2 * Psip * XB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) +
          AlphaLp * M3 * YB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * cosTheta +
          AlphaRp * M2 * YB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * cosTheta +
          2.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta +
          2.0 * M3 * Psip * ZB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * M2 * Psip * ZB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * sinTheta +
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * sinTheta +
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M3 * Psip * YB3 * ds * cosB * sinAlphaL * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Psip * YB2 * ds * cosB * sinAlphaR * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Phip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi +
          2.0 * M2 * Phip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * ZB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPhi -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * cosTheta * sinPhi -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * cosTheta * sinPhi -
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * cosPhi * sinPhi * sinTheta -
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * cosPhi * sinPhi * sinTheta +
          2.0 * M2 * Psip * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta * sinTheta +
          2.0 * M2 * Psip * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi +
          2.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi -
          2.0 * Iyy3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * Iyy2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * Izz3 * Psip * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * Izz2 * Psip * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Psip * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Psip * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Psip * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Psip * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi * sinTheta,
      Ixz1 * Thetap * cosPhi - 1.0 * AlphaLp * Ixx3 * cosAlphaL * sinAlphaL -
          1.0 * AlphaRp * Ixx2 * cosAlphaR * sinAlphaR + AlphaLp * Iyy3 * cosAlphaL * sinAlphaL +
          AlphaRp * Iyy2 * cosAlphaR * sinAlphaR + Ixx1 * Thetap * cosTheta * sinTheta -
          1.0 * Iyy1 * Thetap * cosTheta * sinTheta + Iyy2 * Thetap * cosTheta * sinTheta +
          Iyy3 * Thetap * cosTheta * sinTheta - 1.0 * Izz2 * Thetap * cosTheta * sinTheta -
          1.0 * Izz3 * Thetap * cosTheta * sinTheta - 2.0 * Ixz1 * Thetap * cosPhi * pow(cosTheta, 2) -
          1.0 * Ixx3 * Thetap * cosAlphaL * cosPhi * sinAlphaL - 1.0 * Ixx2 * Thetap * cosAlphaR * cosPhi * sinAlphaR +
          Iyy3 * Thetap * cosAlphaL * cosPhi * sinAlphaL + Iyy2 * Thetap * cosAlphaR * cosPhi * sinAlphaR -
          1.0 * AlphaLp * Ixx3 * cosPhi * cosTheta * sinTheta - 1.0 * AlphaRp * Ixx2 * cosPhi * cosTheta * sinTheta +
          AlphaLp * Iyy3 * cosPhi * cosTheta * sinTheta + AlphaRp * Iyy2 * cosPhi * cosTheta * sinTheta -
          1.0 * M1 * Thetap * pow(XB1, 2) * cosTheta * sinTheta -
          1.0 * M2 * Thetap * pow(XB2, 2) * cosTheta * sinTheta -
          1.0 * M3 * Thetap * pow(XB3, 2) * cosTheta * sinTheta + M1 * Thetap * pow(YB1, 2) * cosTheta * sinTheta +
          M2 * Thetap * pow(YB2, 2) * cosTheta * sinTheta + M3 * Thetap * pow(YB3, 2) * cosTheta * sinTheta +
          Ixz1 * Phip * cosTheta * sinPhi * sinTheta + M2 * Thetap * pow(ds, 2) * cosTheta * sinTheta +
          M3 * Thetap * pow(ds, 2) * cosTheta * sinTheta - 1.0 * AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL +
          AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          AlphaLp * Ixx3 * cosAlphaL * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * Ixx2 * cosAlphaR * sinAlphaR * pow(cosTheta, 2) -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * sinAlphaL * pow(cosTheta, 2) -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * sinAlphaR * pow(cosTheta, 2) - 1.0 * M1 * Thetap * XB1 * ZB1 * cosPhi -
          1.0 * M2 * Thetap * XB2 * ZB2 * cosPhi - 1.0 * M3 * Thetap * XB3 * ZB3 * cosPhi -
          1.0 * M1 * Thetap * XB1 * YB1 * sinPhi - 1.0 * M2 * Thetap * XB2 * YB2 * sinPhi -
          1.0 * M3 * Thetap * XB3 * YB3 * sinPhi + Ixx3 * Thetap * pow(cosAlphaL, 2) * cosTheta * sinTheta +
          Ixx2 * Thetap * pow(cosAlphaR, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * cosTheta * sinTheta -
          2.0 * Iyy2 * Thetap * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * Iyy3 * Thetap * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * Izz2 * Thetap * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * Izz3 * Thetap * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Ixx2 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx3 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi + Iyy1 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz1 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi + Izz2 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi +
          Izz3 * Phip * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx2 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Ixx3 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy1 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz1 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta +
          Izz2 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta + Izz3 * Thetap * pow(cosPhi, 2) * cosTheta * sinTheta +
          M1 * Phip * YB1 * ZB1 * pow(cosTheta, 2) + M2 * Phip * YB2 * ZB2 * pow(cosTheta, 2) +
          M3 * Phip * YB3 * ZB3 * pow(cosTheta, 2) - 1.0 * AlphaLp * M3 * ZB3 * ds * cosB * sinAlphaL -
          1.0 * AlphaRp * M2 * ZB2 * ds * cosB * sinAlphaR -
          1.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR +
          Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL +
          Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * Iyy3 * Phip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) +
          Iyy2 * Phip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) +
          Izz3 * Phip * cosAlphaL * cosB * sinB * pow(cosTheta, 2) -
          1.0 * Izz2 * Phip * cosAlphaR * cosB * sinB * pow(cosTheta, 2) -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL -
          1.0 * AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR +
          2.0 * AlphaLp * Ixx3 * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Ixx3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * AlphaRp * Ixx2 * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Ixx2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Iyy3 * Thetap * cosAlphaL * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Iyy2 * Thetap * cosAlphaR * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          1.0 * AlphaLp * Iyy3 * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * AlphaRp * Iyy2 * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          AlphaLp * Izz3 * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          AlphaRp * Izz2 * pow(cosB, 2) * cosPhi * cosTheta * sinTheta - 1.0 * M2 * Thetap * XB2 * ds * sinB * sinPhi +
          M3 * Thetap * XB3 * ds * sinB * sinPhi - 1.0 * M1 * Phip * pow(YB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Phip * pow(YB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Phip * pow(YB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M1 * Phip * pow(ZB1, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Phip * pow(ZB2, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Phip * pow(ZB3, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          2.0 * M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * M1 * Thetap * pow(YB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Thetap * pow(YB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Thetap * pow(YB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M1 * Thetap * pow(ZB1, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Thetap * pow(ZB2, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Thetap * pow(ZB3, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Phip * pow(ds, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * M3 * Thetap * pow(ds, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) +
          AlphaLp * Ixx3 * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * Ixx2 * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * AlphaRp * Iyy2 * cosAlphaR * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M1 * Thetap * XB1 * ZB1 * cosPhi * pow(cosTheta, 2) +
          2.0 * M2 * Thetap * XB2 * ZB2 * cosPhi * pow(cosTheta, 2) +
          2.0 * M3 * Thetap * XB3 * ZB3 * cosPhi * pow(cosTheta, 2) +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          M2 * Phip * ZB2 * ds * sinB * pow(cosTheta, 2) - 1.0 * M3 * Phip * ZB3 * ds * sinB * pow(cosTheta, 2) +
          Ixx3 * Phip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Ixx2 * Phip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy3 * Phip * pow(cosAlphaL, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Iyy2 * Phip * pow(cosAlphaR, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy2 * Phip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy3 * Phip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Phip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Phip * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          2.0 * M1 * Thetap * XB1 * YB1 * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Thetap * XB2 * YB2 * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Thetap * XB3 * YB3 * pow(cosTheta, 2) * sinPhi +
          Ixx3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Ixx2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Thetap * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy3 * Thetap * cosB * sinAlphaL * sinB * sinPhi - 1.0 * Iyy2 * Thetap * cosB * sinAlphaR * sinB * sinPhi -
          1.0 * Izz3 * Thetap * cosB * sinAlphaL * sinB * sinPhi + Izz2 * Thetap * cosB * sinAlphaR * sinB * sinPhi -
          2.0 * M1 * Phip * YB1 * ZB1 * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M2 * Phip * YB2 * ZB2 * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M3 * Phip * YB3 * ZB3 * pow(cosPhi, 2) * pow(cosTheta, 2) +
          AlphaLp * M3 * XB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) +
          AlphaRp * M2 * XB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) +
          M3 * Phip * YB3 * ds * cosAlphaL * cosB * pow(cosTheta, 2) +
          M2 * Phip * YB2 * ds * cosAlphaR * cosB * pow(cosTheta, 2) +
          2.0 * Iyy3 * Phip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * Iyy2 * Phip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * Izz3 * Phip * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * Izz2 * Phip * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * AlphaLp * Iyy3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Iyy3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * AlphaRp * Iyy2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * Iyy2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) -
          2.0 * AlphaLp * Izz3 * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Izz3 * Thetap * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) -
          2.0 * AlphaRp * Izz2 * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          2.0 * Izz2 * Thetap * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * pow(cosTheta, 2) +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosTheta * sinTheta +
          2.0 * M2 * Thetap * XB2 * ds * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Thetap * XB3 * ds * sinB * pow(cosTheta, 2) * sinPhi +
          M2 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Phip * pow(ds, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M3 * Thetap * pow(ds, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          AlphaLp * Iyy3 * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * Iyy2 * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * AlphaLp * Izz3 * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) -
          1.0 * AlphaRp * Izz2 * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          Iyy3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          Iyy2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz3 * Phip * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Izz2 * Phip * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi -
          1.0 * Ixx3 * Phip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Ixx2 * Phip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta +
          Iyy3 * Phip * cosAlphaL * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Iyy2 * Phip * cosAlphaR * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M3 * Thetap * pow(ds, 2) * cosB * sinAlphaL * sinB * sinPhi -
          1.0 * M2 * Thetap * pow(ds, 2) * cosB * sinAlphaR * sinB * sinPhi +
          Iyy3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          Iyy2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz3 * Thetap * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * Izz2 * Thetap * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * M2 * Phip * ZB2 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * M3 * Phip * ZB3 * ds * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          1.0 * M3 * Thetap * XB3 * ds * cosAlphaL * cosB * cosPhi -
          1.0 * M2 * Thetap * XB2 * ds * cosAlphaR * cosB * cosPhi -
          1.0 * M3 * Thetap * ZB3 * ds * cosB * cosPhi * sinAlphaL -
          1.0 * M2 * Thetap * ZB2 * ds * cosB * cosPhi * sinAlphaR -
          1.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL -
          1.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * sinB * pow(cosTheta, 2) +
          M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * sinB * pow(cosTheta, 2) -
          1.0 * M3 * Thetap * YB3 * ds * cosB * sinAlphaL * sinPhi -
          1.0 * M2 * Thetap * YB2 * ds * cosB * sinAlphaR * sinPhi +
          M1 * Phip * XB1 * YB1 * cosPhi * cosTheta * sinTheta + M2 * Phip * XB2 * YB2 * cosPhi * cosTheta * sinTheta +
          M3 * Phip * XB3 * YB3 * cosPhi * cosTheta * sinTheta -
          2.0 * Iyy3 * Thetap * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * Iyy2 * Thetap * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * Izz3 * Thetap * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * Izz2 * Thetap * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta -
          1.0 * M1 * Phip * XB1 * ZB1 * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Phip * XB2 * ZB2 * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Phip * XB3 * ZB3 * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Thetap * YB2 * ds * sinB * cosTheta * sinTheta -
          2.0 * M3 * Thetap * YB3 * ds * sinB * cosTheta * sinTheta -
          2.0 * M2 * Thetap * YB2 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta +
          2.0 * M3 * Thetap * YB3 * ds * pow(cosPhi, 2) * sinB * cosTheta * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta +
          AlphaRp * Iyy2 * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta +
          AlphaLp * Izz3 * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * AlphaRp * Izz2 * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Phip * YB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) -
          2.0 * M2 * Phip * YB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * pow(cosTheta, 2) +
          AlphaLp * M3 * ZB3 * ds * cosB * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * M2 * ZB2 * ds * cosB * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) +
          AlphaLp * M3 * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaL * pow(cosTheta, 2) +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * pow(cosPhi, 2) * sinAlphaR * pow(cosTheta, 2) -
          1.0 * Iyy3 * Phip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta +
          Iyy2 * Phip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          Izz3 * Phip * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta -
          1.0 * Izz2 * Phip * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          M3 * Phip * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M2 * Phip * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * pow(cosTheta, 2) * sinPhi +
          M3 * Thetap * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta +
          M2 * Thetap * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * pow(cosPhi, 2) * cosTheta * sinTheta -
          1.0 * AlphaLp * Iyy3 * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          AlphaRp * Iyy2 * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          AlphaLp * Izz3 * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * AlphaRp * Izz2 * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          2.0 * M3 * Thetap * XB3 * ds * cosB * sinAlphaL * cosTheta * sinTheta -
          2.0 * M2 * Thetap * XB2 * ds * cosB * sinAlphaR * cosTheta * sinTheta -
          1.0 * Iyy3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * Iyy2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          Izz3 * Phip * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta +
          Izz2 * Phip * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          M2 * Phip * XB2 * ds * cosPhi * sinB * cosTheta * sinTheta -
          1.0 * M3 * Phip * XB3 * ds * cosPhi * sinB * cosTheta * sinTheta +
          2.0 * M1 * Thetap * YB1 * ZB1 * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Thetap * YB2 * ZB2 * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Thetap * YB3 * ZB3 * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Thetap * pow(ds, 2) * cosB * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Thetap * pow(ds, 2) * cosB * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Thetap * XB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) +
          2.0 * M2 * Thetap * XB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) +
          2.0 * M3 * Thetap * ZB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * M2 * Thetap * ZB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) -
          2.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * cosB * pow(cosPhi, 2) * sinB * pow(cosTheta, 2) +
          2.0 * AlphaLp * M3 * pow(ds, 2) * pow(cosAlphaL, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * cosPhi * sinAlphaL * pow(cosTheta, 2) +
          2.0 * AlphaRp * M2 * pow(ds, 2) * pow(cosAlphaR, 2) * pow(cosB, 2) * cosPhi * cosTheta * sinTheta +
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * cosPhi * sinAlphaR * pow(cosTheta, 2) +
          2.0 * M3 * Thetap * YB3 * ds * cosB * sinAlphaL * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Thetap * YB2 * ds * cosB * sinAlphaR * pow(cosTheta, 2) * sinPhi -
          2.0 * M2 * Phip * YB2 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Phip * YB3 * ds * cosPhi * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Phip * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * cosTheta * sinTheta +
          M2 * Phip * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * cosTheta * sinTheta +
          AlphaLp * M3 * ZB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinTheta +
          AlphaRp * M2 * ZB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinTheta -
          1.0 * AlphaLp * M3 * XB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta -
          1.0 * AlphaRp * M2 * XB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta +
          AlphaLp * M3 * YB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta +
          AlphaRp * M2 * YB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Phip * XB3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Phip * XB2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinTheta +
          M3 * Phip * YB3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinTheta +
          M2 * Phip * YB2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosB * cosPhi * sinAlphaL * sinB * pow(cosTheta, 2) * sinPhi +
          AlphaRp * M2 * pow(ds, 2) * cosB * cosPhi * sinAlphaR * sinB * pow(cosTheta, 2) * sinPhi -
          1.0 * M3 * Phip * ZB3 * ds * cosB * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Phip * ZB2 * ds * cosB * sinAlphaR * cosTheta * sinPhi * sinTheta -
          1.0 * M3 * Phip * pow(ds, 2) * cosAlphaL * pow(cosB, 2) * sinAlphaL * cosTheta * sinPhi * sinTheta -
          1.0 * M2 * Phip * pow(ds, 2) * cosAlphaR * pow(cosB, 2) * sinAlphaR * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Thetap * ZB2 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Thetap * ZB3 * ds * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          AlphaLp * M3 * YB3 * ds * cosB * cosPhi * sinAlphaL * pow(cosTheta, 2) * sinPhi +
          AlphaRp * M2 * YB2 * ds * cosB * cosPhi * sinAlphaR * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Phip * ZB3 * ds * cosAlphaL * cosB * cosPhi * pow(cosTheta, 2) * sinPhi +
          2.0 * M2 * Phip * ZB2 * ds * cosAlphaR * cosB * cosPhi * pow(cosTheta, 2) * sinPhi +
          2.0 * M3 * Thetap * ZB3 * ds * cosAlphaL * cosB * pow(cosPhi, 2) * cosTheta * sinTheta +
          2.0 * M2 * Thetap * ZB2 * ds * cosAlphaR * cosB * pow(cosPhi, 2) * cosTheta * sinTheta -
          2.0 * Iyy3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * Iyy2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * Izz3 * Thetap * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          2.0 * Izz2 * Thetap * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * pow(ds, 2) * cosAlphaL * cosB * sinB * cosTheta * sinPhi * sinTheta +
          AlphaRp * M2 * pow(ds, 2) * cosAlphaR * cosB * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M3 * Thetap * YB3 * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Thetap * YB2 * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPhi * sinTheta -
          2.0 * M3 * Thetap * pow(ds, 2) * cosAlphaL * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta +
          2.0 * M2 * Thetap * pow(ds, 2) * cosAlphaR * cosB * cosPhi * sinB * cosTheta * sinPhi * sinTheta,
      0, 0, 0,
      -AlphaRp * (1.0 * M2 * ds * cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                  1.0 * M2 * ds * cosB * cosPsi * sinAlphaR * cosTheta) -
          Psip * (1.0 * M2 * ds * cosAlphaR * cosB * cosTheta * sinPsi +
                  1.0 * M2 * ds * cosB * cosPsi * sinAlphaR * sinPhi -
                  M2 * ds * cosB * cosPhi * sinAlphaR * sinPsi * sinTheta) -
          Thetap * (1.0 * M2 * ds * cosAlphaR * cosB * cosPsi * sinTheta +
                    1.0 * M2 * ds * cosB * cosPhi * cosPsi * sinAlphaR * cosTheta) -
          1.0 * M2 * Phip * ds * cosB * sinAlphaR * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta),
      -AlphaLp * (1.0 * M3 * ds * cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                  1.0 * M3 * ds * cosB * cosPsi * sinAlphaL * cosTheta) -
          Psip * (1.0 * M3 * ds * cosAlphaL * cosB * cosTheta * sinPsi +
                  1.0 * M3 * ds * cosB * cosPsi * sinAlphaL * sinPhi -
                  M3 * ds * cosB * cosPhi * sinAlphaL * sinPsi * sinTheta) -
          Thetap * (1.0 * M3 * ds * cosAlphaL * cosB * cosPsi * sinTheta +
                    1.0 * M3 * ds * cosB * cosPhi * cosPsi * sinAlphaL * cosTheta) -
          1.0 * M3 * Phip * ds * cosB * sinAlphaL * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta),
      Phip * (M1 * (YB1 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) -
                    1.0 * ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
              M2 * (ds * (sinB * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) -
                          1.0 * cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
                    YB2 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) -
                    1.0 * ZB2 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) -
              1.0 * M3 *
                  (ds * (sinB * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) -
                   1.0 * YB3 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) +
                   ZB3 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))) -
          Thetap * (1.0 * M1 * ZB1 * cosPsi * cosTheta * sinPhi - M2 * YB2 * cosPhi * cosPsi * cosTheta -
                    M3 * YB3 * cosPhi * cosPsi * cosTheta - M1 * YB1 * cosPhi * cosPsi * cosTheta +
                    1.0 * M2 * ZB2 * cosPsi * cosTheta * sinPhi + 1.0 * M3 * ZB3 * cosPsi * cosTheta * sinPhi -
                    M2 * ds * cosPhi * cosPsi * sinB * cosTheta + 1.0 * M3 * ds * cosPhi * cosPsi * sinB * cosTheta +
                    1.0 * M3 * ds * cosAlphaL * cosB * cosPsi * cosTheta * sinPhi +
                    1.0 * M2 * ds * cosAlphaR * cosB * cosPsi * cosTheta * sinPhi) +
          Psip * (M2 * (ds * (sinB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) +
                              cosAlphaR * cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) +
                        YB2 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) +
                        ZB2 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) +
                  M3 * (YB3 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                        1.0 * ds *
                            (sinB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                             1.0 * cosAlphaL * cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) +
                        ZB3 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) +
                  M1 * (YB1 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) +
                        ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) -
          1.0 * AlphaLp * M3 * ds * cosB * sinAlphaL * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) -
          1.0 * AlphaRp * M2 * ds * cosB * sinAlphaR * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta),
      M1 * Psip * XB1 * sinPsi * sinTheta - 1.0 * M2 * Thetap * XB2 * cosPsi * cosTheta -
          1.0 * M3 * Thetap * XB3 * cosPsi * cosTheta - 1.0 * M1 * Thetap * XB1 * cosPsi * cosTheta +
          M2 * Psip * XB2 * sinPsi * sinTheta + M3 * Psip * XB3 * sinPsi * sinTheta +
          M1 * Phip * YB1 * cosPhi * cosPsi * cosTheta + M2 * Phip * YB2 * cosPhi * cosPsi * cosTheta +
          M3 * Phip * YB3 * cosPhi * cosPsi * cosTheta - 1.0 * M1 * Phip * ZB1 * cosPsi * cosTheta * sinPhi -
          1.0 * M2 * Phip * ZB2 * cosPsi * cosTheta * sinPhi - 1.0 * M3 * Phip * ZB3 * cosPsi * cosTheta * sinPhi -
          1.0 * M1 * Psip * ZB1 * cosPhi * cosTheta * sinPsi - 1.0 * M2 * Psip * ZB2 * cosPhi * cosTheta * sinPsi -
          1.0 * M3 * Psip * ZB3 * cosPhi * cosTheta * sinPsi - 1.0 * M1 * Thetap * ZB1 * cosPhi * cosPsi * sinTheta -
          1.0 * M2 * Thetap * ZB2 * cosPhi * cosPsi * sinTheta - 1.0 * M3 * Thetap * ZB3 * cosPhi * cosPsi * sinTheta -
          1.0 * M1 * Psip * YB1 * cosTheta * sinPhi * sinPsi - 1.0 * M2 * Psip * YB2 * cosTheta * sinPhi * sinPsi -
          1.0 * M3 * Psip * YB3 * cosTheta * sinPhi * sinPsi - 1.0 * M1 * Thetap * YB1 * cosPsi * sinPhi * sinTheta -
          1.0 * M2 * Thetap * YB2 * cosPsi * sinPhi * sinTheta - 1.0 * M3 * Thetap * YB3 * cosPsi * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * ds * cosAlphaL * cosB * cosPsi * sinTheta -
          1.0 * AlphaRp * M2 * ds * cosAlphaR * cosB * cosPsi * sinTheta -
          1.0 * M3 * Thetap * ds * cosB * cosPsi * sinAlphaL * cosTheta -
          1.0 * M2 * Thetap * ds * cosB * cosPsi * sinAlphaR * cosTheta +
          M2 * Phip * ds * cosPhi * cosPsi * sinB * cosTheta -
          1.0 * M3 * Phip * ds * cosPhi * cosPsi * sinB * cosTheta +
          M3 * Psip * ds * cosB * sinAlphaL * sinPsi * sinTheta +
          M2 * Psip * ds * cosB * sinAlphaR * sinPsi * sinTheta -
          1.0 * M2 * Psip * ds * sinB * cosTheta * sinPhi * sinPsi +
          M3 * Psip * ds * sinB * cosTheta * sinPhi * sinPsi -
          1.0 * M2 * Thetap * ds * cosPsi * sinB * sinPhi * sinTheta +
          M3 * Thetap * ds * cosPsi * sinB * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * ds * cosB * cosPhi * cosPsi * sinAlphaL * cosTheta -
          1.0 * AlphaRp * M2 * ds * cosB * cosPhi * cosPsi * sinAlphaR * cosTheta -
          1.0 * M3 * Phip * ds * cosAlphaL * cosB * cosPsi * cosTheta * sinPhi -
          1.0 * M2 * Phip * ds * cosAlphaR * cosB * cosPsi * cosTheta * sinPhi -
          1.0 * M3 * Psip * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPsi -
          1.0 * M2 * Psip * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPsi -
          1.0 * M3 * Thetap * ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta -
          1.0 * M2 * Thetap * ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta,
      M1 * Phip * ZB1 * cosPhi * cosPsi + M2 * Phip * ZB2 * cosPhi * cosPsi + M3 * Phip * ZB3 * cosPhi * cosPsi -
          1.0 * M1 * Psip * XB1 * cosPsi * cosTheta - 1.0 * M2 * Psip * XB2 * cosPsi * cosTheta -
          1.0 * M3 * Psip * XB3 * cosPsi * cosTheta + M1 * Phip * YB1 * cosPsi * sinPhi +
          M2 * Phip * YB2 * cosPsi * sinPhi + M3 * Phip * YB3 * cosPsi * sinPhi + M1 * Psip * YB1 * cosPhi * sinPsi +
          M2 * Psip * YB2 * cosPhi * sinPsi + M3 * Psip * YB3 * cosPhi * sinPsi -
          1.0 * M1 * Psip * ZB1 * sinPhi * sinPsi - 1.0 * M2 * Psip * ZB2 * sinPhi * sinPsi -
          1.0 * M3 * Psip * ZB3 * sinPhi * sinPsi + M1 * Thetap * XB1 * sinPsi * sinTheta +
          M2 * Thetap * XB2 * sinPsi * sinTheta + M3 * Thetap * XB3 * sinPsi * sinTheta -
          1.0 * M1 * Psip * ZB1 * cosPhi * cosPsi * sinTheta - 1.0 * M2 * Psip * ZB2 * cosPhi * cosPsi * sinTheta -
          1.0 * M3 * Psip * ZB3 * cosPhi * cosPsi * sinTheta - 1.0 * M1 * Thetap * ZB1 * cosPhi * cosTheta * sinPsi -
          1.0 * M2 * Thetap * ZB2 * cosPhi * cosTheta * sinPsi - 1.0 * M3 * Thetap * ZB3 * cosPhi * cosTheta * sinPsi +
          M2 * Phip * ds * cosPsi * sinB * sinPhi - 1.0 * M3 * Phip * ds * cosPsi * sinB * sinPhi +
          M2 * Psip * ds * cosPhi * sinB * sinPsi - 1.0 * M3 * Psip * ds * cosPhi * sinB * sinPsi -
          1.0 * M1 * Phip * YB1 * cosPhi * sinPsi * sinTheta - 1.0 * M2 * Phip * YB2 * cosPhi * sinPsi * sinTheta -
          1.0 * M3 * Phip * YB3 * cosPhi * sinPsi * sinTheta - 1.0 * M1 * Psip * YB1 * cosPsi * sinPhi * sinTheta -
          1.0 * M2 * Psip * YB2 * cosPsi * sinPhi * sinTheta - 1.0 * M3 * Psip * YB3 * cosPsi * sinPhi * sinTheta -
          1.0 * M1 * Thetap * YB1 * cosTheta * sinPhi * sinPsi - 1.0 * M2 * Thetap * YB2 * cosTheta * sinPhi * sinPsi -
          1.0 * M3 * Thetap * YB3 * cosTheta * sinPhi * sinPsi + M1 * Phip * ZB1 * sinPhi * sinPsi * sinTheta +
          M2 * Phip * ZB2 * sinPhi * sinPsi * sinTheta + M3 * Phip * ZB3 * sinPhi * sinPsi * sinTheta +
          M3 * Phip * ds * cosAlphaL * cosB * cosPhi * cosPsi + M2 * Phip * ds * cosAlphaR * cosB * cosPhi * cosPsi -
          1.0 * AlphaLp * M3 * ds * cosAlphaL * cosB * cosTheta * sinPsi -
          1.0 * AlphaRp * M2 * ds * cosAlphaR * cosB * cosTheta * sinPsi -
          1.0 * AlphaLp * M3 * ds * cosB * cosPsi * sinAlphaL * sinPhi -
          1.0 * AlphaRp * M2 * ds * cosB * cosPsi * sinAlphaR * sinPhi -
          1.0 * M3 * Psip * ds * cosB * cosPsi * sinAlphaL * cosTheta -
          1.0 * M2 * Psip * ds * cosB * cosPsi * sinAlphaR * cosTheta -
          1.0 * M3 * Psip * ds * cosAlphaL * cosB * sinPhi * sinPsi -
          1.0 * M2 * Psip * ds * cosAlphaR * cosB * sinPhi * sinPsi +
          M3 * Thetap * ds * cosB * sinAlphaL * sinPsi * sinTheta +
          M2 * Thetap * ds * cosB * sinAlphaR * sinPsi * sinTheta -
          1.0 * M2 * Phip * ds * cosPhi * sinB * sinPsi * sinTheta +
          M3 * Phip * ds * cosPhi * sinB * sinPsi * sinTheta -
          1.0 * M2 * Psip * ds * cosPsi * sinB * sinPhi * sinTheta +
          M3 * Psip * ds * cosPsi * sinB * sinPhi * sinTheta -
          1.0 * M2 * Thetap * ds * sinB * cosTheta * sinPhi * sinPsi +
          M3 * Thetap * ds * sinB * cosTheta * sinPhi * sinPsi -
          1.0 * M3 * Psip * ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta -
          1.0 * M2 * Psip * ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta -
          1.0 * M3 * Thetap * ds * cosAlphaL * cosB * cosPhi * cosTheta * sinPsi -
          1.0 * M2 * Thetap * ds * cosAlphaR * cosB * cosPhi * cosTheta * sinPsi +
          AlphaLp * M3 * ds * cosB * cosPhi * sinAlphaL * sinPsi * sinTheta +
          AlphaRp * M2 * ds * cosB * cosPhi * sinAlphaR * sinPsi * sinTheta +
          M3 * Phip * ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta +
          M2 * Phip * ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta,
      0, 0, 0,
      AlphaRp * (M2 * ds * cosAlphaR * cosB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                 1.0 * M2 * ds * cosB * sinAlphaR * cosTheta * sinPsi) -
          Thetap * (1.0 * M2 * ds * cosAlphaR * cosB * sinPsi * sinTheta +
                    1.0 * M2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPsi) -
          Psip * (1.0 * M2 * ds * cosB * sinAlphaR * sinPhi * sinPsi - M2 * ds * cosAlphaR * cosB * cosPsi * cosTheta +
                  1.0 * M2 * ds * cosB * cosPhi * cosPsi * sinAlphaR * sinTheta) +
          M2 * Phip * ds * cosB * sinAlphaR * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta),
      AlphaLp * (M3 * ds * cosAlphaL * cosB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                 1.0 * M3 * ds * cosB * sinAlphaL * cosTheta * sinPsi) -
          Thetap * (1.0 * M3 * ds * cosAlphaL * cosB * sinPsi * sinTheta +
                    1.0 * M3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPsi) -
          Psip * (1.0 * M3 * ds * cosB * sinAlphaL * sinPhi * sinPsi - M3 * ds * cosAlphaL * cosB * cosPsi * cosTheta +
                  1.0 * M3 * ds * cosB * cosPhi * cosPsi * sinAlphaL * sinTheta) +
          M3 * Phip * ds * cosB * sinAlphaL * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta),
      Phip * (1.0 * M1 *
                  (1.0 * ZB1 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                   YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) -
              1.0 * M2 *
                  (ds * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         1.0 * cosAlphaR * cosB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta)) -
                   1.0 * ZB2 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) +
                   YB2 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) +
              M3 * (ds * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                          cosAlphaL * cosB * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta)) +
                    ZB3 * (cosPsi * sinPhi - 1.0 * cosPhi * sinPsi * sinTheta) -
                    1.0 * YB3 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) -
          Thetap * (1.0 * M1 * ZB1 * cosTheta * sinPhi * sinPsi - M2 * YB2 * cosPhi * cosTheta * sinPsi -
                    M3 * YB3 * cosPhi * cosTheta * sinPsi - M1 * YB1 * cosPhi * cosTheta * sinPsi +
                    1.0 * M2 * ZB2 * cosTheta * sinPhi * sinPsi + 1.0 * M3 * ZB3 * cosTheta * sinPhi * sinPsi -
                    M2 * ds * cosPhi * sinB * cosTheta * sinPsi + 1.0 * M3 * ds * cosPhi * sinB * cosTheta * sinPsi +
                    1.0 * M3 * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinPsi +
                    1.0 * M2 * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinPsi) +
          Psip * (M2 * (ds * (sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosAlphaR * cosB * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta)) +
                        ZB2 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) +
                        YB2 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
                  M3 * (ZB3 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) -
                        1.0 * ds *
                            (sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                             1.0 * cosAlphaL * cosB * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta)) +
                        YB3 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
                  M1 * (ZB1 * (cosPhi * sinPsi - 1.0 * cosPsi * sinPhi * sinTheta) +
                        YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))) +
          AlphaLp * M3 * ds * cosB * sinAlphaL * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
          AlphaRp * M2 * ds * cosB * sinAlphaR * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta),
      M1 * Psip * ZB1 * cosPhi * cosPsi * cosTheta - 1.0 * M2 * Psip * XB2 * cosPsi * sinTheta -
          1.0 * M3 * Psip * XB3 * cosPsi * sinTheta - 1.0 * M1 * Thetap * XB1 * cosTheta * sinPsi -
          1.0 * M2 * Thetap * XB2 * cosTheta * sinPsi - 1.0 * M3 * Thetap * XB3 * cosTheta * sinPsi -
          1.0 * M1 * Psip * XB1 * cosPsi * sinTheta + M2 * Psip * ZB2 * cosPhi * cosPsi * cosTheta +
          M3 * Psip * ZB3 * cosPhi * cosPsi * cosTheta + M1 * Phip * YB1 * cosPhi * cosTheta * sinPsi +
          M2 * Phip * YB2 * cosPhi * cosTheta * sinPsi + M3 * Phip * YB3 * cosPhi * cosTheta * sinPsi +
          M1 * Psip * YB1 * cosPsi * cosTheta * sinPhi + M2 * Psip * YB2 * cosPsi * cosTheta * sinPhi +
          M3 * Psip * YB3 * cosPsi * cosTheta * sinPhi - 1.0 * M1 * Phip * ZB1 * cosTheta * sinPhi * sinPsi -
          1.0 * M2 * Phip * ZB2 * cosTheta * sinPhi * sinPsi - 1.0 * M3 * Phip * ZB3 * cosTheta * sinPhi * sinPsi -
          1.0 * M1 * Thetap * ZB1 * cosPhi * sinPsi * sinTheta - 1.0 * M2 * Thetap * ZB2 * cosPhi * sinPsi * sinTheta -
          1.0 * M3 * Thetap * ZB3 * cosPhi * sinPsi * sinTheta - 1.0 * M1 * Thetap * YB1 * sinPhi * sinPsi * sinTheta -
          1.0 * M2 * Thetap * YB2 * sinPhi * sinPsi * sinTheta - 1.0 * M3 * Thetap * YB3 * sinPhi * sinPsi * sinTheta -
          1.0 * AlphaLp * M3 * ds * cosAlphaL * cosB * sinPsi * sinTheta -
          1.0 * AlphaRp * M2 * ds * cosAlphaR * cosB * sinPsi * sinTheta -
          1.0 * M3 * Psip * ds * cosB * cosPsi * sinAlphaL * sinTheta -
          1.0 * M2 * Psip * ds * cosB * cosPsi * sinAlphaR * sinTheta -
          1.0 * M3 * Thetap * ds * cosB * sinAlphaL * cosTheta * sinPsi -
          1.0 * M2 * Thetap * ds * cosB * sinAlphaR * cosTheta * sinPsi +
          M2 * Phip * ds * cosPhi * sinB * cosTheta * sinPsi -
          1.0 * M3 * Phip * ds * cosPhi * sinB * cosTheta * sinPsi +
          M2 * Psip * ds * cosPsi * sinB * cosTheta * sinPhi -
          1.0 * M3 * Psip * ds * cosPsi * sinB * cosTheta * sinPhi -
          1.0 * M2 * Thetap * ds * sinB * sinPhi * sinPsi * sinTheta +
          M3 * Thetap * ds * sinB * sinPhi * sinPsi * sinTheta +
          M3 * Psip * ds * cosAlphaL * cosB * cosPhi * cosPsi * cosTheta +
          M2 * Psip * ds * cosAlphaR * cosB * cosPhi * cosPsi * cosTheta -
          1.0 * AlphaLp * M3 * ds * cosB * cosPhi * sinAlphaL * cosTheta * sinPsi -
          1.0 * AlphaRp * M2 * ds * cosB * cosPhi * sinAlphaR * cosTheta * sinPsi -
          1.0 * M3 * Phip * ds * cosAlphaL * cosB * cosTheta * sinPhi * sinPsi -
          1.0 * M2 * Phip * ds * cosAlphaR * cosB * cosTheta * sinPhi * sinPsi -
          1.0 * M3 * Thetap * ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta -
          1.0 * M2 * Thetap * ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta,
      M1 * Phip * ZB1 * cosPhi * sinPsi - 1.0 * M2 * Psip * YB2 * cosPhi * cosPsi -
          1.0 * M3 * Psip * YB3 * cosPhi * cosPsi - 1.0 * M1 * Psip * YB1 * cosPhi * cosPsi +
          M2 * Phip * ZB2 * cosPhi * sinPsi + M3 * Phip * ZB3 * cosPhi * sinPsi + M1 * Psip * ZB1 * cosPsi * sinPhi +
          M2 * Psip * ZB2 * cosPsi * sinPhi + M3 * Psip * ZB3 * cosPsi * sinPhi -
          1.0 * M1 * Psip * XB1 * cosTheta * sinPsi - 1.0 * M2 * Psip * XB2 * cosTheta * sinPsi -
          1.0 * M3 * Psip * XB3 * cosTheta * sinPsi - 1.0 * M1 * Thetap * XB1 * cosPsi * sinTheta -
          1.0 * M2 * Thetap * XB2 * cosPsi * sinTheta - 1.0 * M3 * Thetap * XB3 * cosPsi * sinTheta +
          M1 * Phip * YB1 * sinPhi * sinPsi + M2 * Phip * YB2 * sinPhi * sinPsi + M3 * Phip * YB3 * sinPhi * sinPsi +
          M1 * Thetap * ZB1 * cosPhi * cosPsi * cosTheta + M2 * Thetap * ZB2 * cosPhi * cosPsi * cosTheta +
          M3 * Thetap * ZB3 * cosPhi * cosPsi * cosTheta - 1.0 * M2 * Psip * ds * cosPhi * cosPsi * sinB +
          M3 * Psip * ds * cosPhi * cosPsi * sinB + M1 * Phip * YB1 * cosPhi * cosPsi * sinTheta +
          M2 * Phip * YB2 * cosPhi * cosPsi * sinTheta + M3 * Phip * YB3 * cosPhi * cosPsi * sinTheta +
          M1 * Thetap * YB1 * cosPsi * cosTheta * sinPhi + M2 * Thetap * YB2 * cosPsi * cosTheta * sinPhi +
          M3 * Thetap * YB3 * cosPsi * cosTheta * sinPhi - 1.0 * M1 * Phip * ZB1 * cosPsi * sinPhi * sinTheta -
          1.0 * M2 * Phip * ZB2 * cosPsi * sinPhi * sinTheta - 1.0 * M3 * Phip * ZB3 * cosPsi * sinPhi * sinTheta -
          1.0 * M1 * Psip * ZB1 * cosPhi * sinPsi * sinTheta - 1.0 * M2 * Psip * ZB2 * cosPhi * sinPsi * sinTheta -
          1.0 * M3 * Psip * ZB3 * cosPhi * sinPsi * sinTheta + M2 * Phip * ds * sinB * sinPhi * sinPsi -
          1.0 * M3 * Phip * ds * sinB * sinPhi * sinPsi - 1.0 * M1 * Psip * YB1 * sinPhi * sinPsi * sinTheta -
          1.0 * M2 * Psip * YB2 * sinPhi * sinPsi * sinTheta - 1.0 * M3 * Psip * YB3 * sinPhi * sinPsi * sinTheta +
          AlphaLp * M3 * ds * cosAlphaL * cosB * cosPsi * cosTheta +
          AlphaRp * M2 * ds * cosAlphaR * cosB * cosPsi * cosTheta +
          M3 * Phip * ds * cosAlphaL * cosB * cosPhi * sinPsi + M2 * Phip * ds * cosAlphaR * cosB * cosPhi * sinPsi +
          M3 * Psip * ds * cosAlphaL * cosB * cosPsi * sinPhi + M2 * Psip * ds * cosAlphaR * cosB * cosPsi * sinPhi -
          1.0 * AlphaLp * M3 * ds * cosB * sinAlphaL * sinPhi * sinPsi -
          1.0 * AlphaRp * M2 * ds * cosB * sinAlphaR * sinPhi * sinPsi -
          1.0 * M3 * Psip * ds * cosB * sinAlphaL * cosTheta * sinPsi -
          1.0 * M2 * Psip * ds * cosB * sinAlphaR * cosTheta * sinPsi -
          1.0 * M3 * Thetap * ds * cosB * cosPsi * sinAlphaL * sinTheta -
          1.0 * M2 * Thetap * ds * cosB * cosPsi * sinAlphaR * sinTheta +
          M2 * Phip * ds * cosPhi * cosPsi * sinB * sinTheta -
          1.0 * M3 * Phip * ds * cosPhi * cosPsi * sinB * sinTheta +
          M2 * Thetap * ds * cosPsi * sinB * cosTheta * sinPhi -
          1.0 * M3 * Thetap * ds * cosPsi * sinB * cosTheta * sinPhi -
          1.0 * M2 * Psip * ds * sinB * sinPhi * sinPsi * sinTheta +
          M3 * Psip * ds * sinB * sinPhi * sinPsi * sinTheta +
          M3 * Thetap * ds * cosAlphaL * cosB * cosPhi * cosPsi * cosTheta +
          M2 * Thetap * ds * cosAlphaR * cosB * cosPhi * cosPsi * cosTheta -
          1.0 * AlphaLp * M3 * ds * cosB * cosPhi * cosPsi * sinAlphaL * sinTheta -
          1.0 * AlphaRp * M2 * ds * cosB * cosPhi * cosPsi * sinAlphaR * sinTheta -
          1.0 * M3 * Phip * ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta -
          1.0 * M2 * Phip * ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta -
          1.0 * M3 * Psip * ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta -
          1.0 * M2 * Psip * ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta,
      0, 0, 0,
      M2 * ds * cosB *
          (AlphaRp * sinAlphaR * sinTheta - 1.0 * Thetap * cosAlphaR * cosTheta -
           1.0 * AlphaRp * cosAlphaR * cosPhi * cosTheta + Phip * sinAlphaR * cosTheta * sinPhi +
           Thetap * cosPhi * sinAlphaR * sinTheta),
      M3 * ds * cosB *
          (AlphaLp * sinAlphaL * sinTheta - 1.0 * Thetap * cosAlphaL * cosTheta -
           1.0 * AlphaLp * cosAlphaL * cosPhi * cosTheta + Phip * sinAlphaL * cosTheta * sinPhi +
           Thetap * cosPhi * sinAlphaL * sinTheta),
      Thetap * (M1 * ZB1 * sinPhi * sinTheta - 1.0 * M2 * YB2 * cosPhi * sinTheta - 1.0 * M3 * YB3 * cosPhi * sinTheta -
                1.0 * M1 * YB1 * cosPhi * sinTheta + M2 * ZB2 * sinPhi * sinTheta + M3 * ZB3 * sinPhi * sinTheta -
                1.0 * M2 * ds * cosPhi * sinB * sinTheta + M3 * ds * cosPhi * sinB * sinTheta +
                M3 * ds * cosAlphaL * cosB * sinPhi * sinTheta + M2 * ds * cosAlphaR * cosB * sinPhi * sinTheta) -
          Phip * (1.0 * M3 * cosTheta *
                      (ZB3 * cosPhi + YB3 * sinPhi - 1.0 * ds * sinB * sinPhi + ds * cosAlphaL * cosB * cosPhi) +
                  1.0 * M2 * cosTheta *
                      (ZB2 * cosPhi + YB2 * sinPhi + ds * sinB * sinPhi + ds * cosAlphaR * cosB * cosPhi) +
                  1.0 * M1 * cosTheta * (ZB1 * cosPhi + YB1 * sinPhi)) +
          AlphaLp * M3 * ds * cosB * sinAlphaL * cosTheta * sinPhi +
          AlphaRp * M2 * ds * cosB * sinAlphaR * cosTheta * sinPhi,
      M1 * Thetap * XB1 * sinTheta + M2 * Thetap * XB2 * sinTheta + M3 * Thetap * XB3 * sinTheta -
          1.0 * M1 * Thetap * ZB1 * cosPhi * cosTheta - 1.0 * M2 * Thetap * ZB2 * cosPhi * cosTheta -
          1.0 * M3 * Thetap * ZB3 * cosPhi * cosTheta - 1.0 * M1 * Phip * YB1 * cosPhi * sinTheta -
          1.0 * M2 * Phip * YB2 * cosPhi * sinTheta - 1.0 * M3 * Phip * YB3 * cosPhi * sinTheta -
          1.0 * M1 * Thetap * YB1 * cosTheta * sinPhi - 1.0 * M2 * Thetap * YB2 * cosTheta * sinPhi -
          1.0 * M3 * Thetap * YB3 * cosTheta * sinPhi + M1 * Phip * ZB1 * sinPhi * sinTheta +
          M2 * Phip * ZB2 * sinPhi * sinTheta + M3 * Phip * ZB3 * sinPhi * sinTheta -
          1.0 * AlphaLp * M3 * ds * cosAlphaL * cosB * cosTheta -
          1.0 * AlphaRp * M2 * ds * cosAlphaR * cosB * cosTheta + M3 * Thetap * ds * cosB * sinAlphaL * sinTheta +
          M2 * Thetap * ds * cosB * sinAlphaR * sinTheta - 1.0 * M2 * Phip * ds * cosPhi * sinB * sinTheta +
          M3 * Phip * ds * cosPhi * sinB * sinTheta - 1.0 * M2 * Thetap * ds * sinB * cosTheta * sinPhi +
          M3 * Thetap * ds * sinB * cosTheta * sinPhi - 1.0 * M3 * Thetap * ds * cosAlphaL * cosB * cosPhi * cosTheta -
          1.0 * M2 * Thetap * ds * cosAlphaR * cosB * cosPhi * cosTheta +
          AlphaLp * M3 * ds * cosB * cosPhi * sinAlphaL * sinTheta +
          AlphaRp * M2 * ds * cosB * cosPhi * sinAlphaR * sinTheta +
          M3 * Phip * ds * cosAlphaL * cosB * sinPhi * sinTheta + M2 * Phip * ds * cosAlphaR * cosB * sinPhi * sinTheta,
      0, 0, 0, 0;

  return C;
}
