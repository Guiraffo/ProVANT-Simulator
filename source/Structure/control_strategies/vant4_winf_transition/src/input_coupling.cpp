/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the implementation of the function that returns the
 * input coupling matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#include "vant4_winf_transition/input_coupling.h"

#include "vant4_winf_transition/uav4_parameters.h"

#include <cmath>

Eigen::MatrixXd InputCouplingMatrix(Eigen::VectorXd q)
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

  Eigen::MatrixXd B1(8, 2);
  Eigen::MatrixXd B2(8, 2);
  Eigen::MatrixXd BInputCoupling(8, 4);
  Eigen::MatrixXd JvL(3, 8);
  Eigen::MatrixXd JvR(3, 8);
  Eigen::MatrixXd JwR(3, 8);
  Eigen::MatrixXd JwL(3, 8);
  Eigen::VectorXd az(3);
  Eigen::MatrixXd RIC2(3, 3);
  Eigen::MatrixXd RIC3(3, 3);

  RIC2 << cos(AlphaR) * cos(Psi) * cos(Theta) - sin(AlphaR) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)),
      -sin(B) * (cos(AlphaR) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)) +
                 cos(Psi) * sin(AlphaR) * cos(Theta)) -
          cos(B) * (cos(Phi) * sin(Psi) - cos(Psi) * sin(Phi) * sin(Theta)),
      cos(B) * (cos(AlphaR) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)) +
                cos(Psi) * sin(AlphaR) * cos(Theta)) -
          sin(B) * (cos(Phi) * sin(Psi) - cos(Psi) * sin(Phi) * sin(Theta)),
      sin(AlphaR) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) + cos(AlphaR) * cos(Theta) * sin(Psi),
      cos(B) * (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)) +
          sin(B) * (cos(AlphaR) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) -
                    sin(AlphaR) * cos(Theta) * sin(Psi)),
      sin(B) * (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)) -
          cos(B) * (cos(AlphaR) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) -
                    sin(AlphaR) * cos(Theta) * sin(Psi)),
      -cos(AlphaR) * sin(Theta) - cos(Phi) * sin(AlphaR) * cos(Theta),
      sin(B) * (sin(AlphaR) * sin(Theta) - cos(AlphaR) * cos(Phi) * cos(Theta)) + cos(B) * cos(Theta) * sin(Phi),
      sin(B) * cos(Theta) * sin(Phi) - cos(B) * (sin(AlphaR) * sin(Theta) - cos(AlphaR) * cos(Phi) * cos(Theta));

  RIC3 << cos(AlphaL) * cos(Psi) * cos(Theta) - sin(AlphaL) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)),
      sin(B) * (cos(AlphaL) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)) +
                cos(Psi) * sin(AlphaL) * cos(Theta)) -
          cos(B) * (cos(Phi) * sin(Psi) - cos(Psi) * sin(Phi) * sin(Theta)),
      cos(B) * (cos(AlphaL) * (sin(Phi) * sin(Psi) + cos(Phi) * cos(Psi) * sin(Theta)) +
                cos(Psi) * sin(AlphaL) * cos(Theta)) +
          sin(B) * (cos(Phi) * sin(Psi) - cos(Psi) * sin(Phi) * sin(Theta)),
      sin(AlphaL) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) + cos(AlphaL) * cos(Theta) * sin(Psi),
      cos(B) * (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)) -
          sin(B) * (cos(AlphaL) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) -
                    sin(AlphaL) * cos(Theta) * sin(Psi)),
      -cos(B) * (cos(AlphaL) * (cos(Psi) * sin(Phi) - cos(Phi) * sin(Psi) * sin(Theta)) -
                 sin(AlphaL) * cos(Theta) * sin(Psi)) -
          sin(B) * (cos(Phi) * cos(Psi) + sin(Phi) * sin(Psi) * sin(Theta)),
      -cos(AlphaL) * sin(Theta) - cos(Phi) * sin(AlphaL) * cos(Theta),
      cos(B) * cos(Theta) * sin(Phi) - sin(B) * (sin(AlphaL) * sin(Theta) - cos(AlphaL) * cos(Phi) * cos(Theta)),
      -cos(B) * (sin(AlphaL) * sin(Theta) - cos(AlphaL) * cos(Phi) * cos(Theta)) - sin(B) * cos(Theta) * sin(Phi);

  az << 0, 0, 1;

  JvR << ds * cos(AlphaR) * cos(B) * cos(Psi) * cos(Theta) - ds * cos(B) * sin(AlphaR) * sin(Phi) * sin(Psi) -
             ds * cos(B) * cos(Phi) * cos(Psi) * sin(AlphaR) * sin(Theta),
      0,
      ZB2 * cos(Phi) * sin(Psi) + YB2 * sin(Phi) * sin(Psi) + ds * sin(B) * sin(Phi) * sin(Psi) +
          YB2 * cos(Phi) * cos(Psi) * sin(Theta) - ZB2 * cos(Psi) * sin(Phi) * sin(Theta) +
          ds * cos(AlphaR) * cos(B) * cos(Phi) * sin(Psi) + ds * cos(Phi) * cos(Psi) * sin(B) * sin(Theta) -
          ds * cos(AlphaR) * cos(B) * cos(Psi) * sin(Phi) * sin(Theta),
      ZB2 * cos(Phi) * cos(Psi) * cos(Theta) - XB2 * cos(Psi) * sin(Theta) + YB2 * cos(Psi) * cos(Theta) * sin(Phi) -
          ds * cos(B) * cos(Psi) * sin(AlphaR) * sin(Theta) + ds * cos(Psi) * sin(B) * cos(Theta) * sin(Phi) +
          ds * cos(AlphaR) * cos(B) * cos(Phi) * cos(Psi) * cos(Theta),
      ZB2 * cos(Psi) * sin(Phi) - YB2 * cos(Phi) * cos(Psi) - XB2 * cos(Theta) * sin(Psi) -
          YB2 * sin(Phi) * sin(Psi) * sin(Theta) - ds * cos(Phi) * cos(Psi) * sin(B) -
          ZB2 * cos(Phi) * sin(Psi) * sin(Theta) + ds * cos(AlphaR) * cos(B) * cos(Psi) * sin(Phi) -
          ds * cos(B) * sin(AlphaR) * cos(Theta) * sin(Psi) - ds * sin(B) * sin(Phi) * sin(Psi) * sin(Theta) -
          ds * cos(AlphaR) * cos(B) * cos(Phi) * sin(Psi) * sin(Theta),
      1, 0, 0,
      ds * cos(AlphaR) * cos(B) * cos(Theta) * sin(Psi) + ds * cos(B) * cos(Psi) * sin(AlphaR) * sin(Phi) -
          ds * cos(B) * cos(Phi) * sin(AlphaR) * sin(Psi) * sin(Theta),
      0,
      YB2 * cos(Phi) * sin(Psi) * sin(Theta) - YB2 * cos(Psi) * sin(Phi) - ZB2 * sin(Phi) * sin(Psi) * sin(Theta) -
          ds * cos(Psi) * sin(B) * sin(Phi) - ZB2 * cos(Phi) * cos(Psi) -
          ds * cos(AlphaR) * cos(B) * cos(Phi) * cos(Psi) + ds * cos(Phi) * sin(B) * sin(Psi) * sin(Theta) -
          ds * cos(AlphaR) * cos(B) * sin(Phi) * sin(Psi) * sin(Theta),
      ZB2 * cos(Phi) * cos(Theta) * sin(Psi) - XB2 * sin(Psi) * sin(Theta) + YB2 * cos(Theta) * sin(Phi) * sin(Psi) -
          ds * cos(B) * sin(AlphaR) * sin(Psi) * sin(Theta) + ds * sin(B) * cos(Theta) * sin(Phi) * sin(Psi) +
          ds * cos(AlphaR) * cos(B) * cos(Phi) * cos(Theta) * sin(Psi),
      XB2 * cos(Psi) * cos(Theta) - YB2 * cos(Phi) * sin(Psi) + ZB2 * sin(Phi) * sin(Psi) +
          ZB2 * cos(Phi) * cos(Psi) * sin(Theta) - ds * cos(Phi) * sin(B) * sin(Psi) +
          YB2 * cos(Psi) * sin(Phi) * sin(Theta) + ds * cos(B) * cos(Psi) * sin(AlphaR) * cos(Theta) +
          ds * cos(AlphaR) * cos(B) * sin(Phi) * sin(Psi) + ds * cos(Psi) * sin(B) * sin(Phi) * sin(Theta) +
          ds * cos(AlphaR) * cos(B) * cos(Phi) * cos(Psi) * sin(Theta),
      0, 1, 0, -ds * cos(AlphaR) * cos(B) * sin(Theta) - ds * cos(B) * cos(Phi) * sin(AlphaR) * cos(Theta), 0,
      YB2 * cos(Phi) * cos(Theta) - ZB2 * cos(Theta) * sin(Phi) + ds * cos(Phi) * sin(B) * cos(Theta) -
          ds * cos(AlphaR) * cos(B) * cos(Theta) * sin(Phi),
      -XB2 * cos(Theta) - ZB2 * cos(Phi) * sin(Theta) - YB2 * sin(Phi) * sin(Theta) -
          ds * sin(B) * sin(Phi) * sin(Theta) - ds * cos(B) * sin(AlphaR) * cos(Theta) -
          ds * cos(AlphaR) * cos(B) * cos(Phi) * sin(Theta),
      0, 0, 0, 1;

  JvL << 0,
      ds * cos(AlphaL) * cos(B) * cos(Psi) * cos(Theta) - ds * cos(B) * sin(AlphaL) * sin(Phi) * sin(Psi) -
          ds * cos(B) * cos(Phi) * cos(Psi) * sin(AlphaL) * sin(Theta),
      ZB3 * cos(Phi) * sin(Psi) + YB3 * sin(Phi) * sin(Psi) - ds * sin(B) * sin(Phi) * sin(Psi) +
          YB3 * cos(Phi) * cos(Psi) * sin(Theta) - ZB3 * cos(Psi) * sin(Phi) * sin(Theta) +
          ds * cos(AlphaL) * cos(B) * cos(Phi) * sin(Psi) - ds * cos(Phi) * cos(Psi) * sin(B) * sin(Theta) -
          ds * cos(AlphaL) * cos(B) * cos(Psi) * sin(Phi) * sin(Theta),
      ZB3 * cos(Phi) * cos(Psi) * cos(Theta) - XB3 * cos(Psi) * sin(Theta) + YB3 * cos(Psi) * cos(Theta) * sin(Phi) -
          ds * cos(B) * cos(Psi) * sin(AlphaL) * sin(Theta) - ds * cos(Psi) * sin(B) * cos(Theta) * sin(Phi) +
          ds * cos(AlphaL) * cos(B) * cos(Phi) * cos(Psi) * cos(Theta),
      ZB3 * cos(Psi) * sin(Phi) - YB3 * cos(Phi) * cos(Psi) - XB3 * cos(Theta) * sin(Psi) -
          YB3 * sin(Phi) * sin(Psi) * sin(Theta) + ds * cos(Phi) * cos(Psi) * sin(B) -
          ZB3 * cos(Phi) * sin(Psi) * sin(Theta) + ds * cos(AlphaL) * cos(B) * cos(Psi) * sin(Phi) -
          ds * cos(B) * sin(AlphaL) * cos(Theta) * sin(Psi) + ds * sin(B) * sin(Phi) * sin(Psi) * sin(Theta) -
          ds * cos(AlphaL) * cos(B) * cos(Phi) * sin(Psi) * sin(Theta),
      1, 0, 0, 0,
      ds * cos(AlphaL) * cos(B) * cos(Theta) * sin(Psi) + ds * cos(B) * cos(Psi) * sin(AlphaL) * sin(Phi) -
          ds * cos(B) * cos(Phi) * sin(AlphaL) * sin(Psi) * sin(Theta),
      ds * cos(Psi) * sin(B) * sin(Phi) - YB3 * cos(Psi) * sin(Phi) - ZB3 * sin(Phi) * sin(Psi) * sin(Theta) -
          ZB3 * cos(Phi) * cos(Psi) + YB3 * cos(Phi) * sin(Psi) * sin(Theta) -
          ds * cos(AlphaL) * cos(B) * cos(Phi) * cos(Psi) - ds * cos(Phi) * sin(B) * sin(Psi) * sin(Theta) -
          ds * cos(AlphaL) * cos(B) * sin(Phi) * sin(Psi) * sin(Theta),
      ZB3 * cos(Phi) * cos(Theta) * sin(Psi) - XB3 * sin(Psi) * sin(Theta) + YB3 * cos(Theta) * sin(Phi) * sin(Psi) -
          ds * cos(B) * sin(AlphaL) * sin(Psi) * sin(Theta) - ds * sin(B) * cos(Theta) * sin(Phi) * sin(Psi) +
          ds * cos(AlphaL) * cos(B) * cos(Phi) * cos(Theta) * sin(Psi),
      XB3 * cos(Psi) * cos(Theta) - YB3 * cos(Phi) * sin(Psi) + ZB3 * sin(Phi) * sin(Psi) +
          ZB3 * cos(Phi) * cos(Psi) * sin(Theta) + ds * cos(Phi) * sin(B) * sin(Psi) +
          YB3 * cos(Psi) * sin(Phi) * sin(Theta) + ds * cos(B) * cos(Psi) * sin(AlphaL) * cos(Theta) +
          ds * cos(AlphaL) * cos(B) * sin(Phi) * sin(Psi) - ds * cos(Psi) * sin(B) * sin(Phi) * sin(Theta) +
          ds * cos(AlphaL) * cos(B) * cos(Phi) * cos(Psi) * sin(Theta),
      0, 1, 0, 0, -ds * cos(AlphaL) * cos(B) * sin(Theta) - ds * cos(B) * cos(Phi) * sin(AlphaL) * cos(Theta),
      YB3 * cos(Phi) * cos(Theta) - ZB3 * cos(Theta) * sin(Phi) - ds * cos(Phi) * sin(B) * cos(Theta) -
          ds * cos(AlphaL) * cos(B) * cos(Theta) * sin(Phi),
      ds * sin(B) * sin(Phi) * sin(Theta) - ZB3 * cos(Phi) * sin(Theta) - YB3 * sin(Phi) * sin(Theta) -
          XB3 * cos(Theta) - ds * cos(B) * sin(AlphaL) * cos(Theta) - ds * cos(AlphaL) * cos(B) * cos(Phi) * sin(Theta),
      0, 0, 0, 1;

  JwR << cos(Psi) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Theta) -
             cos(Phi) * pow(sin(AlphaR), 2) * sin(Psi) * pow(sin(Theta), 2) -
             pow(cos(AlphaR), 2) * cos(Phi) * pow(cos(Theta), 2) * sin(Psi) -
             pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * sin(Psi) * pow(sin(Theta), 2) -
             pow(cos(B), 2) * cos(Phi) * pow(sin(AlphaR), 2) * pow(cos(Theta), 2) * sin(Psi) -
             cos(Phi) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
             pow(cos(AlphaR), 2) * cos(Phi) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) +
             pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Psi) * sin(Phi) * sin(Theta) +
             pow(cos(AlphaR), 2) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
             cos(AlphaR) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) -
             cos(AlphaR) * pow(cos(B), 2) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) +
             cos(AlphaR) * pow(cos(Phi), 2) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) -
             cos(AlphaR) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
             cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) -
             cos(AlphaR) * pow(cos(B), 2) * pow(cos(Phi), 2) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) -
             cos(AlphaR) * pow(cos(Phi), 2) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
             cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) +
             cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Phi),
      0,
      cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * cos(Theta) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) +
          pow(cos(AlphaR), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) +
          cos(AlphaR) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Psi) * cos(Theta) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(Phi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) +
          cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) -
          cos(AlphaR) * sin(AlphaR) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaR), 2) * cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta),
      cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Theta) -
          pow(cos(B), 2) * pow(sin(AlphaR), 2) * pow(cos(Theta), 2) * sin(Psi) -
          pow(sin(AlphaR), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
          pow(cos(Phi), 2) * pow(sin(AlphaR), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(B), 2) * pow(sin(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaR), 2) * pow(cos(Theta), 2) * sin(Psi) -
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
          2 * cos(AlphaR) * cos(Phi) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          2 * cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * cos(Theta) * sin(Psi) * sin(Theta) -
          2 * cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta),
      0, 0, 0, 0,
      pow(cos(AlphaR), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(Theta), 2) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          pow(sin(AlphaR), 2) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(cos(Theta), 2) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) +
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Theta) +
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) +
          cos(AlphaR) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) -
          cos(AlphaR) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) -
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi),
      0,
      pow(cos(Phi), 2) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) +
          pow(cos(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) +
          pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) +
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * sin(Phi) * pow(sin(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Theta) * sin(Psi) +
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * sin(Psi) * sin(Theta) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Theta) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * sin(Phi) * pow(sin(Theta), 2) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * sin(Phi) * pow(sin(Theta), 2) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Theta),
      pow(cos(AlphaR), 2) * cos(Psi) * pow(sin(Theta), 2) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(AlphaR), 2) * pow(cos(Theta), 2) +
          pow(cos(B), 2) * cos(Psi) * pow(cos(Theta), 2) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(Theta), 2) +
          cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) * pow(sin(Phi), 2) +
          cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(cos(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) +
          2 * cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) -
          2 * cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Theta) -
          2 * cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Theta),
      pow(cos(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) +
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) -
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(Phi), 2) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          pow(sin(AlphaR), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * sin(Phi) * sin(Theta) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) +
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * sin(Phi) * sin(Theta),
      0, 0, 0,
      pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(AlphaR), 2) * pow(cos(Psi), 2) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaR), 2) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(Phi), 2) * sin(Psi) -
          cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * pow(sin(Phi), 2) * sin(Psi) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Psi) +
          cos(AlphaR) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * pow(sin(B), 2) * sin(Phi) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * sin(AlphaR) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * cos(Phi) * sin(AlphaR) * pow(sin(B), 2) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaR) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2),
      0,
      cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * cos(Theta) -
          pow(cos(B), 2) * pow(cos(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(Phi), 2) * sin(Theta) -
          pow(cos(Phi), 2) * pow(sin(B), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Theta) -
          pow(sin(AlphaR), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * sin(Theta) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Psi) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(B), 2) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * cos(Theta) -
          cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta),
      cos(AlphaR) * sin(AlphaR) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(AlphaR), 2) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          cos(Phi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) -
          cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaR) * pow(cos(B), 2) * sin(AlphaR) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(B), 2) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaR) * sin(AlphaR) * pow(sin(B), 2) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(AlphaR), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          pow(cos(B), 2) * cos(Psi) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) -
          cos(Psi) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(Psi) * pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(AlphaR), 2) * cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) -
          cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
          cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaR) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaR) * cos(Phi) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta),
      pow(cos(AlphaR), 2) * pow(cos(Psi), 2) * pow(cos(Theta), 2) +
          pow(cos(B), 2) * pow(cos(Phi), 2) * pow(sin(Psi), 2) + pow(cos(Phi), 2) * pow(sin(B), 2) * pow(sin(Psi), 2) +
          pow(sin(AlphaR), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(AlphaR), 2) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(Phi), 2) * pow(sin(Theta), 2) +
          pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaR), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) -
          2 * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * cos(Phi) * cos(Psi) * pow(sin(AlphaR), 2) * sin(Phi) * sin(Psi) * sin(Theta) -
          2 * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) -
          2 * cos(AlphaR) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) -
          2 * cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * cos(Theta) * sin(Theta) +
          2 * cos(AlphaR) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * cos(Theta) * sin(Theta) +
          2 * cos(AlphaR) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Theta) +
          2 * pow(cos(AlphaR), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * pow(cos(AlphaR), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * cos(AlphaR) * pow(cos(B), 2) * cos(Psi) * sin(AlphaR) * cos(Theta) * sin(Phi) * sin(Psi) +
          2 * cos(AlphaR) * cos(Psi) * sin(AlphaR) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi),
      0, 0, 0;

  JwL << 0,
      cos(Psi) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Theta) -
          cos(Phi) * pow(sin(AlphaL), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaL), 2) * cos(Phi) * pow(cos(Theta), 2) * sin(Psi) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(B), 2) * cos(Phi) * pow(sin(AlphaL), 2) * pow(cos(Theta), 2) * sin(Psi) -
          cos(Phi) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
          pow(cos(AlphaL), 2) * cos(Phi) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Psi) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaL), 2) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
          cos(AlphaL) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * pow(cos(Phi), 2) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) -
          cos(AlphaL) * pow(cos(B), 2) * pow(cos(Phi), 2) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * pow(cos(Phi), 2) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) +
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Phi),
      cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * cos(Theta) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) +
          pow(cos(AlphaL), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) +
          cos(AlphaL) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Psi) * cos(Theta) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(Phi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) +
          cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) -
          cos(AlphaL) * sin(AlphaL) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaL), 2) * cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta),
      cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Theta) -
          pow(cos(B), 2) * pow(sin(AlphaL), 2) * pow(cos(Theta), 2) * sin(Psi) -
          pow(sin(AlphaL), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
          pow(cos(Phi), 2) * pow(sin(AlphaL), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(B), 2) * pow(sin(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaL), 2) * pow(cos(Theta), 2) * sin(Psi) -
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
          2 * cos(AlphaL) * cos(Phi) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          2 * cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * cos(Theta) * sin(Psi) * sin(Theta) -
          2 * cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta),
      0, 0, 0, 0, 0,
      pow(cos(AlphaL), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(Theta), 2) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          pow(sin(AlphaL), 2) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(cos(Theta), 2) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) +
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Theta) +
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) +
          cos(AlphaL) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) -
          cos(AlphaL) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) -
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi),
      pow(cos(Phi), 2) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) +
          pow(cos(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) +
          pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) +
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * sin(Phi) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Theta) * sin(Psi) +
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * sin(Psi) * sin(Theta) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Theta) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * sin(Phi) * pow(sin(Theta), 2) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * sin(Phi) * pow(sin(Theta), 2) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Theta),
      pow(cos(AlphaL), 2) * cos(Psi) * pow(sin(Theta), 2) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(AlphaL), 2) * pow(cos(Theta), 2) +
          pow(cos(B), 2) * cos(Psi) * pow(cos(Theta), 2) * pow(sin(Phi), 2) +
          pow(cos(B), 2) * cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(Theta), 2) +
          cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) * pow(sin(Phi), 2) +
          cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(cos(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * pow(cos(Theta), 2) +
          2 * cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) -
          2 * cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Theta) -
          2 * cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Theta),
      pow(cos(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) +
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) -
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(Phi), 2) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          pow(sin(AlphaL), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * sin(Phi) * sin(Theta) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Phi) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) +
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * sin(Phi) * sin(Theta),
      0, 0, 0, 0,
      pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(AlphaL), 2) * pow(cos(Psi), 2) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) +
          pow(cos(AlphaL), 2) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * cos(Theta) * sin(Phi) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(Phi), 2) * sin(Psi) -
          cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * sin(Phi) * sin(Theta) -
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * pow(sin(Phi), 2) * sin(Psi) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Psi) +
          cos(AlphaL) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * sin(Phi) * sin(Theta) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * pow(sin(B), 2) * sin(Phi) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * sin(AlphaL) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * cos(Phi) * sin(AlphaL) * pow(sin(B), 2) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaL) * pow(cos(Phi), 2) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2),
      cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * cos(Theta) -
          pow(cos(B), 2) * pow(cos(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(Phi), 2) * sin(Theta) -
          pow(cos(Phi), 2) * pow(sin(B), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * sin(Theta) -
          pow(sin(AlphaL), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * sin(Theta) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * sin(Theta) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Psi) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(B), 2) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) * sin(Theta) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * cos(Theta) -
          cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) -
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) +
          pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) +
          cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) +
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi) * sin(Theta),
      cos(AlphaL) * sin(AlphaL) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(AlphaL), 2) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) -
          pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          cos(Phi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) -
          cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaL) * pow(cos(B), 2) * sin(AlphaL) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) -
          pow(cos(B), 2) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) -
          cos(AlphaL) * sin(AlphaL) * pow(sin(B), 2) * sin(Phi) * pow(sin(Psi), 2) * sin(Theta) +
          pow(cos(Phi), 2) * cos(Psi) * pow(sin(AlphaL), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          pow(cos(B), 2) * cos(Psi) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) -
          cos(Psi) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta) +
          cos(Psi) * pow(sin(B), 2) * cos(Theta) * pow(sin(Phi), 2) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) +
          pow(cos(AlphaL), 2) * cos(Phi) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * pow(sin(Psi), 2) -
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(cos(Theta), 2) * sin(Psi) +
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * pow(cos(Theta), 2) * sin(Psi) -
          cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(AlphaL) * sin(Psi) * pow(sin(Theta), 2) -
          cos(AlphaL) * cos(Phi) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * sin(Psi) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * cos(Psi) * cos(Theta) * sin(Psi) * sin(Theta) +
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * cos(Psi) * pow(sin(B), 2) * cos(Theta) * sin(Psi) * sin(Theta),
      pow(cos(AlphaL), 2) * pow(cos(Psi), 2) * pow(cos(Theta), 2) +
          pow(cos(B), 2) * pow(cos(Phi), 2) * pow(sin(Psi), 2) + pow(cos(Phi), 2) * pow(sin(B), 2) * pow(sin(Psi), 2) +
          pow(sin(AlphaL), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * pow(sin(B), 2) * pow(cos(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Psi), 2) +
          pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(AlphaL), 2) * pow(sin(Theta), 2) +
          pow(cos(B), 2) * pow(cos(Psi), 2) * pow(sin(Phi), 2) * pow(sin(Theta), 2) +
          pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Phi), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(B), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(Theta), 2) +
          pow(cos(AlphaL), 2) * pow(cos(Phi), 2) * pow(cos(Psi), 2) * pow(sin(B), 2) * pow(sin(Theta), 2) -
          2 * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * cos(Phi) * cos(Psi) * pow(sin(AlphaL), 2) * sin(Phi) * sin(Psi) * sin(Theta) -
          2 * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) -
          2 * cos(AlphaL) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) -
          2 * cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * cos(Theta) * sin(Theta) +
          2 * cos(AlphaL) * pow(cos(B), 2) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * cos(Theta) * sin(Theta) +
          2 * cos(AlphaL) * cos(Phi) * pow(cos(Psi), 2) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Theta) +
          2 * pow(cos(AlphaL), 2) * pow(cos(B), 2) * cos(Phi) * cos(Psi) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * pow(cos(AlphaL), 2) * cos(Phi) * cos(Psi) * pow(sin(B), 2) * sin(Phi) * sin(Psi) * sin(Theta) +
          2 * cos(AlphaL) * pow(cos(B), 2) * cos(Psi) * sin(AlphaL) * cos(Theta) * sin(Phi) * sin(Psi) +
          2 * cos(AlphaL) * cos(Psi) * sin(AlphaL) * pow(sin(B), 2) * cos(Theta) * sin(Phi) * sin(Psi),
      0, 0, 0;

  B2 << 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

  B1 << JvR.transpose() * RIC2 * az + JwR.transpose() * RIC2 * az * (Kt / b),
      JvL.transpose() * RIC3 * az - JwL.transpose() * RIC3 * az * (Kt / b);

  // Adjust the Input coupling matrix considering q = [AlphaR AlphaL Phi Theta Psi X Y Z];

  BInputCoupling << B1, B2;

  return BInputCoupling;
}
