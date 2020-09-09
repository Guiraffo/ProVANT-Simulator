/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the implementation for the function that returns the
 * gravitational forces vector for the UAV 4.0.
 * @author Daniel Cardoso
 */

#include "vant4_winf_transition/gravitational_vector.h"

#include "vant4_winf_transition/uav4_parameters.h"

#include <cmath>

Eigen::VectorXd GravitationVector(Eigen::VectorXd q)
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

  Eigen::VectorXd G(8);

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

  // vetor gravidade
  G << -M2 * ds * g * (cosAlphaR * cosB * sinTheta + cosB * cosPhi * sinAlphaR * cosTheta),
      -M3 * ds * g * (cosAlphaL * cosB * sinTheta + cosB * cosPhi * sinAlphaL * cosTheta),
      M2 * g *
              (ds * (cosPhi * sinB * cosTheta - cosAlphaR * cosB * cosTheta * sinPhi) + YB2 * cosPhi * cosTheta -
               ZB2 * cosTheta * sinPhi) -
          M3 * g *
              (ds * (cosPhi * sinB * cosTheta + cosAlphaL * cosB * cosTheta * sinPhi) - YB3 * cosPhi * cosTheta +
               ZB3 * cosTheta * sinPhi) +
          M1 * g * (YB1 * cosPhi * cosTheta - ZB1 * cosTheta * sinPhi),
      -M1 * g * (XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta) -
          M3 * g *
              (ds * (cosB * sinAlphaL * cosTheta - sinB * sinPhi * sinTheta + cosAlphaL * cosB * cosPhi * sinTheta) +
               XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta) -
          M2 * g *
              (ds * (cosB * sinAlphaR * cosTheta + sinB * sinPhi * sinTheta + cosAlphaR * cosB * cosPhi * sinTheta) +
               XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta),
      0, 0, 0, M1 * g + M2 * g + M3 * g;

  return G;
}
