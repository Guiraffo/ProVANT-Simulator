/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the implementation for the function that returns 
 * the inertia matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#include "vant4_winf_transition/inertia_matrix.h"
#include "vant4_winf_transition/uav4_parameters.h"

#include <cmath>

Eigen::MatrixXd InertiaMatrix(Eigen::VectorXd q)
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

  Eigen::MatrixXd M(8, 8);

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

  M << (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
               (Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                    ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                         (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                     (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                         (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) -
                Izz2 *
                    ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                          cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                          cosB * sinAlphaR * cosTheta * sinPsi) -
                     (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                          sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosB * cosPsi * sinAlphaR * cosTheta) +
                     cosTheta * sinPhi *
                         (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                          cosAlphaR * cosB * cosPhi * cosTheta)) *
                    (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                     sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                Iyy2 *
                    ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                          cosPsi * sinAlphaR * sinB * cosTheta) +
                     (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                          cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                          sinAlphaR * sinB * cosTheta * sinPsi) +
                     cosTheta * sinPhi *
                         (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                          cosAlphaR * cosPhi * sinB * cosTheta)) *
                    (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                     cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     cosPsi * sinAlphaR * sinB * cosTheta)) +
           (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
               (Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                    ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                         (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                     (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                         (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
                Izz2 *
                    ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                          cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                          cosB * sinAlphaR * cosTheta * sinPsi) -
                     (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                          sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosB * cosPsi * sinAlphaR * cosTheta) +
                     cosTheta * sinPhi *
                         (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                          cosAlphaR * cosB * cosPhi * cosTheta)) *
                    (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     cosB * sinAlphaR * cosTheta * sinPsi) +
                Iyy2 *
                    (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                     cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                     sinAlphaR * sinB * cosTheta * sinPsi) *
                    ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                          cosPsi * sinAlphaR * sinB * cosTheta) +
                     (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                          cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                          sinAlphaR * sinB * cosTheta * sinPsi) +
                     cosTheta * sinPhi *
                         (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                          cosAlphaR * cosPhi * sinB * cosTheta))) +
           M2 * (pow(ds, 2) * pow(cosB, 2) * pow(cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta, 2) +
                 pow(ds, 2) * pow(cosB, 2) *
                     pow(sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta +
                             cosPhi * cosPsi * sinAlphaR * sinTheta,
                         2) +
                 pow(ds, 2) * pow(cosB, 2) *
                     pow(cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi -
                             cosPhi * sinAlphaR * sinPsi * sinTheta,
                         2)) +
           cosTheta * sinPhi *
               (Izz2 *
                    ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                          cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                          cosB * sinAlphaR * cosTheta * sinPsi) -
                     (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                          sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosB * cosPsi * sinAlphaR * cosTheta) +
                     cosTheta * sinPhi *
                         (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                          cosAlphaR * cosB * cosPhi * cosTheta)) *
                    (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) -
                Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                    ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                         (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                     (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                         (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
                Iyy2 *
                    ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                         (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                          cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                          cosPsi * sinAlphaR * sinB * cosTheta) +
                     (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                         (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                          cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                          sinAlphaR * sinB * cosTheta * sinPsi) +
                     cosTheta * sinPhi *
                         (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                          cosAlphaR * cosPhi * sinB * cosTheta)) *
                    (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)),
      0,
      cosTheta * sinPsi *
              (Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
               Izz2 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    cosTheta * sinPhi *
                        (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                         cosAlphaR * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                         cosAlphaR * cosPhi * sinB * cosTheta))) -
          M2 * (ds * cosB *
                    (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta +
                     cosPhi * cosPsi * sinAlphaR * sinTheta) *
                    (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                     YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                     ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                     ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) +
                ds * cosB *
                    (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi -
                     cosPhi * sinAlphaR * sinPsi * sinTheta) *
                    (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                     ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                     ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                     ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta) +
                ds * cosB * cosTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                    (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi)) -
          sinTheta *
              (Izz2 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    cosTheta * sinPhi *
                        (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                         cosAlphaR * cosB * cosPhi * cosTheta)) *
                   (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) -
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
               Iyy2 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                         cosAlphaR * cosPhi * sinB * cosTheta)) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) -
          cosPsi * cosTheta *
              (Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) -
               Izz2 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    cosTheta * sinPhi *
                        (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                         cosAlphaR * cosB * cosPhi * cosTheta)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Iyy2 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                         cosAlphaR * cosPhi * sinB * cosTheta)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta)),
      sinPsi *
              (Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) -
               Izz2 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    cosTheta * sinPhi *
                        (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                         cosAlphaR * cosB * cosPhi * cosTheta)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Iyy2 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                         cosAlphaR * cosPhi * sinB * cosTheta)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta)) +
          cosPsi *
              (Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
               Izz2 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    cosTheta * sinPhi *
                        (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                         cosAlphaR * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                         cosAlphaR * cosPhi * sinB * cosTheta))) +
          M2 *
              (ds * cosB * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
               ds * cosB * cosPsi *
                   (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta +
                    cosPhi * cosPsi * sinAlphaR * sinTheta) *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) +
               ds * cosB * sinPsi *
                   (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi -
                    cosPhi * sinAlphaR * sinPsi * sinTheta) *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta)),
      M2 * (ds * cosB *
                (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi - cosPhi * sinAlphaR * sinPsi * sinTheta) *
                (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                 ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                 ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                 ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) +
            ds * cosB *
                (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaR * sinTheta) *
                (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                 YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                 ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                 ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) -
          Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
              ((sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
               (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
               cosTheta * sinPhi * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta)) +
          Izz2 *
              ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               cosTheta * sinPhi *
                   (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta)) *
              (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) +
          Iyy2 *
              ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) +
               (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) +
               cosTheta * sinPhi *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) *
              (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta),
      -M2 * ds * cosB *
          (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaR * sinTheta),
      M2 * ds * cosB *
          (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi - cosPhi * sinAlphaR * sinPsi * sinTheta),
      -M2 * ds * cosB * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta), 0,
      (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               Iyy3 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) +
          (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta))) +
          M3 * (pow(ds, 2) * pow(cosB, 2) * pow(cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta, 2) +
                pow(ds, 2) * pow(cosB, 2) *
                    pow(sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta +
                            cosPhi * cosPsi * sinAlphaL * sinTheta,
                        2) +
                pow(ds, 2) * pow(cosB, 2) *
                    pow(cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi -
                            cosPhi * sinAlphaL * sinPsi * sinTheta,
                        2)) +
          cosTheta * sinPhi *
              (Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) -
               Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Iyy3 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta)) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)),
      cosTheta * sinPsi *
              (Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta))) -
          M3 * (ds * cosB *
                    (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta +
                     cosPhi * cosPsi * sinAlphaL * sinTheta) *
                    (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                     YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
                ds * cosB *
                    (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi -
                     cosPhi * sinAlphaL * sinPsi * sinTheta) *
                    (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                     ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                     ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                     ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) -
                ds * cosB * cosTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                    (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi)) -
          sinTheta *
              (Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) -
               Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Iyy3 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta)) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) -
          cosPsi * cosTheta *
              (Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               Iyy3 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)),
      sinPsi *
              (Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               Iyy3 *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) +
          cosPsi *
              (Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                        (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
               Izz3 *
                   ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
                    (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    cosTheta * sinPhi *
                        (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                         cosAlphaL * cosB * cosPhi * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    cosTheta * sinPhi *
                        (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                         cosAlphaL * cosPhi * sinB * cosTheta))) +
          M3 *
              (ds * cosB * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) +
               ds * cosB * cosPsi *
                   (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta +
                    cosPhi * cosPsi * sinAlphaL * sinTheta) *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta) -
               ds * cosB * sinPsi *
                   (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi -
                    cosPhi * sinAlphaL * sinPsi * sinTheta) *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta)),
      M3 * (ds * cosB *
                (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi - cosPhi * sinAlphaL * sinPsi * sinTheta) *
                (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                 ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                 ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                 ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
            ds * cosB *
                (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaL * sinTheta) *
                (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                 YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                 ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                 ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) -
          Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
              ((sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
               (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
               cosTheta * sinPhi * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta)) +
          Izz3 *
              ((cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               cosTheta * sinPhi *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta)) *
              (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) +
          Iyy3 *
              ((cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) -
               (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
               cosTheta * sinPhi *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) *
              (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta),
      -M3 * ds * cosB *
          (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaL * sinTheta),
      M3 * ds * cosB *
          (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi - cosPhi * sinAlphaL * sinPsi * sinTheta),
      -M3 * ds * cosB * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta),
      (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) +
               Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))) -
          (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Iyy2 *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))) -
          M2 * (ds * cosB *
                    (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta +
                     cosPhi * cosPsi * sinAlphaR * sinTheta) *
                    (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                     YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                     ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                     ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) +
                ds * cosB *
                    (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi -
                     cosPhi * sinAlphaR * sinPsi * sinTheta) *
                    (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                     ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                     ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                     ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta) +
                ds * cosB * cosTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                    (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi)) -
          cosTheta * sinPhi *
              (Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) +
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) -
               Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi))),
      (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) -
               Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) -
          (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
               Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) -
          M3 * (ds * cosB *
                    (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta +
                     cosPhi * cosPsi * sinAlphaL * sinTheta) *
                    (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                     YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
                ds * cosB *
                    (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi -
                     cosPhi * sinAlphaL * sinPsi * sinTheta) *
                    (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                     ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                     ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                     ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) -
                ds * cosB * cosTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                    (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi)) -
          cosTheta * sinPhi *
              (Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) -
               Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) +
               Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi))),
      sinTheta *
              (Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) -
               Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) +
               Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi))) +
          sinTheta *
              (Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) +
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) -
               Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi))) +
          sinTheta *
              (sinTheta *
                   (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Ixz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) -
               cosPhi * cosTheta *
                   (Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Izz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) +
               Iyy1 * cosTheta * sinPhi *
                   (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
                    cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) +
          M3 * (pow(cosTheta, 2) *
                    pow(ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi, 2) +
                pow(ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                        ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                        ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                        ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta,
                    2) +
                pow(ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                        YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                        ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                        ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta,
                    2)) +
          M2 * (pow(cosTheta, 2) *
                    pow(YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi, 2) +
                pow(ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                        ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                        ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                        ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta,
                    2) +
                pow(ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                        YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                        ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                        ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta,
                    2)) +
          M1 * (pow(cosTheta, 2) * pow(YB1 * cosPhi - ZB1 * sinPhi, 2) +
                pow(YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                        ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta),
                    2) +
                pow(YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                        ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta),
                    2)) +
          cosPsi * cosTheta *
              ((Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                Izz1 *
                    (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) *
                   (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
               cosPsi * cosTheta *
                   (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Ixz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) +
               Iyy1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
                    cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) -
          cosTheta * sinPsi *
              ((Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                Izz1 *
                    (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) *
                   (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
               cosTheta * sinPsi *
                   (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Ixz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) +
               Iyy1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
                    cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) +
          cosPsi * cosTheta *
              (Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
               Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) +
          cosPsi * cosTheta *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Iyy2 *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))) +
          cosTheta * sinPsi *
              (Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) -
               Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) +
          cosTheta * sinPsi *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) +
               Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))),
      cosPsi *
              (Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) -
               Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Ixx3 * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) +
          cosPsi *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) +
               Ixx2 * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))) -
          M1 * (cosTheta * (YB1 * cosPhi - ZB1 * sinPhi) *
                    (XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta) -
                cosPsi *
                    (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                    (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) +
                sinPsi *
                    (YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) *
                    (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi)) -
          sinPsi *
              ((Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                Izz1 *
                    (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) *
                   (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
               cosPsi * cosTheta *
                   (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Ixz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) +
               Iyy1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
                   (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
                    cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) -
          cosPsi *
              ((Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                Izz1 *
                    (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) *
                   (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
               cosTheta * sinPsi *
                   (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
                    Ixz1 * (cosPhi * cosTheta * sinTheta -
                            cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                            cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) +
               Iyy1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
                   (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
                    cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta))) +
          M3 *
              (cosTheta * (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi) *
                   (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) -
               cosPsi *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                   (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                    YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                    ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                    ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
               sinPsi *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                   (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                    ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                    ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                    ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta)) -
          M2 *
              (cosTheta * (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi) *
                   (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
               cosPsi *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                   (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                    YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                    ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                    ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) +
               sinPsi *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                   (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                    ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                    ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                    ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta)) -
          sinPsi *
              (Izz3 *
                   (sinTheta * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi -
                                cosAlphaL * cosB * cosPhi * cosTheta) +
                    cosPsi * cosTheta *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 *
                   (cosPsi * cosTheta *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
                    sinTheta * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta +
                                cosAlphaL * cosPhi * sinB * cosTheta) +
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
               Ixx3 * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi))) -
          sinPsi *
              (Izz2 *
                   (cosPsi * cosTheta *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
                    sinTheta * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta +
                                cosAlphaR * cosB * cosPhi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Iyy2 *
                   (sinTheta * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta -
                                cosAlphaR * cosPhi * sinB * cosTheta) +
                    cosPsi * cosTheta *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    cosTheta * sinPsi *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               Ixx2 * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) *
                   (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
                    cosPsi * cosTheta *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
                    cosTheta * sinPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi))),
      Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
              (cosPsi * cosTheta *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) -
               sinTheta *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) +
               cosTheta * sinPsi *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi)) -
          M2 * ((ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                 ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                 ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                 ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta) *
                    (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                     ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) +
                (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                 YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                 ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                 ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) *
                    (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                     YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) -
          M1 * ((YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                 ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) *
                    (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                     YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta) +
                (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                 ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                    (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi)) -
          sinTheta *
              (Ixx1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
               Ixz1 *
                   (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) -
          M3 * ((ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                 ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) *
                    (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                     ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
                (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                 YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                 ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) *
                    (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                     YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) -
          Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
              (sinTheta *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) +
               cosPsi * cosTheta *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               cosTheta * sinPsi *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi)) +
          cosPhi * cosTheta *
              (Ixz1 * (pow(sinTheta, 2) + pow(cosPsi, 2) * pow(cosTheta, 2) + pow(cosTheta, 2) * pow(sinPsi, 2)) -
               Izz1 *
                   (cosPhi * cosTheta * sinTheta - cosPsi * cosTheta * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosTheta * sinPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta))) -
          Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
              (sinTheta * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
               cosPsi * cosTheta *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) +
               cosTheta * sinPsi *
                   (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) -
          Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
              (sinTheta * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) -
               cosPsi * cosTheta *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
               cosTheta * sinPsi *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) -
          Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
              (sinTheta *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) +
               cosPsi * cosTheta *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               cosTheta * sinPsi *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi)) +
          Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
              (cosPsi * cosTheta *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
               sinTheta *
                   (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) +
               cosTheta * sinPsi *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi)) -
          Iyy1 * cosTheta * sinPhi *
              (cosPsi * cosTheta * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosTheta * sinPhi * sinTheta -
               cosTheta * sinPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)),
      M1 * (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
            ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) +
          M3 * (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
          M2 * (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta),
      -M1 * (YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
             ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) -
          M3 * (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) -
          M2 * (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta),
      M2 * cosTheta * (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi) -
          M3 * cosTheta * (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi) +
          M1 * cosTheta * (YB1 * cosPhi - ZB1 * sinPhi),
      (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) +
          M2 *
              (ds * cosB * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
               ds * cosB * cosPsi *
                   (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta +
                    cosPhi * cosPsi * sinAlphaR * sinTheta) *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) +
               ds * cosB * sinPsi *
                   (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi -
                    cosPhi * sinAlphaR * sinPsi * sinTheta) *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta)) +
          (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) +
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) +
          cosTheta * sinPhi *
              (Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) -
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) +
               Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)),
      (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) +
          M3 *
              (ds * cosB * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) +
               ds * cosB * cosPsi *
                   (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta +
                    cosPhi * cosPsi * sinAlphaL * sinTheta) *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta) -
               ds * cosB * sinPsi *
                   (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi -
                    cosPhi * sinAlphaL * sinPsi * sinTheta) *
                   (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                    ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                    ds * cosAlphaL * cosB * cosPhi * cosTheta)) +
          (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) +
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) +
          cosTheta * sinPhi *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) -
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) +
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)),
      M3 * (cosTheta * (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi) *
                (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
                 ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) -
            cosPsi *
                (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
                 ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                 YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                 ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
            sinPsi *
                (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
                 ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                 ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta)) -
          sinTheta *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) -
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) +
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) -
          sinTheta *
              (Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) -
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) +
               Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) -
          M1 * (cosTheta * (YB1 * cosPhi - ZB1 * sinPhi) *
                    (XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta) -
                cosPsi *
                    (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                     ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                    (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) +
                sinPsi *
                    (YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) *
                    (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi)) -
          M2 *
              (cosTheta * (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi) *
                   (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                    ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
               cosPsi *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                   (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                    YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                    ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                    ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) +
               sinPsi *
                   (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                    ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                    ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                   (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                    ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                    ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                    ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta)) -
          sinTheta * (Ixz1 * sinTheta *
                          (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                           sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) -
                      Izz1 * cosPhi * cosTheta *
                          (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                           sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
                      Iyy1 * cosTheta * sinPhi *
                          (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                           sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta))) -
          cosPsi * cosTheta *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) -
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) -
          cosPsi * cosTheta *
              (Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta) -
               Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) +
          cosTheta * sinPsi *
              (Iyy1 *
                   (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                   (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
               Izz1 *
                   (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) *
                   (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
               Ixz1 * cosTheta * sinPsi *
                   (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))) +
          cosTheta * sinPsi *
              (Izz3 *
                   (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosB * cosPsi * sinAlphaL * cosTheta) +
                    cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              cosB * sinAlphaL * cosTheta * sinPsi)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              sinAlphaL * sinB * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosPsi * sinAlphaL * sinB * cosTheta)) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) +
               Ixx3 *
                   (cosPsi *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
                   (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) +
          cosTheta * sinPsi *
              (Izz2 *
                   (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                              cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                              cosB * sinAlphaR * cosTheta * sinPsi) -
                    sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                              sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosB * cosPsi * sinAlphaR * cosTheta)) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                              cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                              cosPsi * sinAlphaR * sinB * cosTheta) +
                    cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                              cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                              sinAlphaR * sinB * cosTheta * sinPsi)) *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) +
               Ixx2 *
                   (cosPsi *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
                    sinPsi *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) -
          cosPsi * cosTheta *
              (Iyy1 *
                   (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                   (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
               Izz1 *
                   (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) *
                   (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
               Ixz1 * cosPsi * cosTheta *
                   (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))),
      sinPsi * (Iyy1 *
                    (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                     sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                    (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                Izz1 *
                    (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) *
                    (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                Ixz1 * cosPsi * cosTheta *
                    (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))) +
          cosPsi * (Iyy1 *
                        (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                        (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    Izz1 *
                        (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) *
                        (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    Ixz1 * cosTheta * sinPsi *
                        (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta))) +
          sinPsi * (Izz3 *
                        (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                                   cosB * cosPsi * sinAlphaL * cosTheta) +
                         cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                                   cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                                   cosB * sinAlphaL * cosTheta * sinPsi)) *
                        (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) -
                    Iyy3 *
                        (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                                   cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   sinAlphaL * sinB * cosTheta * sinPsi) -
                         sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosPsi * sinAlphaL * sinB * cosTheta)) *
                        (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta) +
                    Ixx3 *
                        (cosPsi * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosAlphaL * cosTheta * sinPsi) +
                         sinPsi * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosAlphaL * cosPsi * cosTheta)) *
                        (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) +
          sinPsi * (Iyy2 *
                        (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                                   cosPsi * sinAlphaR * sinB * cosTheta) +
                         cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                                   cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                                   sinAlphaR * sinB * cosTheta * sinPsi)) *
                        (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) -
                    Izz2 *
                        (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                                   cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosB * sinAlphaR * cosTheta * sinPsi) -
                         sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosB * cosPsi * sinAlphaR * cosTheta)) *
                        (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                         sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) +
                    Ixx2 *
                        (cosPsi * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosAlphaR * cosTheta * sinPsi) +
                         sinPsi * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosAlphaR * cosPsi * cosTheta)) *
                        (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) +
          cosPsi * (Izz3 *
                        (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                                   cosB * cosPsi * sinAlphaL * cosTheta) +
                         cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                                   cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                                   cosB * sinAlphaL * cosTheta * sinPsi)) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi) +
                    Iyy3 *
                        (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                                   cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   sinAlphaL * sinB * cosTheta * sinPsi) -
                         sinPsi * (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosPsi * sinAlphaL * sinB * cosTheta)) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) +
                    Ixx3 *
                        (cosPsi * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosAlphaL * cosTheta * sinPsi) +
                         sinPsi * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosAlphaL * cosPsi * cosTheta)) *
                        (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi)) +
          cosPsi * (Izz2 *
                        (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                                   cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosB * sinAlphaR * cosTheta * sinPsi) -
                         sinPsi * (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosB * cosPsi * sinAlphaR * cosTheta)) *
                        (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) +
                    Iyy2 *
                        (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                                   cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                                   cosPsi * sinAlphaR * sinB * cosTheta) +
                         cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                                   cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                                   sinAlphaR * sinB * cosTheta * sinPsi)) *
                        (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi) +
                    Ixx2 *
                        (cosPsi * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                                   cosAlphaR * cosTheta * sinPsi) +
                         sinPsi * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                                   cosAlphaR * cosPsi * cosTheta)) *
                        (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi)) +
          M1 * (pow(cosPsi, 2) * pow(ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi, 2) +
                pow(sinPsi, 2) * pow(ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi, 2) +
                pow(XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta, 2)) +
          M3 * (pow(XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
                        ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta,
                    2) +
                pow(cosPsi, 2) * pow(XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                                         ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                                         ds * cosAlphaL * cosB * cosPhi * cosTheta,
                                     2) +
                pow(sinPsi, 2) * pow(XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                                         ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                                         ds * cosAlphaL * cosB * cosPhi * cosTheta,
                                     2)) +
          M2 * (pow(XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                        ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta,
                    2) +
                pow(cosPsi, 2) * pow(ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                                         ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                                         ds * cosAlphaR * cosB * cosPhi * cosTheta,
                                     2) +
                pow(sinPsi, 2) * pow(ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                                         ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                                         ds * cosAlphaR * cosB * cosPhi * cosTheta,
                                     2)),
      M2 * (sinPsi *
                (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
                 ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                 ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                 ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                 ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) -
            cosPsi *
                (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
                 ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                 YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                 ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                 ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) -
          M3 * (sinPsi *
                    (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                     ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                     ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                    (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                     ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) -
                cosPsi *
                    (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                     ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                     ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                    (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                     YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) -
          M1 * (cosPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) *
                    (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi) -
                sinPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) *
                    (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                     YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta)) -
          Ixx3 *
              (cosPsi * (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
               sinPsi * (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta)) *
              (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) -
          Ixx2 *
              (cosPsi * (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
               sinPsi * (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta)) *
              (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) +
          Ixz1 * sinTheta *
              (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
               sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
          Izz3 *
              (sinPsi * (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosB * cosPsi * sinAlphaL * cosTheta) +
               cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         cosB * sinAlphaL * cosTheta * sinPsi)) *
              (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) +
          Izz2 *
              (cosPsi * (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         cosB * sinAlphaR * cosTheta * sinPsi) -
               sinPsi *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta)) *
              (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) +
          Iyy3 *
              (cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                         cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                         sinAlphaL * sinB * cosTheta * sinPsi) -
               sinPsi *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) *
              (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) +
          Iyy2 *
              (sinPsi * (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                         cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                         cosPsi * sinAlphaR * sinB * cosTheta) +
               cosPsi * (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                         cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                         sinAlphaR * sinB * cosTheta * sinPsi)) *
              (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) -
          Izz1 * cosPhi * cosTheta *
              (cosPsi * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
               sinPsi * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta)) +
          Iyy1 * cosTheta * sinPhi *
              (cosPsi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
               sinPsi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)),
      M1 * cosPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) -
          M3 * cosPsi *
              (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
               ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) +
          M2 * cosPsi *
              (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
               ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta),
      M1 * sinPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) -
          M3 * sinPsi *
              (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
               ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) +
          M2 * sinPsi *
              (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
               ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta),
      -M3 * (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
             ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) -
          M2 * (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
          M1 * (XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta),
      M2 * (ds * cosB *
                (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi - cosPhi * sinAlphaR * sinPsi * sinTheta) *
                (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                 ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                 ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                 ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) +
            ds * cosB *
                (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaR * sinTheta) *
                (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                 YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                 ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                 ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) +
          (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) -
          (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
               Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
               Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta)) +
          cosTheta * sinPhi *
              (Izz2 * pow(sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta,
                          2) +
               Iyy2 * pow(cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta,
                          2) +
               Ixx2 * pow(cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta, 2)),
      M3 * (ds * cosB *
                (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi - cosPhi * sinAlphaL * sinPsi * sinTheta) *
                (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                 ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                 ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                 ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
            ds * cosB *
                (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaL * sinTheta) *
                (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                 YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                 ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                 ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) +
          (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) *
              (Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) -
               Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) -
          (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) *
              (Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) -
               Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) +
          cosTheta * sinPhi *
              (Izz3 * pow(cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta,
                          2) +
               Iyy3 * pow(cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta,
                          2) +
               Ixx3 * pow(cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta, 2)),
      cosTheta * sinPsi *
              ((cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) -
               cosTheta * sinPsi * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) +
               Iyy1 * cosTheta * sinPhi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) -
          M2 * ((ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                 ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                 ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                 ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta) *
                    (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                     ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) +
                (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                 YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                 ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                 ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta) *
                    (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                     YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) -
          M1 * ((YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                 ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) *
                    (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                     YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta) +
                (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                 ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) *
                    (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi)) -
          sinTheta * (sinTheta * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) -
                      cosPhi * cosTheta * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) +
                      Iyy1 * pow(cosTheta, 2) * pow(sinPhi, 2)) -
          sinTheta *
              (Izz3 * pow(cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta,
                          2) +
               Iyy3 * pow(cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta,
                          2) +
               Ixx3 * pow(cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta, 2)) -
          sinTheta *
              (Izz2 * pow(sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta,
                          2) +
               Iyy2 * pow(cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta,
                          2) +
               Ixx2 * pow(cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta, 2)) -
          M3 * ((ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                 ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                 ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) *
                    (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                     ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
                (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                 YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                 ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                 ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) *
                    (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                     YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) +
          cosPsi * cosTheta *
              (Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) -
               Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) +
          cosPsi * cosTheta *
              (Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
               Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
               Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta)) +
          cosTheta * sinPsi *
              (Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    cosB * sinAlphaL * cosTheta * sinPsi) -
               Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
               Iyy3 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    sinAlphaL * sinB * cosTheta * sinPsi) *
                   (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) +
          cosTheta * sinPsi *
              (Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) -
          cosPsi * cosTheta *
              ((sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) +
               cosPsi * cosTheta * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) +
               Iyy1 * cosTheta * sinPhi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)),
      cosPsi * (Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                    (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                     cosAlphaL * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                     cosB * sinAlphaL * cosTheta * sinPsi) -
                Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                    (sinAlphaL * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaL * cosTheta * sinPsi) +
                Iyy3 *
                    (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     cosAlphaL * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                     sinAlphaL * sinB * cosTheta * sinPsi) *
                    (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta)) -
          M1 * (cosPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) *
                    (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                     ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi) -
                sinPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) *
                    (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                     YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta)) +
          cosPsi *
              (Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                    cosAlphaR * cosB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
                    cosB * sinAlphaR * cosTheta * sinPsi) -
               Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + cosAlphaR * cosTheta * sinPsi) +
               Iyy2 *
                   (cosB * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) +
                    cosAlphaR * sinB * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) -
                    sinAlphaR * sinB * cosTheta * sinPsi) *
                   (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta)) -
          M3 * (sinPsi *
                    (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                     ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                     ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                    (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                     ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) -
                cosPsi *
                    (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi +
                     ds * cosB * sinAlphaL * sinTheta + ds * sinB * cosTheta * sinPhi -
                     ds * cosAlphaL * cosB * cosPhi * cosTheta) *
                    (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                     YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta)) +
          M2 * (sinPsi *
                    (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                     ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                     ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                    (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                     ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                     ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                     ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta) -
                cosPsi *
                    (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi -
                     ds * cosB * sinAlphaR * sinTheta + ds * sinB * cosTheta * sinPhi +
                     ds * cosAlphaR * cosB * cosPhi * cosTheta) *
                    (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                     YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                     ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                     ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta)) +
          sinPsi * ((sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) +
                    cosPsi * cosTheta * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) +
                    Iyy1 * cosTheta * sinPhi * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) +
          cosPsi * ((cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) -
                    cosTheta * sinPsi * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) +
                    Iyy1 * cosTheta * sinPhi * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) -
          sinPsi *
              (Ixx3 * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta) *
                   (sinAlphaL * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaL * cosPsi * cosTheta) -
               Izz3 * (cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta) *
                   (sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaL * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosB * cosPsi * sinAlphaL * cosTheta) +
               Iyy3 * (cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta) *
                   (cosAlphaL * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosPsi * sinAlphaL * sinB * cosTheta)) -
          sinPsi *
              (Ixx2 * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta) *
                   (sinAlphaR * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) - cosAlphaR * cosPsi * cosTheta) +
               Izz2 * (sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta) *
                   (cosAlphaR * cosB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                    sinB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + cosB * cosPsi * sinAlphaR * cosTheta) -
               Iyy2 * (cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta) *
                   (cosB * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) +
                    cosAlphaR * sinB * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
                    cosPsi * sinAlphaR * sinB * cosTheta)),
      sinTheta * (Ixx1 * sinTheta - Ixz1 * cosPhi * cosTheta) +
          M1 * (pow(YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
                        ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi,
                    2) +
                pow(ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
                        YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta,
                    2)) +
          Izz3 * pow(cosB * sinAlphaL * sinTheta + sinB * cosTheta * sinPhi - cosAlphaL * cosB * cosPhi * cosTheta, 2) +
          Izz2 * pow(sinB * cosTheta * sinPhi - cosB * sinAlphaR * sinTheta + cosAlphaR * cosB * cosPhi * cosTheta, 2) +
          Iyy3 * pow(cosB * cosTheta * sinPhi - sinAlphaL * sinB * sinTheta + cosAlphaL * cosPhi * sinB * cosTheta, 2) +
          Iyy2 * pow(cosB * cosTheta * sinPhi + sinAlphaR * sinB * sinTheta - cosAlphaR * cosPhi * sinB * cosTheta, 2) +
          M3 * (pow(XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                        ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi +
                        YB3 * cosPsi * sinPhi * sinTheta + ds * cosB * cosPsi * sinAlphaL * cosTheta +
                        ds * cosAlphaL * cosB * sinPhi * sinPsi - ds * cosPsi * sinB * sinPhi * sinTheta +
                        ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta,
                    2) +
                pow(YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                        YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB +
                        ZB3 * cosPhi * sinPsi * sinTheta - ds * cosAlphaL * cosB * cosPsi * sinPhi +
                        ds * cosB * sinAlphaL * cosTheta * sinPsi - ds * sinB * sinPhi * sinPsi * sinTheta +
                        ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta,
                    2)) +
          M2 * (pow(XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                        ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi +
                        YB2 * cosPsi * sinPhi * sinTheta + ds * cosB * cosPsi * sinAlphaR * cosTheta +
                        ds * cosAlphaR * cosB * sinPhi * sinPsi + ds * cosPsi * sinB * sinPhi * sinTheta +
                        ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta,
                    2) +
                pow(YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                        YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB +
                        ZB2 * cosPhi * sinPsi * sinTheta - ds * cosAlphaR * cosB * cosPsi * sinPhi +
                        ds * cosB * sinAlphaR * cosTheta * sinPsi + ds * sinB * sinPhi * sinPsi * sinTheta +
                        ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta,
                    2)) +
          Ixx3 * pow(cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta, 2) +
          Ixx2 * pow(cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta, 2) -
          cosPhi * cosTheta * (Ixz1 * sinTheta - Izz1 * cosPhi * cosTheta) + Iyy1 * pow(cosTheta, 2) * pow(sinPhi, 2),
      -M1 * (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
             ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi) -
          M3 * (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta) -
          M2 * (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta),
      M1 * (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
            YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta) +
          M3 * (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
          M2 * (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta),
      0,
      -M2 * ds * cosB *
          (sinAlphaR * sinPhi * sinPsi - cosAlphaR * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaR * sinTheta),
      -M3 * ds * cosB *
          (sinAlphaL * sinPhi * sinPsi - cosAlphaL * cosPsi * cosTheta + cosPhi * cosPsi * sinAlphaL * sinTheta),
      M1 * (YB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) +
            ZB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta)) +
          M3 * (ZB3 * cosPhi * sinPsi + YB3 * sinPhi * sinPsi - ds * sinB * sinPhi * sinPsi +
                YB3 * cosPhi * cosPsi * sinTheta - ZB3 * cosPsi * sinPhi * sinTheta +
                ds * cosAlphaL * cosB * cosPhi * sinPsi - ds * cosPhi * cosPsi * sinB * sinTheta -
                ds * cosAlphaL * cosB * cosPsi * sinPhi * sinTheta) +
          M2 * (ZB2 * cosPhi * sinPsi + YB2 * sinPhi * sinPsi + ds * sinB * sinPhi * sinPsi +
                YB2 * cosPhi * cosPsi * sinTheta - ZB2 * cosPsi * sinPhi * sinTheta +
                ds * cosAlphaR * cosB * cosPhi * sinPsi + ds * cosPhi * cosPsi * sinB * sinTheta -
                ds * cosAlphaR * cosB * cosPsi * sinPhi * sinTheta),
      M1 * cosPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) -
          M3 * cosPsi *
              (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
               ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) +
          M2 * cosPsi *
              (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
               ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta),
      -M1 * (YB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta) -
             ZB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) + XB1 * cosTheta * sinPsi) -
          M3 * (YB3 * cosPhi * cosPsi - ZB3 * cosPsi * sinPhi + XB3 * cosTheta * sinPsi +
                YB3 * sinPhi * sinPsi * sinTheta - ds * cosPhi * cosPsi * sinB + ZB3 * cosPhi * sinPsi * sinTheta -
                ds * cosAlphaL * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaL * cosTheta * sinPsi -
                ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaL * cosB * cosPhi * sinPsi * sinTheta) -
          M2 * (YB2 * cosPhi * cosPsi - ZB2 * cosPsi * sinPhi + XB2 * cosTheta * sinPsi +
                YB2 * sinPhi * sinPsi * sinTheta + ds * cosPhi * cosPsi * sinB + ZB2 * cosPhi * sinPsi * sinTheta -
                ds * cosAlphaR * cosB * cosPsi * sinPhi + ds * cosB * sinAlphaR * cosTheta * sinPsi +
                ds * sinB * sinPhi * sinPsi * sinTheta + ds * cosAlphaR * cosB * cosPhi * sinPsi * sinTheta),
      M1 + M2 + M3, 0, 0,
      M2 * ds * cosB *
          (cosAlphaR * cosTheta * sinPsi + cosPsi * sinAlphaR * sinPhi - cosPhi * sinAlphaR * sinPsi * sinTheta),
      M3 * ds * cosB *
          (cosAlphaL * cosTheta * sinPsi + cosPsi * sinAlphaL * sinPhi - cosPhi * sinAlphaL * sinPsi * sinTheta),
      -M1 * (YB1 * (cosPsi * sinPhi - cosPhi * sinPsi * sinTheta) +
             ZB1 * (cosPhi * cosPsi + sinPhi * sinPsi * sinTheta)) -
          M3 * (ZB3 * cosPhi * cosPsi + YB3 * cosPsi * sinPhi + ZB3 * sinPhi * sinPsi * sinTheta -
                ds * cosPsi * sinB * sinPhi - YB3 * cosPhi * sinPsi * sinTheta +
                ds * cosAlphaL * cosB * cosPhi * cosPsi + ds * cosPhi * sinB * sinPsi * sinTheta +
                ds * cosAlphaL * cosB * sinPhi * sinPsi * sinTheta) -
          M2 * (ZB2 * cosPhi * cosPsi + YB2 * cosPsi * sinPhi + ZB2 * sinPhi * sinPsi * sinTheta +
                ds * cosPsi * sinB * sinPhi - YB2 * cosPhi * sinPsi * sinTheta +
                ds * cosAlphaR * cosB * cosPhi * cosPsi - ds * cosPhi * sinB * sinPsi * sinTheta +
                ds * cosAlphaR * cosB * sinPhi * sinPsi * sinTheta),
      M1 * sinPsi * (ZB1 * cosPhi * cosTheta - XB1 * sinTheta + YB1 * cosTheta * sinPhi) -
          M3 * sinPsi *
              (XB3 * sinTheta - ZB3 * cosPhi * cosTheta - YB3 * cosTheta * sinPhi + ds * cosB * sinAlphaL * sinTheta +
               ds * sinB * cosTheta * sinPhi - ds * cosAlphaL * cosB * cosPhi * cosTheta) +
          M2 * sinPsi *
              (ZB2 * cosPhi * cosTheta - XB2 * sinTheta + YB2 * cosTheta * sinPhi - ds * cosB * sinAlphaR * sinTheta +
               ds * sinB * cosTheta * sinPhi + ds * cosAlphaR * cosB * cosPhi * cosTheta),
      M1 * (ZB1 * (sinPhi * sinPsi + cosPhi * cosPsi * sinTheta) -
            YB1 * (cosPhi * sinPsi - cosPsi * sinPhi * sinTheta) + XB1 * cosPsi * cosTheta) +
          M3 * (XB3 * cosPsi * cosTheta - YB3 * cosPhi * sinPsi + ZB3 * sinPhi * sinPsi +
                ZB3 * cosPhi * cosPsi * sinTheta + ds * cosPhi * sinB * sinPsi + YB3 * cosPsi * sinPhi * sinTheta +
                ds * cosB * cosPsi * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * sinPhi * sinPsi -
                ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaL * cosB * cosPhi * cosPsi * sinTheta) +
          M2 * (XB2 * cosPsi * cosTheta - YB2 * cosPhi * sinPsi + ZB2 * sinPhi * sinPsi +
                ZB2 * cosPhi * cosPsi * sinTheta - ds * cosPhi * sinB * sinPsi + YB2 * cosPsi * sinPhi * sinTheta +
                ds * cosB * cosPsi * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * sinPhi * sinPsi +
                ds * cosPsi * sinB * sinPhi * sinTheta + ds * cosAlphaR * cosB * cosPhi * cosPsi * sinTheta),
      0, M1 + M2 + M3, 0, -M2 * ds * cosB * (cosAlphaR * sinTheta + cosPhi * sinAlphaR * cosTheta),
      -M3 * ds * cosB * (cosAlphaL * sinTheta + cosPhi * sinAlphaL * cosTheta),
      M2 * cosTheta * (YB2 * cosPhi - ZB2 * sinPhi + ds * cosPhi * sinB - ds * cosAlphaR * cosB * sinPhi) -
          M3 * cosTheta * (ZB3 * sinPhi - YB3 * cosPhi + ds * cosPhi * sinB + ds * cosAlphaL * cosB * sinPhi) +
          M1 * cosTheta * (YB1 * cosPhi - ZB1 * sinPhi),
      -M3 * (XB3 * cosTheta + ZB3 * cosPhi * sinTheta + YB3 * sinPhi * sinTheta - ds * sinB * sinPhi * sinTheta +
             ds * cosB * sinAlphaL * cosTheta + ds * cosAlphaL * cosB * cosPhi * sinTheta) -
          M2 * (XB2 * cosTheta + ZB2 * cosPhi * sinTheta + YB2 * sinPhi * sinTheta + ds * sinB * sinPhi * sinTheta +
                ds * cosB * sinAlphaR * cosTheta + ds * cosAlphaR * cosB * cosPhi * sinTheta) -
          M1 * (XB1 * cosTheta + ZB1 * cosPhi * sinTheta + YB1 * sinPhi * sinTheta),
      0, 0, 0, M1 + M2 + M3;

  return M;
}
