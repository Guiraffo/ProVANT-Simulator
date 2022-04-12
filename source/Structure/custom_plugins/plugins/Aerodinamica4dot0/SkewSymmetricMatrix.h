/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https: //github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @file SkewSymmetricMatrix.h
 * @brief Contains the implementation of the SkewSymmetricMatrix function.
 *
 * @author Júnio Eduardo de Morais Aquino
 */

#include <eigen3/Eigen/Eigen>

/**
 * @brief Creates a 3x3 skew symmetric matrix from a tridimensional vector.
 *
 * @param Vector
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd SkewSymmetricMatrix(Eigen::VectorXd Vector)
{
  // Place Vet in the Skew Symmetric matrix S
  Eigen::MatrixXd SkewMatrix(3, 3);
  SkewMatrix << 0, -Vector(2), Vector(1), Vector(2), 0, -Vector(0), -Vector(1), Vector(0), 0;

  return SkewMatrix;
}
