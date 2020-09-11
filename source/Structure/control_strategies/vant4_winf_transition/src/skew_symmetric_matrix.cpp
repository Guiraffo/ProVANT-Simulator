/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the implementation of the function that returns the
 * a skew symmetric matrix for a given vector.
 * @author Daniel Cardoso
 */

#include "vant4_winf_transition/skew_symmetric_matrix.h"

#include <cmath>

Eigen::MatrixXd SkewSymmetricMatrix(Eigen::VectorXd Vector)
{
  // Place Vet in the Skew Symmetric matrix S
  Eigen::MatrixXd SkewMatrix(3, 3);
  SkewMatrix << 0, -Vector(2), Vector(1), Vector(2), 0, -Vector(0), -Vector(1), Vector(0), 0;

  return SkewMatrix;
}
