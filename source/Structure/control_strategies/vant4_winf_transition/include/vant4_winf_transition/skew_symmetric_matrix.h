/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration of the function that returns the
 * a skew symmetric matrix for a given vector.
 * @author Daniel Cardoso
 */

#ifndef SKEW_SYMMETRIC_MATRIX
#define SKEW_SYMMETRIC_MATRIX

#include <eigen3/Eigen/Eigen>

/**
 * @brief Returns a 3x3 skew symmetric matrix for a given vector of dimension 3.
 *
 * @param Vector Vector of 3 real numbers to return a skew symmetric matrix
 * representation.
 * @return Eigen::MatrixXd 3x3 skew symmetric matrix.
 */
Eigen::MatrixXd SkewSymmetricMatrix(Eigen::VectorXd Vector);

#endif  // SKEW_SYMMETRIC_MATRIX
