/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration of the function that returns the
 * Coriollis foces matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#ifndef CORIOLLIS_MATRIX
#define CORIOLLIS_MATRIX

#include <eigen3/Eigen/Eigen>

/**
 * @brief Calculates the Coriollis Forces Matrix for the UAV 4.0.
 *
 * @param q Vector of generalized variables.
 * @param qp Vector of generalized variables time derivatives.
 * @return Eigen::MatrixXd
 */
Eigen::MatrixXd coriolisMatrix(Eigen::VectorXd q, Eigen::VectorXd qp);

#endif  // CORIOLLIS_MATRIX
