/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration for the function that returns the
 * input coupling matrix for the UAV 4.0.
 * @author Daniel Cardoso
 * @todo Check this documentation, at the time of writing this block, I could
 * not understand the diference between the input coupling and input coupling
 * aero matrices.
 */

#ifndef INPUT_COUPLING_AERO
#define INPUT_COUPLING_AERO

#include <eigen3/Eigen/Eigen>

/**
 * @brief Calculates the input coupling matrix.
 *
 * @param q Vector of generalized variables.
 * @param qp Vector of generalized variables time derivatives.
 * @param ub
 * @return Eigen::MatrixXd Input coupling matrix
 */
Eigen::MatrixXd InputCouplingMatrixAero(Eigen::VectorXd q, Eigen::VectorXd qp, double* ub);

#endif  // INPUT_COUPLING_AERO
