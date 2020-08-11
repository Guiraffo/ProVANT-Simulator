/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration for the function that returns the
 * inertia matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#ifndef INERTIA_MATRIX
#define INERTIA_MATRIX

#include <eigen3/Eigen/Eigen>

/**
 * @brief Calculates the inertia matrix of the UAV 4.0 Euler Lagrange model.
 * 
 * @param q Vector of generalized variables.
 * @return Eigen::MatrixXd 
 */
Eigen::MatrixXd InertiaMatrix(Eigen::VectorXd q);

#endif //INERTIA_MATRIX
