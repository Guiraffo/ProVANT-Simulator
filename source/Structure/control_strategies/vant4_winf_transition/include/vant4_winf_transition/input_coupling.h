/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration for the function that returns the
 * input coupling matrix for the UAV 4.0.
 * @author Daniel Cardoso
 */

#ifndef INPUT_COUPLING
#define INPUT_COUPLING

#include <eigen3/Eigen/Eigen>

/**
 * @brief Calculates the Input Coupling matrix.
 * 
 * @param q Vector of generalized variables.
 * @return Eigen::MatrixXd Input Coupling Matrix.
 */
Eigen::MatrixXd InputCouplingMatrix(Eigen::VectorXd q);

#endif //INPUT_COUPLING
