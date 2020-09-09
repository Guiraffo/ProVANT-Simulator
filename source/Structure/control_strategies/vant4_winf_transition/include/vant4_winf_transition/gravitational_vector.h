/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the declaration for the function that returns the
 * gravitational forces vector for the UAV 4.0.
 * @author Daniel Cardoso
 */

#ifndef GRAVITATIONAL_VECTOR
#define GRAVITATIONAL_VECTOR

#include <eigen3/Eigen/Eigen>

/**
 * @brief Calculates the gravitational forces vector.
 *
 * @param q Vector of generalized variables.
 * @return Eigen::VectorXd
 */
Eigen::VectorXd GravitationVector(Eigen::VectorXd q);

#endif  // GRAVITATIONAL_VECTOR
