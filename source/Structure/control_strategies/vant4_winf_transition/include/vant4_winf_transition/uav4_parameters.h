/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file contains the modelling parameters of the UAV 4.0 used in the
 * determination of the Euler Lagrange matrices of the robot.
 *
 * @todo Document the UAV parameters.
 */

#ifndef UAV4_PARAMETERS
#define UAV4_PARAMETERS

constexpr double Ixx1 = 0.1489;
constexpr double Iyy1 = 0.1789;
constexpr double Izz1 = 0.3011;
constexpr double Ixz1 = -0.0189;
constexpr double Ixx2 = 0.0007103;
constexpr double Iyy2 = 0.00071045;
constexpr double Izz2 = 0.00021337;
constexpr double Ixx3 = 0.0007103;
constexpr double Iyy3 = 0.00071045;
constexpr double Izz3 = 0.00021337;
constexpr double ds = 0.02;
constexpr double B = 0.0524;  // rad
constexpr double g = 9.8;
constexpr double b = 9.5e-6;
constexpr double Kt = 1.7e-7;
constexpr double M1 = 7.0;
constexpr double M2 = 0.3;
constexpr double M3 = 0.3;
constexpr double XB1 = 0.06684;
constexpr double YB1 = 0;
constexpr double ZB1 = 0.005392;
constexpr double pi = 3.14159265358979323846264338327950288419716939937510582097494459230781640628;
constexpr double XB2 = 0.078;
constexpr double YB2 = -0.6073;
constexpr double ZB2 = 0.1235;
constexpr double XB3 = 0.078;
constexpr double YB3 = 0.6073;
constexpr double ZB3 = 0.1235;

#endif  // UAV4_PARAMETERS
