/*
 * This file is part of the ProVANT simulator project.
 * Licensed under the terms of the MIT open source license. More details at
 * https://github.com/Guiraffo/ProVANT-Simulator/blob/master/LICENSE.md
 */
/**
 * @brief This file
 * @author Daniel Cardoso
 * @author Brenner Santana Rego
 * @todo Document this block and the functions declared in this file.
 * @todo As this Class contains only static methods, it can be removed in
 * favor of the declaration and implementation of this methods as functions.
 * If the cplexutils namespace is needed, please include a namespace block
 * replacing the class declaration.
 *
 * E-mail: brennersr7@ufmg.br
 */

#ifndef CPLEXUTILS_H
#define CPLEXUTILS_H

#include <eigen3/Eigen/Eigen>

class cplexutils
{
public:
  cplexutils()
  {
  }
  ~cplexutils()
  {
  }

  static void lp_old(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq,
                     Eigen::MatrixXd lb, Eigen::MatrixXd ub, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);

  static void lp(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq,
                 Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);

  static void lp_minmax(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq,
                        Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvaluemin,
                        double& objvaluemax, Eigen::MatrixXd& xstarmin, Eigen::MatrixXd& xstarmax, int& exitflag);

  static void lp_minmax_manyf_fixconst(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq,
                                       Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB,
                                       Eigen::MatrixXd& objvaluemin, Eigen::MatrixXd& objvaluemax, int& exitflag);

  static void qp(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq,
                 Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar,
                 int& exitflag);

  static void free_and_null(char** ptr);
};

#endif
