#include <iostream>
#include <eigen3/Eigen/Eigen>
#include "math.h"
#include "stdio.h"
#include <ilcplex/ilocplex.h>
#include <limits>

//using namespace Eigen;

/* A first implementation of constrained zonotopes in C++.

   Author: Brenner Santana Rego
   E-mail: brennersr7@ufmg.br   */


#ifndef CPLEXUTILS_H
#define CPLEXUTILS_H


class cplexutils
{

        public:
        
	cplexutils()
	{ 
		
	}
	~cplexutils()
	{
		
	}
	

        static void lp_old(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void lp(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void lp_minmax(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvaluemin, double& objvaluemax, Eigen::MatrixXd& xstarmin, Eigen::MatrixXd& xstarmax, int& exitflag);
        
        static void lp_minmax_manyf_fixconst(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, Eigen::MatrixXd& objvaluemin, Eigen::MatrixXd& objvaluemax, int& exitflag);                
        
        static void qp(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag);
        
        static void free_and_null(char **ptr);

};	

#endif

