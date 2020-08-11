#include "cplexutils.h"

//using namespace Eigen;

/* A first implementation of constrained zonotopes in C++.

   Author: Brenner Santana Rego
   E-mail: brennersr7@ufmg.br   */



void cplexutils::lp_old(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd lb, Eigen::MatrixXd ub, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{
        
	// CPLEX LP wrapper: Concert C++ version

	// Calls CPLEX to solve min f'x subject to
	//
	//                      A*x <= b
	//                      Aeq*x = beq
	//                      lb <= x <= ub

	// f, A, b, Aeq, beq, lb and ub are MatrixXd objects of appropriate dimensions

	// Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex

	// Code based on populatebynonzero

	IloEnv env;
	try {

                // Create CPLEX model, decision variables, constraints
		IloModel model(env);

		IloNumVarArray var(env);
		IloRangeArray con(env);

		// if (( argc != 2 )                         ||
		  // ( argv[1][0] != '-' )                 ||
		  // ( strchr ("rcn", argv[1][1]) == NULL )   ) {
		 // usage (argv[0]);
		 // throw(-1);
		// }

		//IloNumVarArray var(env);
		//IloRangeArray con(env);

                // Objective function
                IloObjective obj = IloMinimize(env);
                

                // Decision variables and lower/upper bounds
		// Populate by row code
		
		// switch (argv[1][1]) {
		 // case 'r':
			// populatebyrow (model, var, con);
			// break;
		 // case 'c':
			// populatebycolumn (model, var, con);
			// break;
		 // case 'n':
			// populatebynonzero (model, var, con);
			// break;
		// }		
		
                // Populate by nonzero code
		// populatebynonzero (IloModel model, IloNumVarArray x, IloRangeArray c)		
	  
                // IloEnv env = model.getEnv();

                //IloObjective obj = IloMaximize(env);

                if((lb.size()==0)||(ub.size()==0)) // No lower/upper bounds
                {
                        //printf("Entered [no lower/upper bounds]\n");
                        for(int j=0; j<f.rows(); j++)
                        {
                                //var.add(IloNumVar(env));
                                var.add(IloNumVar(env, -IloInfinity, IloInfinity)); // Corrected strange errors
                        }
                }
                else // With lower/upper bounds (only works if bounds are specified for EVERY decision variable)
                {
                        //printf("Entered [with lower/upper bounds]\n");
                        for(int j=0; j<f.rows(); j++)
                        {
                                var.add(IloNumVar(env, lb(j,0), ub(j,0)));
                                //std::cout << lb(j,0); printf(" "); std::cout << ub(j,0); printf("\n");
                        }
                }    
                
                
                
                // Objective function coefficients
                //printf("Entered [objective function coefficients]\n");                
                for(int j=0; j<f.rows(); j++)
                {
                        obj.setLinearCoef(var[j], f(j,0));
                        //std::cout << f(j,0); printf(" ");
                }                        
               // printf("\n");
   
   
                // Inequality constraints             
                for(int j=0; j<A.rows(); j++)
                {
                        //printf("Entered [inequality constraints]\n");  
                        con.add(IloRange(env, -IloInfinity, b(j,0)));
                        for(int i=0; i<A.cols(); i++)
                        {
                                                     
                               con[j].setLinearCoef(var[i], A(j,i)); 
                               //std::cout << A(j,i);  printf(" ");
                        }
                        //std::cout << b(j,0); printf("\n");
                } 
                
                
                // Equality constraints             
                for(int j=0; j<Aeq.rows(); j++)
                {
                        //printf("Entered [equality constraints]\n");                 
                        con.add(IloRange(env, beq(j,0), beq(j,0)));
                        for(int i=0; i<Aeq.cols(); i++)
                        {
                                                 
                               con[A.rows()+j].setLinearCoef(var[i], Aeq(j,i)); 
                               //std::cout << Aeq(j,i); printf(" ");
                        }
                        //std::cout << beq(j,0); printf("\n");                        
                }        
      
                
                model.add(obj);
                model.add(con);    
                
		IloCplex cplex(model);
		//cplex.exportModel("lpex1.lp");
		
		// Verbose = OFF
                cplex.setOut(env.getNullStream());


                exitflag = 0;
		// Optimize the problem and obtain solution.
		if ( !cplex.solve() ) {
		        env.error() << "Failed to optimize LP" << std::endl;
		        exitflag = -1;
		        //throw(-1);
		}  
		
                IloNumArray vals(env);
                cplex.getValues(vals, var);		
                
                objvalue = cplex.getObjValue(); // Optimal value of the objective function
                //std::cout << objvalue; std::cout << exitflag;
                for(int j=0; j<f.rows(); j++)
                {
                        xstar(j,0) = vals[j]; // Optimal solution
                }
		
//		IloNumArray vals(env);
//		env.out() << "Solution status = " << cplex.getStatus() << endl;
//		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
//		cplex.getValues(vals, var);
//		env.out() << "Values        = " << vals << endl;
//		cplex.getSlacks(vals, con);
//		env.out() << "Slacks        = " << vals << endl;
//		cplex.getDuals(vals, con);
//		env.out() << "Duals         = " << vals << endl;
//		cplex.getReducedCosts(vals, var);
//		env.out() << "Reduced Costs = " << vals << endl;		                                   
                

//                c.add(IloRange(env, -IloInfinity, 20.0));
//                c.add(IloRange(env, -IloInfinity, 30.0));

//                x.add(IloNumVar(env, 0.0, 40.0));
//                x.add(IloNumVar(env));
//                x.add(IloNumVar(env));

//                obj.setLinearCoef(x[0], 1.0);
//                obj.setLinearCoef(x[1], 2.0);
//                obj.setLinearCoef(x[2], 3.0);

//                c[0].setLinearCoef(x[0], -1.0);
//                c[0].setLinearCoef(x[1],  1.0);
//                c[0].setLinearCoef(x[2],  1.0);
//                c[1].setLinearCoef(x[0],  1.0);
//                c[1].setLinearCoef(x[1], -3.0);
//                c[1].setLinearCoef(x[2],  1.0);

//                c[0].setName("c1");
//                c[1].setName("c2");

//                x[0].setName("x1");
//                x[1].setName("x2");
//                x[2].setName("x3");

//                model.add(obj);
//                model.add(c);  
//	  
//	  
//	  

//		IloCplex cplex(model);
//		cplex.exportModel("lpex1.lp");

//		// Optimize the problem and obtain solution.
//		if ( !cplex.solve() ) {
//		 env.error() << "Failed to optimize LP" << endl;
//		 throw(-1);
//		}

//		IloNumArray vals(env);
//		env.out() << "Solution status = " << cplex.getStatus() << endl;
//		env.out() << "Solution value  = " << cplex.getObjValue() << endl;
//		cplex.getValues(vals, var);
//		env.out() << "Values        = " << vals << endl;
//		cplex.getSlacks(vals, con);
//		env.out() << "Slacks        = " << vals << endl;
//		cplex.getDuals(vals, con);
//		env.out() << "Duals         = " << vals << endl;
//		cplex.getReducedCosts(vals, var);
//		env.out() << "Reduced Costs = " << vals << endl;
	}
	catch (IloException& e) {
		std::cerr << "Concert exception caught: " << e << std::endl;
	}
	catch (...) {
		std::cerr << "Unknown exception caught" << std::endl;
	}

	env.end();	

}


void cplexutils::lp(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{
        
        // CPLEX LP Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];    
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
		int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup.\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem a in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvalue = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstar(j,0) = x[j]; // Optimal solution
        }
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                  
                   

}

void cplexutils::lp_minmax(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvaluemin, double& objvaluemax, Eigen::MatrixXd& xstarmin, Eigen::MatrixXd& xstarmax, int& exitflag)        
{
        
        // CPLEX LP Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];    
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
		int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Minimization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem b in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvaluemin = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstarmin(j,0) = x[j]; // Optimal solution
        }
        

       
        // Maximization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
        goto TERMINATE;
        }          
        
        
        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

//        cur_numrows = CPXgetnumrows (env, lp);
//        cur_numcols = CPXgetnumcols (env, lp);

//        x = (double *) malloc (cur_numcols * sizeof(double));
//        slack = (double *) malloc (cur_numrows * sizeof(double));
//        dj = (double *) malloc (cur_numcols * sizeof(double));
//        pi = (double *) malloc (cur_numrows * sizeof(double));

//        if ( x     == NULL ||
//        slack == NULL ||
//        dj    == NULL ||
//        pi    == NULL   ) {
//        status = CPXERR_NO_MEMORY;
//        fprintf (stderr, "Could not allocate memory for solution.\n");
//        goto TERMINATE;
//        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }      
        
        objvaluemax = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstarmax(j,0) = x[j]; // Optimal solution
        }          
        
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                  
                   

}

void cplexutils::lp_minmax_manyf_fixconst(Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, Eigen::MatrixXd& objvaluemin, Eigen::MatrixXd& objvaluemax, int& exitflag)
{
               
        // CPLEX LP Wrapper: Callable Libraries version

        // Calls CPLEX to solve min f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_LPs = f.cols();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.col(0).data();        
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows]; 
        int      objind[nof_x];
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        objind[j] = j;
                        
                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                              
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);
                        
                        objind[j] = j;                        

                        matbeg[j] = j*totalrows;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }                            
                }
        }                 
        
        // Allocating arrays (several for loops)
        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  
//        
//        if(nof_bounds==0)
//        {
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] = -CPX_INFBOUND;
//                        ub[j] =  CPX_INFBOUND;
//                }
//        }        
//        else
//        {
//                
//                for(j = 0; j<nof_x; j++)
//                {
//                        lb[j] =  LB(j,0);
//                        ub[j] =  UB(j,0);
//                }
//        }   
//        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
////        std::cout << "sense ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << sense[j] << " ";
////        }
////        std::cout << std::endl;
////        
////        std::cout << "rhs ";
////        for(j=0; j<totalrows; j++)
////        {
////                std::cout << rhs[j] << " ";
////        }
////        std::cout << std::endl;        
//        
////        
////        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
////                std::cout << matbeg[j] << " ";
//        }
////        std::cout << std::endl;        
//        
////        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
////                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
////        std::cout << std::endl;   


                        
        
//        double* fdata = f.data();
//        double* Adata = A.data();
//        double* bdata = b.data();
//        double* Aeqdata = Aeq.data();
//        double* beqdata = beq.data();        

	// Code based on lpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
        
//        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
//        Eigen::MatrixXd btotal(nof_ineq+nof_eq, nof_x);
//        if(nof_ineq!=0 && nof_eq==0)
//        {
//                Atotal = A;
//                btotal = b;
//        }
//        else if(nof_ineq==0 && nof_eq!=0)
//        {
//                Atotal = Aeq;
//                btotal = beq;
//        }
//        else
//        {
//                Atotal << A, Aeq;
//                btotal << b, beq;
//        }
        
//        int totalrows = nof_ineq+nof_eq;
//        
//        
//        int      matbeg[nof_x];
//        int      matind[nof_x*totalrows];        
//        double*  matval = Atotal.data();
//        double*  rhs = btotal.data();
//        char     sense[totalrows];  

//        /* Now create the new rows.  First, populate the arrays. */

//        rowname[0] = "c1";
//        sense[0]   = 'L';
//        rhs[0]     = 20.0;

//        rowname[1] = "c2";
//        sense[1]   = 'L';
//        rhs[1]     = 30.0;

//        status = CPXnewrows (env, lp, NUMROWS, rhs, sense, NULL, rowname);
//        if ( status )   goto TERMINATE;

//        /* Now add the new columns.  First, populate the arrays. */

//        obj[0] = 1.0;      obj[1] = 2.0;           obj[2] = 3.0;

//        matbeg[0] = 0;     matbeg[1] = 2;          matbeg[2] = 4;

//        matind[0] = 0;     matind[2] = 0;          matind[4] = 0;
//        matval[0] = -1.0;  matval[2] = 1.0;        matval[4] = 1.0;

//        matind[1] = 1;     matind[3] = 1;          matind[5] = 1;
//        matval[1] = 1.0;   matval[3] = -3.0;       matval[5] = 1.0;

//        lb[0] = 0.0;       lb[1] = 0.0;            lb[2] = 0.0;
//        ub[0] = 40.0;      ub[1] = CPX_INFBOUND;   ub[2] = CPX_INFBOUND;

//        colname[0] = "x1"; colname[1] = "x2";      colname[2] = "x3";

//        status = CPXaddcols (env, lp, NUMCOLS, NUMNZ, obj, matbeg, matind,
//                        matval, lb, ub, colname);
//        if ( status )  goto TERMINATE;
        
//        for(j=0; j<nof_ineq; j++)
//        {
//                sense[j] = 'L';
//        }
//        for(j=nof_ineq; j<nof_ineq+nof_eq; j++)
//        {
//                sense[j] = 'E';
//        }      
//        
//        std::cout << "sense ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << sense[j] << " ";
//        }
//        std::cout << std::endl;
//        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        
//        std::cout << "matbeg ";
//        for(j=0; j<nof_x; j++)
//        {
//                matbeg[j] = j*totalrows;
//                std::cout << matbeg[j] << " ";
//        }
//        std::cout << std::endl;        
//        
//        std::cout << "matind ";
//        for(j=0; j<nof_x; j++)
//        {
//                for(i=0; i<totalrows; i++)
//                {
//                        matind[j*totalrows + i] = i;
//                        std::cout << matind[j*totalrows+i] << " ";                        
//                }
//        }
//        std::cout << std::endl;        
        
        
        // Minimization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem c in: add cols.\n");
        goto TERMINATE;
        }
        

        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }


        // Get objective value
        objvaluemin(0,0) = objval;

       
        // Maximization
        
        // Setup problem
        
        status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
        goto TERMINATE;
        }          
        
        
        /* Optimize the problem and obtain solution. */

        status = CPXlpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize LP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

//        cur_numrows = CPXgetnumrows (env, lp);
//        cur_numcols = CPXgetnumcols (env, lp);

//        x = (double *) malloc (cur_numcols * sizeof(double));
//        slack = (double *) malloc (cur_numrows * sizeof(double));
//        dj = (double *) malloc (cur_numcols * sizeof(double));
//        pi = (double *) malloc (cur_numrows * sizeof(double));

//        if ( x     == NULL ||
//        slack == NULL ||
//        dj    == NULL ||
//        pi    == NULL   ) {
//        status = CPXERR_NO_MEMORY;
//        fprintf (stderr, "Could not allocate memory for solution.\n");
//        goto TERMINATE;
//        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }      
        
        
        // Get objective value
        objvaluemax(0,0) = objval;      
        
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;   

        
        // Solve remaining LPs
        for(int j=1; j<nof_LPs; j++)
        {
                // Change LP
                
                obj = f.col(j).data(); 
                status = CPXchgobj(env, lp, nof_x, objind, obj);  /* Problem is minimization */
                if ( status ){
                fprintf (stderr, "Failed to change objective function.\n");
                goto TERMINATE;
                }                   
                
                // Minimization
                
                // Setup problem
                
                status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
                if ( status ){
                fprintf (stderr, "Failed to populate problem in: problem setup (min).\n");
                goto TERMINATE;
                }      
                
                /* Optimize the problem and obtain solution. */

                status = CPXlpopt (env, lp);
                if ( status ) {
                fprintf (stderr, "Failed to optimize LP.\n");
                exitflag = -1;
                goto TERMINATE;
                }         
                
                
                status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
                if ( status ) {
                exitflag = -1;
                fprintf (stderr, "Failed to obtain solution.\n");
                goto TERMINATE;
                }      


                // Get objective value                
                objvaluemin(j,0) = objval;
              
                
                // Maximization
                
                // Setup problem
                
                status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
                if ( status ){
                fprintf (stderr, "Failed to populate problem in: problem setup (max).\n");
                goto TERMINATE;
                }          
                
                
                /* Optimize the problem and obtain solution. */

                status = CPXlpopt (env, lp);
                if ( status ) {
                fprintf (stderr, "Failed to optimize LP.\n");
                exitflag = -1;
                goto TERMINATE;
                }      
                
                status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
                if ( status ) {
                exitflag = -1;
                fprintf (stderr, "Failed to obtain solution.\n");
                goto TERMINATE;
                }      
                
                // Get objective value                  
                objvaluemax(j,0) = objval;                                            
                        
        }      

        TERMINATE:

        /* Free up the solution */

        cplexutils::free_and_null ((char **) &x);
        cplexutils::free_and_null ((char **) &slack);
        cplexutils::free_and_null ((char **) &dj);
        cplexutils::free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
                     
        
}

void cplexutils::qp(Eigen::MatrixXd H, Eigen::MatrixXd f, Eigen::MatrixXd A, Eigen::MatrixXd b, Eigen::MatrixXd Aeq, Eigen::MatrixXd beq, Eigen::MatrixXd LB, Eigen::MatrixXd UB, double& objvalue, Eigen::MatrixXd& xstar, int& exitflag)
{

        // CPLEX Quadratic Programming Wrapper: Callable Libraries version

        // Calls CPLEX to solve min 0.5*x'Hx + f'x subject to
        //
        //                      A*x <= b
        //                      Aeq*x = beq
        //                      LB <= x <= UB

        // H, f, A, b, Aeq, beq, LB and UB are MatrixXd objects of appropriate dimensions

        // Returns the optimal value of the objective function, the optimal value of x, and the exit flag from cplex
        
        
        exitflag = 0;
        
        int nof_x = f.rows();
        int nof_ineq = b.rows();
        int nof_eq = beq.rows();
        int nof_bounds = UB.rows();
        
        double* obj = f.data();
        double lb[nof_x];
        double ub[nof_x];    
        
        int i, j;        
        
        Eigen::MatrixXd Atotal(nof_ineq+nof_eq, nof_x);
        Eigen::MatrixXd btotal(nof_ineq+nof_eq, 1);
        if(nof_ineq!=0 && nof_eq==0)
        {
                Atotal = A;
                btotal = b;
        }
        else if(nof_ineq==0 && nof_eq!=0)
        {
                Atotal = Aeq;
                btotal = beq;
        }
        else
        {
                Atotal << A, Aeq;
                btotal << b, beq;
        }     
        
//        std::cout << "btotal " << btotal << std::endl;
        
        int totalrows = nof_ineq+nof_eq;
        
//        std::cout << "totalrows " << totalrows << std::endl;
        
        // Bounds on decision variables
        
//        double* obj = f.data();
//        double lb[nof_x];
//        double ub[nof_x];

        
        // Allocating arrays (one nested for loop)
        
        int      matbeg[nof_x];
        int      matind[nof_x*totalrows];        
        double*  matval = Atotal.data();
        double*  rhs = btotal.data();
        char     sense[totalrows];   
        int      qmatbeg[nof_x];
        int      qmatcnt[nof_x];
        int      qmatind[nof_x*nof_x];
        double   *qmatval = H.data();         
        
        if(nof_bounds==0)
        {
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] = -CPX_INFBOUND;
                        ub[j] =  CPX_INFBOUND;
                        
                        matbeg[j] = j*totalrows;
                        
                        qmatbeg[j] = j*nof_x;
                        qmatcnt[j] = nof_x;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }   
                         
                        for(i=0; i<nof_x; i++)
                        {
                                qmatind[j*nof_x + i] = i;
                        }                          
                                          
                }
        }        
        else
        {
                
                for(j = 0; j<nof_x; j++)
                {
                        lb[j] =  LB(j,0);
                        ub[j] =  UB(j,0);

                        matbeg[j] = j*totalrows;
                        
                        qmatbeg[j] = j*nof_x;
                        qmatcnt[j] = nof_x;
                        
                        for(i=0; i<nof_ineq; i++)
                        {
                                sense[i] = 'L';
                                matind[j*totalrows + i] = i;                      
                        }
                        for(i=nof_ineq; i<totalrows; i++)
                        {
                                sense[i] = 'E';
                                matind[j*totalrows + i] = i;                      
                        }   
                         
                        for(i=0; i<nof_x; i++)
                        {
                                qmatind[j*nof_x + i] = i;
                        }                             
                }
        }                 
        

	// Code based on lpex1.c and qpex1.c, populatebyrows and populatebycols
	
	int      solstat;
        double   objval;
        double   *x = NULL;
        double   *pi = NULL;
        double   *slack = NULL;
        double   *dj = NULL;
        
        CPXENVptr     env = NULL;
        CPXLPptr      lp = NULL;
        int           status = 0;
        int           cur_numrows, cur_numcols;    
        
        /* Initialize the CPLEX environment */

        env = CPXopenCPLEX (&status);

        /* If an error occurs, the status value indicates the reason for
        failure.  A call to CPXgeterrorstring will produce the text of
        the error message.  Note that CPXopenCPLEX produces no output,
        so the only way to see the cause of the error is to use
        CPXgeterrorstring.  For other CPLEX routines, the errors will
        be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON.  */

        if ( env == NULL ) {
        char  errmsg[CPXMESSAGEBUFSIZE];
        fprintf (stderr, "Could not open CPLEX environment.\n");
        CPXgeterrorstring (env, status, errmsg);
        fprintf (stderr, "%s", errmsg);
        goto TERMINATE;
        }
        
        /* Turn on output to the screen */

        status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_ON);
        //status = CPXsetintparam (env, CPXPARAM_ScreenOutput, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }
        status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_ON);
        //status = CPXsetintparam (env, CPX_PARAM_SCRIND, CPX_OFF);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on screen indicator, error %d.\n", status);
        goto TERMINATE;
        }        
        

        /* Turn on data checking */

        status = CPXsetintparam (env, CPXPARAM_Read_DataCheck,
	                    CPX_DATACHECK_WARN);
        if ( status ) {
        fprintf (stderr, 
               "Failure to turn on data checking, error %d.\n", status);
        goto TERMINATE;
        }   
        
        /* Create the problem. */

        lp = CPXcreateprob (env, &status, "lpex1");

        /* A returned pointer of NULL may mean that not enough memory
        was available or there was some other problem.  In the case of 
        failure, an error message will have been written to the error 
        channel from inside CPLEX.  In this example, the setting of
        the parameter CPXPARAM_ScreenOutput causes the error message to
        appear on stdout.  */

        if ( lp == NULL ) {
        fprintf (stderr, "Failed to create LP.\n");
        goto TERMINATE;
        }             
        
            
        
        
        // Setup problem
        
        //status = CPXchgobjsen (env, lp, CPX_MAX);  /* Problem is maximization */
        status = CPXchgobjsen (env, lp, CPX_MIN);  /* Problem is minimization */
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: problem setup.\n");
        goto TERMINATE;
        }     
        
        
        // Create rows     
        
        status = CPXnewrows (env, lp, totalrows, rhs, sense, NULL, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem in: create rows.\n");
        goto TERMINATE;
        }        
        
        // Add cols
        
        status = CPXaddcols (env, lp, nof_x, nof_x*totalrows, obj, matbeg, matind,
                                matval, lb, ub, NULL);
        if ( status ){
        fprintf (stderr, "Failed to populate problem d in: add cols.\n");
        goto TERMINATE;
        }
        
        
        /* Now copy the QP part of the problem data into the lp */

        status = CPXcopyquad (env, lp, qmatbeg, qmatcnt, qmatind, qmatval);
        if ( status ) {
        fprintf (stderr, "Failed to copy quadratic matrix.\n");
        goto TERMINATE;
        }        
        

        /* Optimize the problem and obtain solution. */

        status = CPXqpopt (env, lp);
        if ( status ) {
        fprintf (stderr, "Failed to optimize QP.\n");
        exitflag = -1;
        goto TERMINATE;
        }

        /* The size of the problem should be obtained by asking CPLEX what
        the actual size is, rather than using sizes from when the problem
        was built.  cur_numrows and cur_numcols store the current number 
        of rows and columns, respectively.  */

        cur_numrows = CPXgetnumrows (env, lp);
        cur_numcols = CPXgetnumcols (env, lp);

        x = (double *) malloc (cur_numcols * sizeof(double));
        slack = (double *) malloc (cur_numrows * sizeof(double));
        dj = (double *) malloc (cur_numcols * sizeof(double));
        pi = (double *) malloc (cur_numrows * sizeof(double));

        if ( x     == NULL ||
        slack == NULL ||
        dj    == NULL ||
        pi    == NULL   ) {
        status = CPXERR_NO_MEMORY;
        fprintf (stderr, "Could not allocate memory for solution.\n");
        goto TERMINATE;
        }

        status = CPXsolution (env, lp, &solstat, &objval, x, pi, slack, dj);
        if ( status ) {
        exitflag = -1;
        fprintf (stderr, "Failed to obtain solution.\n");
        goto TERMINATE;
        }

//        /* Write the output to the screen. */

//        printf ("\nSolution status = %d\n", solstat);
//        printf ("Solution value  = %f\n\n", objval);

//        for (i = 0; i < cur_numrows; i++) {
//        printf ("Row %d:  Slack = %10f  Pi = %10f\n", i, slack[i], pi[i]);
//        }

//        for (j = 0; j < cur_numcols; j++) {
//        printf ("Column %d:  Value = %10f  Reduced cost = %10f\n",
//              j, x[j], dj[j]);
//        }

//        /* Finally, write a copy of the problem to a file. */

//        status = CPXwriteprob (env, lp, "lpex1.lp", NULL);
//        if ( status ) {
//        fprintf (stderr, "Failed to write LP to disk.\n");
//        goto TERMINATE;
//        }

        objvalue = objval;
        for(int j=0; j<nof_x; j++)
        {
                xstar(j,0) = x[j]; // Optimal solution
        }
        
//        std::cout << "rhs ";
//        for(j=0; j<totalrows; j++)
//        {
//                std::cout << rhs[j] << " ";
//        }
//        std::cout << std::endl;         

        TERMINATE:

        /* Free up the solution */

        free_and_null ((char **) &x);
        free_and_null ((char **) &slack);
        free_and_null ((char **) &dj);
        free_and_null ((char **) &pi);

        /* Free up the problem as allocated by CPXcreateprob, if necessary */

        if ( lp != NULL ) {
        status = CPXfreeprob (env, &lp);
        if ( status ) {
         fprintf (stderr, "CPXfreeprob failed, error code %d.\n", status);
        }
        }

        /* Free up the CPLEX environment, if necessary */

        if ( env != NULL ) {
        status = CPXcloseCPLEX (&env);

        /* Note that CPXcloseCPLEX produces no output,
         so the only way to see the cause of the error is to use
         CPXgeterrorstring.  For other CPLEX routines, the errors will
         be seen if the CPXPARAM_ScreenOutput indicator is set to CPX_ON. */

        if ( status ) {
         char  errmsg[CPXMESSAGEBUFSIZE];
         fprintf (stderr, "Could not close CPLEX environment.\n");
         CPXgeterrorstring (env, status, errmsg);
         fprintf (stderr, "%s", errmsg);
        }
        }
        
        
}


void cplexutils::free_and_null (char **ptr)
{
   if ( *ptr != NULL ) {
      free (*ptr);
      *ptr = NULL;
   }
} /* END free_and_null */ 



//};	
