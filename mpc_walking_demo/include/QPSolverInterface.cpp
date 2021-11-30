#pragma once

#include <Eigen/Geometry>
#include <Eigen/Core>
#include <iostream>

#include <blasfeo_target.h>
#include <blasfeo_common.h>
#include <blasfeo_v_aux_ext_dep.h>
#include <blasfeo_d_aux_ext_dep.h>
#include <blasfeo_i_aux_ext_dep.h>
#include <blasfeo_d_aux.h>
#include <blasfeo_d_blas.h>

#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_timing.h>


inline Eigen::VectorXd solveQP_hpipm(Eigen::MatrixXd& costFunctionH, Eigen::VectorXd& costFunctionF, 
		Eigen::MatrixXd& AConstraint, Eigen::VectorXd& bConstraintMin, Eigen::VectorXd& bConstraintMax, Eigen::MatrixXd& AeqZ, Eigen::VectorXd& beqZ) {

    int n_variables = costFunctionH.cols();
    int n_constr = AConstraint.rows() ;

    int nv = n_variables;
    int ne = AeqZ.rows();

    int nb = 0; 
    int ng = n_constr;
    int ns = 0; 
    int nsb = 0; 
    int nsg = 0;
    int idxb[n_variables] = {};
    double H[n_variables*n_variables] = {};
    double g[n_variables] = {};
    double d_lb[n_constr] = {};
    double d_ub[n_constr] = {};
    double _lb[n_constr] = {};
    double _ub[n_constr] = {};
    double C[n_constr*n_variables] = {};
    
    double A[ne*n_variables] = {};
    double b[ne] = {};

    for (int i = 0; i<n_variables; i++) {
        //idxb[i] = i;
        g[i] = costFunctionF(i);

        for (int j = 0; j<n_variables; j++) {
            H[j*n_variables+i] = costFunctionH(i,j);
        }
    }


    for (int k = 0; k<n_constr; k++) {
        d_lb[k] = bConstraintMin(k); //bConstraintMin(k);
        d_ub[k] = bConstraintMax(k); //bConstraintMax(k);

        for (int j = 0; j<n_variables; j++) {
            C[j*n_constr+k] = AConstraint(k,j); //AConstraint(k,j);
        }
    }

    for (int k = 0; k<ne; k++) {
        b[k] = beqZ(k); 
        for (int j = 0; j<n_variables; j++) {
            A[j*ne+k] = AeqZ(k,j); //AConstraint(k,j);
        }
    }

    int dim_size = d_dense_qp_dim_memsize();
    void *dim_mem = malloc(dim_size);
    struct d_dense_qp_dim dim;
    d_dense_qp_dim_create(&dim, dim_mem);

    d_dense_qp_dim_set_all(nv, ne, nb, ng, nsb, nsg, &dim);

    int qp_size = d_dense_qp_memsize(&dim);
    void *qp_mem = malloc(qp_size);
    struct d_dense_qp qp;
    d_dense_qp_create(&dim, &qp, qp_mem);

    d_dense_qp_set_H(H, &qp);
    d_dense_qp_set_g(g, &qp);

    d_dense_qp_set_C(C, &qp);
    d_dense_qp_set_lg(d_lb, &qp);
    d_dense_qp_set_ug(d_ub, &qp);

    d_dense_qp_set_A(A, &qp);
    d_dense_qp_set_b(b, &qp);

    // allocate memory for the solution

    int qp_sol_size = d_dense_qp_sol_memsize(&dim);
    void *qp_sol_mem = malloc(qp_sol_size);
    struct d_dense_qp_sol qp_sol;
    d_dense_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

    // allocate memory for ipm solver and its workspace

    int ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dim);
    void *ipm_arg_mem = malloc(ipm_arg_size);
    struct d_dense_qp_ipm_arg arg;
    d_dense_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
    enum hpipm_mode mode = SPEED;//ROBUST;   // set mode ROBUST, SPEED, BALANCE, SPEED_ABS
    d_dense_qp_ipm_arg_set_default(mode, &arg);

    int ipm_size = d_dense_qp_ipm_ws_memsize(&dim, &arg);
    void *ipm_mem = malloc(ipm_size);
    struct d_dense_qp_ipm_ws workspace;
    d_dense_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

    // solve QP

    d_dense_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);

    // store solution in an Eigen Vector

    int nu_max = n_variables;   // see here what is missing
    Eigen::VectorXd u_store = Eigen::VectorXd::Zero(nu_max);
    double *u = (double *) malloc(nu_max*sizeof(double));
    d_dense_qp_sol_get_v(&qp_sol, u);
    for (int i=0; i<n_variables; i++) u_store(i) = u[i];

    // free memory

    free(u);
    free(dim_mem);
    free(qp_mem);
    free(qp_sol_mem);
    free(ipm_arg_mem);
    free(ipm_mem);

    return u_store;
}



