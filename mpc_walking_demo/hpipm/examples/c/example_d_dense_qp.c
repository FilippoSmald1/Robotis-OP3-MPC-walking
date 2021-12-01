/*
#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

#include <blasfeo_d_aux_ext_dep.h>

#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_timing.h>
/**/


#include <stdlib.h>
#include <stdio.h>
#include <sys/time.h>

#include <blasfeo_d_aux_ext_dep.h>

#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>

// QP size

// horizon lenght
int N = 5;
// number of input
static int nnu[6] = {1, 1, 1, 1, 1, 0};
// number of states
static int nnx[6] = {2, 2, 2, 2, 2, 2};
// number of input box constraints
static int nnbu[6] = {0, 0, 0, 0, 0, 0};
// number of states box constraints
static int nnbx[6] = {2, 0, 0, 0, 0, 0};
// number of general constraints
static int nng[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on state box constraints
static int nnsbx[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on input box constraints
static int nnsbu[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on general constraints
static int nnsg[6] = {0, 0, 0, 0, 0, 0};


// QP data

//
static double A[] = {1, 0, 1, 1};
//
static double B[] = {0, 1};
//
static double b[] = {0, 0};

//
static double Q[] = {1, 0, 0, 1};
//
static double R[] = {1};
//
static double S[] = {0, 0};
//
static double q[] = {1, 1};
//
static double r[] = {0};

//
static double lbx0[] = {1, 1};
//
static double ubx0[] = {1, 1};
//
static int idxbx0[] = {0, 1};

//
static double u_guess[] = {0};
//
static double x_guess[] = {0, 0};
//
static double sl_guess[] = {};
//
static double su_guess[] = {};

// array of pointers

//
static double *AA[5] = {A, A, A, A, A};
//
static double *BB[5] = {B, B, B, B, B};
//
static double *bb[5] = {b, b, b, b, b};
//
static double *QQ[6] = {Q, Q, Q, Q, Q, Q};
//
static double *RR[6] = {R, R, R, R, R, R};
//
static double *SS[6] = {S, S, S, S, S, S};
//
static double *qq[6] = {q, q, q, q, q, q};
//
static double *rr[6] = {r, r, r, r, r, r};
//
static int *iidxbx[6] = {idxbx0, NULL, NULL, NULL, NULL, NULL};
//
static double *llbx[6] = {lbx0, NULL, NULL, NULL, NULL, NULL};
//
static double *uubx[6] = {ubx0, NULL, NULL, NULL, NULL, NULL};
//
static int *iidxbu[6] = {};
//
static double *llbu[6] = {};
//
static double *uubu[6] = {};
//
static double *CC[6] = {};
//
static double *DD[6] = {};
//
static double *llg[6] = {};
//
static double *uug[6] = {};
//
static double *ZZl[6] = {};
//
static double *ZZu[6] = {};
//
static double *zzl[6] = {};
//
static double *zzu[6] = {};
//
static int *iidxs[6] = {};
//
static double *llls[6] = {};
//
static double *llus[6] = {};

//
static double *uu_guess[6] = {u_guess, u_guess, u_guess, u_guess, u_guess, u_guess};
//
static double *xx_guess[6] = {x_guess, x_guess, x_guess, x_guess, x_guess, x_guess};
//
static double *ssl_guess[6] = {sl_guess, sl_guess, sl_guess, sl_guess, sl_guess, sl_guess};
//
static double *ssu_guess[6] = {su_guess, su_guess, su_guess, su_guess, su_guess, su_guess};



// export as global data

int *nu = nnu;
int *nx = nnx;
int *nbu = nnbu;
int *nbx = nnbx;
int *ng = nng;
int *nsbx = nnsbx;
int *nsbu = nnsbu;
int *nsg = nnsg;

double **hA = AA;
double **hB = BB;
double **hb = bb;
double **hQ = QQ;
double **hR = RR;
double **hS = SS;
double **hq = qq;
double **hr = rr;
int **hidxbx = iidxbx;
double **hlbx = llbx;
double **hubx = uubx;
int **hidxbu = iidxbu;
double **hlbu = llbu;
double **hubu = uubu;
double **hC = CC;
double **hD = DD;
double **hlg = llg;
double **hug = uug;
double **hZl = ZZl;
double **hZu = ZZu;
double **hzl = zzl;
double **hzu = zzu;
int **hidxs = iidxs;
double **hlls = llls;
double **hlus = llus;

double **hu_guess = uu_guess;
double **hx_guess = xx_guess;
double **hsl_guess = ssl_guess;
double **hsu_guess = ssu_guess;

// arg
int mode = 1;
int iter_max = 30;
double alpha_min = 1e-8;
double mu0 = 1e4;
double tol_stat = 1e-4;
double tol_eq = 1e-5;
double tol_ineq = 1e-5;
double tol_comp = 1e-5;
double reg_prim = 1e-12;
int warm_start = 0;
int pred_corr = 1;
int ric_alg = 0;



// main
int main()
	{

        int c = getchar();

	int ii, jj;

	int hpipm_status;

	int rep, nrep=10;

	struct timeval tv0, tv1;

/************************************************
* ocp qp dim
************************************************/

	int dim_size = d_ocp_qp_dim_memsize(N);
	void *dim_mem = malloc(dim_size);

	struct d_ocp_qp_dim dim;
	d_ocp_qp_dim_create(N, &dim, dim_mem);

	d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, &dim);

/************************************************
* ocp qp
************************************************/

	int qp_size = d_ocp_qp_memsize(&dim);
	void *qp_mem = malloc(qp_size);

	struct d_ocp_qp qp;
	d_ocp_qp_create(&dim, &qp, qp_mem);

	d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, &qp);

/************************************************
* ocp qp sol
************************************************/

	int qp_sol_size = d_ocp_qp_sol_memsize(&dim);
	void *qp_sol_mem = malloc(qp_sol_size);

	struct d_ocp_qp_sol qp_sol;
	d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);

/************************************************
* ipm arg
************************************************/

	int ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
	void *ipm_arg_mem = malloc(ipm_arg_size);

	struct d_ocp_qp_ipm_arg arg;
	d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);

	d_ocp_qp_ipm_arg_set_default(mode, &arg);



/************************************************
* ipm workspace
************************************************/

	int ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
	void *ipm_mem = malloc(ipm_size);

	struct d_ocp_qp_ipm_ws workspace;
	d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);

/************************************************
* ipm solver
************************************************/

//	gettimeofday(&tv0, NULL); // start
	hpipm_timer timer;
	hpipm_tic(&timer);

	for(rep=0; rep<nrep; rep++)
		{
		// solution guess
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_u(ii, hu_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_x(ii, hx_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_sl(ii, hsl_guess[ii], &qp_sol);
//		for(ii=0; ii<=N; ii++)
//			d_cvt_colmaj_to_ocp_qp_sol_su(ii, hsu_guess[ii], &qp_sol);

		// call solver
		d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
		d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
		}

//	gettimeofday(&tv1, NULL); // stop
//	double time_ipm = (tv1.tv_sec-tv0.tv_sec)/(nrep+0.0)+(tv1.tv_usec-tv0.tv_usec)/(nrep*1e6);
	double time_ipm = hpipm_toc(&timer) / nrep;

// XXX
//exit(1);

/************************************************
* print solution info
************************************************/

    printf("\nHPIPM returned with flag %i.\n", hpipm_status);
    if(hpipm_status == 0)
		{
        printf("\n -> QP solved!\n");
		}
	else if(hpipm_status==1)
		{
        printf("\n -> Solver failed! Maximum number of iterations reached\n");
		}
	else if(hpipm_status==2)
		{
        printf("\n -> Solver failed! Minimum step lenght reached\n");
		}
	else if(hpipm_status==2)
		{
        printf("\n -> Solver failed! NaN in computations\n");
		}
	else
		{
        printf("\n -> Solver failed! Unknown return flag\n");
		}
    printf("\nAverage solution time over %i runs: %e [s]\n", nrep, time_ipm);
	printf("\n\n");

/************************************************
* extract and print solution
************************************************/

	// u

	int nu_max = nu[0];
	for(ii=1; ii<=N; ii++)
		if(nu[ii]>nu_max)
			nu_max = nu[ii];

	double *u = malloc(nu_max*sizeof(double));

	printf("\nu = \n");
	for(ii=0; ii<=N; ii++)
		{
		d_ocp_qp_sol_get_u(ii, &qp_sol, u);
		d_print_mat(1, nu[ii], u, 1);
		}

	// x

	int nx_max = nx[0];
	for(ii=1; ii<=N; ii++)
		if(nx[ii]>nx_max)
			nx_max = nx[ii];

	double *x = malloc(nx_max*sizeof(double));

	printf("\nx = \n");
	for(ii=0; ii<=N; ii++)
		{
		d_ocp_qp_sol_get_x(ii, &qp_sol, x);
		d_print_mat(1, nx[ii], x, 1);
		}

	// pi
	double *pi = malloc(nx_max*sizeof(double));

	printf("\npi = \n");
	for(ii=0; ii<N; ii++)
		{
		d_ocp_qp_sol_get_pi(ii, &qp_sol, pi);
		d_print_mat(1, nx[ii+1], pi, 1);
		}

/************************************************
* print ipm statistics
************************************************/

	int iter; d_ocp_qp_ipm_get_iter(&workspace, &iter);
	double res_stat; d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
	double res_eq; d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
	double res_ineq; d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
	double res_comp; d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
	double *stat; d_ocp_qp_ipm_get_stat(&workspace, &stat);
	int stat_m; d_ocp_qp_ipm_get_stat_m(&workspace, &stat_m);

	printf("\nipm return = %d\n", hpipm_status);
	printf("\nipm residuals max: res_g = %e, res_b = %e, res_d = %e, res_m = %e\n", res_stat, res_eq, res_ineq, res_comp);

	printf("\nipm iter = %d\n", iter);
	printf("\nalpha_aff\tmu_aff\t\tsigma\t\talpha_prim\talpha_dual\tmu\t\tres_stat\tres_eq\t\tres_ineq\tres_comp\tlq fact\t\titref pred\titref corr\tlin res stat\tlin res eq\tlin res ineq\tlin res comp\n");
	d_print_exp_tran_mat(stat_m, iter+1, stat, stat_m);

	printf("\nocp ipm time = %e [s]\n\n", time_ipm);

/************************************************
* get riccati matrices and vectors
************************************************/

#if 1
	printf("\nget Riccati recursion matrices and vectors\n");

	double *Lr0 = malloc(nu[0]*nu[0]*sizeof(double));
	double *Ls0 = malloc(nx[0]*nu[0]*sizeof(double));
	double *P0 = malloc(nx[0]*nx[0]*sizeof(double));
	double *lr0 = malloc(nu[0]*sizeof(double));
	double *p0 = malloc(nx[0]*sizeof(double));
	double *K0 = malloc(nu[0]*nx[0]*sizeof(double));
	double *k0 = malloc(nu[0]*sizeof(double));

	double *Lr1 = malloc(nu[1]*nu[1]*sizeof(double));
	double *Ls1 = malloc(nx[1]*nu[1]*sizeof(double));
	double *P1 = malloc(nx[1]*nx[1]*sizeof(double));
	double *lr1 = malloc(nu[1]*sizeof(double));
	double *p1 = malloc(nx[1]*sizeof(double));
	double *K1 = malloc(nu[1]*nx[1]*sizeof(double));
	double *k1 = malloc(nu[1]*sizeof(double));

	//
	d_ocp_qp_ipm_get_ric_Lr(&qp, &arg, &workspace, 0, Lr0);
	printf("\nLr0\n");
	d_print_exp_mat(nu[0], nu[0], Lr0, nu[0]);
	//
	d_ocp_qp_ipm_get_ric_Ls(&qp, &arg, &workspace, 0, Ls0);
	printf("\nLs0\n");
	d_print_exp_mat(nx[0], nu[0], Ls0, nx[0]);
	//
	d_ocp_qp_ipm_get_ric_P(&qp, &arg, &workspace, 0, P0);
	printf("\nP0\n");
	d_print_exp_mat(nx[0], nx[0], P0, nx[0]);
	//
	d_ocp_qp_ipm_get_ric_lr(&qp, &arg, &workspace, 0, lr0);
	printf("\nlr0\n");
	d_print_exp_mat(1, nu[0], lr0, 1);
	//
	d_ocp_qp_ipm_get_ric_p(&qp, &arg, &workspace, 0, p0);
	printf("\np0\n");
	d_print_exp_mat(1, nx[0], p0, 1);
	//
	d_ocp_qp_ipm_get_ric_K(&qp, &arg, &workspace, 0, K0);
	printf("\nK0\n");
	d_print_exp_mat(nu[0], nx[0], K0, nu[0]);
	//
	d_ocp_qp_ipm_get_ric_k(&qp, &arg, &workspace, 0, k0);
	printf("\nk0\n");
	d_print_exp_mat(1, nu[0], k0, 1);

	//
	d_ocp_qp_ipm_get_ric_Lr(&qp, &arg, &workspace, 1, Lr1);
	printf("\nLr1\n");
	d_print_exp_mat(nu[1], nu[1], Lr1, nu[1]);
	//
	d_ocp_qp_ipm_get_ric_Ls(&qp, &arg, &workspace, 1, Ls1);
	printf("\nLs1\n");
	d_print_exp_mat(nx[1], nu[1], Ls1, nx[1]);
	//
	d_ocp_qp_ipm_get_ric_P(&qp, &arg, &workspace, 1, P1);
	printf("\nP1\n");
	d_print_exp_mat(nx[1], nx[1], P1, nx[1]);
	//
	d_ocp_qp_ipm_get_ric_lr(&qp, &arg, &workspace, 1, lr1);
	printf("\nlr1\n");
	d_print_exp_mat(1, nu[1], lr1, 1);
	//
	d_ocp_qp_ipm_get_ric_p(&qp, &arg, &workspace, 1, p1);
	printf("\np1\n");
	d_print_exp_mat(1, nx[1], p1, 1);
	//
	d_ocp_qp_ipm_get_ric_K(&qp, &arg, &workspace, 1, K1);
	printf("\nK1\n");
	d_print_exp_mat(nu[1], nx[1], K1, nu[1]);
	//
	d_ocp_qp_ipm_get_ric_k(&qp, &arg, &workspace, 1, k1);
	printf("\nk1\n");
	d_print_exp_mat(1, nu[1], k1, 1);

	free(Lr0);
	free(Ls0);
	free(P0);
	free(lr0);
	free(p0);
	free(K0);
	free(k0);

	free(Lr1);
	free(Ls1);
	free(P1);
	free(lr1);
	free(p1);
	free(K1);
	free(k1);
#endif

/************************************************
* free memory and return
************************************************/

    free(dim_mem);
    free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem);

	free(u);
	free(x);
	free(pi);

        c = getchar();
	return 0;

	}


