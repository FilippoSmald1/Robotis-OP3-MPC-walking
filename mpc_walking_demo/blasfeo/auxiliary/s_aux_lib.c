/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "../include/blasfeo_common.h"

#define REAL float
#define MAT blasfeo_smat
#define VEC blasfeo_svec



#if defined(LA_REFERENCE) | defined(LA_EXTERNAL_BLAS_WRAPPER)


#define MEMSIZE_MAT blasfeo_memsize_smat
#define MEMSIZE_DIAG_MAT blasfeo_memsize_diag_smat
#define MEMSIZE_VEC blasfeo_memsize_svec

#define CREATE_MAT blasfeo_create_smat
#define CREATE_VEC blasfeo_create_svec

#define PACK_MAT blasfeo_pack_smat
#define PACK_TRAN_MAT blasfeo_pack_tran_smat
#define PACK_VEC blasfeo_pack_svec
#define UNPACK_MAT blasfeo_unpack_smat
#define UNPACK_TRAN_MAT blasfeo_unpack_tran_smat
#define UNPACK_VEC blasfeo_unpack_svec
#define CAST_MAT2STRMAT s_cast_mat2strmat
#define CAST_DIAG_MAT2STRMAT s_cast_diag_mat2strmat
#define CAST_VEC2VECMAT s_cast_vec2vecmat


#define GEAD_LIBSTR blasfeo_sgead
#define GECP_LIBSTR blasfeo_sgecp
#define GECPSC_LIBSTR blasfeo_sgecpsc
#define GESC_LIBSTR blasfeo_sgesc
#define GESE_LIBSTR blasfeo_sgese




// insert element into strmat
void blasfeo_sgein1(float a, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	pA[0] = a;
	return;
	}



// extract element from strmat
float blasfeo_sgeex1(struct blasfeo_smat *sA, int ai, int aj)
	{
	if (ai==aj)
		{
		// invalidate stored inverse diagonal
		sA->use_dA = 0;
		}

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	return pA[0];
	}



// insert element into strvec
void blasfeo_svecin1(float a, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	x[0] = a;
	return;
	}



// extract element from strvec
float blasfeo_svecex1(struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	return x[0];
	}



// set all elements of a strvec to a value
void blasfeo_svecse(int m, float alpha, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<m; ii++)
		x[ii] = alpha;
	return;
	}



// extract diagonal to vector
void blasfeo_sdiaex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = alpha*pA[ii*(lda+1)];
	return;
	}



// insert a vector into diagonal
void blasfeo_sdiain(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii*(lda+1)] = alpha*x[ii];
	return;
	}



// add scalar to diagonal
void blasfeo_sdiare(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii*(lda+1)] += alpha;
	return;
	}



// extract a row into a vector
void blasfeo_srowex(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = alpha*pA[ii*lda];
	return;
	}



// insert a vector into a row
void blasfeo_srowin(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii*lda] = alpha*x[ii];
	return;
	}



// add a vector to a row
void blasfeo_srowad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii*lda] += alpha*x[ii];
	return;
	}



// swap two rows of two matrix structs
void blasfeo_srowsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*lda;
	int ii;
	float tmp;
	for(ii=0; ii<kmax; ii++)
		{
		tmp = pA[ii*lda];
		pA[ii*lda] = pC[ii*ldc];
		pC[ii*ldc] = tmp;
		}
	return;
	}



// permute the rows of a matrix struct
void blasfeo_srowpe(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_srowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// inverse permute the rows of a matrix struct
void blasfeo_srowpei(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_srowsw(sA->n, sA, ii, 0, sA, ipiv[ii], 0);
		}
	return;
	}



// extract a vector from a column of a matrix struct
void blasfeo_scolex(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_svec *sx, int xi)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		x[ii] = pA[ii];
	return;
	}



// insert a vector into a column of a matrix struct
void blasfeo_scolin(int kmax, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii] = x[ii];
	return;
	}



// add a scaled vector to a column of a matrix struct
void blasfeo_scolad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii] += alpha*x[ii];
	return;
	}



// scale a column
void blasfeo_scolsc(int kmax, float alpha, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii] *= alpha;
	return;
	}



// swap two cols of two matrix structs
void blasfeo_scolsw(int kmax, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*lda;
	int ii;
	float tmp;
	for(ii=0; ii<kmax; ii++)
		{
		tmp = pA[ii];
		pA[ii] = pC[ii];
		pC[ii] = tmp;
		}
	return;
	}



// permute the cols of a matrix struct
void blasfeo_scolpe(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			blasfeo_scolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// inverse permute the cols of a matrix struct
void blasfeo_scolpei(int kmax, int *ipiv, struct blasfeo_smat *sA)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int ii;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			blasfeo_scolsw(sA->m, sA, 0, ii, sA, 0, ipiv[ii]);
		}
	return;
	}



// copy a strvec into a strvec
void blasfeo_sveccp(int m, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] = pa[ii+0];
		pc[ii+1] = pa[ii+1];
		pc[ii+2] = pa[ii+2];
		pc[ii+3] = pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] = pa[ii+0];
		}
	return;
	}



// scale a strvec
void blasfeo_svecsc(int m, float alpha, struct blasfeo_svec *sa, int ai)
	{
	float *pa = sa->pa + ai;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pa[ii+0] *= alpha;
		pa[ii+1] *= alpha;
		pa[ii+2] *= alpha;
		pa[ii+3] *= alpha;
		}
	for(; ii<m; ii++)
		{
		pa[ii+0] *= alpha;
		}
	return;
	}



// copy and scale a strvec into a strvec
void blasfeo_sveccpsc(int m, float alpha, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] = alpha*pa[ii+0];
		pc[ii+1] = alpha*pa[ii+1];
		pc[ii+2] = alpha*pa[ii+2];
		pc[ii+3] = alpha*pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] = alpha*pa[ii+0];
		}
	return;
	}



// copy a lower triangular strmat into a lower triangular strmat
void blasfeo_strcp_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*ldc;
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = jj;
		for(; ii<m; ii++)
			{
			pC[ii+0+jj*ldc] = pA[ii+0+jj*lda];
			}
		}
	return;
	}



// scales and adds a strvec into a strvec
void blasfeo_svecad(int m, float alpha, struct blasfeo_svec *sa, int ai, struct blasfeo_svec *sc, int ci)
	{
	float *pa = sa->pa + ai;
	float *pc = sc->pa + ci;
	int ii;
	ii = 0;
	for(; ii<m-3; ii+=4)
		{
		pc[ii+0] += alpha*pa[ii+0];
		pc[ii+1] += alpha*pa[ii+1];
		pc[ii+2] += alpha*pa[ii+2];
		pc[ii+3] += alpha*pa[ii+3];
		}
	for(; ii<m; ii++)
		{
		pc[ii+0] += alpha*pa[ii+0];
		}
	return;
	}



// copy and transpose a generic strmat into a generic strmat
void blasfeo_sgetr(int m, int n, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*ldc;
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		ii = 0;
		for(; ii<m-3; ii+=4)
			{
			pC[jj+(ii+0)*ldc] = pA[ii+0+jj*lda];
			pC[jj+(ii+1)*ldc] = pA[ii+1+jj*lda];
			pC[jj+(ii+2)*ldc] = pA[ii+2+jj*lda];
			pC[jj+(ii+3)*ldc] = pA[ii+3+jj*lda];
			}
		for(; ii<m; ii++)
			{
			pC[jj+(ii+0)*ldc] = pA[ii+0+jj*lda];
			}
		}
	return;
	}



// copy and transpose a lower triangular strmat into an upper triangular strmat
void blasfeo_strtr_l(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*ldc;
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = jj;
		for(; ii<m; ii++)
			{
			pC[jj+(ii+0)*ldc] = pA[ii+0+jj*lda];
			}
		}
	return;
	}



// copy and transpose an upper triangular strmat into a lower triangular strmat
void blasfeo_strtr_u(int m, struct blasfeo_smat *sA, int ai, int aj, struct blasfeo_smat *sC, int ci, int cj)
	{
	// invalidate stored inverse diagonal
	sC->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	int ldc = sC->m;
	float *pC = sC->pA + ci + cj*ldc;
	int ii, jj;
	for(jj=0; jj<m; jj++)
		{
		ii = 0;
		for(; ii<=jj; ii++)
			{
			pC[jj+(ii+0)*ldc] = pA[ii+0+jj*lda];
			}
		}
	return;
	}



// insert a strvec to the diagonal of a strmat, sparse formulation
void blasfeo_sdiain_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	float *x = sx->pa + xi;
	int ldd = sD->m;
	float *pD = sD->pA + di + dj*ldd;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*(ldd+1)] = alpha * x[jj];
		}
	return;
	}



// extract the diagonal of a strmat from a strvec , sparse formulation
void blasfeo_sdiaex_sp(int kmax, float alpha, int *idx, struct blasfeo_smat *sD, int di, int dj, struct blasfeo_svec *sx, int xi)
	{
	float *x = sx->pa + xi;
	int ldd = sD->m;
	float *pD = sD->pA + di + dj*ldd;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		x[jj] = alpha * pD[ii*(ldd+1)];
		}
	return;
	}



// add a vector to diagonal of a matrix struct
void blasfeo_sdiaad(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_smat *sA, int ai, int aj)
	{
	// invalidate stored inverse diagonal
	sA->use_dA = 0;

	int lda = sA->m;
	float *pA = sA->pA + ai + aj*lda;
	float *x = sx->pa + xi;
	int ii;
	for(ii=0; ii<kmax; ii++)
		pA[ii*(lda+1)] += alpha*x[ii];
	return;
	}



// add scaled strvec to another strvec and insert to diagonal of strmat, sparse formulation
void blasfeo_sdiaad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	float *x = sx->pa + xi;
	int ldd = sD->m;
	float *pD = sD->pA + di + dj*ldd;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*(ldd+1)] += alpha * x[jj];
		}
	return;
	}



// add scaled strvec to another strvec and insert to diagonal of strmat, sparse formulation
void blasfeo_sdiaadin_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sy, int yi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	float *x = sx->pa + xi;
	float *y = sy->pa + yi;
	int ldd = sD->m;
	float *pD = sD->pA + di + dj*ldd;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*(ldd+1)] = y[jj] + alpha * x[jj];
		}
	return;
	}



// add scaled strvec to row of strmat, sparse formulation
void blasfeo_srowad_sp(int kmax, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_smat *sD, int di, int dj)
	{
	// invalidate stored inverse diagonal
	sD->use_dA = 0;

	float *x = sx->pa + xi;
	int ldd = sD->m;
	float *pD = sD->pA + di + dj*ldd;
	int ii, jj;
	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*ldd] += alpha * x[jj];
		}
	return;
	}



// add scaled strvec to another strvec, sparse formulation
void blasfeo_svecad_sp(int m, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] += alpha * x[ii];
	return;
	}



// insert  scaled strvec to another strvec, sparse formulation
void blasfeo_svecin_sp(int m, float alpha, struct blasfeo_svec *sx, int xi, int *idx, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[idx[ii]] = alpha * x[ii];
	return;
	}


// extract scaled strvec to strvec, sparse formulation
void blasfeo_svecex_sp(int m, float alpha, int *idx, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sz, int zi)
	{
	float *x = sx->pa + xi;
	float *z = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		z[ii] = alpha * x[idx[ii]];
	return;
	}


// clip strvec between two strvec
void blasfeo_sveccl(int m, struct blasfeo_svec *sxm, int xim, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sxp, int xip, struct blasfeo_svec *sz, int zi)
	{
	float *xm = sxm->pa + xim;
	float *x  = sx->pa + xi;
	float *xp = sxp->pa + xip;
	float *z  = sz->pa + zi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		if(x[ii]>=xp[ii])
			{
			z[ii] = xp[ii];
			}
		else if(x[ii]<=xm[ii])
			{
			z[ii] = xm[ii];
			}
		else
			{
			z[ii] = x[ii];
			}
		}
	return;
	}



// clip strvec between two strvec, with mask
void blasfeo_sveccl_mask(int m, struct blasfeo_svec *sxm, int xim, struct blasfeo_svec *sx, int xi, struct blasfeo_svec *sxp, int xip, struct blasfeo_svec *sz, int zi, struct blasfeo_svec *sm, int mi)
	{
	float *xm = sxm->pa + xim;
	float *x  = sx->pa + xi;
	float *xp = sxp->pa + xip;
	float *z  = sz->pa + zi;
	float *mask  = sm->pa + mi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		if(x[ii]>=xp[ii])
			{
			z[ii] = xp[ii];
			mask[ii] = 1.0;
			}
		else if(x[ii]<=xm[ii])
			{
			z[ii] = xm[ii];
			mask[ii] = -1.0;
			}
		else
			{
			z[ii] = x[ii];
			mask[ii] = 0.0;
			}
		}
	return;
	}


// zero out strvec, with mask
void blasfeo_svecze(int m, struct blasfeo_svec *sm, int mi, struct blasfeo_svec *sv, int vi, struct blasfeo_svec *se, int ei)
	{
	float *mask = sm->pa + mi;
	float *v = sv->pa + vi;
	float *e = se->pa + ei;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		if(mask[ii]==0)
			{
			e[ii] = v[ii];
			}
		else
			{
			e[ii] = 0;
			}
		}
	return;
	}



// compute inf norm of strvec
void blasfeo_svecnrm_inf(int m, struct blasfeo_svec *sx, int xi, float *ptr_norm)
	{
	int ii;
	float *x = sx->pa + xi;
	float norm = 0.0;
	float tmp;
	for(ii=0; ii<m; ii++)
		{
#ifdef USE_C99_MATH
		norm = fmaxf(norm, fabsf(x[ii]));
#else
		tmp = fabs(x[ii]);
		norm = tmp>norm ? tmp : norm;
#endif
		}
	*ptr_norm = norm;
	return;
	}



// permute elements of a vector struct
void blasfeo_svecpe(int kmax, int *ipiv, struct blasfeo_svec *sx, int xi)
	{
	int ii;
	float tmp;
	float *x = sx->pa + xi;
	for(ii=0; ii<kmax; ii++)
		{
		if(ipiv[ii]!=ii)
			{
			tmp = x[ipiv[ii]];
			x[ipiv[ii]] = x[ii];
			x[ii] = tmp;
			}
		}
	return;
	}



// inverse permute elements of a vector struct
void blasfeo_svecpei(int kmax, int *ipiv, struct blasfeo_svec *sx, int xi)
	{
	int ii;
	float tmp;
	float *x = sx->pa + xi;
	for(ii=kmax-1; ii>=0; ii--)
		{
		if(ipiv[ii]!=ii)
			{
			tmp = x[ipiv[ii]];
			x[ipiv[ii]] = x[ii];
			x[ii] = tmp;
			}
		}
	return;
	}



#else

#error : wrong LA choice

#endif

// LA_REFERENCE | LA_EXTERNAL_BLAS_WRAPPER
#include "x_aux_lib.c"
