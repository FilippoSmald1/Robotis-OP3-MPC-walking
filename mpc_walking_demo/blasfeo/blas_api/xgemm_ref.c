/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS for embedded optimization.                                                      *
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



void GEMM(char *ta, char *tb, int *pm, int *pn, int *pk, REAL *palpha, REAL *A, int *plda, REAL *B, int *pldb, REAL *pbeta, REAL *C, int *pldc)
	{

	struct MAT sA;
	sA.pA = A;
	sA.m = *plda;

	struct MAT sB;
	sB.pA = B;
	sB.m = *pldb;

	struct MAT sC;
	sC.pA = C;
	sC.m = *pldc;

	if(*ta=='n' | *ta=='N')
		{
		if(*tb=='n' | *tb=='N')
			{
			GEMM_NN(*pm, *pn, *pk, *palpha, &sA, 0, 0, &sB, 0, 0, *pbeta, &sC, 0, 0, &sC, 0, 0);
			}
		else if(*tb=='t' | *tb=='T' | *tb=='c' | *tb=='C')
			{
			GEMM_NT(*pm, *pn, *pk, *palpha, &sA, 0, 0, &sB, 0, 0, *pbeta, &sC, 0, 0, &sC, 0, 0);
			}
		else
			{
			printf("\nBLASFEO: gemm: wrong value for tb\n");
			return;
			}
		}
	else if(*ta=='t' | *ta=='T' | *ta=='c' | *ta=='C')
		{
		if(*tb=='n' | *tb=='N')
			{
			GEMM_TN(*pm, *pn, *pk, *palpha, &sA, 0, 0, &sB, 0, 0, *pbeta, &sC, 0, 0, &sC, 0, 0);
			}
		else if(*tb=='t' | *tb=='T' | *tb=='c'| *tb=='C')
			{
			GEMM_TT(*pm, *pn, *pk, *palpha, &sA, 0, 0, &sB, 0, 0, *pbeta, &sC, 0, 0, &sC, 0, 0);
			}
		else
			{
			printf("\nBLASFEO: gemm: wrong value for tb\n");
			return;
			}
		}
	else
		{
		printf("\nBLASFEO: gemm: wrong value for ta\n");
		return;
		}

	return;

	}
