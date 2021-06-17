/**
 *   @file  MATRIX_multi8x8MatInv.c
 *
 *   @brief
 *      8x8 matrix inversion function for input of multiple matrices
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <modules/postProcessing/matrixFunc/api/MATRIX_8x8MatInv.h>
#ifdef _TMS320C6600

extern void	step1 (  
             IN      cplx32_t * RESTRICT Input,
             IN      int32_t	  N,
             IN      __float2_t   * RESTRICT localScratch,
             OUT     cplxf_t  * RESTRICT invA);

extern void step2(  
             IN      cplxf_t * RESTRICT invA,
             IN      cplxf_t * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT B);

extern void step3(  
             IN      cplx32_t * RESTRICT Input,
             IN      cplxf_t  * RESTRICT B,
             IN      cplxf_t  * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT T);

extern void step4 (  
             IN      cplxf_t * RESTRICT T,
             IN      int32_t	  N,
             IN      __float2_t   * RESTRICT localScratch,
             OUT     cplxf_t  * RESTRICT invT);

extern void step5 (  
             IN      cplxf_t * RESTRICT invT,
             IN      cplxf_t * RESTRICT B,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT C);

extern void step6 (  
             IN      cplxf_t * RESTRICT invA,
             IN      cplxf_t * RESTRICT B,
             IN      cplxf_t  * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT output);


#define DEBUGOUTPUT (0)

/** 
 *  \fn     void MATRIX_multi8x8MatInvFltOut(
 *                            IN      Cplx32  * restrict Input,
 *                            IN      int32_t   N,
 *                            IN      int32_t * RESTRICT scratch,
 *                            OUT     cplxf_t  * output);
 *
 *  \brief   Multiple 8x8 matrices (complex positive semi-definitive Hermitian) inversion using block wise method with floating-point output.
 * 
 *  \param[in]    Input
 *              Input pointer to N of 8x8 matrices that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1).... (0,7), (1,0), (1,1)...(7,7). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of 8x8 matrix to be inverted. .
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 6 * N * 2 * 16 = 192 * N 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[out]   output
 *              Output 8x8 matrices. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */


void MATRIX_multi8x8MatInvFltOut (  
                             IN      cplx32_t * RESTRICT Input,
                             IN      int32_t	  N,
                             IN      int32_t * RESTRICT scratch,
                             OUT     cplxf_t  * output)
{
	cplxf_t		* RESTRICT B;
	cplxf_t		* RESTRICT C;
	cplxf_t		* RESTRICT T;
	cplxf_t		* RESTRICT invT;
	cplxf_t		* RESTRICT invA;
	int32_t		ii, jj;
	int32_t		scratchIndx;
	int64_t		* RESTRICT inPtr;
	__float2_t		* RESTRICT tempOutPtr;
	__float2_t		* RESTRICT tempInPtr;
	__float2_t		* RESTRICT localScratch;


	scratchIndx		=	0;
	B				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	C				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	T				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	invA			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	invT			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	localScratch	=	(__float2_t  *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;



#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* input matrix */ 
		fid			=	fopen("input.dat", "w");
		//for (jj = 8 * 8; jj < 2 * 8 * 8; jj++)
		for (jj = 0; jj < 8 * 8; jj++)
			fprintf(fid, "\t%d\t\t%d\n", Input[jj].real, Input[jj].imag);
		fclose(fid);
	}
#endif



	/****************************************************/
	/****************************************************/
	/********** Step 0: Copy C (SW of 4x4) **************/
	/****************************************************/
	/****************************************************/
	{
		inPtr		=	(int64_t * ) Input;
		tempOutPtr	=	(__float2_t *) C;
		for ( ii = 0; ii < N; ii++ )
		{
			#ifndef _WIN32
			#pragma UNROLL(4);
			#endif
			for ( jj = 0; jj < 4; jj++ )
			{
				_amem8_f2(&tempOutPtr[jj * 4 + 0])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 0]));
				_amem8_f2(&tempOutPtr[jj * 4 + 1])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 1]));
				_amem8_f2(&tempOutPtr[jj * 4 + 2])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 2]));
				_amem8_f2(&tempOutPtr[jj * 4 + 3])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 3]));
			}
			inPtr			+=	64;
			tempOutPtr		+=	16;
		}
	}// end of step 0

	/****************************************************/
	/****************************************************/
	/******* Step 1: inversion of A (NW of 4x4) *********/
	/****************************************************/
	/****************************************************/
	step1 ( Input, N, localScratch, invA );

	/****************************************************/
	/****************************************************/
	/******* Step 2: B=C*invA, C=SW 4x4         *********/
	/****************************************************/
	/****************************************************/
	step2( invA, C, N, B);		
		

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invA.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invA[jj].real, invA[jj].imag);
		fclose(fid);

	}
#endif


#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("CinvA.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", B[jj].real, B[jj].imag);
		fclose(fid);

	}
#endif


	/****************************************************/
	/****************************************************/
	/**** Step 3: T = D - C*inv(A)*conj(C), C=SW 4x4 ****/
	/****************************************************/
	/****************************************************/
	step3( Input, B, C, N, T);


#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("Tmat.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", T[jj].real, T[jj].imag);
		fclose(fid);

	}
#endif

	/****************************************************/
	/****************************************************/
	/***************** Step 4: invT = inv(T) ************/
	/****************************************************/
	/****************************************************/
	step4 ( T, N, localScratch, invT);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invT.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invT[jj].real, invT[jj].imag);
		fclose(fid);

	}
#endif


	/****************************************************/
	/****************************************************/
	/********** Step 4.5: output SE 4x4 matrix  *********/
	/****************************************************/
	/****************************************************/
	{
		tempInPtr	=	(__float2_t * ) invT;
		tempOutPtr	=	(__float2_t *) output;
		for ( ii = 0; ii < N; ii++ )
		{
			#ifndef _WIN32
			#pragma UNROLL(4);
			#endif
			for ( jj = 0; jj < 4; jj++ )
			{
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 4])	=	_amem8_f2(&tempInPtr[jj * 4 + 0]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 5])	=	_amem8_f2(&tempInPtr[jj * 4 + 1]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 6])	=	_amem8_f2(&tempInPtr[jj * 4 + 2]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 7])	=	_amem8_f2(&tempInPtr[jj * 4 + 3]);
			}
			tempInPtr		+=	16;
			tempOutPtr		+=	64;
		}
	} // end of step 4.5


	/************************************************************/
	/************************************************************/
	/*** Step 5:output SW (- invT*C*invA) and NE 4x4 matrix  ****/
	/************************************************************/
	/************************************************************/
	step5 ( invT, B, N, C);


	/**************************************************************/
	/**************************************************************/
	/*** Step 5.5:output SW (- invT*C*invA) and NE 4x4 matrix  ****/
	/**************************************************************/
	/**************************************************************/
	{
		__float2_t  dtemp, dtemp1, dtemp2, dtemp3, zeros;

		zeros		=	_ftof2(0.f, 0.f);
		tempOutPtr	=	(__float2_t *) output;
		tempInPtr	=	(__float2_t *) C;
		for ( ii = 0; ii < N; ii++ )
		{
			dtemp								=	_amem8_f2(&tempInPtr[0]);
			_amem8_f2(&tempOutPtr[32 + 0])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 0 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[4]);
			_amem8_f2(&tempOutPtr[40 + 0])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 0 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[8]);
			_amem8_f2(&tempOutPtr[48 + 0])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 0 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[12]);
			_amem8_f2(&tempOutPtr[56 + 0])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 0 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[1]);
			_amem8_f2(&tempOutPtr[32 + 1])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 1 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[5]);
			_amem8_f2(&tempOutPtr[40 + 1])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 1 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[9]);
			_amem8_f2(&tempOutPtr[48 + 1])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 1 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[13]);
			_amem8_f2(&tempOutPtr[56 + 1])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 1 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[2]);
			_amem8_f2(&tempOutPtr[32 + 2])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 2 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[6]);
			_amem8_f2(&tempOutPtr[40 + 2])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 2 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[10]);
			_amem8_f2(&tempOutPtr[48 + 2])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 2 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[14]);
			_amem8_f2(&tempOutPtr[56 + 2])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 2 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[3]);
			_amem8_f2(&tempOutPtr[32 + 3])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 3 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[7]);
			_amem8_f2(&tempOutPtr[40 + 3])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 3 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[11]);
			_amem8_f2(&tempOutPtr[48 + 3])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 3 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[15]);
			_amem8_f2(&tempOutPtr[56 + 3])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 3 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			tempInPtr		+=	16;
			tempOutPtr		+=	64;
		}
	}

	/************************************************************/
	/************************************************************/
	/****** Step 6:output NW (invA + invA*B*invT*C*invA)  *******/
	/************************************************************/
	/************************************************************/
	step6 ( invA, B, C, N, output);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invRn.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 64; jj++)
			fprintf(fid, "\t%e\t\t%e\n", output[jj].real, output[jj].imag);
		fclose(fid);

	}
#endif

}



/** 
 *  \fn     void MATRIX_multi8x8MatInvFxdOut(
 *                            IN      Cplx32  * restrict Input,
 *                            IN      int32_t   N,
 *                            IN      int32_t * RESTRICT scratch,
 *							  IN      int32_t         range,
 *							  OUT     int32_t	 * RESTRICT Qinv,
 *                            OUT     cplx16_t  * output);
 *
 *  \brief   Multiple 8x8 matrices (complex positive semi-definitive Hermitian) inversion using block wise method with fixed-point output.
 * 
 *  \param[in]    Input
 *              Input pointer to N of 8x8 matrices that needs to be inversed, stored sequentially 
 *              in format (0,0), (0,1).... (0,7), (1,0), (1,1)...(7,7). 
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]    N
 *              Number of 8x8 matrix to be inverted. .
 *
 *  \param[out]    scratch
 *              Input pointer to the scratch memory. Must be of size 6 * N * 2 * 16 + 64 * 2 * N = 320 * N 32-bit words.
 *              Must be aligned to 8-byte boundary.
 *
 *  \param[in]   range
 *              The full range of the output. For example, if we set it to 14, that means the real and imaginary
 *              part of the output elements will not exceed 14-bit with sign.
 *
 *  \param[out]   Qinv
 *              Output block Q value for the inverse matrices.
 *
 *  \param[out]   output
 *              Output 8x8 matrices. Stored sequentially as the input.
 *              Must be aligned to 8-byte boundary.
 *
 *  \pre     Input matrix must be complex positive semi-definitive Hermitian matrix
 *
 *  \post  
 * 
 *  \sa   
 * 
 */


void MATRIX_multi8x8MatInvFxdOut (  
                             IN      cplx32_t * RESTRICT Input,
                             IN      int32_t	  N,
                             IN      int32_t * RESTRICT scratch,
							 IN      int32_t         range,
							 OUT     int32_t	 * RESTRICT Qinv,
                             OUT     cplx16_t  * output)
{
	cplxf_t		* RESTRICT B;
	cplxf_t		* RESTRICT C;
	cplxf_t		* RESTRICT T;
	cplxf_t		* RESTRICT invT;
	cplxf_t		* RESTRICT invA;
	cplxf_t		* RESTRICT foutput;
	int32_t		ii, jj;
	int32_t		scratchIndx;
	int64_t		* RESTRICT inPtr;
	__float2_t		* RESTRICT tempOutPtr;
	__float2_t		* RESTRICT tempInPtr;
	__float2_t		* RESTRICT localScratch;


	scratchIndx		=	0;
	B				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	C				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	T				=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	invA			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	invT			=	(cplxf_t *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	localScratch	=	(__float2_t  *) &scratch[scratchIndx];
	scratchIndx		+=	N * 16 * 2;
	foutput			=	(cplxf_t  *) &scratch[scratchIndx];
	scratchIndx		+=	N * 64 * 2;



#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* input matrix */ 
		fid			=	fopen("input.dat", "w");
		//for (jj = 8 * 8; jj < 2 * 8 * 8; jj++)
		for (jj = 0; jj < 8 * 8; jj++)
			fprintf(fid, "\t%d\t\t%d\n", Input[jj].real, Input[jj].imag);
		fclose(fid);
	}
#endif



	/****************************************************/
	/****************************************************/
	/********** Step 0: Copy C (SW of 4x4) **************/
	/****************************************************/
	/****************************************************/
	{
		inPtr		=	(int64_t * ) Input;
		tempOutPtr	=	(__float2_t *) C;
		for ( ii = 0; ii < N; ii++ )
		{
			#ifndef _WIN32
			#pragma UNROLL(4);
			#endif
			for ( jj = 0; jj < 4; jj++ )
			{
				_amem8_f2(&tempOutPtr[jj * 4 + 0])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 0]));
				_amem8_f2(&tempOutPtr[jj * 4 + 1])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 1]));
				_amem8_f2(&tempOutPtr[jj * 4 + 2])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 2]));
				_amem8_f2(&tempOutPtr[jj * 4 + 3])	=	_dintsp((int64_t)_amem8(&inPtr[(jj + 4) * 8 + 3]));
			}
			inPtr			+=	64;
			tempOutPtr		+=	16;
		}
	}// end of step 0

	/****************************************************/
	/****************************************************/
	/******* Step 1: inversion of A (NW of 4x4) *********/
	/****************************************************/
	/****************************************************/
	step1 ( Input, N, localScratch, invA );

	/****************************************************/
	/****************************************************/
	/******* Step 2: B=C*invA, C=SW 4x4         *********/
	/****************************************************/
	/****************************************************/
	step2( invA, C, N, B);		
		

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invA.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invA[jj].real, invA[jj].imag);
		fclose(fid);

	}
#endif


#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("CinvA.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", B[jj].real, B[jj].imag);
		fclose(fid);

	}
#endif


	/****************************************************/
	/****************************************************/
	/**** Step 3: T = D - C*inv(A)*conj(C), C=SW 4x4 ****/
	/****************************************************/
	/****************************************************/
	step3( Input, B, C, N, T);


#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("Tmat.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", T[jj].real, T[jj].imag);
		fclose(fid);

	}
#endif

	/****************************************************/
	/****************************************************/
	/***************** Step 4: invT = inv(T) ************/
	/****************************************************/
	/****************************************************/
	step4 ( T, N, localScratch, invT);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invT.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 16; jj++)
			fprintf(fid, "\t%e\t\t%e\n", invT[jj].real, invT[jj].imag);
		fclose(fid);

	}
#endif


	/****************************************************/
	/****************************************************/
	/********** Step 4.5: output SE 4x4 matrix  *********/
	/****************************************************/
	/****************************************************/
	{
		tempInPtr	=	(__float2_t * ) invT;
		tempOutPtr	=	(__float2_t *) foutput;
		for ( ii = 0; ii < N; ii++ )
		{
			#ifndef _WIN32
			#pragma UNROLL(4);
			#endif
			for ( jj = 0; jj < 4; jj++ )
			{
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 4])	=	_amem8_f2(&tempInPtr[jj * 4 + 0]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 5])	=	_amem8_f2(&tempInPtr[jj * 4 + 1]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 6])	=	_amem8_f2(&tempInPtr[jj * 4 + 2]);
				_amem8_f2(&tempOutPtr[(jj + 4) * 8 + 7])	=	_amem8_f2(&tempInPtr[jj * 4 + 3]);
			}
			tempInPtr		+=	16;
			tempOutPtr		+=	64;
		}
	} // end of step 4.5


	/************************************************************/
	/************************************************************/
	/*** Step 5:output SW (- invT*C*invA) and NE 4x4 matrix  ****/
	/************************************************************/
	/************************************************************/
	step5 ( invT, B, N, C);


	/**************************************************************/
	/**************************************************************/
	/*** Step 5.5:output SW (- invT*C*invA) and NE 4x4 matrix  ****/
	/**************************************************************/
	/**************************************************************/
	{
		__float2_t  dtemp, dtemp1, dtemp2, dtemp3, zeros;

		zeros		=	_ftof2(0.f, 0.f);
		tempOutPtr	=	(__float2_t *) foutput;
		tempInPtr	=	(__float2_t *) C;
		for ( ii = 0; ii < N; ii++ )
		{
			dtemp								=	_amem8_f2(&tempInPtr[0]);
			_amem8_f2(&tempOutPtr[32 + 0])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 0 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[4]);
			_amem8_f2(&tempOutPtr[40 + 0])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 0 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[8]);
			_amem8_f2(&tempOutPtr[48 + 0])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 0 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[12]);
			_amem8_f2(&tempOutPtr[56 + 0])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 0 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[1]);
			_amem8_f2(&tempOutPtr[32 + 1])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 1 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[5]);
			_amem8_f2(&tempOutPtr[40 + 1])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 1 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[9]);
			_amem8_f2(&tempOutPtr[48 + 1])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 1 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[13]);
			_amem8_f2(&tempOutPtr[56 + 1])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 1 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[2]);
			_amem8_f2(&tempOutPtr[32 + 2])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 2 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[6]);
			_amem8_f2(&tempOutPtr[40 + 2])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 2 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[10]);
			_amem8_f2(&tempOutPtr[48 + 2])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 2 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[14]);
			_amem8_f2(&tempOutPtr[56 + 2])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 2 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			dtemp								=	_amem8_f2(&tempInPtr[3]);
			_amem8_f2(&tempOutPtr[32 + 3])		=	_dsubsp(zeros, dtemp);
			_amem8_f2(&tempOutPtr[8 * 3 + 4])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
			dtemp1								=	_amem8_f2(&tempInPtr[7]);
			_amem8_f2(&tempOutPtr[40 + 3])		=	_dsubsp(zeros, dtemp1);
			_amem8_f2(&tempOutPtr[8 * 3 + 5])	=	_lltof2(_f2toll(dtemp1) ^ 0x8000000000000000);
			dtemp2								=	_amem8_f2(&tempInPtr[11]);
			_amem8_f2(&tempOutPtr[48 + 3])		=	_dsubsp(zeros, dtemp2);
			_amem8_f2(&tempOutPtr[8 * 3 + 6])	=	_lltof2(_f2toll(dtemp2) ^ 0x8000000000000000);
			dtemp3								=	_amem8_f2(&tempInPtr[15]);
			_amem8_f2(&tempOutPtr[56 + 3])		=	_dsubsp(zeros, dtemp3);
			_amem8_f2(&tempOutPtr[8 * 3 + 7])	=	_lltof2(_f2toll(dtemp3) ^ 0x8000000000000000);

			tempInPtr		+=	16;
			tempOutPtr		+=	64;
		}
	}

	/************************************************************/
	/************************************************************/
	/****** Step 6:output NW (invA + invA*B*invT*C*invA)  *******/
	/************************************************************/
	/************************************************************/
	step6 ( invA, B, C, N, foutput);

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;

		/* A matrix */ 
		fid			=	fopen("invRn.dat", "w");
		//for (jj = 16; jj < 2 * 16; jj++)
		for (jj = 0; jj < 64; jj++)
			fprintf(fid, "\t%e\t\t%e\n", foutput[jj].real, foutput[jj].imag);
		fclose(fid);

	}
#endif

	/************************************************************/
	/************************************************************/
	/****** Step 7:Convert flt output to block fixed output  ****/
	/************************************************************/
	/************************************************************/
	{
		int64_t		max, lltemp1, lltemp2;
		float		magic;
		__float2_t  magicPair;
		float		* RESTRICT magicPtr;
		int32_t     head = 16 - range;
		int32_t		* RESTRICT OutPtr;
		int32_t		jj;

		OutPtr	=	(int32_t *) output;
		tempInPtr	=	(__float2_t *) foutput;
		magicPtr	=	(float *) B;
		for ( ii = 0; ii < N; ii++ )
		{
			max			=	_dmax2(_amem8_const(&tempInPtr[0]), _amem8_const(&tempInPtr[9]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[18]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[27]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[36]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[45]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[54]));
			max			=	_dmax2(max, _amem8_const(&tempInPtr[63]));
			magic		=	_itof(0x04C00000 + (head<<23) + (_hill(max) & 0xFF800000));
			magicPair	=	_fdmv_f2(magic, magic);
			magicPtr[ii] =	magic;
			Qinv[ii]	=	(127 + 23) - (_ftoi(magic) >> 23);

			#ifdef _BIG_ENDIAN
			#ifdef _TMS320C6X
			#pragma UNROLL (16);
			#endif
			for ( jj = 0; jj < 32; jj += 2 )
			{
				lltemp1				=	_f2toll(_daddsp(tempInPtr[jj], magicPair));
				lltemp2				=	_f2toll(_daddsp(tempInPtr[jj + 1], magicPair));
				_amem8(&OutPtr[jj])	=	_itoll(_pack2(_hill(lltemp1), _loll(lltemp1)), _pack2(_hill(lltemp2), _loll(lltemp2)));
			}
			#else
			#ifdef _TMS320C6X
			#pragma UNROLL (16);
			#endif
			for ( jj = 0; jj < 32; jj += 2 )
			{
				lltemp1				=	_f2toll(_daddsp(tempInPtr[jj], magicPair));
				lltemp2				=	_f2toll(_daddsp(tempInPtr[jj + 1], magicPair));
				_amem8(&OutPtr[jj])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), _loll(lltemp1)));
			}
			#endif

			OutPtr		+=	64;
			tempInPtr	+=	64;
		}
		
		OutPtr	=	((int32_t *) output) + 32;
		tempInPtr	=	((__float2_t *) foutput) + 32;
		magicPtr	=	(float *) B;
		for ( ii = 0; ii < N; ii++ )
		{
			magic		=	magicPtr[ii];
			magicPair	=	_fdmv_f2(magic, magic);

			#ifdef _BIG_ENDIAN
			#ifdef _TMS320C6X
			#pragma UNROLL (16);
			#endif
			for ( jj = 0; jj < 32; jj += 2 )
			{
				lltemp1				=	_f2toll(_daddsp(tempInPtr[jj], magicPair));
				lltemp2				=	_f2toll(_daddsp(tempInPtr[jj + 1], magicPair));
				_amem8(&OutPtr[jj])	=	_itoll(_pack2(_hill(lltemp1), _loll(lltemp1)), _pack2(_hill(lltemp2), _loll(lltemp2)));
			}
			#else
			#ifdef _TMS320C6X
			#pragma UNROLL (16);
			#endif
			for ( jj = 0; jj < 32; jj += 2 )
			{
				lltemp1				=	_f2toll(_daddsp(tempInPtr[jj], magicPair));
				lltemp2				=	_f2toll(_daddsp(tempInPtr[jj + 1], magicPair));
				_amem8(&OutPtr[jj])	=	_itoll(_pack2(_hill(lltemp2), _loll(lltemp2)), _pack2(_hill(lltemp1), _loll(lltemp1)));
			}
			#endif

			OutPtr		+=	64;
			tempInPtr	+=	64;
		}	

	} // end of Step 7



}







/****************************************************/
/****************************************************/
/******* Step 1: inversion of A (NW of 4x4) *********/
/****************************************************/
/****************************************************/

void step1 (  
             IN      cplx32_t * RESTRICT Input,
             IN      int32_t	  N,
             IN      __float2_t   * RESTRICT localScratch,
             OUT     cplxf_t  * RESTRICT invA)
{
	int32_t		ii;
	int64_t		* RESTRICT inPtr;
	__float2_t		* RESTRICT tempOutPtr;
	float  A0, A3, D0, D3, F0, F3;
	float  detA, oneOverdetA;
	__float2_t A1, D1, F1, C0, C1, C2, C3;
	__float2_t B0, B1, B2, B3;
	__float2_t dtemp, dtemp1, dtemp2, doneOverdetA;
	__x128_t results;
	__float2_t		* RESTRICT CC;
	__float2_t		* RESTRICT FF;
	__float2_t		* RESTRICT DD;
	__float2_t		* RESTRICT CPtr;
	__float2_t		* RESTRICT DPtr;
	__float2_t		* RESTRICT FPtr;


	CC		=	&localScratch[0];
	DD		=	&localScratch[4 * N];
	FF		=	&localScratch[6 * N];

#if DEBUGOUTPUT
	if (1)
	{
		FILE * fid;
		int32_t  jj;

		/* input matrix */ 
		fid			=	fopen("input.dat", "w");
		//for (ii = 8 * 8; ii < 2 * 8 * 8; ii++)
		for (ii = 0; ii < 8 * 8; ii++)
			fprintf(fid, "\t%d\t\t%d\n", Input[ii].real, Input[ii].imag);
		fclose(fid);
	}
#endif

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;
	for ( ii = 0; ii < N; ii++ )
	{
		/* Calculate d = inv(d) */
		dtemp			=   _dintsp(_amem8(&inPtr[18]));
		D0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[27]));
		D3				=	_hif2(dtemp);
		D1				=	_dintsp(_amem8(&inPtr[19]));
		inPtr			+=	64;

		dtemp			=	_dmpysp(D1, D1);
		detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

		dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
		D0				=	_hif2(dtemp1);
		D3				=	_lof2(dtemp1);
		D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&DPtr[0])	=	dtemp1;
		_amem8_f2(&DPtr[1])	=	D1;
		DPtr			+=	2;
	}

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(int64_t * ) Input;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load b */
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[10]));
		B3				=	_dintsp(_amem8(&inPtr[11]));
		inPtr			+=	64;

		dtemp 			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);
		DPtr			+=	2;

		/* calculate c = b*inv(d) */
		results			=	_cmpysp(D1, B1);
		C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B0);
		C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
		results			=	_cmpysp(D1, B3);
		C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B2);
		C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));
		_amem8_f2(&CPtr[0])	=	C0;
		_amem8_f2(&CPtr[1])	=	C1;
		_amem8_f2(&CPtr[2])	=	C2;
		_amem8_f2(&CPtr[3])	=	C3;
		CPtr			+=	4;
	}
	
	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	inPtr		=	(int64_t * ) Input;
	tempOutPtr	=	(__float2_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load a */
		dtemp			=	_dintsp(_amem8(&inPtr[0]));
		A0				=	_hif2(dtemp);
		dtemp			=	_dintsp(_amem8(&inPtr[9]));
		A3				=	_hif2(dtemp);
		A1				=	_dintsp(_amem8(&inPtr[1]));

		/* load b */
		B0				=	_dintsp(_amem8(&inPtr[2]));
		B1				=	_dintsp(_amem8(&inPtr[3]));
		B2				=	_dintsp(_amem8(&inPtr[10]));
		B3				=	_dintsp(_amem8(&inPtr[11]));
		inPtr			+=	64;
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);
		CPtr			+=	4;

		/* calculate f = a - b *inv(d) * conj(b) -- Hermitian */
		results			=	_cmpysp(B0, C0);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B1, C1);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F0				=	A0 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C2);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B3, C3);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F3				=	A3 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C0);
		F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(B3, C1);
		F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		/* Calculate f = inv(f) */
		dtemp			=	_dmpysp(F1, F1);
		detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);
		
		dtemp1			=	_dmpysp(_ftof2(F3, F0), doneOverdetA);
		F1				=	_dmpysp(F1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&FPtr[0])	=	dtemp1;
		_amem8_f2(&FPtr[1])	=	F1;
		FPtr			+=	2;

		/* NW output */
		_amem8_f2(&tempOutPtr[0])	=	_lltof2(_f2toll(dtemp1) & 0xFFFFFFFF00000000); //_ftof2(_hif2(dtemp1), 0.f); //_lltof2(_f2toll(dtemp1) & 0xFFFFFFFF00000000);
		_amem8_f2(&tempOutPtr[1])	=	F1;
		_amem8_f2(&tempOutPtr[5])	=	_ftof2(_lof2(dtemp1), 0.f);
		_amem8_f2(&tempOutPtr[4])	=	_lltof2(_f2toll(F1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	(__float2_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);

		dtemp			=	_amem8_f2(&FPtr[0]);
		F0				=	_hif2(dtemp);
		F3				=	_lof2(dtemp);
		F1				=	_amem8_f2(&FPtr[1]);
		CPtr			+=	4;
		DPtr			+=	2;
		FPtr			+=	2;

		/* NE output = - f * c, SW = conj(NW)*/
		results			=	_cmpysp(F1, C2);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
		_amem8_f2(&tempOutPtr[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[8])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C3);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
		B1				=	dtemp;
		_amem8_f2(&tempOutPtr[3])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[12])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C0);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
		_amem8_f2(&tempOutPtr[6])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[9])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
		B3				=	dtemp;
		_amem8_f2(&tempOutPtr[7])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[13])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
		

		
		/* SE output */
		/* inv(d) - conj(c) * inv(f) * c, whrer c = b * inv(d) */
		dtemp			=	_dmpysp(C0, C0);
		A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C2, C2);
		A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C0, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A0				+=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(C1, C1);
		A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C3, C3);
		A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C1, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A3				+=	2.f * _hif2(dtemp);
		_amem8_f2(&tempOutPtr[10])=	_ftof2(A0, 0.f);
		_amem8_f2(&tempOutPtr[15])=	_ftof2(A3, 0.f);

		results			=	_cmpysp(C0, B1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A1				=	_daddsp(D1, dtemp);
		results			=	_cmpysp(C2, B3);
		A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		_amem8_f2(&tempOutPtr[11])=	A1;
		_amem8_f2(&tempOutPtr[14])=	_lltof2(_f2toll(A1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 
} // end of step 1


/****************************************************/
/****************************************************/
/******* Step 2: B=C*invA, C=SW 4x4         *********/
/****************************************************/
/****************************************************/
void step2(  
             IN      cplxf_t * RESTRICT invA,
             IN      cplxf_t * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT B)
{
	__float2_t		* RESTRICT tempOutPtr;
	__float2_t		* RESTRICT tempInPtr;
	__float2_t		* RESTRICT tempInPtr1;
	int32_t	ii, kk;
	__float2_t   * RESTRICT A1, * RESTRICT A2, * RESTRICT A3, * RESTRICT A4;
	__float2_t  * RESTRICT C1, * RESTRICT C2, * RESTRICT C3, * RESTRICT C4;
	__float2_t  dtemp, dtemp1, dtemp2, dtemp3;
	__x128_t result;


	tempInPtr1	=	(__float2_t *) C;
	tempOutPtr	=	(__float2_t *) B;
	tempInPtr	=	(__float2_t *) invA;
	for ( ii = 0; ii < N; ii++ )
	{


		A1 				=	(__float2_t *) &tempInPtr1[0];
		A2 				=	(__float2_t *) &tempInPtr1[4];
		A3 				=	(__float2_t *) &tempInPtr1[8];
		A4 				=	(__float2_t *) &tempInPtr1[12];
		C1 				=	(__float2_t *) &tempOutPtr[0];
		C2 				=	(__float2_t *) &tempOutPtr[4];
		C3 				=	(__float2_t *) &tempOutPtr[8];
		C4 				=	(__float2_t *) &tempOutPtr[12];
		for ( kk = 0; kk < 4; kk++ )
		{
			result		=	_cmpysp(_amem8_f2(&A1[0]), _amem8_f2(&tempInPtr[4 * 0 + kk]));
			dtemp		=	_daddsp(_hif2_128(result), _lof2_128(result));
			result		=	_cmpysp(_amem8_f2(&A2[0]), _amem8_f2(&tempInPtr[4 * 0 + kk]));
			dtemp1		=	_daddsp(_hif2_128(result), _lof2_128(result));
			result		=	_cmpysp(_amem8_f2(&A3[0]), _amem8_f2(&tempInPtr[4 * 0 + kk]));
			dtemp2		=	_daddsp(_hif2_128(result), _lof2_128(result));
			result		=	_cmpysp(_amem8_f2(&A4[0]), _amem8_f2(&tempInPtr[4 * 0 + kk]));
			dtemp3		=	_daddsp(_hif2_128(result), _lof2_128(result));

			result		=	_cmpysp(_amem8_f2(&A1[1]), _amem8_f2(&tempInPtr[4 * 1 + kk]));
			dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A2[1]), _amem8_f2(&tempInPtr[4 * 1 + kk]));
			dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A3[1]), _amem8_f2(&tempInPtr[4 * 1 + kk]));
			dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A4[1]), _amem8_f2(&tempInPtr[4 * 1 + kk]));
			dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

			result		=	_cmpysp(_amem8_f2(&A1[2]), _amem8_f2(&tempInPtr[4 * 2 + kk]));
			dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A2[2]), _amem8_f2(&tempInPtr[4 * 2 + kk]));
			dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A3[2]), _amem8_f2(&tempInPtr[4 * 2 + kk]));
			dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A4[2]), _amem8_f2(&tempInPtr[4 * 2 + kk]));
			dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

			result		=	_cmpysp(_amem8_f2(&A1[3]), _amem8_f2(&tempInPtr[4 * 3 + kk]));
			dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A2[3]), _amem8_f2(&tempInPtr[4 * 3 + kk]));
			dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A3[3]), _amem8_f2(&tempInPtr[4 * 3 + kk]));
			dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
			result		=	_cmpysp(_amem8_f2(&A4[3]), _amem8_f2(&tempInPtr[4 * 3 + kk]));
			dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

			_amem8_f2(&C1[kk])	=	dtemp;
			_amem8_f2(&C2[kk])	=	dtemp1;
			_amem8_f2(&C3[kk])	=	dtemp2;
			_amem8_f2(&C4[kk])	=	dtemp3;
		}
		tempInPtr1		+=	16;
		tempOutPtr		+=	16;
		tempInPtr		+=	16;
	}
} // end of step 2



/****************************************************/
/****************************************************/
/**** Step 3: T = D - C*inv(A)*conj(C), C=SW 4x4 ****/
/****************************************************/
/****************************************************/
void step3(  
             IN      cplx32_t * RESTRICT Input,
             IN      cplxf_t  * RESTRICT B,
             IN      cplxf_t  * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT T)
{
	int32_t		ii;
	__x128_t	results;
	__float2_t		dtemp1;
	int64_t		* RESTRICT DPtr;
	__float2_t		* RESTRICT TPtr;
	__float2_t		* RESTRICT BPtr;
	__float2_t		* RESTRICT CPtr;


	DPtr	=	(int64_t * ) Input;
	CPtr	=	(__float2_t *) C;
	TPtr	=	(__float2_t *) T;
	BPtr	=	(__float2_t *) B;
	for ( ii = 0; ii < N; ii++ )
	{

		results			=	_cmpysp(_amem8_f2(&CPtr[0]), _amem8_f2(&BPtr[0]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[1]), _amem8_f2(&BPtr[1]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[2]), _amem8_f2(&BPtr[2]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[3]), _amem8_f2(&BPtr[3]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[36])), dtemp1);
		_amem8_f2(&TPtr[0])	=	dtemp1;

		results			=	_cmpysp(_amem8_f2(&CPtr[4]), _amem8_f2(&BPtr[0]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[5]), _amem8_f2(&BPtr[1]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[6]), _amem8_f2(&BPtr[2]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[7]), _amem8_f2(&BPtr[3]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[37])), dtemp1);
		_amem8_f2(&TPtr[1])	=	dtemp1;
		_amem8_f2(&TPtr[4])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&CPtr[4]), _amem8_f2(&BPtr[4]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[5]), _amem8_f2(&BPtr[5]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[6]), _amem8_f2(&BPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[7]), _amem8_f2(&BPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[45])), dtemp1);
		_amem8_f2(&TPtr[5])	=	dtemp1;
		
		results			=	_cmpysp(_amem8_f2(&CPtr[8]), _amem8_f2(&BPtr[0]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[9]), _amem8_f2(&BPtr[1]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[10]), _amem8_f2(&BPtr[2]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[11]), _amem8_f2(&BPtr[3]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[38])), dtemp1);
		_amem8_f2(&TPtr[2])	=	dtemp1;
		_amem8_f2(&TPtr[8])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&CPtr[8]), _amem8_f2(&BPtr[4]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[9]), _amem8_f2(&BPtr[5]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[10]), _amem8_f2(&BPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[11]), _amem8_f2(&BPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[46])), dtemp1);
		_amem8_f2(&TPtr[6])	=	dtemp1;
		_amem8_f2(&TPtr[9])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		

		DPtr		+=	64;
		TPtr		+=	16;
		BPtr		+=	16;
		CPtr		+=	16;
	}




	DPtr	=	(int64_t * ) Input;
	CPtr	=	(__float2_t *) C;
	TPtr	=	(__float2_t *) T;
	BPtr	=	(__float2_t *) B;
	for ( ii = 0; ii < N; ii++ )
	{

		results			=	_cmpysp(_amem8_f2(&CPtr[8]), _amem8_f2(&BPtr[8]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[9]), _amem8_f2(&BPtr[9]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[10]), _amem8_f2(&BPtr[10]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[11]), _amem8_f2(&BPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[54])), dtemp1);
		_amem8_f2(&TPtr[10])	=	dtemp1;

		results			=	_cmpysp(_amem8_f2(&CPtr[12]), _amem8_f2(&BPtr[0]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[13]), _amem8_f2(&BPtr[1]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[14]), _amem8_f2(&BPtr[2]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[15]), _amem8_f2(&BPtr[3]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[39])), dtemp1);
		_amem8_f2(&TPtr[3])	=	dtemp1;
		_amem8_f2(&TPtr[12])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&CPtr[12]), _amem8_f2(&BPtr[4]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[13]), _amem8_f2(&BPtr[5]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[14]), _amem8_f2(&BPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[15]), _amem8_f2(&BPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[47])), dtemp1);
		_amem8_f2(&TPtr[7])	=	dtemp1;
		_amem8_f2(&TPtr[13])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&CPtr[12]), _amem8_f2(&BPtr[8]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[13]), _amem8_f2(&BPtr[9]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[14]), _amem8_f2(&BPtr[10]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[15]), _amem8_f2(&BPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[55])), dtemp1);
		_amem8_f2(&TPtr[11])	=	dtemp1;
		_amem8_f2(&TPtr[14])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&CPtr[12]), _amem8_f2(&BPtr[12]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&CPtr[13]), _amem8_f2(&BPtr[13]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[14]), _amem8_f2(&BPtr[14]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&CPtr[15]), _amem8_f2(&BPtr[15]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_dsubsp(_dintsp(_amem8(&DPtr[63])), dtemp1);
		_amem8_f2(&TPtr[15])	=	dtemp1;

		DPtr		+=	64;
		TPtr		+=	16;
		BPtr		+=	16;
		CPtr		+=	16;
	}
} // end of step 3



/****************************************************/
/****************************************************/
/***************** Step 4: invT = inv(T) ************/
/****************************************************/
/****************************************************/
void step4 (  
             IN      cplxf_t * RESTRICT T,
             IN      int32_t	  N,
             IN      __float2_t  * RESTRICT localScratch,
             OUT     cplxf_t  * RESTRICT invT)
{
	int32_t	ii;
	float  A0, A3, D0, D3, F0, F3;
	float  detA, oneOverdetA;
	__float2_t A1, D1, F1, C0, C1, C2, C3;
	__float2_t B0, B1, B2, B3;
	__float2_t dtemp, dtemp1, dtemp2, doneOverdetA;
	__x128_t results;
	__float2_t		* RESTRICT tempOutPtr;
	__float2_t		* RESTRICT inPtr;
	__float2_t		* RESTRICT CC;
	__float2_t		* RESTRICT DD;
	__float2_t		* RESTRICT FF;
	__float2_t		* RESTRICT CPtr;
	__float2_t		* RESTRICT DPtr;
	__float2_t		* RESTRICT FPtr;


	CC		=	&localScratch[0];
	DD		=	&localScratch[4 * N];
	FF		=	&localScratch[6 * N];

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(__float2_t * ) T;
	for ( ii = 0; ii < N; ii++ )
	{
		/* Calculate d = inv(d) */
		dtemp			=   _amem8_f2(&inPtr[10]);
		D0				=	_hif2(dtemp);
		dtemp			=	_amem8_f2(&inPtr[15]);
		D3				=	_hif2(dtemp);
		D1				=	_amem8_f2(&inPtr[11]);
		inPtr			+=	16;

		dtemp			=	_dmpysp(D1, D1);
		detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		//oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);

		dtemp1			=	_dmpysp(_ftof2(D3, D0),  doneOverdetA);
		D0				=	_hif2(dtemp1);
		D3				=	_lof2(dtemp1);
		D1				=	_dmpysp(D1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&DPtr[0])	=	dtemp1;
		_amem8_f2(&DPtr[1])	=	D1;
		DPtr			+=	2;
	}

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	inPtr		=	(__float2_t * ) T;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load b */
		B0				=	_amem8_f2(&inPtr[2]);
		B1				=	_amem8_f2(&inPtr[3]);
		B2				=	_amem8_f2(&inPtr[6]);
		B3				=	_amem8_f2(&inPtr[7]);
		inPtr			+=	16;

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);
		DPtr			+=	2;

		/* calculate c = b*inv(d) */
		results			=	_cmpysp(D1, B1);
		C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B0);
		C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
		results			=	_cmpysp(D1, B3);
		C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B2);
		C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
		C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));
		_amem8_f2(&CPtr[0])	=	C0;
		_amem8_f2(&CPtr[1])	=	C1;
		_amem8_f2(&CPtr[2])	=	C2;
		_amem8_f2(&CPtr[3])	=	C3;
		CPtr			+=	4;
	}
	
	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	inPtr		=	(__float2_t * ) T;
	tempOutPtr	=	(__float2_t *) invT;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load a */
		dtemp			=	_amem8_f2(&inPtr[0]);
		A0				=	_hif2(dtemp);
		dtemp			=	_amem8_f2(&inPtr[5]);
		A3				=	_hif2(dtemp);
		A1				=	_amem8_f2(&inPtr[1]);
		B0				=	_amem8_f2(&inPtr[2]);
		B1				=	_amem8_f2(&inPtr[3]);
		B2				=	_amem8_f2(&inPtr[6]);
		B3				=	_amem8_f2(&inPtr[7]);
		inPtr			+=	16;
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);
		CPtr			+=	4;

		/* calculate f = a - b *inv(d) * conj(b) -- Hermitian */
		results			=	_cmpysp(B0, C0);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B1, C1);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F0				=	A0 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C2);
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(B3, C3);
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		F3				=	A3 - _hif2(dtemp1);
		
		results			=	_cmpysp(B2, C0);
		F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(B3, C1);
		F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		/* Calculate f = inv(f) */
		dtemp			=	_dmpysp(F1, F1);
		detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		//oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		doneOverdetA	=	_fdmv_f2(oneOverdetA, oneOverdetA);
		
		dtemp1			=	_dmpysp(_ftof2(F3, F0), doneOverdetA);
		F1				=	_dmpysp(F1, _lltof2(_f2toll(doneOverdetA) ^ 0x8000000080000000));
		_amem8_f2(&FPtr[0])	=	dtemp1;
		_amem8_f2(&FPtr[1])	=	F1;
		FPtr			+=	2;

		/* NW output */
		_amem8_f2(&tempOutPtr[0])	=	_lltof2(_f2toll(dtemp1) & 0xFFFFFFFF00000000); //_ftof2(_hif2(dtemp1), 0.f); //_lltof2(_f2toll(dtemp1) & 0xFFFFFFFF00000000);
		_amem8_f2(&tempOutPtr[1])	=	F1;
		_amem8_f2(&tempOutPtr[5])	=	_ftof2(_lof2(dtemp1), 0.f);
		_amem8_f2(&tempOutPtr[4])	=	_lltof2(_f2toll(F1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	(__float2_t *) invT;
	for ( ii = 0; ii < N; ii++ )
	{
		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);

		dtemp			=	_amem8_f2(&FPtr[0]);
		F0				=	_hif2(dtemp);
		F3				=	_lof2(dtemp);
		F1				=	_amem8_f2(&FPtr[1]);
		CPtr			+=	4;
		DPtr			+=	2;
		FPtr			+=	2;

		/* NE output = - f * c, SW = conj(NW)*/
		results			=	_cmpysp(F1, C2);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
		_amem8_f2(&tempOutPtr[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[8])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C3);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
		B1				=	dtemp;
		_amem8_f2(&tempOutPtr[3])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[12])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C0);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
		_amem8_f2(&tempOutPtr[6])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[9])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
		B3				=	dtemp;
		_amem8_f2(&tempOutPtr[7])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[13])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
		

		
		/* SE output */
		/* inv(d) - conj(c) * inv(f) * c, whrer c = b * inv(d) */
		dtemp			=	_dmpysp(C0, C0);
		A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C2, C2);
		A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C0, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A0				+=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(C1, C1);
		A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C3, C3);
		A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C1, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A3				+=	2.f * _hif2(dtemp);
		_amem8_f2(&tempOutPtr[10])=	_ftof2(A0, 0.f);
		_amem8_f2(&tempOutPtr[15])=	_ftof2(A3, 0.f);

		results			=	_cmpysp(C0, B1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A1				=	_daddsp(D1, dtemp);
		results			=	_cmpysp(C2, B3);
		A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		_amem8_f2(&tempOutPtr[11])=	A1;
		_amem8_f2(&tempOutPtr[14])=	_lltof2(_f2toll(A1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 

#if 0
	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempInPtr	=	(__float2_t * ) T;
	tempOutPtr	=	(__float2_t *) invT;
	for ( ii = 0; ii < N; ii++ )
	{
		/* load A */
		dtemp			=	_amem8_f2(&tempInPtr[0]);
		A0				=	_hif2(dtemp);
		dtemp			=	_amem8_f2(&tempInPtr[5]);
		A3				=	_hif2(dtemp);
		A1				=	_amem8_f2(&tempInPtr[1]);

		/* Calculate D = inv(D) */
		dtemp			=	(__float2_t) _amem8_f2(&tempInPtr[10]);
		D0				=	_hif2(dtemp);
		dtemp			=	(__float2_t) _amem8_f2(&tempInPtr[15]);
		D3				=	_hif2(dtemp);
		D1				=	(__float2_t) _amem8_f2(&tempInPtr[11]);
		dtemp			=	_dmpysp(D1, D1);
		detA			=	D0 * D3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);

		ftemp1			=	D0 * oneOverdetA;
		D0				=	D3 * oneOverdetA;
		D3				=	ftemp1;
		D1				=	_dmpysp(D1, _ftof2(-oneOverdetA, -oneOverdetA));
		_amem8_f2(&DPtr[0])	=	_ftof2(D0, D3);
		_amem8_f2(&DPtr[1])	=	D1;
		DPtr			+=	2;

		/* load B */
		B0				=	_amem8_f2(&tempInPtr[2]);
		B1				=	_amem8_f2(&tempInPtr[3]);
		B2				=	_amem8_f2(&tempInPtr[6]);
		B3				=	_amem8_f2(&tempInPtr[7]);
		tempInPtr		+=	16;

		/* calculate C = B*inv(D) */
		results			=	_cmpysp(D1, B1);
		C0				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C0				=	_daddsp(C0, _dmpysp(B0, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B0);
		C1				=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	C1;
		C1				=	_daddsp(C1, _dmpysp(B1, _ftof2(D3, D3)));
		results			=	_cmpysp(D1, B3);
		C2				=	_dsubsp(_hif2_128(results), _lof2_128(results));
		C2				=	_daddsp(C2, _dmpysp(B2, _ftof2(D0, D0)));
		results			=	_cmpysp(D1, B2);
		C3				=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	C3;
		C3				=	_daddsp(C3, _dmpysp(B3, _ftof2(D3, D3)));
		_amem8_f2(&CPtr[0])	=	C0;
		_amem8_f2(&CPtr[1])	=	C1;
		_amem8_f2(&CPtr[2])	=	C2;
		_amem8_f2(&CPtr[3])	=	C3;
		CPtr			+=	4;

		/* calculate F = A - B *inv(D) * conj(B) -- Hermitian */
		dtemp			=	_dmpysp(B0, B0);
		F0				=	A0 - D0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(B1, B1);
		F0				-=	D3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(B1, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		F0				-=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(B2, B2);
		F3				=	A3 - D0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(B3, B3);
		F3				-=	D3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(B3, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		F3				-=	2.f * _hif2(dtemp);

		results			=	_cmpysp(B2, C0);
		F1				=	_dsubsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(B3, C1);
		F1				=	_dsubsp(F1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		/* Calculate F = inv(F) */
		dtemp			=	_dmpysp(F1, F1);
		detA			=	F0 * F3 - _hif2(dtemp) - _lof2(dtemp);
		oneOverdetA		=	_rcpsp( detA );
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		oneOverdetA		=	oneOverdetA * (2.f - detA * oneOverdetA);
		
		ftemp1			=	F0 * oneOverdetA;
		F0				=	F3 * oneOverdetA;
		F3				=	ftemp1;
		F1				=	_dmpysp(F1, _ftof2(-oneOverdetA, -oneOverdetA));
		_amem8_f2(&FPtr[0])	=	_ftof2(F0, F3);
		_amem8_f2(&FPtr[1])	=	F1;
		FPtr			+=	2;

		/* NW output */
		_amem8_f2(&tempOutPtr[0])	=	_ftof2(F0, 0.f);
		_amem8_f2(&tempOutPtr[1])	=	F1;
		_amem8_f2(&tempOutPtr[5])	=	_ftof2(F3, 0.f);
		_amem8_f2(&tempOutPtr[4])	=	_lltof2(_f2toll(F1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	} 

	CPtr		=	(__float2_t *) CC;
	DPtr		=	(__float2_t *) DD;
	FPtr		=	(__float2_t *) FF;
	tempOutPtr	=	(__float2_t *) invT;
	for ( ii = 0; ii < N; ii++ )
	{
		dtemp			=	_amem8_f2(&FPtr[0]);
		F0				=	_hif2(dtemp);
		F3				=	_lof2(dtemp);
		F1				=	_amem8_f2(&FPtr[1]);
		FPtr			+=	2;

		C0				=	_amem8_f2(&CPtr[0]);
		C1				=	_amem8_f2(&CPtr[1]);
		C2				=	_amem8_f2(&CPtr[2]);
		C3				=	_amem8_f2(&CPtr[3]);
		CPtr			+=	4;

		dtemp			=	_amem8_f2(&DPtr[0]);
		D0				=	_hif2(dtemp);
		D3				=	_lof2(dtemp);
		D1				=	_amem8_f2(&DPtr[1]);
		DPtr			+=	2;


		/* NE output = - F * C, SW = conj(NW)*/
		results			=	_cmpysp(F1, C2);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp1			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C0, _ftof2(F0, F0)));
		_amem8_f2(&tempOutPtr[2])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[8])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C3);
		dtemp			=	_daddsp(_hif2_128(results), _lof2_128(results));
		dtemp2			=	dtemp;
		dtemp			=	_daddsp(dtemp, _dmpysp(C1, _ftof2(F0, F0)));
		B1				=	dtemp;
		_amem8_f2(&tempOutPtr[3])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[12])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C0);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C2, _ftof2(F3, F3)));
		_amem8_f2(&tempOutPtr[6])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[9])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);

		results			=	_cmpysp(F1, C1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		dtemp			=	_daddsp(dtemp, _dmpysp(C3, _ftof2(F3, F3)));
		B3				=	dtemp;
		_amem8_f2(&tempOutPtr[7])		=	_lltof2(_f2toll(dtemp) ^ 0x8000000080000000);
		_amem8_f2(&tempOutPtr[13])	=	_lltof2(_f2toll(dtemp) ^ 0x8000000000000000);
		

		
		/* SE output */
		/* inv(D) - conj(C) * inv(F) * C, whrer C = B * inv(D) */
		dtemp			=	_dmpysp(C0, C0);
		A0				=	D0 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C2, C2);
		A0				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C0, dtemp1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A0				+=	2.f * _hif2(dtemp);

		dtemp			=	_dmpysp(C1, C1);
		A3				=	D3 + F0 * (_hif2(dtemp) + _lof2(dtemp));
		dtemp			=	_dmpysp(C3, C3);
		A3				+=	F3 * (_hif2(dtemp) + _lof2(dtemp));
		results			=	_cmpysp(C1, dtemp2);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A3				+=	2.f * _hif2(dtemp);
		_amem8_f2(&tempOutPtr[10])=	_ftof2(A0, 0.f);
		_amem8_f2(&tempOutPtr[15])=	_ftof2(A3, 0.f);

		results			=	_cmpysp(C0, B1);
		dtemp			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		A1				=	_daddsp(D1, dtemp);
		results			=	_cmpysp(C2, B3);
		A1				=	_daddsp(A1, _dsubsp(_hif2_128(results), _lof2_128(results)));

		_amem8_f2(&tempOutPtr[11])=	A1;
		_amem8_f2(&tempOutPtr[14])=	_lltof2(_f2toll(A1) ^ 0x0000000080000000);

		tempOutPtr		+=	16;
	}
#endif	
} // end of step 4


/************************************************************/
/************************************************************/
/*** Step 5:output SW (- invT*C*invA) and NE 4x4 matrix  ****/
/************************************************************/
/************************************************************/
void step5 (  
             IN      cplxf_t * RESTRICT invT,
             IN      cplxf_t * RESTRICT B,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT C)
{
	int32_t	ii;
	__float2_t   * RESTRICT A1, * RESTRICT A2, * RESTRICT A3, * RESTRICT A4;
	__float2_t  * RESTRICT C1, * RESTRICT C2, * RESTRICT C3, * RESTRICT C4;
	__float2_t  dtemp, dtemp1, dtemp2, dtemp3;
	__x128_t result;
	__float2_t		* RESTRICT tempOutPtr;
	__float2_t		* RESTRICT tempInPtr;
	__float2_t		* RESTRICT tempInPtr1;


	tempInPtr1	=	(__float2_t * ) invT;
	tempInPtr	=	(__float2_t *) B;
	tempOutPtr	=	(__float2_t *) C;
	for ( ii = 0; ii < N; ii++ )
	{


		A1 				=	(__float2_t *) &tempInPtr1[0];
		A2 				=	(__float2_t *) &tempInPtr1[4];
		A3 				=	(__float2_t *) &tempInPtr1[8];
		A4 				=	(__float2_t *) &tempInPtr1[12];
		C1 				=	(__float2_t *) &tempOutPtr[0];
		C2 				=	(__float2_t *) &tempOutPtr[4];
		C3 				=	(__float2_t *) &tempOutPtr[8];
		C4 				=	(__float2_t *) &tempOutPtr[12];
		result		=	_cmpysp(_amem8_f2(&A1[0]), _amem8_f2(&tempInPtr[4 * 0 + 0]));
		dtemp		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A2[0]), _amem8_f2(&tempInPtr[4 * 0 + 0]));
		dtemp1		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A3[0]), _amem8_f2(&tempInPtr[4 * 0 + 0]));
		dtemp2		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A4[0]), _amem8_f2(&tempInPtr[4 * 0 + 0]));
		dtemp3		=	_daddsp(_hif2_128(result), _lof2_128(result));

		result		=	_cmpysp(_amem8_f2(&A1[1]), _amem8_f2(&tempInPtr[4 * 1 + 0]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[1]), _amem8_f2(&tempInPtr[4 * 1 + 0]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[1]), _amem8_f2(&tempInPtr[4 * 1 + 0]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[1]), _amem8_f2(&tempInPtr[4 * 1 + 0]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[2]), _amem8_f2(&tempInPtr[4 * 2 + 0]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[2]), _amem8_f2(&tempInPtr[4 * 2 + 0]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[2]), _amem8_f2(&tempInPtr[4 * 2 + 0]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[2]), _amem8_f2(&tempInPtr[4 * 2 + 0]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[3]), _amem8_f2(&tempInPtr[4 * 3 + 0]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[3]), _amem8_f2(&tempInPtr[4 * 3 + 0]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[3]), _amem8_f2(&tempInPtr[4 * 3 + 0]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[3]), _amem8_f2(&tempInPtr[4 * 3 + 0]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		_amem8_f2(&C1[0])	=	dtemp;
		_amem8_f2(&C2[0])	=	dtemp1;
		_amem8_f2(&C3[0])	=	dtemp2;
		_amem8_f2(&C4[0])	=	dtemp3;

		tempInPtr		+=	16;
		tempInPtr1		+=	16;
		tempOutPtr		+=	16;
	}

	tempInPtr1	=	(__float2_t * ) invT;
	tempInPtr	=	(__float2_t *) B;
	tempOutPtr	=	(__float2_t *) C;
	for ( ii = 0; ii < N; ii++ )
	{


		A1 				=	(__float2_t *) &tempInPtr1[0];
		A2 				=	(__float2_t *) &tempInPtr1[4];
		A3 				=	(__float2_t *) &tempInPtr1[8];
		A4 				=	(__float2_t *) &tempInPtr1[12];
		C1 				=	(__float2_t *) &tempOutPtr[0 + 1];
		C2 				=	(__float2_t *) &tempOutPtr[4 + 1];
		C3 				=	(__float2_t *) &tempOutPtr[8 + 1];
		C4 				=	(__float2_t *) &tempOutPtr[12 + 1];

		result		=	_cmpysp(_amem8_f2(&A1[0]), _amem8_f2(&tempInPtr[4 * 0 + 1]));
		dtemp		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A2[0]), _amem8_f2(&tempInPtr[4 * 0 + 1]));
		dtemp1		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A3[0]), _amem8_f2(&tempInPtr[4 * 0 + 1]));
		dtemp2		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A4[0]), _amem8_f2(&tempInPtr[4 * 0 + 1]));
		dtemp3		=	_daddsp(_hif2_128(result), _lof2_128(result));

		result		=	_cmpysp(_amem8_f2(&A1[1]), _amem8_f2(&tempInPtr[4 * 1 + 1]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[1]), _amem8_f2(&tempInPtr[4 * 1 + 1]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[1]), _amem8_f2(&tempInPtr[4 * 1 + 1]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[1]), _amem8_f2(&tempInPtr[4 * 1 + 1]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[2]), _amem8_f2(&tempInPtr[4 * 2 + 1]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[2]), _amem8_f2(&tempInPtr[4 * 2 + 1]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[2]), _amem8_f2(&tempInPtr[4 * 2 + 1]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[2]), _amem8_f2(&tempInPtr[4 * 2 + 1]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[3]), _amem8_f2(&tempInPtr[4 * 3 + 1]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[3]), _amem8_f2(&tempInPtr[4 * 3 + 1]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[3]), _amem8_f2(&tempInPtr[4 * 3 + 1]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[3]), _amem8_f2(&tempInPtr[4 * 3 + 1]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		_amem8_f2(&C1[0])	=	dtemp;
		_amem8_f2(&C2[0])	=	dtemp1;
		_amem8_f2(&C3[0])	=	dtemp2;
		_amem8_f2(&C4[0])	=	dtemp3;

		tempInPtr		+=	16;
		tempInPtr1		+=	16;
		tempOutPtr		+=	16;
	}
	tempInPtr1	=	(__float2_t * ) invT;
	tempInPtr	=	(__float2_t *) B;
	tempOutPtr	=	(__float2_t *) C;
	for ( ii = 0; ii < N; ii++ )
	{


		A1 				=	(__float2_t *) &tempInPtr1[0];
		A2 				=	(__float2_t *) &tempInPtr1[4];
		A3 				=	(__float2_t *) &tempInPtr1[8];
		A4 				=	(__float2_t *) &tempInPtr1[12];
		C1 				=	(__float2_t *) &tempOutPtr[0 + 2];
		C2 				=	(__float2_t *) &tempOutPtr[4 + 2];
		C3 				=	(__float2_t *) &tempOutPtr[8 + 2];
		C4 				=	(__float2_t *) &tempOutPtr[12 + 2];

		result		=	_cmpysp(_amem8_f2(&A1[0]), _amem8_f2(&tempInPtr[4 * 0 + 2]));
		dtemp		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A2[0]), _amem8_f2(&tempInPtr[4 * 0 + 2]));
		dtemp1		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A3[0]), _amem8_f2(&tempInPtr[4 * 0 + 2]));
		dtemp2		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A4[0]), _amem8_f2(&tempInPtr[4 * 0 + 2]));
		dtemp3		=	_daddsp(_hif2_128(result), _lof2_128(result));

		result		=	_cmpysp(_amem8_f2(&A1[1]), _amem8_f2(&tempInPtr[4 * 1 + 2]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[1]), _amem8_f2(&tempInPtr[4 * 1 + 2]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[1]), _amem8_f2(&tempInPtr[4 * 1 + 2]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[1]), _amem8_f2(&tempInPtr[4 * 1 + 2]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[2]), _amem8_f2(&tempInPtr[4 * 2 + 2]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[2]), _amem8_f2(&tempInPtr[4 * 2 + 2]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[2]), _amem8_f2(&tempInPtr[4 * 2 + 2]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[2]), _amem8_f2(&tempInPtr[4 * 2 + 2]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[3]), _amem8_f2(&tempInPtr[4 * 3 + 2]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[3]), _amem8_f2(&tempInPtr[4 * 3 + 2]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[3]), _amem8_f2(&tempInPtr[4 * 3 + 2]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[3]), _amem8_f2(&tempInPtr[4 * 3 + 2]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		_amem8_f2(&C1[0])	=	dtemp;
		_amem8_f2(&C2[0])	=	dtemp1;
		_amem8_f2(&C3[0])	=	dtemp2;
		_amem8_f2(&C4[0])	=	dtemp3;

		tempInPtr		+=	16;
		tempInPtr1		+=	16;
		tempOutPtr		+=	16;
	}
	tempInPtr1	=	(__float2_t * ) invT;
	tempInPtr	=	(__float2_t *) B;
	tempOutPtr	=	(__float2_t *) C;
	for ( ii = 0; ii < N; ii++ )
	{


		A1 				=	(__float2_t *) &tempInPtr1[0];
		A2 				=	(__float2_t *) &tempInPtr1[4];
		A3 				=	(__float2_t *) &tempInPtr1[8];
		A4 				=	(__float2_t *) &tempInPtr1[12];
		C1 				=	(__float2_t *) &tempOutPtr[0 + 3];
		C2 				=	(__float2_t *) &tempOutPtr[4 + 3];
		C3 				=	(__float2_t *) &tempOutPtr[8 + 3];
		C4 				=	(__float2_t *) &tempOutPtr[12 + 3];

		result		=	_cmpysp(_amem8_f2(&A1[0]), _amem8_f2(&tempInPtr[4 * 0 + 3]));
		dtemp		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A2[0]), _amem8_f2(&tempInPtr[4 * 0 + 3]));
		dtemp1		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A3[0]), _amem8_f2(&tempInPtr[4 * 0 + 3]));
		dtemp2		=	_daddsp(_hif2_128(result), _lof2_128(result));
		result		=	_cmpysp(_amem8_f2(&A4[0]), _amem8_f2(&tempInPtr[4 * 0 + 3]));
		dtemp3		=	_daddsp(_hif2_128(result), _lof2_128(result));

		result		=	_cmpysp(_amem8_f2(&A1[1]), _amem8_f2(&tempInPtr[4 * 1 + 3]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[1]), _amem8_f2(&tempInPtr[4 * 1 + 3]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[1]), _amem8_f2(&tempInPtr[4 * 1 + 3]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[1]), _amem8_f2(&tempInPtr[4 * 1 + 3]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[2]), _amem8_f2(&tempInPtr[4 * 2 + 3]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[2]), _amem8_f2(&tempInPtr[4 * 2 + 3]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[2]), _amem8_f2(&tempInPtr[4 * 2 + 3]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[2]), _amem8_f2(&tempInPtr[4 * 2 + 3]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		result		=	_cmpysp(_amem8_f2(&A1[3]), _amem8_f2(&tempInPtr[4 * 3 + 3]));
		dtemp		=	_daddsp(dtemp, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A2[3]), _amem8_f2(&tempInPtr[4 * 3 + 3]));
		dtemp1		=	_daddsp(dtemp1, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A3[3]), _amem8_f2(&tempInPtr[4 * 3 + 3]));
		dtemp2		=	_daddsp(dtemp2, _daddsp(_hif2_128(result), _lof2_128(result)));
		result		=	_cmpysp(_amem8_f2(&A4[3]), _amem8_f2(&tempInPtr[4 * 3 + 3]));
		dtemp3		=	_daddsp(dtemp3, _daddsp(_hif2_128(result), _lof2_128(result)));

		_amem8_f2(&C1[0])	=	dtemp;
		_amem8_f2(&C2[0])	=	dtemp1;
		_amem8_f2(&C3[0])	=	dtemp2;
		_amem8_f2(&C4[0])	=	dtemp3;
		tempInPtr		+=	16;
		tempInPtr1		+=	16;
		tempOutPtr		+=	16;
	}
} // end of step 5


/************************************************************/
/************************************************************/
/****** Step 6:output NW (invA + invA*B*invT*C*invA)  *******/
/************************************************************/
/************************************************************/
void step6 (  
             IN      cplxf_t * RESTRICT invA,
             IN      cplxf_t * RESTRICT B,
             IN      cplxf_t  * RESTRICT C,
             IN      int32_t	  N,
             OUT     cplxf_t  * RESTRICT output)
{
	int32_t		ii;
	__x128_t	results;
	__float2_t		dtemp1;
	__float2_t		* RESTRICT invAPtr;
	__float2_t		* RESTRICT BPtr;
	__float2_t		* RESTRICT CPtr;
	__float2_t		* RESTRICT tempOutPtr;


	invAPtr		=	(__float2_t * ) invA;
	CPtr		=	(__float2_t * ) C;
	tempOutPtr	=	(__float2_t *) output;
	BPtr		=	(__float2_t *) B;
	for ( ii = 0; ii < N; ii++ )
	{

		results			=	_cmpysp(_amem8_f2(&BPtr[0]), _amem8_f2(&CPtr[0]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[4]), _amem8_f2(&CPtr[4]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[8]), _amem8_f2(&CPtr[8]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[12]), _amem8_f2(&CPtr[12]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[0]), dtemp1);
		_amem8_f2(&tempOutPtr[0])	=	dtemp1;

		results			=	_cmpysp(_amem8_f2(&BPtr[0]), _amem8_f2(&CPtr[1]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[4]), _amem8_f2(&CPtr[5]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[8]), _amem8_f2(&CPtr[9]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[12]), _amem8_f2(&CPtr[13]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[1]), dtemp1);
		_amem8_f2(&tempOutPtr[1])	=	dtemp1;
		_amem8_f2(&tempOutPtr[8])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&BPtr[1]), _amem8_f2(&CPtr[1]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[5]), _amem8_f2(&CPtr[5]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[9]), _amem8_f2(&CPtr[9]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[13]), _amem8_f2(&CPtr[13]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[5]), dtemp1);
		_amem8_f2(&tempOutPtr[9])	=	dtemp1;
		
		results			=	_cmpysp(_amem8_f2(&BPtr[0]), _amem8_f2(&CPtr[2]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[4]), _amem8_f2(&CPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[8]), _amem8_f2(&CPtr[10]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[12]), _amem8_f2(&CPtr[14]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[2]), dtemp1);
		_amem8_f2(&tempOutPtr[2])	=	dtemp1;
		_amem8_f2(&tempOutPtr[16])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&BPtr[1]), _amem8_f2(&CPtr[2]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[5]), _amem8_f2(&CPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[9]), _amem8_f2(&CPtr[10]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[13]), _amem8_f2(&CPtr[14]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[6]), dtemp1);
		_amem8_f2(&tempOutPtr[10])	=	dtemp1;
		_amem8_f2(&tempOutPtr[17])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		tempOutPtr		+=	64;
		invAPtr			+=	16;
		BPtr			+=	16;
		CPtr			+=	16;
	}

	invAPtr		=	(__float2_t * ) invA;
	CPtr		=	(__float2_t * ) C;
	tempOutPtr	=	(__float2_t *) output;
	BPtr		=	(__float2_t *) B;
	for ( ii = 0; ii < N; ii++ )
	{

	
		results			=	_cmpysp(_amem8_f2(&BPtr[2]), _amem8_f2(&CPtr[2]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[6]), _amem8_f2(&CPtr[6]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[10]), _amem8_f2(&CPtr[10]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[14]), _amem8_f2(&CPtr[14]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[10]), dtemp1);
		_amem8_f2(&tempOutPtr[18])	=	dtemp1;

		results			=	_cmpysp(_amem8_f2(&BPtr[0]), _amem8_f2(&CPtr[3]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[4]), _amem8_f2(&CPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[8]), _amem8_f2(&CPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[12]), _amem8_f2(&CPtr[15]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[3]), dtemp1);
		_amem8_f2(&tempOutPtr[3])	=	dtemp1;
		_amem8_f2(&tempOutPtr[24])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&BPtr[1]), _amem8_f2(&CPtr[3]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[5]), _amem8_f2(&CPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[9]), _amem8_f2(&CPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[13]), _amem8_f2(&CPtr[15]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[7]), dtemp1);
		_amem8_f2(&tempOutPtr[11])	=	dtemp1;
		_amem8_f2(&tempOutPtr[25])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&BPtr[2]), _amem8_f2(&CPtr[3]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[6]), _amem8_f2(&CPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[10]), _amem8_f2(&CPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[14]), _amem8_f2(&CPtr[15]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[11]), dtemp1);
		_amem8_f2(&tempOutPtr[19])	=	dtemp1;
		_amem8_f2(&tempOutPtr[26])	=	_lltof2(_f2toll(dtemp1) ^ 0x0000000080000000);
		
		results			=	_cmpysp(_amem8_f2(&BPtr[3]), _amem8_f2(&CPtr[3]));
		dtemp1			=	_dsubsp(_hif2_128(results), _lof2_128(results));
		results			=	_cmpysp(_amem8_f2(&BPtr[7]), _amem8_f2(&CPtr[7]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[11]), _amem8_f2(&CPtr[11]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		results			=	_cmpysp(_amem8_f2(&BPtr[15]), _amem8_f2(&CPtr[15]));
		dtemp1			=	_daddsp(dtemp1, _dsubsp(_hif2_128(results), _lof2_128(results)));
		dtemp1			=	_daddsp(_amem8_f2(&invAPtr[15]), dtemp1);
		_amem8_f2(&tempOutPtr[27])	=	dtemp1;

		tempOutPtr		+=	64;
		invAPtr			+=	16;
		BPtr			+=	16;
		CPtr			+=	16;
	}

} // end of step 6
#endif
