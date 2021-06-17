/*! 
 * \file   RADARDEMO_aoaEstDML_priv.c
 *
 *  \brief   Estimate the angle of arrival using DML. 
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/

#include "RADARDEMO_aoaEstDM_priv.h"
//#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif
#define PIOVER180 (3.141592653589793/180.0)
#define PI        (3.141592653589793f)


/*! 
   \fn     RADARDEMO_aoaEstimationDML_8ant
 
   \brief   Estimate the angle of arrival of each detected object using DML for 8 Rx antennas. 
  
   \param[in]    Rn
               input covariance matrix from input antenna samples.
 
   \param[in]    steeringVec
               Pointer to steering vector.
 
   \param[in]    steeringVecSize
               Size of the steering vector.
 
   \param[in]    scratchPad
               Scratch memory.
 
   \param[in]    firstStageSearchStep
               First stage search step size, if 1, no second stage search needed.
 
   \param[out]    normVar
               Output normalized variance of the search metric.
			   
   \param[out]    angleEst
               Output angle estimates.
			   
   \ret        Number of output angle estimates. Retuns 0 if nRxAnt !=4 , 
 
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_aoaEstimationDML_8ant(
							IN  cplxf_t * Rn,
							IN  cplxf_t * steeringVec,
							IN  int32_t steeringVecSize,
							IN  float   * scratchPad,
							IN  uint8_t   firstStageSearchStep,
							OUT float     * normVar,
							OUT int32_t   * angleEst)
{
	/*
	 Solve: [theta1, theta2] = argmax(trace(A*inv(A'*A)*A'*Rn)
	 where:
	         A = [steeringVec(theta1) steeringVec(theta2)] is a nRxAnt by 2 matrix
			 Rn is the covariance matrix of the antenna signal
	*/

	__float2_t	a11, a12, a13, a14, a15, a16, a17, a2j;
	float       a11Ma21Re, a12Ma22Re, a13Ma23Re;
	float		R11PR88, R22PR77, R33PR66, R44PR55;
	__float2_t	conja1iMa2j, a1iPa2j;
	__float2_t	invAoffdiag;
	__float2_t	f2temp, Btemp, f2temp1;
	__float2_t	* RESTRICT steeringVecPtr, * RESTRICT tempPtr;
	int32_t	    i, j, i2,steeringVecSizeTemp;
	float 		max, metric, ftemp1, ftemp2, avg, var;
	int32_t     max_i, max_j;
	float 		B11, B22, B33, B44;
	__float2_t  * RESTRICT tempInvAoffdiagBuf;
	float       * RESTRICT tempMetricBuf, temprcp;
			
	/*
	A = [ 1    1
		    a11  a21
			a12  a22
		    a13  a23
			a14  a24
		    a15  a25
			a16  a26
			a17  a27];
	let B = A*inv(A'*A)*A';
	Because antennas is uniform linear, a = exp(-1j*pi*d*[0 1 2 3]*sin([angles(:).']*pi/180)), we can simplify calculation below:
	inv(A'*A) = [8  			-invAoffdiag
		            -invAoffdiag   8			] / (64 - invAoffdiag'*invAoffdiag)
	invAoffdiag	=	1 + sum(a1i'*a2i) for i = 1:7	 
	B11 = B88 = 16 - 2*real(invAoffdiag);
	Bi1 = B(9-i)1 = conj(B1i) = conj(B1(8-i) = 8*(a1i + a2i) - a*a1i - a'*a2i;
	Bij = B(9-i)(9-j) = conj(Bji) = conj(B(9-j)(9-i)) = 8*(a1(i-j) + a1(i-j)) - a*a2j'*a11i - a'*a1j'*a2i;
	B = [B11  B12  B13 B14 ... B18
		    B12' B22  B23 B13 ... B28
			B13' B23' B22 B23 ... B28
			...
			B18' B17' B16' B11]/(64 - invAoffdiag'*invAoffdiag)
	*/
		
	if(firstStageSearchStep > 1)
	{
		tempPtr			=	(__float2_t *) steeringVec;
		steeringVecPtr 	=	(__float2_t *) &scratchPad[3 * (steeringVecSize >> 1) + 2];
		j				=	0;
		for (i = 0; i < steeringVecSize; i+=firstStageSearchStep)
		{
			_amem8_f2(&steeringVecPtr[7 * j + 0]) 	=	_amem8_f2(&tempPtr[7 * i + 0]);
			_amem8_f2(&steeringVecPtr[7 * j + 1]) 	=	_amem8_f2(&tempPtr[7 * i + 1]);
			_amem8_f2(&steeringVecPtr[7 * j + 2]) 	=	_amem8_f2(&tempPtr[7 * i + 2]);
			_amem8_f2(&steeringVecPtr[7 * j + 3]) 	=	_amem8_f2(&tempPtr[7 * i + 3]);
			_amem8_f2(&steeringVecPtr[7 * j + 4]) 	=	_amem8_f2(&tempPtr[7 * i + 4]);
			_amem8_f2(&steeringVecPtr[7 * j + 5]) 	=	_amem8_f2(&tempPtr[7 * i + 5]);
			_amem8_f2(&steeringVecPtr[7 * j + 6]) 	=	_amem8_f2(&tempPtr[7 * i + 6]);
			j++;
		}
		steeringVecSizeTemp 	=	j;
	}
	else
	{
		steeringVecPtr			=	(__float2_t *) steeringVec;
		steeringVecSizeTemp 	=	steeringVecSize;
	}

	max 			=	0.f;
	max_i			=	-1;
	max_j			=	-1;
	R11PR88			=	(Rn[0].real + Rn[35].real);
	R22PR77 		=	(Rn[8].real + Rn[33].real);
	R33PR66 		=	(Rn[15].real + Rn[30].real);
	R44PR55 		=	(Rn[21].real + Rn[26].real);
	for (i = 0; i < steeringVecSizeTemp; i++)
	{
		a11 	=	_amem8_f2(&steeringVecPtr[7*i + 0]);
		a12 	=	_amem8_f2(&steeringVecPtr[7*i + 1]);
		a13 	=	_amem8_f2(&steeringVecPtr[7*i + 2]);
		a14 	=	_amem8_f2(&steeringVecPtr[7*i + 3]);
		a15 	=	_amem8_f2(&steeringVecPtr[7*i + 4]);
		a16 	=	_amem8_f2(&steeringVecPtr[7*i + 5]);
		a17 	=	_amem8_f2(&steeringVecPtr[7*i + 6]);
		/* first loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		/* B11 B22 B33 B44 invAoffdiag*/
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			invAoffdiag		=	_ftof2(1.f, 0);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a11, a2j);
			a11Ma21Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a12, a2j);
			a12Ma22Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a13, a2j);
			a13Ma23Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a14, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B44				=	7.f - _hif2(conja1iMa2j) - 2.f * (a13Ma23Re + a12Ma22Re + a11Ma21Re);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a15, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B33				=	8.f - _hif2(invAoffdiag) - (a12Ma22Re + a11Ma21Re);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a16, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B22				=	8.f - _hif2(invAoffdiag) - a11Ma21Re;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a17, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B11 			=	8.f - _hif2(invAoffdiag);
			metric			=	B11 * R11PR88;
			metric 			+=	B22 * R22PR77;
			metric 			+=	B33 * R33PR66;
			metric 			+=	B44 * R44PR55;

			_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
			*tempMetricBuf++ 				= 	metric;
		}
		/* second loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		/* B21 B32 B43 B54 B81 B71*/
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			a1iPa2j			=	_daddsp(a11, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B21*(R12+R78))/2 i = 1*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a11);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[1]), _amem8_f2(&Rn[34]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B32*(R23+R67))/2 i = 2, j = 1*/
			f2temp			=	_complex_conjugate_mpysp(a2j, a12);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[9]), _amem8_f2(&Rn[31]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B43*(R34+R56))/2 i = 3, j = 2*/
			f2temp1			=	a2j;
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			f2temp			=	_complex_conjugate_mpysp(f2temp1, a13);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[16]), _amem8_f2(&Rn[27]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B54*(R45))/2 i = 4, j = 3*/
			f2temp1			=	a2j;
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(f2temp1, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_conjugate_mpysp(a13, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[22]));
			metric			+=	_hif2(f2temp);

			*tempMetricBuf++ 				= 	metric;
		}

		/* 3rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		/* B31 B42 B53 B61 B72*/
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			/* metric += 2*real(B71*(R17+R28))/2 i = 6*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			a1iPa2j			=	_daddsp(a16, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a16);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[6]), _amem8_f2(&Rn[14]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			a1iPa2j			=	_daddsp(a12, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B31*(R13+R68))/2 i = 2*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a12);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[2]), _amem8_f2(&Rn[32]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B42*(R24+R57))/2 i = 3, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a13);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[10]), _amem8_f2(&Rn[28]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B53*(R35+R46))/2 i = 4, j = 2*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[17]), _amem8_f2(&Rn[23]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B81*(R18))/2 i = 7*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
			a1iPa2j			=	_daddsp(a17, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a17);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[7]));
			metric			+=	_hif2(f2temp);

			*tempMetricBuf++ 				= 	metric;
		}

		/* 4rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		/* B31 B42 B53 B61 B72*/
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			a1iPa2j			=	_daddsp(a14, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B51*(R15+R48))/2 i = 4*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a14);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[4]), _amem8_f2(&Rn[25]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B62*(R26+R37))/2 i = 5, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a15);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[12]), _amem8_f2(&Rn[19]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			a1iPa2j			=	_daddsp(a15, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B61*(R16+R38))/2 i = 5*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a15);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[5]), _amem8_f2(&Rn[20]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B72*(R27))/2 i = 6, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a16);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[13]));
			metric			+=	_hif2(f2temp);


			*tempMetricBuf++ 				= 	metric;
		}

		/* 5rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		/* B41 B52 B63  B51 B62*/ 
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf++;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			a1iPa2j			=	_daddsp(a13, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
				
			/* metric += 2*real(B41*(R14+R58))/2 i = 3*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a13);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[3]), _amem8_f2(&Rn[29]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);


			/* metric += 2*real(B52*(R25+R47))/2 i = 4, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[11]), _amem8_f2(&Rn[24]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B63*(R36))/2 i = 5, j = 2*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a15);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[18]));
			metric			+=	_hif2(f2temp);

			f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
			ftemp1 			=	64.f - _hif2(f2temp1) - _lof2(f2temp1);
			ftemp2			=	_rcpsp(ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			metric 			=	metric * ftemp2;
			if ( metric > max )
			{
				max 		=	metric;
				max_i 		=	i;
				max_j 		=	j;
			}

		}
	}

	steeringVecPtr			=	(__float2_t *) steeringVec;
	if (firstStageSearchStep > 1)
	{
		int32_t  iStart, iStop, jStart, jStop;

		iStart			=	(max_i - 2) * firstStageSearchStep;
		if (iStart < 0)
			iStart		=	0;
		iStop			=	(max_i + 2) * firstStageSearchStep + 1;
		if (iStop > steeringVecSize)
			iStop		=	steeringVecSize;
		jStart			=	(max_j - 2) * firstStageSearchStep;
		if (jStart < 0)
			jStart		=	0;
		jStop			=	(max_j + 2) * firstStageSearchStep + 1;
		if (jStop > steeringVecSize)
			jStop		=	steeringVecSize;

		max 			=	0.f;
		max_i			=	-1;
		max_j			=	-1;

		for (i = iStart; i < iStop; i++)
		{
			a11 	=	_amem8_f2(&steeringVecPtr[7*i + 0]);
			a12 	=	_amem8_f2(&steeringVecPtr[7*i + 1]);
			a13 	=	_amem8_f2(&steeringVecPtr[7*i + 2]);
			a14 	=	_amem8_f2(&steeringVecPtr[7*i + 3]);
			a15 	=	_amem8_f2(&steeringVecPtr[7*i + 4]);
			a16 	=	_amem8_f2(&steeringVecPtr[7*i + 5]);
			a17 	=	_amem8_f2(&steeringVecPtr[7*i + 6]);
			/* first loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			/* B11 B22 B33 B44 invAoffdiag*/
			for (j = jStart; j < jStop; j++)
			{
				invAoffdiag		=	_ftof2(1.f, 0);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a11, a2j);
				a11Ma21Re		=	_hif2(conja1iMa2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a12, a2j);
				a12Ma22Re		=	_hif2(conja1iMa2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a13, a2j);
				a13Ma23Re		=	_hif2(conja1iMa2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a14, a2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				B44				=	7.f - _hif2(conja1iMa2j) - 2.f * (a13Ma23Re + a12Ma22Re + a11Ma21Re);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a15, a2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				B33				=	8.f - _hif2(invAoffdiag) - (a12Ma22Re + a11Ma21Re);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a16, a2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				B22				=	8.f - _hif2(invAoffdiag) - a11Ma21Re;

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
				conja1iMa2j		=	_complex_conjugate_mpysp(a17, a2j);
				invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

				B11 			=	8.f - _hif2(invAoffdiag);
				metric			=	B11 * R11PR88;
				metric 			+=	B22 * R22PR77;
				metric 			+=	B33 * R33PR66;
				metric 			+=	B44 * R44PR55;

				_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
				*tempMetricBuf++ 				= 	metric;
			}
			/* second loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			/* B21 B32 B43 B54 B81 B71*/
			for (j = jStart; j < jStop; j++)
			{
				invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
				metric			=	*tempMetricBuf;

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				a1iPa2j			=	_daddsp(a11, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

				/* metric += 2*real(B21*(R12+R78))/2 i = 1*/
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a11);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[1]), _amem8_f2(&Rn[34]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B32*(R23+R67))/2 i = 2, j = 1*/
				f2temp			=	_complex_conjugate_mpysp(a2j, a12);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j				=	_amem8_f2(&steeringVecPtr[7*j + 1]);
				f2temp			=	_complex_conjugate_mpysp(a11, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[9]), _amem8_f2(&Rn[31]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B43*(R34+R56))/2 i = 3, j = 2*/
				f2temp1			=	a2j;
				a2j				=	_amem8_f2(&steeringVecPtr[7*j + 2]);
				f2temp			=	_complex_conjugate_mpysp(f2temp1, a13);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_conjugate_mpysp(a12, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[16]), _amem8_f2(&Rn[27]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B54*(R45))/2 i = 4, j = 3*/
				f2temp1			=	a2j;
				a2j				=	_amem8_f2(&steeringVecPtr[7*j + 3]);
				f2temp			=	_complex_conjugate_mpysp(f2temp1, a14);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_conjugate_mpysp(a13, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[22]));
				metric			+=	_hif2(f2temp);

				*tempMetricBuf++ 				= 	metric;
			}

			/* 3rd loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			/* B31 B42 B53 B61 B72*/
			for (j = jStart; j < jStop; j++)
			{
				invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
				metric			=	*tempMetricBuf;

				/* metric += 2*real(B71*(R17+R28))/2 i = 6*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
				a1iPa2j			=	_daddsp(a16, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a16);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[6]), _amem8_f2(&Rn[14]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
				a1iPa2j			=	_daddsp(a12, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

				/* metric += 2*real(B31*(R13+R68))/2 i = 2*/
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a12);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[2]), _amem8_f2(&Rn[32]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B42*(R24+R57))/2 i = 3, j = 1*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a13);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
				f2temp			=	_complex_conjugate_mpysp(a11, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[10]), _amem8_f2(&Rn[28]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B53*(R35+R46))/2 i = 4, j = 2*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a14);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
				f2temp			=	_complex_conjugate_mpysp(a12, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[17]), _amem8_f2(&Rn[23]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B81*(R18))/2 i = 7*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
				a1iPa2j			=	_daddsp(a17, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a17);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[7]));
				metric			+=	_hif2(f2temp);

				*tempMetricBuf++ 				= 	metric;
			}

			/* 4rd loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			/* B31 B42 B53 B61 B72*/
			for (j = jStart; j < jStop; j++)
			{
				invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
				metric			=	*tempMetricBuf;

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
				a1iPa2j			=	_daddsp(a14, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

				/* metric += 2*real(B51*(R15+R48))/2 i = 4*/
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a14);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[4]), _amem8_f2(&Rn[25]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B62*(R26+R37))/2 i = 5, j = 1*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a15);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
				f2temp			=	_complex_conjugate_mpysp(a11, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[12]), _amem8_f2(&Rn[19]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
				a1iPa2j			=	_daddsp(a15, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

				/* metric += 2*real(B61*(R16+R38))/2 i = 5*/
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a15);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[5]), _amem8_f2(&Rn[20]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B72*(R27))/2 i = 6, j = 1*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a16);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
				f2temp			=	_complex_conjugate_mpysp(a11, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[13]));
				metric			+=	_hif2(f2temp);


				*tempMetricBuf++ 				= 	metric;
			}

			/* 5rd loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			/* B41 B52 B63  B51 B62*/ 
			for (j = jStart; j < jStop; j++)
			{
				invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
				metric			=	*tempMetricBuf++;

				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
				a1iPa2j			=	_daddsp(a13, a2j);
				a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
				
				/* metric += 2*real(B41*(R14+R58))/2 i = 3*/
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				f2temp			=	_complex_mpysp(invAoffdiag, a13);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[3]), _amem8_f2(&Rn[29]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);


				/* metric += 2*real(B52*(R25+R47))/2 i = 4, j = 1*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a14);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
				f2temp			=	_complex_conjugate_mpysp(a11, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_daddsp(_amem8_f2(&Rn[11]), _amem8_f2(&Rn[24]));
				f2temp			=	_complex_mpysp(Btemp, f2temp);
				metric			+=	_hif2(f2temp);

				/* metric += 2*real(B63*(R36))/2 i = 5, j = 2*/
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
				f2temp			=	_complex_conjugate_mpysp(a2j, a15);
				f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(a1iPa2j, f2temp);
				a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
				f2temp			=	_complex_conjugate_mpysp(a12, a2j);
				f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
				Btemp			=	_dsubsp(Btemp, f2temp);
				f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[18]));
				metric			+=	_hif2(f2temp);

				f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
				ftemp1 			=	64.f - _hif2(f2temp1) - _lof2(f2temp1);
				ftemp2			=	_rcpsp(ftemp1);
				ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
				ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
				metric 			=	metric * ftemp2;
				if ( metric > max )
				{
					max 		=	metric;
					max_i 		=	i;
					max_j 		=	j;
				}

			}
		}
	}

	/* find whether there is 1 target or 2 */
	for (i = 0; i < 2; i++)
	{
		if ( i == 0 )
			i2 = max_i;
		else
			i2 = max_j;
		a11 	=	_amem8_f2(&steeringVecPtr[7*max_i + 0]);
		a12 	=	_amem8_f2(&steeringVecPtr[7*max_i + 1]);
		a13 	=	_amem8_f2(&steeringVecPtr[7*max_i + 2]);
		a14 	=	_amem8_f2(&steeringVecPtr[7*max_i + 3]);
		a15 	=	_amem8_f2(&steeringVecPtr[7*max_i + 4]);
		a16 	=	_amem8_f2(&steeringVecPtr[7*max_i + 5]);
		a17 	=	_amem8_f2(&steeringVecPtr[7*max_i + 6]);
		/* first loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		/* B11 B22 B33 B44 invAoffdiag*/
		for (j = 0; j < steeringVecSize; j++)
		{
			invAoffdiag		=	_ftof2(1.f, 0);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a11, a2j);
			a11Ma21Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a12, a2j);
			a12Ma22Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a13, a2j);
			a13Ma23Re		=	_hif2(conja1iMa2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a14, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B44				=	7.f - _hif2(conja1iMa2j) - 2.f * (a13Ma23Re + a12Ma22Re + a11Ma21Re);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a15, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B33				=	8.f - _hif2(invAoffdiag) - (a12Ma22Re + a11Ma21Re);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a16, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B22				=	8.f - _hif2(invAoffdiag) - a11Ma21Re;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
			conja1iMa2j		=	_complex_conjugate_mpysp(a17, a2j);
			invAoffdiag		=	_daddsp(invAoffdiag, conja1iMa2j);

			B11 			=	8.f - _hif2(invAoffdiag);
			metric			=	B11 * R11PR88;
			metric 			+=	B22 * R22PR77;
			metric 			+=	B33 * R33PR66;
			metric 			+=	B44 * R44PR55;

			_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
			*tempMetricBuf++ 				= 	metric;
		}
		/* second loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		/* B21 B32 B43 B54 B81 B71*/
		for (j = 0; j < steeringVecSize; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			a1iPa2j			=	_daddsp(a11, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B21*(R12+R78))/2 i = 1*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a11);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[1]), _amem8_f2(&Rn[34]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B32*(R23+R67))/2 i = 2, j = 1*/
			f2temp			=	_complex_conjugate_mpysp(a2j, a12);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[9]), _amem8_f2(&Rn[31]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B43*(R34+R56))/2 i = 3, j = 2*/
			f2temp1			=	a2j;
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			f2temp			=	_complex_conjugate_mpysp(f2temp1, a13);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[16]), _amem8_f2(&Rn[27]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B54*(R45))/2 i = 4, j = 3*/
			f2temp1			=	a2j;
			a2j				=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(f2temp1, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_conjugate_mpysp(a13, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[22]));
			metric			+=	_hif2(f2temp);

			*tempMetricBuf++ 				= 	metric;
		}

		/* 3rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		/* B31 B42 B53 B61 B72*/
		for (j = 0; j < steeringVecSize; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			/* metric += 2*real(B71*(R17+R28))/2 i = 6*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			a1iPa2j			=	_daddsp(a16, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a16);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[6]), _amem8_f2(&Rn[14]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			a1iPa2j			=	_daddsp(a12, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B31*(R13+R68))/2 i = 2*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a12);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[2]), _amem8_f2(&Rn[32]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B42*(R24+R57))/2 i = 3, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a13);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[10]), _amem8_f2(&Rn[28]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B53*(R35+R46))/2 i = 4, j = 2*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[17]), _amem8_f2(&Rn[23]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B81*(R18))/2 i = 7*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 6]);
			a1iPa2j			=	_daddsp(a17, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a17);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[7]));
			metric			+=	_hif2(f2temp);

			*tempMetricBuf++ 				= 	metric;
		}

		/* 4rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		/* B31 B42 B53 B61 B72*/
		for (j = 0; j < steeringVecSize; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			a1iPa2j			=	_daddsp(a14, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B51*(R15+R48))/2 i = 4*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a14);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[4]), _amem8_f2(&Rn[25]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B62*(R26+R37))/2 i = 5, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a15);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[12]), _amem8_f2(&Rn[19]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			a1iPa2j			=	_daddsp(a15, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B61*(R16+R38))/2 i = 5*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a15);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[5]), _amem8_f2(&Rn[20]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B72*(R27))/2 i = 6, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a16);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 5]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[13]));
			metric			+=	_hif2(f2temp);


			*tempMetricBuf++ 				= 	metric;
		}

		/* 5rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		/* B41 B52 B63  B51 B62*/
		for (j = 0; j < steeringVecSize; j++)
		{
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf++;

			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 2]);
			a1iPa2j			=	_daddsp(a13, a2j);
			a1iPa2j			=	_dmpysp(_ftof2(8.f, 8.f), a1iPa2j);

			/* metric += 2*real(B41*(R14+R58))/2 i = 3*/
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, a2j);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			f2temp			=	_complex_mpysp(invAoffdiag, a13);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[3]), _amem8_f2(&Rn[29]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);


			/* metric += 2*real(B52*(R25+R47))/2 i = 4, j = 1*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 0]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a14);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 3]);
			f2temp			=	_complex_conjugate_mpysp(a11, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_daddsp(_amem8_f2(&Rn[11]), _amem8_f2(&Rn[24]));
			f2temp			=	_complex_mpysp(Btemp, f2temp);
			metric			+=	_hif2(f2temp);

			/* metric += 2*real(B63*(R36))/2 i = 5, j = 2*/
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 1]);
			f2temp			=	_complex_conjugate_mpysp(a2j, a15);
			f2temp			=	_complex_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(a1iPa2j, f2temp);
			a2j 			=	_amem8_f2(&steeringVecPtr[7*j + 4]);
			f2temp			=	_complex_conjugate_mpysp(a12, a2j);
			f2temp			=	_complex_conjugate_mpysp(invAoffdiag, f2temp);
			Btemp			=	_dsubsp(Btemp, f2temp);
			f2temp			=	_complex_mpysp(Btemp, _amem8_f2(&Rn[18]));
			metric			+=	_hif2(f2temp);

			f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
			ftemp1 			=	64.f - _hif2(f2temp1) - _lof2(f2temp1);
			ftemp2			=	_rcpsp(ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			metric 			=	metric * ftemp2;
			if (j == i2) metric = 0.f;
			avg				+=	metric;
			*tempMetricBuf++ = 	metric;
		}
		tempMetricBuf	=	(float *) &scratchPad[2 * steeringVecSize];
		ftemp1			=	_rcpsp((float)steeringVecSize);
		ftemp1			=   ftemp1 * (2.f - ftemp1 * (float)steeringVecSize);
		ftemp1			=   ftemp1 * (2.f - ftemp1 * (float)steeringVecSize);
		avg				=	avg * ftemp1;
		f2temp			=	_ftof2(avg, avg);
		var				=	0.f;
		f2temp1			=	_ftof2(var, var);
		for (j = 0; j < steeringVecSize; j++)
		{
			metric		=	tempMetricBuf[j];
			ftemp1		=	metric - avg;
			if (j == i2) ftemp1 = 0.f;
			var			+=	ftemp1 * ftemp1;
		}
		ftemp1			=	avg * avg * (float)steeringVecSize;
		temprcp			=	_rcpsp(ftemp1);	
		temprcp			=	temprcp * (2.f - ftemp1 * temprcp);
		temprcp			=	temprcp * (2.f - ftemp1 * temprcp);
		ftemp2			=	var * temprcp;
		
		normVar[i] = ftemp2;
	}
	angleEst[0]		=	max_i;
	angleEst[1]		=	max_j;
	i = 2;
	return(i);
}


/*! 
   \fn     RADARDEMO_aoaEstimationDML_4ant
 
   \brief   Estimate the angle of arrival of each detected object using DML for 4 Rx antennas. 
  
   \param[in]    Rn
               input covariance matrix from input antenna samples.
 
   \param[in]    steeringVec
               Pointer to steering vector.
 
   \param[in]    steeringVecSize
               Size of the steering vector.
 
   \param[in]    scratchPad
               Scratch memory.
 
   \param[in]    firstStageSearchStep
               First stage search step size, if 1, no second stage search needed.
 
   \param[out]    normVar
               Output normalized variance of the search metric.
			   
   \param[out]    angleEst
               Output angle estimates.
			   
   \ret        Number of output angle estimates. Retuns 0 if nRxAnt !=4 , 
 
   \pre       none
 
   \post      none
  
 
 */

int32_t	RADARDEMO_aoaEstimationDML_4ant(
							IN  cplxf_t * Rn,
							IN  cplxf_t * steeringVec,
							IN  int32_t steeringVecSize,
							IN  float   * scratchPad,
							IN  uint8_t   firstStageSearchStep,
							OUT float     * normVar,
							OUT int32_t   * angleEst)
{
	/*
	 Solve: [theta1, theta2] = argmax(trace(A*inv(A'*A)*A'*Rn)
	 where:
	         A = [steeringVec(theta1) steeringVec(theta2)] is a nRxAnt by 2 matrix
			 Rn is the covariance matrix of the antenna signal
	*/

	__float2_t	a11, a12, a13, a21, a22, a23;
	__float2_t	conja11Ma21, conja12Ma22, conja13Ma23, a11Pa21;
	__float2_t	R12PR34, R13PR24;
	float	R11PR44, R22PR33;
	__float2_t	invAoffdiag;
	__float2_t	Btemp, f2temp1;
	__float2_t	* RESTRICT steeringVecPtr, * RESTRICT tempPtr;
	int32_t	    i, j, i2,steeringVecSizeTemp;
	float 		max, metric, ftemp1, ftemp2, avg, var;
	int32_t     max_i, max_j;
	float 		B11, B22, temprcp;
	__float2_t  * RESTRICT tempInvAoffdiagBuf;
	float       * RESTRICT tempMetricBuf;
			
	/*
	A = [ 1    1
		    a11  a21
			a12  a22
			a13  a23];
	let B = A*inv(A'*A)*A';
	Because antennas is uniform linear, a = exp(-1j*pi*d*[0 1 2 3]*sin([angles(:).']*pi/180)), we can simplify calculation below:
	inv(A'*A) = [4  			-invAoffdiag
		            -invAoffdiag   4			] / (16 - invAoffdiag'*invAoffdiag
	invAoffdiag	=	1 + a11'*a21 + a12'*a22 + a13'*a23	 
	B11 = 8 - 2*real(invAoffdiag)
	B22 = 6 - 4*real(a11'*a21) - 2*re(a12'*a22);
	B12 = 4*(a11'+a21') - invAoffdiag*a21' - invAoffdiag'*a11';
	B13 = 4*(a12'+a22') - invAoffdiag*a22' - invAoffdiag'*a12';
	B14 = 4*(a13'+a23') - invAoffdiag*a23' - invAoffdiag'*a13';
	B23 = 2*(a11'+a21') - 2*(a11*a22'+a12'*a21);
	B = [B11  B12  B13 B14
		    B12' B22  B23 B13
			B13' B23' B22 B12
			B14' B13' B12' B11]/(16 - invAoffdiag'*invAoffdiag
	*/
		
	R11PR44 			=	Rn[0].real + Rn[9].real;
	R22PR33 			=	Rn[4].real + Rn[7].real;
	R12PR34				=	_daddsp(_amem8_f2(&Rn[1]), _amem8_f2(&Rn[8]));
	R13PR24				=	_daddsp(_amem8_f2(&Rn[2]), _amem8_f2(&Rn[6]));
	max 			=	0.f;
	max_i			=	-1;
	max_j			=	-1;

	if(firstStageSearchStep > 1)
	{
		tempPtr			=	(__float2_t *) steeringVec;
		steeringVecPtr 	=	(__float2_t *) &scratchPad[3 * (steeringVecSize >> 1) + 2];
		j				=	0;
		for (i = 0; i < steeringVecSize; i+=firstStageSearchStep)
		{
			_amem8_f2(&steeringVecPtr[3 * j + 0]) 	=	_amem8_f2(&tempPtr[3 * i + 0]);
			_amem8_f2(&steeringVecPtr[3 * j + 1]) 	=	_amem8_f2(&tempPtr[3 * i + 1]);
			_amem8_f2(&steeringVecPtr[3 * j + 2]) 	=	_amem8_f2(&tempPtr[3 * i + 2]);
			j++;
		}
		steeringVecSizeTemp 	=	j;
	}
	else
	{
		steeringVecPtr			=	(__float2_t *) steeringVec;
		steeringVecSizeTemp 	=	steeringVecSize;
	}

	for (i = 0; i < steeringVecSizeTemp; i++)
	{
		a11 	=	_amem8_f2(&steeringVecPtr[3*i + 0]);
		a12 	=	_amem8_f2(&steeringVecPtr[3*i + 1]);
		a13 	=	_amem8_f2(&steeringVecPtr[3*i + 2]);
		/* first loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
			a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
			a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
			conja11Ma21		=	_complex_conjugate_mpysp(a11, a21);
			conja12Ma22		=	_complex_conjugate_mpysp(a12, a22);
			conja13Ma23		=	_complex_conjugate_mpysp(a13, a23);
			invAoffdiag     =	_daddsp(conja11Ma21, _ftof2(1.f, 0.f));
			invAoffdiag     =	_daddsp(invAoffdiag, conja12Ma22);
			B22 			=	4.f - _hif2(invAoffdiag);
			invAoffdiag     =	_daddsp(invAoffdiag, conja13Ma23);

				
			/* f2temp += (B22*R22 + B22*R33)/2 */
			//B22 			=	3.f - 2.f * _hif2(conja11Ma21) - _hif2(conja12Ma22);
			B22				=	B22 - _hif2(conja11Ma21);
			metric 			=	B22 * R22PR33;

			/* f2temp = (B11*R11 + B11*R44)/2 */
			B11 			=	4.f - _hif2(invAoffdiag);
			metric			+=	B11 * R11PR44;

			/* f2temp += 2*real(B23'*R23)/2 */
			a11Pa21 		= 	_daddsp(a11, a21);
			Btemp 			=	_dsubsp(a11Pa21, _complex_conjugate_mpysp(a11, a22));
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(a21, a12));
			f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[5]));
			metric 			+=	2.f * _hif2(f2temp1);

			_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
			*tempMetricBuf++ 				= 	metric;
		}

		/* 2rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSizeTemp];
		for (j = i + 1; j < steeringVecSizeTemp; j++)
		{
			a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
			a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
			a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf++;

			/* f2temp += 2*real(B12'*(R12+R34))/2 */
			a11Pa21 		= 	_daddsp(a11, a21);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a21));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a11, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, R12PR34);
			metric			+=  _hif2(f2temp1);

			/* f2temp += 2*real(B13'*R13)/2 */
			a11Pa21 		= 	_daddsp(a12, a22);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a22));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a12, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, R13PR24);
			metric 			+=	_hif2(f2temp1);

			/* f2temp += 2*real(B14'*R14)/2 */
			a11Pa21 		= 	_daddsp(a13, a23);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a23));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a13, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[3]));
			metric 			+=	_hif2(f2temp1);

			f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
			ftemp1 			=	16.f - _hif2(f2temp1) - _lof2(f2temp1);
			ftemp2			=	_rcpsp(ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			metric 			=	metric * ftemp2;
			if ( metric > max )
			{
				max 		=	metric;
				max_i 		=	i;
				max_j 		=	j;
			}

		}
	}

	if (firstStageSearchStep > 1)
	{
		int32_t  iStart, iStop, jStart, jStop;

		iStart			=	(max_i - 2) * firstStageSearchStep;
		if (iStart < 0)
			iStart		=	0;
		iStop			=	(max_i + 2) * firstStageSearchStep + 1;
		if (iStop > steeringVecSize)
			iStop		=	steeringVecSize;
		jStart			=	(max_j - 2) * firstStageSearchStep;
		if (jStart < 0)
			jStart		=	0;
		jStop			=	(max_j + 2) * firstStageSearchStep + 1;
		if (jStop > steeringVecSize)
			jStop		=	steeringVecSize;

		max 			=	0.f;
		max_i			=	-1;
		max_j			=	-1;
		steeringVecPtr			=	(__float2_t *) steeringVec;
		for (i = iStart; i < iStop; i++)
		{
			a11 	=	_amem8_f2(&steeringVecPtr[3*i + 0]);
			a12 	=	_amem8_f2(&steeringVecPtr[3*i + 1]);
			a13 	=	_amem8_f2(&steeringVecPtr[3*i + 2]);
			/* first loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			for (j = jStart; j < jStop; j++)
			{
				a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
				a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
				a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
				conja11Ma21		=	_complex_conjugate_mpysp(a11, a21);
				conja12Ma22		=	_complex_conjugate_mpysp(a12, a22);
				conja13Ma23		=	_complex_conjugate_mpysp(a13, a23);
				invAoffdiag     =	_daddsp(conja11Ma21, _ftof2(1.f, 0.f));
				invAoffdiag     =	_daddsp(invAoffdiag, conja12Ma22);
				B22 			=	4.f - _hif2(invAoffdiag);
				invAoffdiag     =	_daddsp(invAoffdiag, conja13Ma23);

				
				/* f2temp += (B22*R22 + B22*R33)/2 */
				//B22 			=	3.f - 2.f * _hif2(conja11Ma21) - _hif2(conja12Ma22);
				B22				=	B22 - _hif2(conja11Ma21);
				metric 			=	B22 * R22PR33;

				/* f2temp = (B11*R11 + B11*R44)/2 */
				B11 			=	4.f - _hif2(invAoffdiag);
				metric			+=	B11 * R11PR44;

				/* f2temp += 2*real(B23'*R23)/2 */
				a11Pa21 		= 	_daddsp(a11, a21);
				Btemp 			=	_dsubsp(a11Pa21, _complex_conjugate_mpysp(a11, a22));
				Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(a21, a12));
				f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[5]));
				metric 			+=	2.f * _hif2(f2temp1);

				_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
				*tempMetricBuf++ 				= 	metric;
			}

			/* 2rd loop*/
			tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
			tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
			for (j = jStart; j < jStop; j++)
			{
				a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
				a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
				a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
				invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
				metric			=	*tempMetricBuf++;

				/* f2temp += 2*real(B12'*(R12+R34))/2 */
				a11Pa21 		= 	_daddsp(a11, a21);
				Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
				Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a21));
				Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a11, invAoffdiag));
				f2temp1 		=	_complex_mpysp(Btemp, R12PR34);
				metric			+=  _hif2(f2temp1);

				/* f2temp += 2*real(B13'*R13)/2 */
				a11Pa21 		= 	_daddsp(a12, a22);
				Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
				Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a22));
				Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a12, invAoffdiag));
				f2temp1 		=	_complex_mpysp(Btemp, R13PR24);
				metric 			+=	_hif2(f2temp1);

				/* f2temp += 2*real(B14'*R14)/2 */
				a11Pa21 		= 	_daddsp(a13, a23);
				Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
				Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a23));
				Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a13, invAoffdiag));
				f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[3]));
				metric 			+=	_hif2(f2temp1);

				f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
				ftemp1 			=	16.f - _hif2(f2temp1) - _lof2(f2temp1);
				ftemp2			=	_rcpsp(ftemp1);
				ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
				ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
				metric 			=	metric * ftemp2;
				if ( metric > max )
				{
					max 		=	metric;
					max_i 		=	i;
					max_j 		=	j;
				}

			}
		}

	}
	/* find whether there is 1 target or 2 */
	steeringVecPtr			=	(__float2_t *) steeringVec;
	for (i = 0; i < 2; i++)
	{
		if ( i == 0 )
			i2 = max_i;
		else
			i2 = max_j;
		a11 	=	_amem8_f2(&steeringVecPtr[3*i2 + 0]);
		a12 	=	_amem8_f2(&steeringVecPtr[3*i2 + 1]);
		a13 	=	_amem8_f2(&steeringVecPtr[3*i2 + 2]);
		/* first loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		for (j = 0; j < steeringVecSize; j++)
		{
			a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
			a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
			a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
			conja11Ma21		=	_complex_conjugate_mpysp(a11, a21);
			conja12Ma22		=	_complex_conjugate_mpysp(a12, a22);
			conja13Ma23		=	_complex_conjugate_mpysp(a13, a23);
			invAoffdiag     =	_daddsp(conja11Ma21, _ftof2(1.f, 0.f));
			invAoffdiag     =	_daddsp(invAoffdiag, conja12Ma22);
			B22 			=	4.f - _hif2(invAoffdiag);
			invAoffdiag     =	_daddsp(invAoffdiag, conja13Ma23);

				
			/* f2temp += (B22*R22 + B22*R33)/2 */
			//B22 			=	3.f - 2.f * _hif2(conja11Ma21) - _hif2(conja12Ma22);
			B22				=	B22 - _hif2(conja11Ma21);
			metric 			=	B22 * R22PR33;

			/* f2temp = (B11*R11 + B11*R44)/2 */
			B11 			=	4.f - _hif2(invAoffdiag);
			metric			+=	B11 * R11PR44;


			/* f2temp += 2*real(B23'*R23)/2 */
			a11Pa21 		= 	_daddsp(a11, a21);
			Btemp 			=	_dsubsp(a11Pa21, _complex_conjugate_mpysp(a11, a22));
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(a21, a12));
			f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[5]));
			metric 			+=	2.f * _hif2(f2temp1);


			_amem8_f2(tempInvAoffdiagBuf++) = 	invAoffdiag;
			*tempMetricBuf++ 				= 	metric;
		}

		/* 2rd loop*/
		tempInvAoffdiagBuf	=	(__float2_t *) &scratchPad[0];
		tempMetricBuf		=	(float *) &scratchPad[2 * steeringVecSize];
		avg					=	0.f;
		for (j = 0; j < steeringVecSize; j++)
		{
			a21 			=	_amem8_f2(&steeringVecPtr[3*j + 0]);
			a22 			=	_amem8_f2(&steeringVecPtr[3*j + 1]);
			a23 			=	_amem8_f2(&steeringVecPtr[3*j + 2]);
			invAoffdiag     =	_amem8_f2(tempInvAoffdiagBuf++);
			metric			=	*tempMetricBuf;

			/* f2temp += 2*real(B12'*(R12+R34))/2 */
			a11Pa21 		= 	_daddsp(a11, a21);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a21));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a11, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, R12PR34);
			metric			+=  _hif2(f2temp1);

			/* f2temp += 2*real(B13'*R13)/2 */
			a11Pa21 		= 	_daddsp(a12, a22);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a22));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a12, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, R13PR24);
			metric 			+=	_hif2(f2temp1);

			/* f2temp += 2*real(B14'*R14)/2 */
			a11Pa21 		= 	_daddsp(a13, a23);
			Btemp			=	_dmpysp(_ftof2(4.f, 4.f), a11Pa21);
			Btemp 			=	_dsubsp(Btemp, _complex_conjugate_mpysp(invAoffdiag, a23));
			Btemp 			=	_dsubsp(Btemp, _complex_mpysp(a13, invAoffdiag));
			f2temp1 		=	_complex_mpysp(Btemp, _amem8_f2(&Rn[3]));
			metric 			+=	_hif2(f2temp1);

			f2temp1			=	_dmpysp(invAoffdiag, invAoffdiag);
			ftemp1 			=	16.f - _hif2(f2temp1) - _lof2(f2temp1);
			ftemp2			=	_rcpsp(ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			ftemp2			=   ftemp2 * (2.f - ftemp2 * ftemp1);
			metric 			=	metric * ftemp2;
			if (j == i2) metric = 0.f;
			avg				+=	metric;
			*tempMetricBuf++ = 	metric;

		}
		tempMetricBuf	=	(float *) &scratchPad[2 * steeringVecSize];
		ftemp1			=	_rcpsp((float)steeringVecSize);
		ftemp1			=   ftemp1 * (2.f - ftemp1 * (float)steeringVecSize);
		ftemp1			=   ftemp1 * (2.f - ftemp1 * (float)steeringVecSize);
		avg				=	avg * ftemp1;
		var				=	0.f;
		f2temp1			=	_ftof2(var, var);
		for (j = 0; j < steeringVecSize; j++)
		{
			metric		=	tempMetricBuf[j];
			ftemp1		=	metric - avg;
			if (j == i2) ftemp1 = 0.f;
			var			+=	ftemp1 * ftemp1;
		}
		ftemp1			=	avg * avg * (float)steeringVecSize;
		temprcp			=	_rcpsp(ftemp1);	
		temprcp			=	temprcp * (2.f - ftemp1 * temprcp);
		temprcp			=	temprcp * (2.f - ftemp1 * temprcp);
		ftemp2			=	var * temprcp;

		normVar[i] = ftemp2;
	}
	angleEst[0]		=	max_i;
	angleEst[1]		=	max_j;
	i = 2;
	return(i);
}

