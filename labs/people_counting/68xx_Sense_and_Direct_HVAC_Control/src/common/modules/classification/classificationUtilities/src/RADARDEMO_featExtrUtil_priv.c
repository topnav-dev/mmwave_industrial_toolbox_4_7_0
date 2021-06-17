/*!
 *   \file   RADARDEMO_featExtrUtil_priv.c
 *
 *   \brief   Utility functions for feature extraction.
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

#include "RADARDEMO_featExtrUtil_priv.h"
#include <math.h>

#ifdef _TMS320C6X
#include "c6x.h"
#endif


//! \copydoc RADARDEMO_featExtrUtilCoMEst
float RADARDEMO_featExtrUtilCoMEst(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT weight)
 {
	float estCoM;
	int32_t i;
	float numAcc, denAcc, ftempIn, ftempW;
	
	numAcc = 0.f;
	denAcc = 0.f;
	
	for ( i = 0; i < numInputs; i++ )
	{
		ftempIn		=	*sigIn++;
		ftempW		=	*weight++;
		numAcc 		+= 	ftempIn * ftempW;
		denAcc		+=  ftempW;
	}
	
#ifdef OPTIMIZED4DSP
	ftempIn =	_rcpsp(denAcc);
	ftempIn =	ftempIn * (2.f - denAcc * ftempIn);
	ftempIn =	ftempIn * (2.f - denAcc * ftempIn);
	estCoM 	=	numAcc * ftempIn;
#else
	estCoM 	=	numAcc/denAcc;
#endif
    return(estCoM);
}

//! \copydoc RADARDEMO_featExtrUtilStatisticsEst
void  RADARDEMO_featExtrUtilStatisticsEst(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT sigIn,
                            IN  float histoBinSize,
                            IN  int16_t numHistoBins,
                            IN  float sigCoM,
                            OUT float   *  RESTRICT min,
                            OUT float   *  RESTRICT max,
                            OUT float   *  RESTRICT mean,
                            OUT float *  RESTRICT std,
                            OUT uint32_t *  RESTRICT histPtr
							)
{
	int32_t i;
	float 	tempMin, tempMax, ftempIn;
	float 	facc;
	
	facc 	=	0.f;
	tempMax	=	-1e10;
	tempMin	=	1e10;
	for ( i = 0; i < numInputs; i++ )
	{
		ftempIn		=	sigIn[i];
		if ( ftempIn > tempMax )
			tempMax	=	ftempIn;
		if ( ftempIn < tempMin )
			tempMin	=	ftempIn;
		facc	+=	ftempIn;
	}
	*min 	= 	tempMin;
	*max 	= 	tempMax;

#ifdef OPTIMIZED4DSP

	ftempIn =	_rcpsp((float)numInputs);
	ftempIn =	ftempIn * (2.f - (float)numInputs * ftempIn);
	ftempIn =	ftempIn * (2.f - (float)numInputs * ftempIn);
	*mean 	=	facc * ftempIn;
#else
	*mean	=	facc/(float)numInputs;
#endif




	if ( histPtr != NULL )
	{
		int32_t  itemp;
		float tempInvBinSize;

#ifdef OPTIMIZED4DSP
		tempInvBinSize	=	_rcpsp(histoBinSize);
		tempInvBinSize	=	tempInvBinSize * (2.f - histoBinSize * tempInvBinSize);
		tempInvBinSize	=	tempInvBinSize * (2.f - histoBinSize * tempInvBinSize);
#else
		tempInvBinSize	=	1.f/histoBinSize;
#endif

		facc 	=	0.f;
		for ( i = 0; i < numInputs; i++ )
		{
			ftempIn		=	sigIn[i];
			facc		+=	(ftempIn - (*mean)) * (ftempIn - (*mean));
			ftempIn		-=	sigCoM;
			ftempIn		=	ftempIn * tempInvBinSize;
			if (ftempIn > 0)
				itemp		=	(int32_t)(ftempIn + 0.5f) + (numHistoBins >> 1);
			else
				itemp		=	(int32_t)(ftempIn - 0.5f) + (numHistoBins >> 1);
			if ( itemp >= numHistoBins )
				itemp = numHistoBins - 1;
			if ( itemp < 0 )
				itemp = 0;
			histPtr[itemp] = histPtr[itemp] + 1;
		}
	}
	else  // no histogram calculation
	{
		facc 	=	0.f;
		for ( i = 0; i < numInputs; i++ )
		{
			ftempIn		=	sigIn[i];
			facc		+=	(ftempIn - *mean) * (ftempIn - *mean);
		}
	}

#ifdef OPTIMIZED4DSP
	ftempIn			=	_rcpsp(facc);
	ftempIn			=	ftempIn * (2.f - facc * ftempIn);
	ftempIn			=	ftempIn * (2.f - facc * ftempIn);
	ftempIn			=	(float)(numInputs - 1) * ftempIn;
	facc			=	_rsqrsp(ftempIn);
    facc			=	facc * (1.5f - (ftempIn*0.5f) * facc *facc); 
    facc			=	facc * (1.5f - (ftempIn*0.5f) * facc *facc); 
	*std			=	facc;
#else
	*std 		=	sqrt(facc/(float)(numInputs - 1));
#endif
	return;
}

//! \copydoc RADARDEMO_featExtrUtil2DStatisticsEst
void  RADARDEMO_featExtrUtil2DStatisticsEst(
                            IN  int8_t  rmCoMinStatsFlag,
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT sigInX,
                            IN  float *  RESTRICT sigInY,
                            IN  float histoBinSize,
                            IN  int16_t numHistoBins,
                            IN  float sigCoMX,
                            IN  float sigCoMY,
                            OUT float   *  RESTRICT min,
                            OUT float   *  RESTRICT max,
                            OUT float   *  RESTRICT mean,
                            OUT float *  RESTRICT std,
                            OUT uint32_t *  RESTRICT histPtr
							)
{
	int32_t i;
	float 	tempMin, tempMax, ftempInX, ftempInY, tempSqr;
	float 	faccX, faccY;
	float   meanX, meanY;
	float   tempCoMX = 0.f, tempCoMY = 0.f;
	
#ifdef OPTIMIZED4DSP
	float ftemp, finvsqrt;
#endif

	faccX 		=	0.f;
	faccY 		=	0.f;
	tempMax	=	-1e10;
	tempMin		=	1e10;
	if (rmCoMinStatsFlag)
	{
		tempCoMX		=	sigCoMX;
		tempCoMY		=	sigCoMY;
	}

	for ( i = 0; i < numInputs; i++ )
	{
		ftempInX		=	sigInX[i];
		ftempInY		=	sigInY[i];
		faccX			+=	ftempInX;
		faccY			+=	ftempInY;
		tempSqr			=	(ftempInX - tempCoMX) * (ftempInX - tempCoMX) + (ftempInY - tempCoMY) * (ftempInY - tempCoMY);
		if ( tempSqr > tempMax )
			tempMax	=	tempSqr;
		if ( tempSqr < tempMin )
			tempMin	=	tempSqr;
	}

#ifdef OPTIMIZED4DSP
	finvsqrt	=	_rsqrsp(tempMin);
    finvsqrt	=	finvsqrt * (1.5f - (tempMin*0.5f) * finvsqrt *finvsqrt); 
    finvsqrt	=	finvsqrt * (1.5f - (tempMin*0.5f) * finvsqrt *finvsqrt); 
	*min 		= 	tempMin * finvsqrt;
	finvsqrt	=	_rsqrsp(tempMax);
    finvsqrt	=	finvsqrt * (1.5f - (tempMax*0.5f) * finvsqrt *finvsqrt); 
    finvsqrt	=	finvsqrt * (1.5f - (tempMax*0.5f) * finvsqrt *finvsqrt); 
	*max 		= 	tempMax * finvsqrt;
	ftempInX	=	_rcpsp((float)numInputs);
	ftempInX	=	ftempInX * (2.f - (float)numInputs * ftempInX);
	ftempInX	=	ftempInX * (2.f - (float)numInputs * ftempInX);
	meanX		=	faccX * ftempInX;
	meanY		=	faccY * ftempInX;
	ftemp		=  meanX * meanX + meanY * meanY;
	finvsqrt	=	_rsqrsp(ftemp);
    finvsqrt	=	finvsqrt * (1.5f - (ftemp*0.5f) * finvsqrt *finvsqrt); 
    finvsqrt	=	finvsqrt * (1.5f - (ftemp*0.5f) * finvsqrt *finvsqrt); 
	*mean 		=	ftemp * finvsqrt;
#else
	*min 		= 	sqrt(tempMin);
	*max 		= 	sqrt(tempMax);
	meanX		=  faccX/(float)numInputs;
	meanY		=  faccY/(float)numInputs;
	*mean 		=	sqrt(meanX * meanX + meanY * meanY);
#endif

	if ( histPtr != NULL )
	{
		int32_t  itemp;
		float tempInvBinSize;

#ifdef OPTIMIZED4DSP
		tempInvBinSize	=	_rcpsp(histoBinSize);
		tempInvBinSize	=	tempInvBinSize * (2.f - histoBinSize * tempInvBinSize);
		tempInvBinSize	=	tempInvBinSize * (2.f - histoBinSize * tempInvBinSize);
#else
		tempInvBinSize	=	1.f/histoBinSize;
#endif

		faccX 	=	0.f;
		for ( i = 0; i < numInputs; i++ )
		{
			ftempInX		=	sigInX[i];
			ftempInY		=	sigInY[i];
			faccX			+=	(ftempInX - meanX) * (ftempInX - meanX) + (ftempInY - meanY) * (ftempInY - meanY);
						
			
			ftempInX		-=	sigCoMX;
			ftempInY		-=	sigCoMY;
			faccY			=	ftempInX * ftempInX + ftempInY * ftempInY;

			#ifdef OPTIMIZED4DSP
			finvsqrt		=	_rsqrsp(faccY);
			finvsqrt		=	finvsqrt * (1.5f - (faccY*0.5f) * finvsqrt *finvsqrt); 
			finvsqrt		=	finvsqrt * (1.5f - (faccY*0.5f) * finvsqrt *finvsqrt); 
			ftempInX		=	faccY * finvsqrt * tempInvBinSize;
			#else
			ftempInX		=	sqrt(faccY) * tempInvBinSize;
			#endif
			if (ftempInX > 0)
				itemp		=	(int32_t)(ftempInX + 0.5f) + (numHistoBins >> 1);
			else
				itemp		=	(int32_t)(ftempInX - 0.5f) + (numHistoBins >> 1);
			if ( itemp >= numHistoBins )
				itemp = numHistoBins - 1;
			if ( itemp < 0 )
				itemp = 0;
			histPtr[itemp] = histPtr[itemp] + 1;
		}
	}
	else  // no histogram calculation
	{
		faccX 	=	0.f;
		for ( i = 0; i < numInputs; i++ )
		{
			ftempInX		=	sigInX[i];
			ftempInY		=	sigInY[i];
			faccX			+=	(ftempInX - meanX) * (ftempInX - meanX) + (ftempInY - meanY) * (ftempInY - meanY);
		}
	}

#ifdef OPTIMIZED4DSP
	ftempInX		=	_rcpsp(faccX);
	ftempInX		=	ftempInX * (2.f - faccX * ftempInX);
	ftempInX		=	ftempInX * (2.f - faccX * ftempInX);
	ftempInX		=	(float)(numInputs - 1) * ftempInX;
	faccX			=	_rsqrsp(ftempInX);
    faccX			=	faccX * (1.5f - (ftempInX*0.5f) * faccX *faccX); 
    faccX			=	faccX * (1.5f - (ftempInX*0.5f) * faccX *faccX); 
	*std			=	faccX;
#else
	*std 		=	sqrt(faccX/(float)(numInputs - 1));
#endif
	return;
}

