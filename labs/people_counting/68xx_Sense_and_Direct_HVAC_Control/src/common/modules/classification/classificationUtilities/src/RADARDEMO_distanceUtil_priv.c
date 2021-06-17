/*!
 *   \file   RADARDEMO_distanceUtil_priv.c
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

#include "RADARDEMO_distanceUtil_priv.h"
#include <math.h>

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifdef OPTIMIZED4DSP
//! \copydoc RADARDEMO_distanceUtilSqrEuc
float RADARDEMO_distanceUtilSqrEuc(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
							OUT uint16_t * RESTRICT minDistCWIndex, 
							OUT float * RESTRICT sqrEucDist)
 {
	int32_t i, j;
	int16_t index;
	float min, dist;
	float delta;
	
	min 	=	1e10;
	index	=	-1;
	if ( codewordDim == 1 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			delta			=	*sigIn - codebook[i];
			dist			=	delta * delta;
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 2 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			delta			=	sigIn[0] - codebook[2 * i + 0];
			dist			=	delta * delta;
			delta			=	sigIn[1] - codebook[2 * i + 1];
			dist			+=	delta * delta;
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}

	}
	else if ( codewordDim == 3 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			delta			=	sigIn[0] - codebook[3 * i + 0];
			dist			=	delta * delta;
			delta			=	sigIn[1] - codebook[3 * i + 1];
			dist			+=	delta * delta;
			delta			=	sigIn[2] - codebook[3 * i + 2];
			dist			+=	delta * delta;
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 4 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			delta			=	sigIn[0] - codebook[4 * i + 0];
			dist			=	delta * delta;
			delta			=	sigIn[1] - codebook[4 * i + 1];
			dist			+=	delta * delta;
			delta			=	sigIn[2] - codebook[4 * i + 2];
			dist			+=	delta * delta;
			delta			=	sigIn[3] - codebook[4 * i + 3];
			dist			+=	delta * delta;
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 5 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			delta			=	sigIn[0] - codebook[5 * i + 0];
			dist			=	delta * delta;
			delta			=	sigIn[1] - codebook[5 * i + 1];
			dist			+=	delta * delta;
			delta			=	sigIn[2] - codebook[5 * i + 2];
			dist			+=	delta * delta;
			delta			=	sigIn[3] - codebook[5 * i + 3];
			dist			+=	delta * delta;
			delta			=	sigIn[4] - codebook[5 * i + 4];
			dist			+=	delta * delta;
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			dist			=	0.f;
			for ( j = 0; j < codewordDim; j++ )
			{
				dist 		+=	(sigIn[j] - codebook[codewordDim * i + j]) * (sigIn[j] - codebook[codewordDim * i + j]);
			}
			sqrEucDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}	
	*minDistCWIndex		=	index;
    return(min);
}

//! \copydoc RADARDEMO_distanceUtilMahalanobis
float RADARDEMO_distanceUtilMahalanobis(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
                            IN  float *  RESTRICT codebookRCholinv,
							OUT uint16_t * RESTRICT minDistCWIndex, 
							OUT float * RESTRICT mahalanobisDist)
 {
	int32_t i, j, k, rLen;
	int16_t index;
	float min, dist, localAcc;
	float * RESTRICT ptrRinv;
	
	min 	=	1e10;
	index	=	-1;
	rLen    =	(codewordDim * (codewordDim + 1)) >> 1;
	if ( codewordDim == 2 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			ptrRinv			=	&codebookRCholinv[rLen * i];
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			dist			=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			mahalanobisDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 3 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			ptrRinv			=	&codebookRCholinv[rLen * i];
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			dist			=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			mahalanobisDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 4 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			ptrRinv			=	&codebookRCholinv[rLen * i];
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			dist			=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			localAcc		+=	(sigIn[3] - codebook[codewordDim * i + 3]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			mahalanobisDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else if ( codewordDim == 5 )
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			ptrRinv			=	&codebookRCholinv[rLen * i];
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			dist			=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			localAcc		+=	(sigIn[3] - codebook[codewordDim * i + 3]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			localAcc		=	(sigIn[0] - codebook[codewordDim * i + 0]) * (*ptrRinv++);
			localAcc		+=	(sigIn[1] - codebook[codewordDim * i + 1]) * (*ptrRinv++);
			localAcc		+=	(sigIn[2] - codebook[codewordDim * i + 2]) * (*ptrRinv++);
			localAcc		+=	(sigIn[3] - codebook[codewordDim * i + 3]) * (*ptrRinv++);
			localAcc		+=	(sigIn[4] - codebook[codewordDim * i + 4]) * (*ptrRinv++);
			dist			+=	localAcc * localAcc;
			mahalanobisDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}
	else
	{
		for ( i = 0; i < numCodewords; i++ )
		{
			dist			=	0.f;
			ptrRinv			=	&codebookRCholinv[rLen * i];
			for ( j = 0; j < codewordDim; j++ )
			{
				localAcc	=	0.f;
				for ( k = 0; k < (j + 1); k++ )
				{
					localAcc	+=	(sigIn[k] - codebook[codewordDim * i + k]) * (*ptrRinv++) ;
				}
				dist		+=	localAcc * localAcc;
			}
			mahalanobisDist[i]	=	dist;
			if ( dist < min )
			{
				min			=	dist;
				index		=	i;
			}
		}
	}	
	*minDistCWIndex		=	index;
    return(min);
}

//! \copydoc RADARDEMO_distanceFindMin
float RADARDEMO_distanceFindMin(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT distIn,
							OUT uint16_t * RESTRICT minDistIndex)
 {
	int32_t i;
	int16_t index;
	float min, dist;
	
	min 	= 1e10;
	index   = -1;
	for ( i = 0; i < numInputs; i++ )
	{
		dist			=	distIn[i];
		if ( dist < min )
		{
			min			=	dist;
			index		=	i;
		}
	}
	*minDistIndex     =	index;
    return(min);
}

#else
//! \copydoc RADARDEMO_distanceUtilSqrEuc
float RADARDEMO_distanceUtilSqrEuc(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
							OUT uint16_t * RESTRICT minDistCWIndex,
							OUT float * RESTRICT sqrEucDist)
 {
	int32_t i, j;
	int16_t index;
	float min, dist;
	
	min 	=	1e10;
	index	=	-1;
	for ( i = 0; i < numCodewords; i++ )
	{
		dist			=	0.f;
		for ( j = 0; j < codewordDim; j++ )
		{
			dist 		+=	(sigIn[j] - codebook[codewordDim * i + j]) * (sigIn[j] - codebook[codewordDim * i + j]);
		}
		sqrEucDist[i]	=	dist;
		if ( dist < min )
		{
			min			=	dist;
			index		=	i;
		}
	}
	
	*minDistCWIndex     =	index;
    return(min);
}

//! \copydoc RADARDEMO_distanceUtilMahalanobis
float RADARDEMO_distanceUtilMahalanobis(
                            IN  int16_t numCodewords,
							IN  int8_t  codewordDim,
                            IN  float *  RESTRICT sigIn,
                            IN  float *  RESTRICT codebook,
                            IN  float *  RESTRICT codebookRCholinv,
							OUT uint16_t * RESTRICT minDistCWIndex,
							OUT float * RESTRICT mahalanobisDist)
 {
	int32_t i, j, k, rLen;
	int16_t index;
	float min, dist, localAcc;
	float *ptrRinv;
	
	min 	=	1e10;
	index	=	-1;
	rLen    = (codewordDim * (codewordDim + 1)) >> 1;
	for ( i = 0; i < numCodewords; i++ )
	{
		dist			=	0.f;
		ptrRinv			=	&codebookRCholinv[rLen * i];
		for ( j = 0; j < codewordDim; j++ )
		{
			localAcc	=	0.f;
			for ( k = 0; k < (j + 1); k++ )
			{
				localAcc	+=	(sigIn[k] - codebook[codewordDim * i + k]) * (*ptrRinv++) ;
			}
			dist		+=	localAcc * localAcc;
		}
		mahalanobisDist[i]	=	dist;
		if ( dist < min )
		{
			min			=	dist;
			index		=	i;
		}
	}
	
	*minDistCWIndex     =	index;
    return(min);
}

//! \copydoc RADARDEMO_distanceFindMin
float RADARDEMO_distanceFindMin(
                            IN  int16_t numInputs,
                            IN  float *  RESTRICT distIn,
							OUT uint16_t * RESTRICT minDistIndex)
 {
	int32_t i;
	int16_t index;
	float min, dist;
	
	min 	= 1e10;
	index   = -1;
	for ( i = 0; i < numInputs; i++ )
	{
		dist			=	distIn[i];
		if ( dist < min )
		{
			min			=	dist;
			index		=	i;
		}
	}
	*minDistIndex     =	index;
    return(min);
}
#endif
