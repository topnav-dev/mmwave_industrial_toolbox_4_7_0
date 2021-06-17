
/**
 *  \file   RADARDEMO_aoaEst2DCaponBF_staticHeatMapEst.c
 *
 *   \brief   Static heatmap generation for Capon.
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "RADARDEMO_aoaEst2DCaponBF_priv.h"

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*!
 *   \fn     RADARDEMO_aoaEstimationBFSinglePeak_static
 *
 *   \brief   Use Bartlett beamforming to generate range azimuth heatmap per range bin for static scene.
 *
 *   \param[in]    sigIn
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[out]    heatmap
 *               Output azimuth-elevation heatmap per range bin, length of number of angle bins, arranged in [elev][azim] format.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    peakVal
 *               Output peak value in angle domain, per range bin.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */

void  RADARDEMO_aoaEstimationBFSinglePeak_static(
                            IN  cplxf_t * sigIn,
							IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
                            OUT  float   * heatmap,
                            OUT float   * peakVal
							)
 {
    int32_t 	i, azimSteps, elevSteps, nRxAnt, staticAzimSearchLen, staticElevSearchLen;
    float 		maxPow;
	__float2_t	*	RESTRICT steeringVec;
	__float2_t	*	RESTRICT azimSteeringVecPtr;
	__float2_t	*	RESTRICT elevSteeringVecPtr;
	__float2_t  acc1f2, f2temp;
	int32_t		azimIdx, elevIdx, scratchOffset = 0;
	float		result;
	float		* RESTRICT heatMapPtr;

	/*
     Solve: spectrum = A'*Rn*A = |A'x|.^2;
     where:
             A = [steeringVec(theta)]  is a nRxAnt by numAngles matrix
             Rn is the covariance matrix of the antenna signal
             x is the input signal vector
    */


	azimSteps			=	(int32_t) capon_handle->staticAzimStepDeciFactor;
	elevSteps			=	(int32_t) capon_handle->staticElevStepDeciFactor;
	steeringVec			=	(__float2_t *) &capon_handle->scratchPad[scratchOffset];
	scratchOffset		=	scratchOffset + 2 * capon_handle->nRxAnt;  /*Store the full steering vec per angle bin*/
	nRxAnt				=	(int32_t)capon_handle->nRxAnt;
	maxPow				=	0.f;
	heatMapPtr			=	&heatmap[capon_handle->staticAzimSearchLen * capon_handle->staticElevSearchLen - 1];
	staticAzimSearchLen =	capon_handle->staticAzimSearchLen;
	staticElevSearchLen =	capon_handle->staticElevSearchLen;

#ifdef _TMS320C6X
	_nassert(nRxAnt %4	==	0);
	_nassert(staticAzimSearchLen >= 10 );
	_nassert(staticElevSearchLen >= 6 );
#endif
	if (azimSteps != 1 )
	{
		for (azimIdx = 0; azimIdx < staticAzimSearchLen; azimIdx++ )
		{

			elevSteeringVecPtr						=	(__float2_t	*) &capon_handle->raHeatMap_handle->steeringVecElev[0];
			azimSteeringVecPtr						=	(__float2_t	*) &capon_handle->raHeatMap_handle->steeringVecAzim[azimSteps * azimIdx * nRxAnt];
			#ifdef _TMS320C6X
			#pragma UNROLL(2)
			#endif
			for (i = 0; i < nRxAnt; i++)
			{
				_amem8_f2(&steeringVec[i])          =   _complex_conjugate_mpysp(_amem8_f2(&sigIn[i]), _amem8_f2(&azimSteeringVecPtr[i]));
			}

			acc1f2									=	_ftof2(0.f, 0.f);

			if (nRxAnt == 12)
			{
				for (elevIdx = 0; elevIdx < staticElevSearchLen; elevIdx++ )
				{
					acc1f2							=	_ftof2(0.f, 0.f);
					#ifdef _TMS320C6X
						#pragma UNROLL(12)
					#endif
					for (i = 0; i < 12; i++)
					{
						acc1f2	=	_daddsp(acc1f2, _complex_mpysp(_amem8_f2(&steeringVec[i]), _amem8_f2(&elevSteeringVecPtr[i])));
					}
					f2temp		=	_dmpysp(acc1f2, acc1f2);
					result		=	_hif2(f2temp) + _lof2(f2temp);

					*heatMapPtr--			=	result;
					if (maxPow < result)
						maxPow				=	result;

					elevSteeringVecPtr				+=	elevSteps * 12;
				}
			}
			else
			{
				for (elevIdx = 0; elevIdx < (int32_t) capon_handle->staticElevSearchLen; elevIdx++ )
				{
					for (i = 0; i < nRxAnt; i++)
					{
						acc1f2	=	_daddsp(acc1f2, _complex_mpysp(_amem8_f2(&steeringVec[i]), _amem8_f2(&elevSteeringVecPtr[i])));
					}
					f2temp		=	_dmpysp(acc1f2, acc1f2);
					result		=	_hif2(f2temp) + _lof2(f2temp);

					*heatMapPtr--			=	result;
					if (maxPow < result)
						maxPow				=	result;

					elevSteeringVecPtr				+=	elevSteps * nRxAnt;
				}
			}
		}
	}
	else
	{
		if (nRxAnt == 12)
		{
			for (azimIdx = 0; azimIdx < staticAzimSearchLen; azimIdx++ )
			{
				for (elevIdx = 0; elevIdx < (int32_t) capon_handle->staticElevSearchLen; elevIdx++ )
				{

					steeringVec						=	(__float2_t	*) &capon_handle->raHeatMap_handle->steeringVec[(elevIdx * staticAzimSearchLen + azimIdx) * nRxAnt];
					acc1f2							=	_ftof2(0.f, 0.f);
					#ifdef _TMS320C6X
						#pragma UNROLL(12)
					#endif
					for (i = 0; i < nRxAnt; i++)
					{
						acc1f2						=	_daddsp(acc1f2, _complex_conjugate_mpysp(_amem8_f2(&sigIn[i]), _amem8_f2(&steeringVec[i])));
					}

					f2temp		=	_dmpysp(acc1f2, acc1f2);
					result		=	_hif2(f2temp) + _lof2(f2temp);

					*heatMapPtr--			=	result;
					if (maxPow < result)
						maxPow				=	result;

				}
			}
		}
		else
		{
			for (azimIdx = 0; azimIdx < staticAzimSearchLen; azimIdx++ )
			{
				for (elevIdx = 0; elevIdx < (int32_t) capon_handle->staticElevSearchLen; elevIdx++ )
				{

					steeringVec						=	(__float2_t	*) &capon_handle->raHeatMap_handle->steeringVec[(elevIdx * staticAzimSearchLen + azimIdx) * nRxAnt];
					acc1f2							=	_ftof2(0.f, 0.f);
					for (i = 0; i < nRxAnt; i++)
					{
						acc1f2						=	_daddsp(acc1f2, _complex_conjugate_mpysp(_amem8_f2(&sigIn[i]), _amem8_f2(&steeringVec[i])));
					}

					f2temp		=	_dmpysp(acc1f2, acc1f2);
					result		=	_hif2(f2temp) + _lof2(f2temp);

					*heatMapPtr--			=	result;
					if (maxPow < result)
						maxPow				=	result;

				}
			}
		}
	}
	* peakVal		=	maxPow;
}

