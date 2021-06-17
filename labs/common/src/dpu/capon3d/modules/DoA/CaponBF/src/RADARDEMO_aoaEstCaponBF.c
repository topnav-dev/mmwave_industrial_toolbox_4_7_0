/*!
 *  \file   RADARDEMO_aoaEstCaponBF.c
 *  \brief   Estimate the angle of arrival using Capon BF.
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

#include <modules/DoA/CaponBF/api/RADARDEMO_aoaEstCaponBF.h>
#include "RADARDEMO_aoaEstCaponBF_priv.h"
//#include <modules/DoA/BF/src/RADARDEMO_aoaEstBF_priv.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif

//! \copydoc RADARDEMO_aoaEstimationBF_create
void    * RADARDEMO_aoaEstCaponBF_create(
                            IN  RADARDEMO_aoaEstCaponBF_config * moduleConfig,
                            OUT RADARDEMO_aoaEstCaponBF_errorCode * errorCode)

{
    uint32_t    i, j, scratchSize;
    double       ftemp1, freal1, fimag1, frealJ, fimagJ;
    RADARDEMO_aoaEstCaponBF_handle * handle;

    *errorCode  =   RADARDEMO_AOACAPONBF_NO_ERROR;

    /* Check antenna spacing, if it's not a uniform linear array, return with NULL */
    if (moduleConfig->antSpacing[0] != 0)
        *errorCode = RADARDEMO_AOACAPONBF_ANTSPACE_NOTSUPPORTED;
    for (i = 1; i < moduleConfig->nRxAnt; i++ )
    {
        if (moduleConfig->antSpacing[i] - moduleConfig->antSpacing[i-1] != 1)
            *errorCode = RADARDEMO_AOACAPONBF_ANTSPACE_NOTSUPPORTED;
    }
    /* Check number of antenna , only supporting up to 8 antenna at this point */
    if ((moduleConfig->nRxAnt != 8) && (moduleConfig->nRxAnt != 4))
        *errorCode = RADARDEMO_AOACAPONBF_NUMANT_NOTSUPPORTED;

    if (*errorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
        return (NULL);

    handle              =   (RADARDEMO_aoaEstCaponBF_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoaEstCaponBF_handle), 1);
    if (handle == NULL)
    {
        *errorCode = RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE;
        return (handle);
    }

    handle->nRxAnt      =   moduleConfig->nRxAnt;
    handle->numInputRangeBins      =   moduleConfig->numInputRangeBins;
   	handle->estAngleRange    =   moduleConfig->estAngleRange;
    handle->estAngleResolution   =   moduleConfig->estAngleResolution;
    handle->maxOutputVar   =   moduleConfig->maxOutputVar;
    handle->gamma       =   moduleConfig->gamma;
	handle->dopplerFFTSize      =   moduleConfig->dopplerFFTSize;
	handle->useCFAR4DopDet      =   moduleConfig->useCFAR4DopDet;

    handle->invRnMatrices =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, handle->numInputRangeBins * handle->nRxAnt * ( 1 + handle->nRxAnt) * sizeof(float), 8);
	
	handle->steeringVecSize =   (uint32_t) ((2.f * handle->estAngleRange) / (handle->estAngleResolution) + 0.5f);

    handle->steeringVec =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->nRxAnt * handle->steeringVecSize * sizeof(cplxf_t), 8);
    if (handle->steeringVec == NULL)
    {
        *errorCode  =   RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }
	
	if (handle->nRxAnt == 4)
		scratchSize  =   82 * sizeof(uint32_t);
	else if (handle->nRxAnt == 8)
		scratchSize  =   360 * sizeof(uint32_t);

	if (scratchSize < (2 * handle->dopplerFFTSize * 2 * sizeof(float)))
		scratchSize = 2 * handle->dopplerFFTSize * 2 * sizeof(float);

	if (handle->useCFAR4DopDet)
		handle->scratchPad  =   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 1, scratchSize, 8);
	else
		handle->scratchPad  =   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, scratchSize, 8);
		
    if (handle->scratchPad == NULL)
    {
        *errorCode  =   RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }
	handle->twiddle			=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * handle->dopplerFFTSize * sizeof(float), 8);
	if (handle->twiddle == NULL)
	{
		*errorCode	=	RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	tw_gen_float(handle->twiddle, handle->dopplerFFTSize);

    // Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    for (i = 0; i < handle->steeringVecSize; i++)
    {
        ftemp1          =   (double) sin((-handle->estAngleRange + (double) i * handle->estAngleResolution) * (double)RADARDEMO_AOAESTBF_PIOVER180);
        freal1          =   (double) cos(-RADARDEMO_AOAESTBF_PI*ftemp1);
        fimag1          =   (double) sin(-RADARDEMO_AOAESTBF_PI*ftemp1);
        frealJ          =   freal1;
        fimagJ          =   fimag1;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].real = (float)frealJ;
        handle->steeringVec[(handle->nRxAnt - 1) * i + 0].imag = (float)fimagJ;
        for (j = 2; j < handle->nRxAnt; j++)
        {
            ftemp1      =   frealJ;
            frealJ      =   frealJ * freal1 - fimagJ * fimag1;
            fimagJ      =   ftemp1 * fimag1 + fimagJ * freal1;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].real = (float)frealJ;
            handle->steeringVec[(handle->nRxAnt - 1) * i + j - 1].imag = (float)fimagJ;
        }
    }


	if (handle->useCFAR4DopDet)
	{
		RADARDEMO_detectionCFAR_config cfarModuleConfig;
		RADARDEMO_detectionCFAR_errorCode cfarErrorCode;


		memset(&cfarModuleConfig, 0, sizeof(RADARDEMO_detectionCFAR_config));
		cfarModuleConfig.cfarType			=	RADARDEMO_DETECTIONCFAR_RA_CASOCFAR;
		cfarModuleConfig.enableSecondPassSearch	=	0;
		cfarModuleConfig.fft1DSize			=	handle->dopplerFFTSize;
		cfarModuleConfig.fft2DSize			=	1;
		cfarModuleConfig.guardSizeRange		=	moduleConfig->dopCfarGuardLen;
		cfarModuleConfig.guardSizeDoppler	=	6;
		cfarModuleConfig.inputType			=	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP;
		cfarModuleConfig.K0					=	moduleConfig->dopCfarThr;
		cfarModuleConfig.searchWinSizeRange	=	16;
		cfarModuleConfig.searchWinSizeDoppler	=	16;
		cfarModuleConfig.maxNumDetObj		=	20;


		handle->dopCFARHandle = (void *) RADARDEMO_detectionCFAR_create(&cfarModuleConfig, &cfarErrorCode);
		if (cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			*errorCode  =   RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE;
		}
		handle->dopCFARInput		=	(RADARDEMO_detectionCFAR_input *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_input), 1);
		
		handle->dopCFARout = (RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_output), 8);
		handle->dopCFARout->rangeInd = (uint16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarModuleConfig.maxNumDetObj * sizeof(uint16_t), 8);
		handle->dopCFARout->dopplerInd = (uint16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, 0 * sizeof(uint16_t), 8);
		handle->dopCFARout->noise = (float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarModuleConfig.maxNumDetObj * sizeof(float), 8);
		handle->dopCFARout->snrEst = (float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarModuleConfig.maxNumDetObj * sizeof(float), 8);
	}

    return((void *)handle);

}

//! \copydoc RADARDEMO_aoaEstimationBF_delete
void    RADARDEMO_aoaEstCaponBF_delete(
                            IN  void * handle)
{
    RADARDEMO_aoaEstCaponBF_handle *aoaEstBFInst;

    aoaEstBFInst    =   (RADARDEMO_aoaEstCaponBF_handle *) handle;

    radarOsal_memFree(aoaEstBFInst->steeringVec, aoaEstBFInst->steeringVecSize * aoaEstBFInst->nRxAnt * sizeof(cplxf_t));
	if (aoaEstBFInst->nRxAnt == 4)
	    radarOsal_memFree(aoaEstBFInst->scratchPad, 82 * sizeof(uint32_t));
	else if (aoaEstBFInst->nRxAnt == 8)
	    radarOsal_memFree(aoaEstBFInst->scratchPad, 360 * sizeof(uint32_t));

    radarOsal_memFree(aoaEstBFInst->invRnMatrices, aoaEstBFInst->numInputRangeBins * aoaEstBFInst->nRxAnt * ( 1 + (aoaEstBFInst->nRxAnt >>1)) * sizeof(float));

    radarOsal_memFree(handle, sizeof(RADARDEMO_aoaEstCaponBF_handle));
}


//! \copydoc RADARDEMO_aoaEstCaponBF_run
RADARDEMO_aoaEstCaponBF_errorCode    RADARDEMO_aoaEstCaponBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEstCaponBF_input * input,
                            OUT RADARDEMO_aoAEstCaponBF_output   * estOutput)

{
    uint32_t     i, rnOffset;
    RADARDEMO_aoaEstCaponBF_handle *aoaEstBFInst;
    cplx16_t     * inputSignal;
    RADARDEMO_aoaEstCaponBF_errorCode errorCode = RADARDEMO_AOACAPONBF_NO_ERROR;

    aoaEstBFInst    =   (RADARDEMO_aoaEstCaponBF_handle *) handle;

    if ( input == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
	else
	{
		inputSignal     =   input->inputAntSamples;
		if ( inputSignal == NULL)
			errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
	}
    if (  estOutput == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
	else
	{
		if ( estOutput->rangeAzimuthHeatMap == NULL)
			errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
	}
    if (  aoaEstBFInst->scratchPad == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
    if ( aoaEstBFInst->steeringVec == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
    if (errorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
        return (errorCode);

	rnOffset	=	(aoaEstBFInst->nRxAnt * (1 + aoaEstBFInst->nRxAnt)) >> 1;
	if(input->processingStepSelector == 0) /* estimate the range-azimuth heatmap*/
	{
		/*Calculate covariance matrix and invert */
		RADARDEMO_aoaEstCaponBF_covInv(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(uint8_t) input->clutterRemovalFlag,
				(float) aoaEstBFInst->gamma,
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t) input->nChirps,
				(int32_t *) &aoaEstBFInst->scratchPad[0],
				(cplx16_t *) input->inputAntSamples,
				(cplxf_t  *) &aoaEstBFInst->invRnMatrices[input->rangeIndx * rnOffset],
				(cplxf_t  *) &estOutput->static_information[input->rangeIndx * aoaEstBFInst->nRxAnt]
			);
		/* Capon beamforming */
		RADARDEMO_aoaEstCaponBF_heatmap(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t)  aoaEstBFInst->steeringVecSize,
				(cplxf_t *) aoaEstBFInst->steeringVec,
				(cplxf_t  *) &aoaEstBFInst->invRnMatrices[input->rangeIndx * rnOffset],
				(float *) estOutput->rangeAzimuthHeatMap
			);
		

	}
	else /*Doppler estimation */
	{
		float * RESTRICT dopplerFFTInput;
		int32_t *  RESTRICT localScratch, scratchOffset;
		float *  RESTRICT dopplerFFTOutput;
		float *  RESTRICT dopplerSpectrum;
		unsigned char *brev = NULL;
		int32_t rad2D;
		float max;
		int32_t index;
		__float2_t f2temp;
		float ftemp;

		scratchOffset		=	0;
		dopplerFFTInput		=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];
		dopplerSpectrum		=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];
		scratchOffset		+=	2 * aoaEstBFInst->dopplerFFTSize;
		localScratch		=	(int32_t *)&aoaEstBFInst->scratchPad[scratchOffset];
		dopplerFFTOutput	=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];

		RADARDEMO_aoaEstCaponBF_dopperEstInput(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t) input->nChirps,
				(cplx16_t *) input->inputAntSamples,
				(cplxf_t *) &aoaEstBFInst->steeringVec[input->azimuthIndx * (aoaEstBFInst->nRxAnt - 1)],
				(cplxf_t  *) &aoaEstBFInst->invRnMatrices[input->rangeIndx * rnOffset],
				(int32_t *) localScratch,
				(float) input->bwDemon,
				(float *) dopplerFFTInput
			);

		for (i = input->nChirps; i < aoaEstBFInst->dopplerFFTSize; i++)
		{
			_amem8_f2(&dopplerFFTInput[2*i]) 	=	_ftof2(0.f, 0.f);
		}
			
		i  = 30 - _norm(aoaEstBFInst->dopplerFFTSize);
		if ((i & 1) == 0)
			rad2D = 4;
		else
			rad2D = 2;

		DSPF_sp_fftSPxSP (
				aoaEstBFInst->dopplerFFTSize,
				dopplerFFTInput,
				(float *)aoaEstBFInst->twiddle, 
				dopplerFFTOutput,
				brev,
				rad2D,
				0,
				aoaEstBFInst->dopplerFFTSize);

		max		= 0.f;
		index	=	0;
		for (i = 0; i < aoaEstBFInst->dopplerFFTSize; i++)
		{
			f2temp	=	_amem8_f2(&dopplerFFTOutput[2 * i]);
			f2temp	=	_dmpysp(f2temp,f2temp);
			ftemp	=	_hif2(f2temp) + _lof2(f2temp);
			dopplerSpectrum[i]	=	ftemp;
			if (ftemp > max)
			{
				max		=	ftemp;
				index	=	i;
			}
		}

		if(aoaEstBFInst->useCFAR4DopDet == 0)
		{
			estOutput->dopplerIdx[0]	=	index;
			estOutput->numDopplerIdx	=	1;
			estOutput->dopplerDetSNR    = 	NULL;
		}
		else
		{
			float *tempInput[1], **dopCFARIn;

			dopCFARIn		=	&tempInput[0];
			tempInput[0]	=	&dopplerSpectrum[0];

			aoaEstBFInst->dopCFARInput->heatmapInput = dopCFARIn;
			aoaEstBFInst->dopCFARInput->azMaxPerRangeBin = (float *)dopCFARIn;
			aoaEstBFInst->dopCFARInput->sidelobeThr = 0;//processInst->sidelobe_dynamic;
			aoaEstBFInst->dopCFARInput->enableSecondPass = 0;
			aoaEstBFInst->dopCFARInput->enable_neighbour_check = 0;

			RADARDEMO_detectionCFAR_run(
                            aoaEstBFInst->dopCFARHandle,
							aoaEstBFInst->dopCFARInput,
							aoaEstBFInst->dopCFARout);
			if (aoaEstBFInst->dopCFARout->numObjDetected > 0)
			{
				for (i = 0; i < aoaEstBFInst->dopCFARout->numObjDetected; i++)
				{
					estOutput->dopplerIdx[i]	=	aoaEstBFInst->dopCFARout->rangeInd[i];
				}
				estOutput->numDopplerIdx	=	aoaEstBFInst->dopCFARout->numObjDetected;
				estOutput->dopplerDetSNR    = 	aoaEstBFInst->dopCFARout->snrEst;
			}
			else
			{
				estOutput->dopplerIdx[0]	=	index;
				estOutput->numDopplerIdx	=	1;
				estOutput->dopplerDetSNR    = 	NULL;
			}
		}
		estOutput->angleEst		=	-aoaEstBFInst->estAngleRange + input->azimuthIndx * aoaEstBFInst->estAngleResolution;
	}
    return (errorCode);
}




//! \copydoc RADARDEMO_aoaEstCaponBF_run
RADARDEMO_aoaEstCaponBF_errorCode    RADARDEMO_aoaEstCaponBF_static_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoAEstCaponBF_input * input,
                            OUT RADARDEMO_aoAEstCaponBF_output   * estOutput,
							IN  float   * scratchPad,
                            OUT float   * peakVal)

{
    //uint32_t     i, rnOffset;
	float	estVar[2];
    int32_t	angleEst[2];
	cplxf_t *sigIn;
	int32_t error_code;
    RADARDEMO_aoaEstCaponBF_handle *aoaEstBFInst;
    RADARDEMO_aoaEstCaponBF_errorCode errorCode = RADARDEMO_AOACAPONBF_NO_ERROR;
    aoaEstBFInst    =   (RADARDEMO_aoaEstCaponBF_handle *) handle;

	sigIn = &(estOutput->static_information[input->rangeIndx * aoaEstBFInst->nRxAnt]);

	error_code = RADARDEMO_aoaEstimationBFSinglePeak_static_use(
                            (int32_t) aoaEstBFInst->nRxAnt,
                            (float)1.0,
                            (float)1.0,
							sigIn,
							aoaEstBFInst->steeringVec,
                            aoaEstBFInst->steeringVecSize,
                            scratchPad,
                            &(peakVal[input->rangeIndx]),
                            estVar,
                            angleEst);	   
	if(error_code==1)
		errorCode = RADARDEMO_AOACAPONBF_NO_ERROR;
	
	return (errorCode);
}

