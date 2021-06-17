/*!
 *  \file   RADARDEMO_aoaEst2DCaponBF.c
 *  \brief   Estimate the 2D angle of arrival using Capon BF.
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

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifndef _TMS320C6600
#include <common/src/dpu/capon3d_overhead/modules/utilities/radar_c674x.h>
#endif

//! \copydoc RADARDEMO_aoaEstimationBF_create
void    * RADARDEMO_aoaEst2DCaponBF_create(
                            IN  RADARDEMO_aoaEst2DCaponBF_config * moduleConfig,
                            OUT RADARDEMO_aoaEst2DCaponBF_errorCode * errorCode)

{
    int32_t    i, j, scratchSize;
    RADARDEMO_aoaEst2DCaponBF_handle * handle;
	float       currentAngle, tempfRe, tempfIm;
	__float2_t  *steerVecPtr;

    *errorCode  =   RADARDEMO_AOACAPONBF_NO_ERROR;

    /* Check antenna spacing, if it's not a uniform linear array, return with NULL */
    /* Check number of antenna , only supporting up to 8 or 12 antenna at this point */
    if ((moduleConfig->nRxAnt != 8) && (moduleConfig->nRxAnt != 12))
        *errorCode = RADARDEMO_AOACAPONBF_NUMANT_NOTSUPPORTED;
	if (moduleConfig->rangeAngleCfg.detectionMethod > 2)
		*errorCode = RADARDEMO_AOACAPONBF_ESTMETHOD_NOTSUPPORTED;

    if (*errorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
        return (NULL);

    handle              =   (RADARDEMO_aoaEst2DCaponBF_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_aoaEst2DCaponBF_handle), 1);
    if (handle == NULL)
    {
        *errorCode = RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE;
        return (handle);
    }
	handle->nRxAnt									=	moduleConfig->nRxAnt;
	handle->detectionMethod							=	moduleConfig->rangeAngleCfg.detectionMethod;

	// initialize handle for range-angle heatmap estimation
	handle->raHeatMap_handle						=	(RADARDEMO_aoaEst2D_RAHeatMap_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_aoaEst2D_RAHeatMap_handle), 1);
	handle->raHeatMap_handle->numInputRangeBins		=	moduleConfig->numInputRangeBins;
	handle->raHeatMap_handle->gamma					=   moduleConfig->rangeAngleCfg.mvdr_alpha;
	handle->raHeatMap_handle->numChirps				=   moduleConfig->numInputChirps;

	handle->tempInputWOstatic						=	(cplx16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 1, handle->nRxAnt * moduleConfig->numInputChirps * sizeof(cplx16_t), 8);

	if (moduleConfig->rangeAngleCfg.detectionMethod	<= 1)	// 0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
															// 1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
	{
		int32_t		numAzimAnt, maxNumElevRows, elevRow2Proc, numAzimBinPerElevRow, kmax;


		handle->raHeatMap_handle->azimOnly				=	1;

		//parse antenna configuration
		numAzimAnt										=	0;
		for (i = 0; i < (int32_t)moduleConfig->nRxAnt; i++ )
		{
			if (numAzimAnt	<	_abs(moduleConfig->m_ind[i]))
				numAzimAnt	=	_abs(moduleConfig->m_ind[i]);
		}
		numAzimAnt										=	numAzimAnt + 1;
		handle->raHeatMap_handle->nRxAnt				=	numAzimAnt;
		handle->raHeatMap_handle->virtAntInd2Proc		=	(uint8_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, numAzimAnt *sizeof(uint8_t), 1);
		
		maxNumElevRows									=	0;
		for (i = 0; i < (int32_t) moduleConfig->nRxAnt; i++ )
		{
			if (maxNumElevRows	<	_abs(moduleConfig->n_ind[i]))
				maxNumElevRows	=	_abs(moduleConfig->n_ind[i]);
		}

		maxNumElevRows									=	maxNumElevRows + 1;
		elevRow2Proc									=	0;
		kmax											=	0;
		for (i = 0; i < (int32_t) maxNumElevRows; i++ )
		{
			numAzimBinPerElevRow						=	0;
			for (j = 0; j < (int32_t) moduleConfig->nRxAnt; j++ )
			{
				if (_abs(moduleConfig->n_ind[j]) == i)
					numAzimBinPerElevRow++;
			}
			if (numAzimBinPerElevRow > kmax)
			{
				kmax									=	numAzimBinPerElevRow;
				elevRow2Proc							=	i;
			}
		}

		j = 0;
		for (i = 0; i < (int32_t) moduleConfig->nRxAnt; i++ )
		{
			if (_abs(moduleConfig->n_ind[i]) == elevRow2Proc)
				handle->raHeatMap_handle->virtAntInd2Proc[j++]	=	i;
		}

	    handle->raHeatMap_handle->nuInit              =   - (float)sinsp_i(moduleConfig->fovCfg[0] * (float)RADARDEMO_AOAESTBF_PIOVER180);
	    handle->raHeatMap_handle->nuStep              =   (float) divsp_i(-handle->raHeatMap_handle->nuInit * moduleConfig->rangeAngleCfg.searchStep, (moduleConfig->fovCfg[0]));
	    handle->raHeatMap_handle->muInit              =   - (float)sinsp_i(moduleConfig->fovCfg[1] * (float)RADARDEMO_AOAESTBF_PIOVER180);
	    handle->raHeatMap_handle->muStep              =   (float) divsp_i(-handle->raHeatMap_handle->muInit * moduleConfig->angle2DEst.azimElevAngleEstCfg.elevSearchStep, (moduleConfig->fovCfg[1]));

	    handle->raHeatMap_handle->azimSearchLen           =   (uint32_t) floor(divsp_i(2.f * moduleConfig->fovCfg[0], (moduleConfig->rangeAngleCfg.searchStep))) + 1;
	    handle->raHeatMap_handle->elevSearchLen           =   (uint32_t) floor(divsp_i(2.f * moduleConfig->fovCfg[1], (moduleConfig->angle2DEst.azimElevAngleEstCfg.elevSearchStep))) + 1;
	    handle->raHeatMap_handle->steeringVecAzim         =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->nRxAnt * handle->raHeatMap_handle->azimSearchLen *sizeof(cplxf_t), 8);
	    handle->raHeatMap_handle->steeringVecElev         =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->nRxAnt * handle->raHeatMap_handle->elevSearchLen *sizeof(cplxf_t), 8);

	    currentAngle                                    =   handle->raHeatMap_handle->nuInit;
	    steerVecPtr                                     =   (__float2_t *)handle->raHeatMap_handle->steeringVecAzim;
	    for (i = 0; i < (int32_t)handle->raHeatMap_handle->azimSearchLen; i++ )
	    {
	        for (j = 0; j < (int32_t)handle->nRxAnt; j++)
	        {
	            tempfRe                                 =   (float)cossp_i(-RADARDEMO_AOAESTBF_PI * moduleConfig->m_ind[j] * currentAngle) * (float) moduleConfig->phaseRot[j];
	            tempfIm                                 =   (float)sinsp_i(-RADARDEMO_AOAESTBF_PI * moduleConfig->m_ind[j] * currentAngle) * (float) moduleConfig->phaseRot[j];
	            _amem8_f2(steerVecPtr++)                =   _complex_mpysp(_ftof2(tempfRe, tempfIm), _amem8_f2(&moduleConfig->phaseCompVect[j]));
	        }
	        currentAngle                                +=  handle->raHeatMap_handle->nuStep;
	    }

	    currentAngle                                    =   handle->raHeatMap_handle->muInit;
	    steerVecPtr                                     =   (__float2_t *)handle->raHeatMap_handle->steeringVecElev;
	    for (i = 0; i < (int32_t)handle->raHeatMap_handle->elevSearchLen; i++ )
	    {
	        for (j = 0; j < (int32_t)handle->nRxAnt; j++)
	        {
	            tempfRe                                 =   (float)cossp_i(-RADARDEMO_AOAESTBF_PI * moduleConfig->n_ind[j] * currentAngle);
	            tempfIm                                 =   (float)sinsp_i(-RADARDEMO_AOAESTBF_PI * moduleConfig->n_ind[j] * currentAngle);
	            _amem8_f2(steerVecPtr++)                =   _ftof2(tempfRe, tempfIm);
	        }
	        currentAngle                                +=  handle->raHeatMap_handle->muStep;
	    }

	    moduleConfig->numRAangleBin						=	handle->raHeatMap_handle->azimSearchLen;
		
		scratchSize										=   2 * numAzimAnt * numAzimAnt * sizeof(cplxf_t);		// 8 x 8 x 8 bytes
		handle->raHeatMap_handle->scratchPad			=   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, scratchSize, 8);
	}
	else                                                    //2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation.
	{
		float currentElev, currentAzim, fov0, fov1, invFov1, invFov0;
		int32_t k, virtAntIdx;

		handle->raHeatMap_handle->azimOnly				=	0;
		handle->raHeatMap_handle->nRxAnt				=	moduleConfig->nRxAnt;
		handle->raHeatMap_handle->virtAntInd2Proc		=	(uint8_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, moduleConfig->nRxAnt *sizeof(uint8_t), 1);

		for (i = 0; i < (int32_t) moduleConfig->nRxAnt; i++ )
		{
			handle->raHeatMap_handle->virtAntInd2Proc[i] =	i;
		}
		fov0											=	moduleConfig->fovCfg[0] - moduleConfig->rangeAngleCfg.searchStep;
		fov1											=	moduleConfig->fovCfg[1] - moduleConfig->rangeAngleCfg.searchStep;

		invFov0											=	_rcpsp(fov0);
		invFov0											=	invFov0 * (2.f - fov0 * invFov0);
		invFov0											=	invFov0 * (2.f - fov0 * invFov0);
		invFov1											=	_rcpsp(fov1);
		invFov1											=	invFov1 * (2.f - fov1 * invFov1);
		invFov1											=	invFov1 * (2.f - fov1 * invFov1);

        //init nu/mu init/step, and steer vector -- this is range-azimuth-elevation search
        handle->raHeatMap_handle->nuInit                =   - (float)sinsp_i(fov0 * (float)RADARDEMO_AOAESTBF_PIOVER180);
        handle->raHeatMap_handle->nuStep                =   (float) -handle->raHeatMap_handle->nuInit * moduleConfig->rangeAngleCfg.searchStep * invFov0;
        handle->raHeatMap_handle->muInit                =   - (float)sinsp_i(fov1 * (float)RADARDEMO_AOAESTBF_PIOVER180);
        handle->raHeatMap_handle->muStep                =   (float) -handle->raHeatMap_handle->muInit * moduleConfig->rangeAngleCfg.searchStep * invFov1;

        handle->raHeatMap_handle->azimSearchLen         =   (uint32_t) floor(divsp_i(2.f * fov0, (moduleConfig->rangeAngleCfg.searchStep))) + 1;
        handle->raHeatMap_handle->elevSearchLen         =   (uint32_t) floor(divsp_i(2.f * fov1,  (moduleConfig->rangeAngleCfg.searchStep))) + 1;
        handle->raHeatMap_handle->steeringVec           =   (cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, moduleConfig->nRxAnt * handle->raHeatMap_handle->elevSearchLen * handle->raHeatMap_handle->azimSearchLen *sizeof(cplxf_t), 1);
        moduleConfig->numRAangleBin                     =   handle->raHeatMap_handle->azimSearchLen * handle->raHeatMap_handle->elevSearchLen;

        currentElev                                     =   handle->raHeatMap_handle->muInit;
        steerVecPtr                                     =   (__float2_t *)handle->raHeatMap_handle->steeringVec;
        for (i = 0; i < (int32_t)handle->raHeatMap_handle->elevSearchLen; i++ )
        {
            currentAzim                                 =   handle->raHeatMap_handle->nuInit;
            for (k = 0; k < (int32_t)handle->raHeatMap_handle->azimSearchLen; k++ )
            {
                for (j = 0; j < (int32_t)moduleConfig->nRxAnt; j++)
                {
                    virtAntIdx                          =   handle->raHeatMap_handle->virtAntInd2Proc[j];
                    tempfRe                             =   (float)cossp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[virtAntIdx] * currentAzim + moduleConfig->n_ind[virtAntIdx] * currentElev)) * (float) moduleConfig->phaseRot[virtAntIdx];
                    tempfIm                             =   (float)sinsp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[virtAntIdx] * currentAzim + moduleConfig->n_ind[virtAntIdx] * currentElev)) * (float) moduleConfig->phaseRot[virtAntIdx];
                    _amem8_f2(steerVecPtr++)            =   _complex_mpysp(_ftof2(tempfRe, tempfIm), _amem8_f2(&moduleConfig->phaseCompVect[virtAntIdx]));
                }
                currentAzim                             +=  handle->raHeatMap_handle->nuStep;
            }
            currentElev                                 +=  handle->raHeatMap_handle->muStep;
        }
		scratchSize										=   2 * handle->nRxAnt * handle->nRxAnt * sizeof(cplxf_t); // 12 x 12 x 8 bytes
		handle->raHeatMap_handle->scratchPad			=   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, scratchSize, 8);
		moduleConfig->numRAangleBin						=	handle->raHeatMap_handle->azimSearchLen * handle->raHeatMap_handle->elevSearchLen;
	}

	// initialize handle for azimuth-elevation heatmap estimation
	moduleConfig->numAzimBins						=	handle->raHeatMap_handle->azimSearchLen;
	moduleConfig->numElevBins						=	handle->raHeatMap_handle->elevSearchLen;
	handle->aeEstimation_handle						=	(RADARDEMO_aoaEst2D_aeEst_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_aoaEst2D_aeEst_handle), 1);
	handle->aeEstimation_handle->numInputRangeBins	=	moduleConfig->numInputRangeBins;
	handle->aeEstimation_handle->numChirps			=   moduleConfig->numInputChirps;
	handle->aeEstimation_handle->nRxAnt				=	moduleConfig->nRxAnt;
	if (moduleConfig->rangeAngleCfg.detectionMethod	<= 1)	// 0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
															// 1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
	{
		handle->aeEstimation_handle->gamma				=   moduleConfig->angle2DEst.azimElevAngleEstCfg.mvdr_alpha;
		handle->aeEstimation_handle->zoomInFlag			=	0;

		handle->aeEstimation_handle->virtAntInd2Proc	=	(uint8_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(uint8_t), 1);

		for (i = 0; i < (int32_t) moduleConfig->nRxAnt; i++ )
		{
			handle->aeEstimation_handle->virtAntInd2Proc[i] =	i;
		}

		handle->aeEstimation_handle->scratchPadSize     =   (uint32_t)4096;
		handle->aeEstimation_handle->scratchPad			=   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 1, handle->aeEstimation_handle->scratchPadSize, 8);

		handle->aeEstimation_handle->maxNpeak2Search	=	moduleConfig->angle2DEst.azimElevAngleEstCfg.maxNpeak2Search;
		handle->aeEstimation_handle->peakExpSamples		=	moduleConfig->angle2DEst.azimElevAngleEstCfg.peakExpSamples;
		handle->aeEstimation_handle->elevOnly			=	moduleConfig->rangeAngleCfg.detectionMethod;
		handle->aeEstimation_handle->sideLobThr			=	moduleConfig->angle2DEst.azimElevAngleEstCfg.sideLobThr;
		handle->aeEstimation_handle->peakExpRelThr		=	moduleConfig->angle2DEst.azimElevAngleEstCfg.peakExpRelThr;
		handle->aeEstimation_handle->peakExpSNRThr		=	moduleConfig->angle2DEst.azimElevAngleEstCfg.peakExpSNRThr;
		handle->aeEstimation_handle->procRngBinMask		=	(uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, (moduleConfig->numInputRangeBins >> 5) *sizeof(uint32_t), 1);
	}
	else                                                    //2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation.
	{
		float       tempfRe, tempfIm;

		handle->aeEstimation_handle->zoomInFlag			=	1;
		handle->aeEstimation_handle->localMaxCheckFlag	=	moduleConfig->angle2DEst.azimElevZoominCfg.localMaxCheckFlag;

		//init nu/mu init/step, and steer vector -- this is azimuth-elevation zoom in
		handle->aeEstimation_handle->nuInit			=	- handle->raHeatMap_handle->nuStep;
		handle->aeEstimation_handle->nuStep			=	divsp_i(handle->raHeatMap_handle->nuStep, ((float)moduleConfig->angle2DEst.azimElevZoominCfg.zoominFactor));
		handle->aeEstimation_handle->muInit			=	- handle->raHeatMap_handle->muStep;
		handle->aeEstimation_handle->muStep			=	divsp_i(handle->raHeatMap_handle->muStep, ((float)moduleConfig->angle2DEst.azimElevZoominCfg.zoominFactor));

		handle->aeEstimation_handle->azimSearchLen		=	2 * moduleConfig->angle2DEst.azimElevZoominCfg.zoominFactor + 1;
		handle->aeEstimation_handle->elevSearchLen		=	2 * moduleConfig->angle2DEst.azimElevZoominCfg.zoominFactor + 1;
		handle->aeEstimation_handle->virtAntInd2Proc	=	(uint8_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(uint8_t), 1);
		for (i = 0; i < (int32_t) moduleConfig->nRxAnt; i++ )
		{
			handle->aeEstimation_handle->virtAntInd2Proc[i] =	i;
		}

		handle->aeEstimation_handle->steeringVecAzimInit	=	(cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(cplxf_t), 8);
		handle->aeEstimation_handle->steeringVecAzimStep	=	(cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(cplxf_t), 8);
		handle->aeEstimation_handle->steeringVecElevInit	=	(cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(cplxf_t), 8);
		handle->aeEstimation_handle->steeringVecElevStep	=	(cplxf_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, handle->aeEstimation_handle->nRxAnt *sizeof(cplxf_t), 8);

		for (j = 0; j < (int32_t)moduleConfig->nRxAnt; j++)
		{
			tempfRe									=	(float)cossp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[j] * handle->aeEstimation_handle->nuInit));
			tempfIm									=	(float)sinsp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[j] * handle->aeEstimation_handle->nuInit));
			_amem8_f2(&handle->aeEstimation_handle->steeringVecAzimInit[j])	=	_ftof2(tempfRe, tempfIm);

			handle->aeEstimation_handle->steeringVecAzimStep[j].real	=	(float)cossp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[j] * handle->aeEstimation_handle->nuStep));
			handle->aeEstimation_handle->steeringVecAzimStep[j].imag	=	(float)sinsp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->m_ind[j] * handle->aeEstimation_handle->nuStep));

			tempfRe									=	(float)cossp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->n_ind[j] * handle->aeEstimation_handle->muInit));
			tempfIm									=	(float)sinsp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->n_ind[j] * handle->aeEstimation_handle->muInit));
			_amem8_f2(&handle->aeEstimation_handle->steeringVecElevInit[j])	=	_ftof2(tempfRe, tempfIm);

			handle->aeEstimation_handle->steeringVecElevStep[j].real	=	(float)cossp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->n_ind[j] * handle->aeEstimation_handle->muStep));
			handle->aeEstimation_handle->steeringVecElevStep[j].imag	=	(float)sinsp_i(-RADARDEMO_AOAESTBF_PI * (moduleConfig->n_ind[j] * handle->aeEstimation_handle->muStep));
		}
		scratchSize  =   (handle->aeEstimation_handle->azimSearchLen * handle->aeEstimation_handle->elevSearchLen) + 3 * handle->aeEstimation_handle->nRxAnt * 2; // 12 x 12 x 8 bytes
		handle->aeEstimation_handle->scratchPad			=   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 1, scratchSize, 8);

		handle->aeEstimation_handle->zoominFactor		=	moduleConfig->angle2DEst.azimElevZoominCfg.zoominFactor;
		handle->aeEstimation_handle->zoominNn8bors		=	moduleConfig->angle2DEst.azimElevZoominCfg.zoominNn8bors;

		handle->aeEstimation_handle->peakExpSamples	=	moduleConfig->angle2DEst.azimElevZoominCfg.peakExpSamples;
		handle->aeEstimation_handle->peakExpRelThr		=	moduleConfig->angle2DEst.azimElevZoominCfg.peakExpRelThr;
		handle->aeEstimation_handle->peakExpSNRThr		=	moduleConfig->angle2DEst.azimElevZoominCfg.peakExpSNRThr;

	}

	//static processing configurations
	handle->staticProcEnabled						=	moduleConfig->staticEstCfg.staticProcEnabled;
	handle->staticAzimStepDeciFactor				=	moduleConfig->staticEstCfg.staticAzimStepDeciFactor;
	handle->staticElevStepDeciFactor				=	moduleConfig->staticEstCfg.staticElevStepDeciFactor;
	handle->staticAzimSearchLen						=	(uint16_t) ceil(divsp_i((float) handle->raHeatMap_handle->azimSearchLen, (float)moduleConfig->staticEstCfg.staticAzimStepDeciFactor));
	handle->staticElevSearchLen						=	(uint16_t) ceil(divsp_i((float) handle->raHeatMap_handle->elevSearchLen, (float)moduleConfig->staticEstCfg.staticElevStepDeciFactor));

	//Doppler estimation configurations
	handle->dopplerFFTSize      =   moduleConfig->dopperFFTSize;
	handle->useCFAR4DopDet      =   moduleConfig->rangeAngleCfg.dopplerEstMethod;

	scratchSize					= 2 * handle->dopplerFFTSize * 2 * sizeof(float);
	handle->scratchPad			=   (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 1, scratchSize, 8);
		
    if (handle->scratchPad == NULL)
    {
        *errorCode  =   RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM;
        return (handle);
    }
	handle->dopTwiddle			=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, 2 * handle->dopplerFFTSize * sizeof(float), 8);
	if (handle->dopTwiddle == NULL)
	{
		*errorCode	=	RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM;
		return (handle);
	}
	tw_gen_float(handle->dopTwiddle, handle->dopplerFFTSize);

	if (handle->useCFAR4DopDet)
	{
		RADARDEMO_detectionCFAR_config cfarModuleConfig;
		RADARDEMO_detectionCFAR_errorCode cfarErrorCode;


		memset(&cfarModuleConfig, 0, sizeof(RADARDEMO_detectionCFAR_config));
		cfarModuleConfig.cfarType			=	RADARDEMO_DETECTIONCFAR_RA_CASOCFAR;
		cfarModuleConfig.enableSecondPassSearch	=	0;
		cfarModuleConfig.fft1DSize			=	handle->dopplerFFTSize;
		cfarModuleConfig.fft2DSize			=	1;
		cfarModuleConfig.guardSizeRange		=	(uint8_t)moduleConfig->dopCfarCfg.guardWinSize;
		cfarModuleConfig.guardSizeDoppler	=	6;
		cfarModuleConfig.inputType			=	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP;
		cfarModuleConfig.K0					=	moduleConfig->dopCfarCfg.thre;
		cfarModuleConfig.searchWinSizeRange	=	(uint8_t)moduleConfig->dopCfarCfg.refWinSize;
		cfarModuleConfig.searchWinSizeDoppler	=	16;
		cfarModuleConfig.maxNumDetObj		=	MAX_DOPCFAR_DET;


		handle->dopCFARHandle = (void *) RADARDEMO_detectionCFAR_create(&cfarModuleConfig, &cfarErrorCode);
		if (cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			*errorCode  =   RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE;
		}
		handle->dopCFARInput			=	(RADARDEMO_detectionCFAR_input *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_detectionCFAR_input), 1);
		
		handle->dopCFARout				=	(RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_detectionCFAR_output), 8);
		handle->dopCFARout->rangeInd	=	(uint16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, MAX_DOPCFAR_DET * sizeof(uint16_t), 8);
		handle->dopCFARout->dopplerInd	=	NULL;
		handle->dopCFARout->noise		=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, MAX_DOPCFAR_DET * sizeof(float), 8);
		handle->dopCFARout->snrEst		=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL1, 0, MAX_DOPCFAR_DET * sizeof(float), 8);
	}

    return((void *)handle);

}

//! \copydoc RADARDEMO_aoaEstimationBF_delete
void    RADARDEMO_aoaEst2DCaponBF_delete(
                            IN  void * handle)
{
    RADARDEMO_aoaEst2DCaponBF_handle *aoaEstBFInst;
	int32_t scratchSize;

    aoaEstBFInst    =   (RADARDEMO_aoaEst2DCaponBF_handle *) handle;

	if (aoaEstBFInst->useCFAR4DopDet)
	{
		radarOsal_memFree(aoaEstBFInst->dopCFARout->rangeInd, MAX_DOPCFAR_DET * sizeof(uint16_t));
		radarOsal_memFree(aoaEstBFInst->dopCFARout->noise, MAX_DOPCFAR_DET * sizeof(float));
		radarOsal_memFree(aoaEstBFInst->dopCFARout->snrEst, MAX_DOPCFAR_DET * sizeof(float));
		radarOsal_memFree(aoaEstBFInst->dopCFARInput, sizeof(RADARDEMO_detectionCFAR_input));
		radarOsal_memFree(aoaEstBFInst->dopCFARout, sizeof(RADARDEMO_detectionCFAR_output));
		RADARDEMO_detectionCFAR_delete(aoaEstBFInst->dopCFARHandle);
	}
	radarOsal_memFree(aoaEstBFInst->dopTwiddle, 2 * aoaEstBFInst->dopplerFFTSize * sizeof(float));
	radarOsal_memFree(aoaEstBFInst->scratchPad, 2 * aoaEstBFInst->dopplerFFTSize * 2 * sizeof(float));


	if (aoaEstBFInst->detectionMethod	<= 1)						// 0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
															// 1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
	{
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->virtAntInd2Proc, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(uint8_t));
		scratchSize  =   (aoaEstBFInst->aeEstimation_handle->nRxAnt * aoaEstBFInst->aeEstimation_handle->nRxAnt) * 2 * sizeof(uint32_t) + 3 * aoaEstBFInst->aeEstimation_handle->nRxAnt * 2 * sizeof(uint32_t); // 12 x 12 x 8 bytes
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->scratchPad, scratchSize);
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->procRngBinMask, (aoaEstBFInst->aeEstimation_handle->numInputRangeBins >> 5) *sizeof(uint32_t));
	}
	else                                                    //2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation.
	{
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->virtAntInd2Proc, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(uint8_t));
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->steeringVecAzimInit, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(cplxf_t));
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->steeringVecAzimStep, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(cplxf_t));
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->steeringVecElevInit, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(cplxf_t));
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->steeringVecElevStep, aoaEstBFInst->aeEstimation_handle->nRxAnt *sizeof(cplxf_t));
		scratchSize  =   (aoaEstBFInst->aeEstimation_handle->nRxAnt * aoaEstBFInst->aeEstimation_handle->nRxAnt) * 2 * sizeof(uint32_t) + 3 * aoaEstBFInst->aeEstimation_handle->nRxAnt * 2 * sizeof(uint32_t); // 12 x 12 x 8 bytes
		radarOsal_memFree(aoaEstBFInst->aeEstimation_handle->scratchPad, scratchSize);
	}
	radarOsal_memFree(aoaEstBFInst->aeEstimation_handle, sizeof(RADARDEMO_aoaEst2D_aeEst_handle));

	// initialize handle for range-angle heatmap estimation
	radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->steeringVecAzim, aoaEstBFInst->nRxAnt * aoaEstBFInst->raHeatMap_handle->azimSearchLen *sizeof(cplxf_t));
	radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->steeringVecElev, aoaEstBFInst->nRxAnt * aoaEstBFInst->raHeatMap_handle->elevSearchLen *sizeof(cplxf_t));

	if (aoaEstBFInst->detectionMethod	<= 1)				// 0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
															// 1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
	{
		radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->virtAntInd2Proc, aoaEstBFInst->raHeatMap_handle->nRxAnt *sizeof(uint8_t));
		
		scratchSize  =   aoaEstBFInst->raHeatMap_handle->nRxAnt * aoaEstBFInst->raHeatMap_handle->nRxAnt * 2 * sizeof(uint32_t);		// 8 x 8 x 8 bytes
		radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->scratchPad, scratchSize);
	}
	else                                                    //2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation.
	{
		radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->virtAntInd2Proc, aoaEstBFInst->raHeatMap_handle->nRxAnt *sizeof(uint8_t));

		scratchSize  =   aoaEstBFInst->raHeatMap_handle->nRxAnt * aoaEstBFInst->raHeatMap_handle->nRxAnt * 2 * sizeof(uint32_t);		// 8 x 8 x 8 bytes
		radarOsal_memFree(aoaEstBFInst->raHeatMap_handle->scratchPad, scratchSize);
	}
    radarOsal_memFree(aoaEstBFInst->raHeatMap_handle, sizeof(RADARDEMO_aoaEst2D_RAHeatMap_handle));


    radarOsal_memFree(handle, sizeof(RADARDEMO_aoaEst2DCaponBF_handle));
}


//! \copydoc RADARDEMO_aoaEst2DCaponBF_run
RADARDEMO_aoaEst2DCaponBF_errorCode    RADARDEMO_aoaEst2DCaponBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoaEst2DCaponBF_input * input,
                            OUT RADARDEMO_aoaEst2DCaponBF_output   * estOutput)

{
    uint32_t     i, rnOffset;
    RADARDEMO_aoaEst2DCaponBF_handle *aoaEstBFInst;
    RADARDEMO_aoaEst2DCaponBF_errorCode errorCode = RADARDEMO_AOACAPONBF_NO_ERROR;
#ifdef CAPON2DMODULEDEBUG
    int32_t     cycleStart;
#endif

    aoaEstBFInst    =   (RADARDEMO_aoaEst2DCaponBF_handle *) handle;

    if ( input == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;

	if (  estOutput == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;

	if (  aoaEstBFInst->scratchPad == NULL)
        errorCode   =   RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT;
    if (errorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
        return (errorCode);

	if (input->processingStepSelector == 0) /* estimate the range-angle heatmap, called per range bin*/
	{
		/*clutter removal, for all the antennas need to be processed for the module*/
		if (input->clutterRemovalFlag)
		{
#ifdef CAPON2DMODULEDEBUG
		    cycleStart = TSCL;
#endif
			RADARDEMO_aoaEst2DCaponBF_clutterRemoval(
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t) input->nChirps,
				(cplx16_t *) input->inputRangeProcOutSamples,
				(cplx16_t *) aoaEstBFInst->tempInputWOstatic,
				(cplxf_t  *) &estOutput->static_information[input->rangeIndx * aoaEstBFInst->nRxAnt]
			);
			memcpy(input->inputRangeProcOutSamples, aoaEstBFInst->tempInputWOstatic, aoaEstBFInst->nRxAnt * input->nChirps * sizeof(cplx16_t));
#ifdef CAPON2DMODULEDEBUG
			estOutput->cyclesLog->crCycles[input->rangeIndx] = TSCL - cycleStart;
#endif
		}
		
		/*Calculate covariance matrix and invert */
#ifdef CAPON2DMODULEDEBUG
        cycleStart = TSCL;
#endif
		rnOffset	=	(aoaEstBFInst->raHeatMap_handle->nRxAnt * (1 + aoaEstBFInst->raHeatMap_handle->nRxAnt)) >> 1;
		RADARDEMO_aoaEst2DCaponBF_covInv(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(float) aoaEstBFInst->raHeatMap_handle->gamma,
				(int32_t) aoaEstBFInst->raHeatMap_handle->nRxAnt,
				(int32_t) input->nChirps,
				(int32_t *) &aoaEstBFInst->raHeatMap_handle->scratchPad[0],
				(uint8_t *) aoaEstBFInst->raHeatMap_handle->virtAntInd2Proc,
				(cplx16_t *) aoaEstBFInst->tempInputWOstatic,
				(cplxf_t  *) &estOutput->invRnMatrices[input->rangeIndx * rnOffset]
			);
#ifdef CAPON2DMODULEDEBUG
       estOutput->cyclesLog->RArnCycles[input->rangeIndx] = TSCL - cycleStart;
#endif

		/* Capon beamforming */
#ifdef CAPON2DMODULEDEBUG
        cycleStart = TSCL;
#endif
		if ( aoaEstBFInst->raHeatMap_handle->azimOnly == 0)
			RADARDEMO_aoaEst2DCaponBF_raHeatmap(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(int32_t) aoaEstBFInst->raHeatMap_handle->nRxAnt,
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t)  aoaEstBFInst->raHeatMap_handle->azimSearchLen,
				(int32_t)  aoaEstBFInst->raHeatMap_handle->elevSearchLen,
				(cplxf_t *) aoaEstBFInst->raHeatMap_handle->steeringVec,
				NULL,
				(uint8_t *) aoaEstBFInst->raHeatMap_handle->virtAntInd2Proc,
				(int32_t *) &aoaEstBFInst->raHeatMap_handle->scratchPad[0],
				(cplxf_t  *) &estOutput->invRnMatrices[input->rangeIndx * rnOffset],
				(float *) &estOutput->malValPerRngBin[input->rangeIndx], 
				(float *) estOutput->rangeAzimuthHeatMap
			);
		else 
		{
			RADARDEMO_aoaEst2DCaponBF_raHeatmap(
				(uint8_t) (input->fallBackToConvBFFlag ^ 1),
				(int32_t) aoaEstBFInst->raHeatMap_handle->nRxAnt,
				(int32_t) aoaEstBFInst->nRxAnt,
				(int32_t)  aoaEstBFInst->raHeatMap_handle->azimSearchLen,
				(int32_t)  1,
				(cplxf_t *) aoaEstBFInst->raHeatMap_handle->steeringVecAzim,
				NULL,
				(uint8_t *) aoaEstBFInst->raHeatMap_handle->virtAntInd2Proc,
				(int32_t *) &aoaEstBFInst->raHeatMap_handle->scratchPad[0],
				(cplxf_t  *) &estOutput->invRnMatrices[input->rangeIndx * rnOffset],
			        (float *) &estOutput->malValPerRngBin[input->rangeIndx], 
				(float *) estOutput->rangeAzimuthHeatMap
				);

			//prepare buffers for next step operation
			if (input->rangeIndx == (aoaEstBFInst->raHeatMap_handle->numInputRangeBins - 1))
				memset(aoaEstBFInst->aeEstimation_handle->procRngBinMask, 0, (aoaEstBFInst->raHeatMap_handle->numInputRangeBins >> 5) *sizeof(uint32_t));
		}
#ifdef CAPON2DMODULEDEBUG
       estOutput->cyclesLog->RAHeatmapCycles[input->rangeIndx] = TSCL - cycleStart;
#endif
		

	} // end of if (input->processingStepSelector == 0)
	else if (input->processingStepSelector == 1) /* estimate 2D angle -- azimuth, elevation and Doppler estimation, called per detected point*/
	{
		int32_t angleDetIdx, dopplerOutCnt;
		float * RESTRICT dopplerFFTInput;
		int32_t scratchOffset;
		float *  RESTRICT dopplerFFTOutput;
		float *  RESTRICT dopplerSpectrum;
		unsigned char *brev = NULL;
		int32_t rad2D;
		float max;
		int32_t index;
		__float2_t f2temp;
		float ftemp;

		if (aoaEstBFInst->aeEstimation_handle->zoomInFlag == 0)
		{
			uint32_t rngMaskIdx, rngMaskOffset, rngMask, azimuthIndx;

			rngMaskIdx		=	(input->rangeIndx >> 5);
			rngMaskOffset	=	(input->rangeIndx & 0x1F);
			rngMask			=	1 << rngMaskOffset;

			azimuthIndx		=	input->angleIndx;
			rnOffset		=	(aoaEstBFInst->aeEstimation_handle->nRxAnt * (1 + aoaEstBFInst->aeEstimation_handle->nRxAnt)) >> 1;
			if ((aoaEstBFInst->aeEstimation_handle->procRngBinMask[rngMaskIdx] & rngMask) == 0)
			{
				// RnInv not available for the range bin, prepare
#ifdef CAPON2DMODULEDEBUG
			     cycleStart = TSCL;
#endif
				aoaEstBFInst->aeEstimation_handle->procRngBinMask[rngMaskIdx]	=	aoaEstBFInst->aeEstimation_handle->procRngBinMask[rngMaskIdx] | rngMask;
				RADARDEMO_aoaEst2DCaponBF_covInv(
						(uint8_t) (input->fallBackToConvBFFlag ^ 1),
						(float) aoaEstBFInst->aeEstimation_handle->gamma,
						(int32_t) aoaEstBFInst->aeEstimation_handle->nRxAnt,
						(int32_t) input->nChirps,
						(int32_t *) &aoaEstBFInst->aeEstimation_handle->scratchPad[0],
						(uint8_t *) aoaEstBFInst->aeEstimation_handle->virtAntInd2Proc,
						(cplx16_t *) input->inputRangeProcOutSamples,
						(cplxf_t  *) &estOutput->invRnMatrices[input->rangeIndx * rnOffset]
					);
#ifdef CAPON2DMODULEDEBUG
				estOutput->cyclesLog->AErnCycles[estOutput->cyclesLog->uniqueRngCnt++] = TSCL - cycleStart;
#endif

				if (aoaEstBFInst->aeEstimation_handle->elevOnly == 0)
					estOutput->numAngleEst = RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim(
						(uint8_t) (input->fallBackToConvBFFlag ^ 1),
						azimuthIndx,
						aoaEstBFInst,
						&estOutput->invRnMatrices[input->rangeIndx * rnOffset],
						estOutput->azimEst, 
						estOutput->elevEst, 
						estOutput->peakPow, 
						estOutput->bwFilter, 
						estOutput->rangeAzimuthHeatMap
					);

			}
#ifdef CAPON2DMODULEDEBUG
             cycleStart = TSCL;
#endif
			if (aoaEstBFInst->aeEstimation_handle->elevOnly == 1)
				estOutput->numAngleEst = RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly(
					(uint8_t) (input->fallBackToConvBFFlag ^ 1),
					azimuthIndx,
					aoaEstBFInst,
					&estOutput->invRnMatrices[input->rangeIndx * rnOffset],
					estOutput->azimEst, 
					estOutput->elevEst, 
					estOutput->peakPow, 
					estOutput->bwFilter, 
					estOutput->rangeAzimuthHeatMap
				);
#ifdef CAPON2DMODULEDEBUG
			if (estOutput->cyclesLog->raDetCnt < MAX_NPNTS )
				estOutput->cyclesLog->AEEstCycles[estOutput->cyclesLog->raDetCnt] = TSCL - cycleStart;
			estOutput->cyclesLog->raDetCnt++;
#endif
		}// end of if (aoaEstBFInst->aeEstimation_handle->zoomInFlag == 0), det method 0 or 1, 2D angle estimation
		else //zoom in
		{
			int32_t		azimuthIndx, elevationIndx, angleIdx, skipEst;
			float       peakPow, sidePow;

#ifdef CAPON2DMODULEDEBUG
			cycleStart = TSCL;
#endif
			angleIdx			=	input->angleIndx;
			for ( elevationIndx = 0; (elevationIndx < (int32_t)aoaEstBFInst->raHeatMap_handle->elevSearchLen) && (angleIdx >= 0); elevationIndx++)
			{
			    angleIdx -= aoaEstBFInst->raHeatMap_handle->azimSearchLen;
			}
			elevationIndx		=	elevationIndx - 1;
			azimuthIndx			=	input->angleIndx - elevationIndx * aoaEstBFInst->raHeatMap_handle->azimSearchLen;
			rnOffset			=	(aoaEstBFInst->aeEstimation_handle->nRxAnt * (1 + aoaEstBFInst->aeEstimation_handle->nRxAnt)) >> 1;

			// check local max
			skipEst			=	0;
			if (aoaEstBFInst->aeEstimation_handle->localMaxCheckFlag == 1) // check elevation domain only
			{
				//estOutput->rangeAzimuthHeatMap has the transposed heatmap, per angle bin all range samples. 
				angleIdx		=	input->angleIndx;
				peakPow			=	estOutput->rangeAzimuthHeatMap[angleIdx * aoaEstBFInst->aeEstimation_handle->numInputRangeBins + input->rangeIndx];

				//left elev
				angleIdx		=	elevationIndx - 1;
				if (angleIdx < 0)
					skipEst	=	1;
				else
				{
					angleIdx	=	angleIdx * aoaEstBFInst->raHeatMap_handle->azimSearchLen + azimuthIndx;
					sidePow     =	estOutput->rangeAzimuthHeatMap[angleIdx * aoaEstBFInst->aeEstimation_handle->numInputRangeBins + input->rangeIndx];
					if (sidePow >	peakPow)
						skipEst	=	1;
				}
				angleIdx		=	elevationIndx + 1;
				if (angleIdx >= aoaEstBFInst->raHeatMap_handle->elevSearchLen)
					skipEst	=	1;
				else
				{
					angleIdx	=	angleIdx * aoaEstBFInst->raHeatMap_handle->azimSearchLen + azimuthIndx;
					sidePow     =	estOutput->rangeAzimuthHeatMap[angleIdx * aoaEstBFInst->aeEstimation_handle->numInputRangeBins + input->rangeIndx];
					if (sidePow >	peakPow)
						skipEst	=	1;
				}
			}
			else if (aoaEstBFInst->aeEstimation_handle->localMaxCheckFlag == 2) // check both domains
			{
				int32_t		leftAzimIndx, rightAzimIndx, leftElevIndx, rightElevIndx, i, j;

				//estOutput->rangeAzimuthHeatMap has the transposed heatmap, per angle bin all range samples. 
				angleIdx		=	input->angleIndx;
				peakPow			=	estOutput->rangeAzimuthHeatMap[angleIdx * aoaEstBFInst->aeEstimation_handle->numInputRangeBins + input->rangeIndx];

				//left elev
				leftAzimIndx	=	azimuthIndx - 1;
				if (leftAzimIndx < 0)
					leftAzimIndx	=	0;
				leftElevIndx	=	elevationIndx - 1;
				if (leftElevIndx < 0)
					leftElevIndx	=	0;

				rightElevIndx	=	elevationIndx + 1;
				if (rightElevIndx >= aoaEstBFInst->raHeatMap_handle->elevSearchLen)
					rightElevIndx	=	aoaEstBFInst->raHeatMap_handle->elevSearchLen - 1;
				rightAzimIndx	=	azimuthIndx + 1;
				if (rightAzimIndx >= aoaEstBFInst->raHeatMap_handle->elevSearchLen)
					rightAzimIndx	=	aoaEstBFInst->raHeatMap_handle->azimSearchLen - 1;

				for (i = leftAzimIndx; i <= rightAzimIndx; i++)
				{
					for (j = leftElevIndx; j <= rightElevIndx; j++)
					{
						angleIdx	=	j * aoaEstBFInst->raHeatMap_handle->azimSearchLen + i;
						sidePow     =	estOutput->rangeAzimuthHeatMap[angleIdx * aoaEstBFInst->aeEstimation_handle->numInputRangeBins + input->rangeIndx];
						if ((sidePow >	peakPow) 
							//&&  (elevationIndx > 1) 
							//&&	(elevationIndx < aoaEstBFInst->raHeatMap_handle->elevSearchLen - 2) 
							//&&	(azimuthIndx > 1)
							//&&	(azimuthIndx < aoaEstBFInst->raHeatMap_handle->elevSearchLen - 2)
							)
							skipEst	=	1;
					}

				}

			}

			if ( skipEst == 0 )
				estOutput->numAngleEst = RADARDEMO_aoaEst2DCaponBF_aeEstZoomin(
					(uint8_t) (input->fallBackToConvBFFlag ^ 1),
					azimuthIndx,
					elevationIndx,
					input->noise,
					aoaEstBFInst->aeEstimation_handle,
					aoaEstBFInst,
					&estOutput->invRnMatrices[input->rangeIndx * rnOffset],
					estOutput->azimEst, 
					estOutput->elevEst, 
					estOutput->peakPow, 
					estOutput->bwFilter
				);
			else
				estOutput->numAngleEst	=	0;
#ifdef CAPON2DMODULEDEBUG
			estOutput->cyclesLog->AEEstCycles[estOutput->cyclesLog->raDetCnt++] = TSCL - cycleStart;
#endif

		}//end of if (aoaEstBFInst->aeEstimation_handle->zoomInFlag == 1), det method 2, 3D detection with zoom-in.

		//Doppler estimation
		dopplerOutCnt			=	0;
		for (angleDetIdx = 0; angleDetIdx < estOutput->numAngleEst; angleDetIdx++ )
		{
#ifdef CAPON2DMODULEDEBUG
            cycleStart = TSCL;
#endif
			scratchOffset		=	0;
			dopplerFFTInput		=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];
			dopplerSpectrum		=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];
			scratchOffset		+=	2 * aoaEstBFInst->dopplerFFTSize;
			dopplerFFTOutput	=	(float *)&aoaEstBFInst->scratchPad[scratchOffset];

			RADARDEMO_aoaEst2DCaponBF_dopperEstInput(
					(int32_t) aoaEstBFInst->aeEstimation_handle->nRxAnt,
					(int32_t) input->nChirps,
					(cplx16_t *) input->inputRangeProcOutSamples,
					(cplxf_t *) &estOutput->bwFilter[angleDetIdx * aoaEstBFInst->aeEstimation_handle->nRxAnt],
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
					(float *)aoaEstBFInst->dopTwiddle, 
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
				estOutput->dopplerIdx[dopplerOutCnt++]	=	index;
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
				aoaEstBFInst->dopCFARInput->sidelobeThr = 0;
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
						estOutput->dopplerIdx[dopplerOutCnt++]	=	aoaEstBFInst->dopCFARout->rangeInd[i];
					}
					estOutput->numDopplerIdx	=	(uint8_t)aoaEstBFInst->dopCFARout->numObjDetected;
					estOutput->dopplerDetSNR    = 	aoaEstBFInst->dopCFARout->snrEst;
				}
				else
				{
					estOutput->dopplerIdx[dopplerOutCnt++]	=	index;
					estOutput->numDopplerIdx	=	1;
					estOutput->dopplerDetSNR    = 	NULL;
				}
			}
#ifdef CAPON2DMODULEDEBUG
			if (estOutput->cyclesLog->dopDetCnt < MAX_NPNTS )
				estOutput->cyclesLog->dopEstCycles[estOutput->cyclesLog->dopDetCnt] = TSCL - cycleStart;
			estOutput->cyclesLog->dopDetCnt++;
#endif
		}// end of Doppler estimation
	}// end of if (input->processingStepSelector == 1) /* estimate 2D angle -- azimuth, elevation and Doppler estimation, called per detected point*/
    return (errorCode);
}


//! \copydoc RADARDEMO_aoaEst2DCaponBF_run
RADARDEMO_aoaEst2DCaponBF_errorCode    RADARDEMO_aoaEst2DCaponBF_static_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoaEst2DCaponBF_input * input,
                            OUT RADARDEMO_aoaEst2DCaponBF_output   * estOutput)

{
    //uint32_t     i, rnOffset;
	cplxf_t *sigIn;
    RADARDEMO_aoaEst2DCaponBF_handle *aoaEstBFInst;
    RADARDEMO_aoaEst2DCaponBF_errorCode errorCode = RADARDEMO_AOACAPONBF_NO_ERROR;
    aoaEstBFInst    =   (RADARDEMO_aoaEst2DCaponBF_handle *) handle;

	if (input->processingStepSelector == 0) /* estimate the range-angle heatmap, per range bin*/
	{
		sigIn = &(estOutput->static_information[input->rangeIndx * aoaEstBFInst->nRxAnt]);

		RADARDEMO_aoaEstimationBFSinglePeak_static(
								sigIn,
								aoaEstBFInst,
								(float *) estOutput->rangeAzimuthHeatMap,
								(float *) &estOutput->malValPerRngBin[input->rangeIndx]);	   
	}
	else if (input->processingStepSelector == 1) /* estimate the 2D angle azimuth and elevation per detected point */
	{
		float	* RESTRICT inputRAHeatMap;
		float	accAzim, accElev, accPow, power, azimCoM, elevCoM, tempElev, tempAzim, invPow;
		int32_t i, j, elevSearchLen, azimSearchLen;
		int32_t	startElev, endElev, startAzim, endAzim;
		int32_t		azimuthIndx, elevationIndx, angleIdx;

		elevSearchLen	=	aoaEstBFInst->staticElevSearchLen;
		azimSearchLen	=	aoaEstBFInst->staticAzimSearchLen;

		angleIdx		=	input->angleIndx;
		//for ( elevationIndx = 0; elevationIndx < (int32_t)elevSearchLen, angleIdx > 0; elevationIndx++, angleIdx -= azimSearchLen);
		//azimuthIndx		=	angleIdx - elevationIndx * azimSearchLen;
		for ( azimuthIndx = 0; (azimuthIndx < (int32_t)azimSearchLen) && (angleIdx >= 0); azimuthIndx++)
		{
		    angleIdx    -=  elevSearchLen;
		}
		azimuthIndx			=	azimuthIndx - 1;
		elevationIndx		=	input->angleIndx - azimuthIndx * elevSearchLen;

		// 2D center of mass calculation for finer estimation.
		inputRAHeatMap		=	(float *) estOutput->rangeAzimuthHeatMap;

		startAzim			=	azimuthIndx - 1;
		if ( startAzim < 0 )
			startAzim		=	0;
		endAzim				=	azimuthIndx + 1;
		if ( endAzim >= azimSearchLen )
			endAzim			=	azimSearchLen - 1;
		startElev			=	elevationIndx - 1;
		if ( startElev < 0 )
			startElev		=	0;
		endElev				=	elevationIndx + 1;
		if ( endElev >= (int32_t)elevSearchLen )
			endElev			=	elevSearchLen - 1;
		
		accPow				=	0.f;
		accAzim				=	0.f;
		accElev				=	0.f;
		for (j = startAzim; j <= endAzim; j++)
		{

			for (i = startElev; i <= endElev; i++)
			{
				power		=	inputRAHeatMap[(j * elevSearchLen + i) * aoaEstBFInst->raHeatMap_handle->numInputRangeBins + input->rangeIndx];
				accPow		+=	power;
				accAzim		+=	power * (float)j;
				accElev		+=	power * (float)i;
			}
		}
		invPow				=	_rcpsp(accPow);
		invPow				=	invPow * (2.f - accPow * invPow);
		invPow				=	invPow * (2.f - accPow * invPow);

		azimCoM				=	accAzim * invPow;
		elevCoM				=	accElev * invPow;

		tempElev			=	(float)asinsp_i(aoaEstBFInst->raHeatMap_handle->muInit + aoaEstBFInst->raHeatMap_handle->muStep * aoaEstBFInst->staticElevStepDeciFactor * elevCoM);
		tempAzim			=	divsp_i((aoaEstBFInst->raHeatMap_handle->nuInit + aoaEstBFInst->raHeatMap_handle->nuStep * aoaEstBFInst->staticAzimStepDeciFactor * azimCoM), (float)cossp_i(tempElev));
		if ( _fabs(tempAzim) < 1.f )
		{
			estOutput->elevEst[0]			=	tempElev; 
			estOutput->azimEst[0]			=	(float)asinsp_i(tempAzim);
			estOutput->peakPow[0]			=	inputRAHeatMap[(azimuthIndx * elevSearchLen + elevationIndx) * aoaEstBFInst->raHeatMap_handle->numInputRangeBins + input->rangeIndx];
			estOutput->numAngleEst			=	1;
		}
		else
			estOutput->numAngleEst			=	0;

	}
	return (errorCode);
}

