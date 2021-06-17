/**
 *   @file  staticdetprocdsp.c
 *
 *   @brief
 *      Implements Data path StaticDet processing Unit using DSP.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ti/mathlib/mathlib.h>
#define DebugP_LOG_ENABLED 1 
#include <ti/drivers/osal/DebugP.h>


/* mmWave SDK driver/common Include Files */
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/alg/mmwavelib/mmwavelib.h>

/* Utils */
#include <ti/utils/mathutils/mathutils.h>
#include <ti/alg/mmwavelib/include/mmwavelib_c674_emulate_c66_intrinsics.h>

/* Data Path Include files */
#include <ti/datapath/dpedma/dpedma.h>
#include <common/src/dpu/staticdetproc/staticdetprocdsp.h>
#include <common/src/dpu/staticdetproc/include/staticdetprocdspinternal.h>


#ifdef XWR68XX_ISK_ANTENNA_PATTERN
// Antenna configuration for ISK, TX order (TX1, TX3, TX2)
//  
//        8, 9,10,11
//  0, 1, 2, 3, 4, 5, 6, 7
//  Notch enable should be zero
int32_t  aDOA_indList[STATICDET_NUM_ADOA_ANT_TOT] = {0, 1, 2, 3, 4, 5, 6, 7};
int32_t  eDOA_indList[STATICDET_NUM_EDOA_ANT_TOT] = {8, 2,-1,-1, 9, 3,-1,-1,
                                                    10, 4,-1,-1,11, 5,-1,-1};
#endif

#ifdef XWR68XX_ODS_ANTENNA_PATTERN
// Antenna configuration for ODS, TX order (TX1, TX2, TX3)
// case 'xWR6843ODS'
// ch-0   ch-3   ch-4    ch-7
// ch-1   ch-2   ch-5    ch-6
//               ch-8    ch-11
//               ch-9    ch-10
// Notch enable should be one
int32_t  aDOA_indList[STATICDET_NUM_ADOA_ANT_TOT] = {1, 2, 5, 6,  0, 3, 4, 7};
int32_t  eDOA_indList[STATICDET_NUM_EDOA_ANT_TOT] = {5, 8, 9, -1, 4, 5, 8, -1,
                                                     6,11,10, -1, 7, 6,11, -1};
#endif

#ifdef XWR68XX_AOP_ANTENNA_PATTERN
// Antenna configuration for AOP ES2, TX order (TX1, TX2, TX3)
// case 'xWR6843AOPES2'

 /* ch-3   ch-1
  * ch-2   ch-0
  * ch-11  ch-9   ch-7   ch-5
  * ch-10  ch-8   ch-6   ch-4    
*/	 
int32_t  aDOA_indList[STATICDET_NUM_ADOA_ANT_TOT] = {10, 8, 6, 4,  11, 9, 7, 5};
int32_t  eDOA_indList[STATICDET_NUM_EDOA_ANT_TOT] = {2, 11, 10, -1, 3, 2, 11, -1,
                                                     0, 9, 8, -1, 1, 0, 9, -1};
#endif

/*===========================================================
 *                    Internal Functions
 *===========================================================*/
/**
 *  @b Description
 *  @n
 *  StaticDet DPU EDMA configuration.
 *  This implementation of staticdet processing involves a Ping/Pong 
 *  mechanism, hence there are two sets of EDMA transfer.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
static inline int32_t DPU_StaticDetProcDSP_configEdma
(
    DPU_StaticDetProcDSP_Obj      *obj,
    DPU_StaticDetProcDSP_Config   *cfg
)
{
    int32_t             retVal = EDMA_NO_ERROR;
    uint16_t            sampleLenInBytes = sizeof(cmplx16ImRe_t);
    //uint16_t            sizeOfDetMatrixElement = sizeof(uint16_t);
    DPEDMA_syncACfg     syncACfg;
    DPEDMA_syncABCfg    syncABCfg;
    cmplx16ImRe_t      *radarCubeBase;

    if(obj == NULL)
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
        goto exit;
    }
    
    radarCubeBase = (cmplx16ImRe_t *)obj->cfg.hwRes.radarCube.data;

    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer data from Radar cube to input buffer (ping)
    ******************************************************************************************/   
    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    syncABCfg.destAddress = (uint32_t)(&obj->cfg.hwRes.pingPongBuf[0]);
    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = cfg->staticCfg.numDopplerChirps;
    syncABCfg.cCount      = 1;/*data for one virtual antenna transferred at a time*/
    syncABCfg.srcBIdx     = cfg->staticCfg.numRxAntennas * cfg->staticCfg.numRangeBins * sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.srcCIdx     = 0U;    
    syncABCfg.dstCIdx     = 0U;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.ping,
                                 NULL,//chainingCfg: No chaining  
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 true,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg
                                 
    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }    
    
    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer data from Radar cube to input buffer (pong)
     ******************************************************************************************/ 

    /* Transfer parameters are the same as ping, except for src/dst addresses */

    /*Ping/Pong srcAddress is reprogrammed in every iteration (except for first ping), therefore
      this srcAddress below will be reprogrammed during processing. In order to avoid recomputing here
      the first pong srcAddress, it will be hardcoded to a valid address (dummy).*/      
    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);/*dummy*/
    syncABCfg.destAddress = (uint32_t)(&obj->cfg.hwRes.pingPongBuf[cfg->staticCfg.numDopplerChirps]);

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.pong,
                                 NULL,//chainingCfg: No chaining  
                                 &syncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 true,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA channel to transfer data from StaticDet DPU internal buffers to Detection matrix
    ******************************************************************************************/    
    syncACfg.srcAddress  = (uint32_t)cfg->hwRes.heatmapLocal;
    syncACfg.destAddress = (uint32_t)&cfg->hwRes.rangeAzimuthHeatmap[0U];
    syncACfg.aCount      = cfg->hwRes.steeringVecSize;
    syncACfg.bCount      = 0U;
    syncACfg.srcBIdx     = 0U;
    syncACfg.dstBIdx     = cfg->hwRes.steeringVecSize;
         
    retVal = DPEDMA_configSyncA_singleFrame(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOut,
                                NULL,//chainingCfg: No chanining
                                &syncACfg,
                                false, //isEventTriggered
                                true,//isIntermediateTransferInterruptEnabled
                                true,//isTransferCompletionEnabled
                                NULL, //transferCompletionCallbackFxn
                                NULL);//transferCompletionCallbackFxnArg

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return(retVal);
} 

/**
 *  @b Description
 *  @n
 *      This function waits for an input (from radar cube to scratch buffer)
 *      EDMA transfer to complete.
 *
 *  @param[in]  obj         DPU object.
 *  @param[in]  pingPongId  Ping/pong ID
 *
 *  \ingroup    DPU_DOPPLERPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       !=0
 */
static inline int32_t DPU_StaticDetProcDSP_waitInData(DPU_StaticDetProcDSP_Obj *obj, uint32_t pingPongId)
{
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t       chId;
    int32_t       retVal = 0;
    
    if(pingPongId == DPU_STATICDETPROCDSP_PING_IDX)
    {
        chId = obj->cfg.hwRes.edmaCfg.edmaIn.ping.channel;
    }
    else
    {
        chId = obj->cfg.hwRes.edmaCfg.edmaIn.pong.channel;
    }
    do 
    {
        retVal = EDMA_isTransferComplete(obj->cfg.hwRes.edmaCfg.edmaHandle,
                                        chId,
                                        (bool *)&isTransferDone);
        if(retVal != EDMA_NO_ERROR)
        {
            return retVal;
        }
    } while (isTransferDone == false);
    
    return 0;
}



void DPU_StaticDetProcDSP_steeringVecGen(    
    DPU_StaticDetProcDSP_Config   *cfg
)
{

    uint32_t    i, j;
    double      ftemp1, freal1, fimag1, frealJ, fimagJ;

    // Ant0's steeringVec is 1 for all angle possiblities, so we don't save them
    for (i = 0; i < cfg->hwRes.steeringVecSize; i++)
    {
        ftemp1          =   (double) sin((-cfg->staticCfg.estAngleRange + (double) i * cfg->staticCfg.angleStepDeg) * (double)DPU_STATICDETPROC_PIOVER180);
        freal1          =   (double) cos(DPU_STATICDETPROC_PI*ftemp1);
        fimag1          =   (double) sin(DPU_STATICDETPROC_PI*ftemp1);
        frealJ          =   freal1;
        fimagJ          =   fimag1;
        cfg->hwRes.steeringVec[(cfg->staticCfg.nAntForHeatmap - 1) * i + 0].real = (float)frealJ;
        cfg->hwRes.steeringVec[(cfg->staticCfg.nAntForHeatmap - 1) * i + 0].imag = (float)fimagJ;
        for (j = 2; j < cfg->staticCfg.nAntForHeatmap; j++)
        {
            ftemp1      =   frealJ;
            frealJ      =   frealJ * freal1 - fimagJ * fimag1;
            fimagJ      =   ftemp1 * fimag1 + fimagJ * freal1;
            cfg->hwRes.steeringVec[(cfg->staticCfg.nAntForHeatmap - 1) * i + j - 1].real = (float)frealJ;
            cfg->hwRes.steeringVec[(cfg->staticCfg.nAntForHeatmap - 1) * i + j - 1].imag = (float)fimagJ;
        }
    }
}




/**
 *  @b Description
 *  @n
 *      This function implements static calculation + phase compensation.
 *
 *  @param[in]  cfg             DPU configuration.
 *  @param[in]  inputBuf        input buffer. Clutter removal algorithm will work on this data.
 *  @param[out]  pMeanVal       calculated zero doppler value
 *
 *  \ingroup    DPU_STATICDETPROC_INTERNAL_FUNCTION
 *
 *  @retval N/A
 */
static inline void DPU_StaticDetProcDSP_zeroDopplerCalc
(
    DPU_StaticDetProcDSP_Config   *cfg,
    cmplx16ImRe_t               *inputBuf,
	cmplx16ImRe_t               phaseRot,
	cplxfImRe_t                 *outVal
)
{
    int32_t sumVal[2]={0};
    
    /*Mmwavelib API requires size to be multiple of 4, which is fine because DPU allows 
      only numDopplerChirps that is multiple of 4*/
    mmwavelib_vecsum((int16_t *) inputBuf,
                     (int32_t *) sumVal,
                     (int32_t)   cfg->staticCfg.numDopplerChirps);
    
	outVal->real = phaseRot.real * (float)(sumVal[1]) - phaseRot.imag * (float)(sumVal[0]);
    outVal->imag = phaseRot.imag * (float)(sumVal[1]) + phaseRot.real * (float)(sumVal[0]);
                                 	
}


/**
 *  @b Description
 *  @n
 *      This function implements highest Doppler calculation + phase compensation.
 *
 *  @param[in]  cfg             DPU configuration.
 *  @param[in]  inputBuf        input buffer. Clutter removal algorithm will work on this data.
 *  @param[out]  pMeanVal       calculated highest doppler value
 *
 *  \ingroup    DPU_STATICDETPROC_INTERNAL_FUNCTION
 *
 *  @retval N/A
 */
static inline void DPU_StaticDetProcDSP_highDopplerCalc
(
    DPU_StaticDetProcDSP_Config   *cfg,
    cmplx16ImRe_t               *inputBuf,
    cmplx16ImRe_t               phaseRot,
    cplxfImRe_t                 *outVal
)
{
    int32_t sumVal[2]={0};
	uint32_t ind; 

    for (ind = 0; ind < cfg->staticCfg.numDopplerChirps; ind = ind + 2)
    {
        sumVal[0] += (int32_t)(inputBuf[ind].real) - (int32_t)(inputBuf[ind+1].real);
        sumVal[1] += (int32_t)(inputBuf[ind].imag) - (int32_t)(inputBuf[ind+1].imag);
    }

    outVal->real = phaseRot.real * (float)(sumVal[1]) - phaseRot.imag * (float)(sumVal[0]);
    outVal->imag = phaseRot.imag * (float)(sumVal[1]) + phaseRot.real * (float)(sumVal[0]);

}

static inline uint32_t DPU_StaticDetProcDSP_detection
(
    DPU_StaticDetProcDSP_Config   *cfg
)
{
    uint32_t  rangeIdx, i, index, index_prev, index_next, maxAInd, maxEInd, detectionStatus; 
	uint32_t  totalNumDet;
	float     temp, tempAbs, maxAVal, maxEVal;
	float     peakVal_current, peakVal_prev, peakVal_next;
	float     peakSum_current, peakSum_record, peakSum_delta, snr; //, noise;
	float     range, aAngle, eAngle;
	float     aAngleInDeg, eAngleInDeg, cosEAngle;
    float     temp_delta, maxEVal_delta, sinAAngle;
    int32_t   maxEInd_delta, eInd_diff, maxEInd_diff;

	totalNumDet = 0;
	index_prev = 0; 
   	index = cfg->hwRes.steeringVecSize; 
	index_next = index + cfg->hwRes.steeringVecSize;	
	for(rangeIdx = 1; rangeIdx < (cfg->staticCfg.numRangeBinsForDetection - 1); rangeIdx ++)
	{
	    //calculate the heatmapDiff and record the peak value and peak index
		maxAInd = 0;
		maxAVal = -1;
        for (i = 0; i < cfg->hwRes.steeringVecSize; i++)
		{
			temp = cfg->hwRes.rangeAzimuthHeatmap[index + i] - cfg->hwRes.rangeAzimuthHeatmap_record[index + i];
			tempAbs = (temp > 0? temp:-temp);
			if (tempAbs > maxAVal)
			{
				maxAInd = i;
				maxAVal = tempAbs;
			}	
            cfg->hwRes.heatmapDiff[i] = tempAbs;
		}
        // the detection condition 
		// 1: The peak index has to be within the minAAngleIndex and maxAAngleIndex
	    // 2: The peak in Diff has to be a local peak cross range, 
		// 3: The peak in Diff has to be bigger than the recorded heatmap or the current heatmap
		// 4: The peak index for Elevation has to be within the minEAngleIndex and maxEAngleIndex
		if ((maxAInd > cfg->staticCfg.minAAngleIndex) && (maxAInd < cfg->staticCfg.maxAAngleIndex))
		{
		    peakVal_current = maxAVal;  
		    temp            = cfg->hwRes.rangeAzimuthHeatmap[index_prev + maxAInd] - cfg->hwRes.rangeAzimuthHeatmap_record[index_prev + maxAInd];
			peakVal_prev    = (temp > 0? temp:-temp);
		    temp            = cfg->hwRes.rangeAzimuthHeatmap[index_next + maxAInd] - cfg->hwRes.rangeAzimuthHeatmap_record[index_next + maxAInd];
			peakVal_next    = (temp > 0? temp:-temp);
					
		    if   ((peakVal_current > (peakVal_prev * cfg->staticCfg.localPeakTH))
		       && (peakVal_current > (peakVal_next * cfg->staticCfg.localPeakTH))
		       && (maxAVal > (cfg->staticCfg.heatmapDiffToNoiseTH * cfg->staticCfg.noiseLevel_record)))
		    {
			    // sum local angle bins for current
			    peakSum_current = 0;
				peakSum_record = 0;
				peakSum_delta = 0;
			    for (i = maxAInd - cfg->staticCfg.numAngleBinToSum; i <= maxAInd + cfg->staticCfg.numAngleBinToSum; i++)
				{
					peakSum_current += cfg->hwRes.rangeAzimuthHeatmap[index + i]; 
					peakSum_record  += cfg->hwRes.rangeAzimuthHeatmap_record[index + i]; 
					peakSum_delta   += cfg->hwRes.heatmapDiff[i]; 
				}
				detectionStatus = 0;
				if (peakSum_delta > (cfg->staticCfg.heatmapDiffTH * peakSum_record))
					detectionStatus = 1;
				if (peakSum_delta > (cfg->staticCfg.heatmapDiffTH * peakSum_current))
					detectionStatus = 2;
				
				// a peak is detected. 
				if (detectionStatus > 0)
				{
					maxEVal = -1;
					maxEInd = 0;
					maxEVal_delta = -1;
					maxEInd_delta = 0;
					for (i = 0; i < cfg->hwRes.steeringVecSize; i++)
					{
						// calculate the maxEInd_delta from the rangeElevation heatmap Difference
						temp_delta = cfg->hwRes.rangeElevationHeatmap[index + i] - cfg->hwRes.rangeElevationHeatmap_record[index + i];
						if  (detectionStatus == 2)
						{
							temp_delta = -temp_delta;
						    snr = peakSum_delta / peakSum_current;
						    //noise = peakSum_current;
					    }
						else
						{
						    snr = peakSum_delta / peakSum_record;
						    //noise = peakSum_record;
						}
                        if (temp_delta > maxEVal_delta)
                        {
                            maxEInd_delta = i;
                            maxEVal_delta = temp_delta;
                        }

                        // calculate the maxEInd from the max of two rangeElevation heatmap 
						if  (detectionStatus == 1)
                            temp = cfg->hwRes.rangeElevationHeatmap[index + i];
                        if  (detectionStatus == 2)
                            temp = cfg->hwRes.rangeElevationHeatmap_record[index + i];
						if (temp > maxEVal)
						{
							maxEInd = i;
							maxEVal = temp;
						}
					}
					eInd_diff = maxEInd_delta - maxEInd;
					maxEInd_diff = (eInd_diff > 0? eInd_diff:-eInd_diff);
					if ((maxEInd_delta > cfg->staticCfg.minEAngleIndex) && (maxEInd_delta < cfg->staticCfg.maxEAngleIndex)
						&& (totalNumDet <  cfg->hwRes.detObjOutMaxSize) && (maxEInd_diff <= cfg->staticCfg.eAngleBinDiffTH))
                    {
						 cfg->hwRes.detObjOut[totalNumDet].velocity = 0;
						 
                         range = (rangeIdx + cfg->staticCfg.minRangeIndex) * cfg->staticCfg.rangeStep; 
                         aAngleInDeg = ((cfg->staticCfg.angleStepDeg * (float)(maxAInd)) - cfg->staticCfg.estAngleRange);
                         eAngleInDeg = ((cfg->staticCfg.angleStepDeg * (float)(maxEInd_delta)) - cfg->staticCfg.estAngleRange);
                         eAngle = (eAngleInDeg - cfg->staticCfg.tiltAngleDeg)*DPU_STATICDETPROC_PIOVER180;
						 cosEAngle = cos(eAngle);
						 sinAAngle = sin(aAngleInDeg * DPU_STATICDETPROC_PIOVER180)/cosEAngle; 
						 
						 if ((sinAAngle < 1) && (sinAAngle > -1))
                         {
							 aAngle = asin(sinAAngle);
							 cfg->hwRes.detObjOut[totalNumDet].x = range * cosEAngle * sinAAngle;
							 cfg->hwRes.detObjOut[totalNumDet].y = range * cosEAngle * cos(aAngle);
							 cfg->hwRes.detObjOut[totalNumDet].z = range * sin(eAngle);
							 cfg->hwRes.detObjOutSideInfo[totalNumDet].snr = (int16_t)(floor(snr));
							 cfg->hwRes.detObjOutSideInfo[totalNumDet].noise = 0;
							 totalNumDet ++;
						 }

			        }
				}		
			}				
		}
		index_prev = index; 
     	index = index_next; 
		index_next = index_next + cfg->hwRes.steeringVecSize;
	}
	return(totalNumDet);
}



static inline void  DPU_StaticDetProcDSP_heatmapGen(
    DPU_StaticDetProcDSP_Config   *cfg,
	int32_t                    *indList,
	cplxfImRe_t                *zeroDopplerOut,
	uint32_t                    numAnt,
    float                       *doaSpectrum	
)
{
    int32_t 	i, index, index2;
    float 		tempPow;
	__float2_t 	input0;
	__float2_t 	*tempSteerVecPtr;
    cplxfImRe_t sigIn[STATICDET_MAX_NUM_ANT];
    cplxfImRe_t	*sig, *phaseRot; 

    phaseRot = &cfg->staticCfg.antNotchPhaseRot; 	
	for (i = 0; i < numAnt; i++)
	{
		index = indList[i]; 
		if (index >= 0)
		{
    		memcpy((void*)&sigIn[i], (void*)&zeroDopplerOut[index], sizeof(cplxfImRe_t));
		    if (cfg->staticCfg.antNotchEnable)
		    {
		        // apply RX beamforming to create a notch, useful for ODS antenna. 		
			    index2 = indList[i+numAnt];     			
				sig = &zeroDopplerOut[index2]; 
			    sigIn[i].real +=  sig->real * phaseRot->real - sig->imag * phaseRot->imag;
			    sigIn[i].imag +=  sig->real * phaseRot->imag + sig->imag * phaseRot->real;
			}
		}
		else 
			memset((void *)&sigIn[i], 0, sizeof(cplxfImRe_t));
	}

    /*
     Solve: spectrum = A'*Rn*A = |A'x|.^2;
     where:
             A = [steeringVec(theta)]  is a nRxAnt by numAngles matrix
             Rn is the covariance matrix of the antenna signal
             x is the input signal vector
    */
	
	/* Compute the spectrum only */
	if ( numAnt == 4 )
	{
		__float2_t input1, input2, input3, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		tempSteerVecPtr =	(__float2_t *) cfg->hwRes.steeringVec;
		
		for (i = 0; i < cfg->hwRes.steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	+=	tempPow;
			tempSteerVecPtr = tempSteerVecPtr + (STATICDET_MAX_NUM_ANT_FOR_HEATMAP - 4);
		}
	}
	else if ( numAnt == 8 )
	{
		__float2_t input1, input2, input3, input4, input5, input6, input7, f2temp1;
		
		input0 			=	_amem8_f2(&sigIn[0]);
		input1 			=	_amem8_f2(&sigIn[1]);
		input2 			=	_amem8_f2(&sigIn[2]);
		input3 			=	_amem8_f2(&sigIn[3]);
		input4 			=	_amem8_f2(&sigIn[4]);
		input5 			=	_amem8_f2(&sigIn[5]);
		input6 			=	_amem8_f2(&sigIn[6]);
		input7 			=	_amem8_f2(&sigIn[7]);
		tempSteerVecPtr =	(__float2_t *) cfg->hwRes.steeringVec;
		
		for (i = 0; i < cfg->hwRes.steeringVecSize; i++ )
		{
			f2temp1		=	_daddsp(input0, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input1));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input2));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input3));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input4));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input5));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input6));
			f2temp1		=	_daddsp(f2temp1, _complex_conjugate_mpysp(_amem8_f2(tempSteerVecPtr++), input7));
			f2temp1		=	_dmpysp(f2temp1, f2temp1);
			tempPow		=	_hif2(f2temp1) + _lof2(f2temp1);
			doaSpectrum[i]	+=	tempPow;
		}
	}
}







/*===========================================================
 *                    StaticDet Proc External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      staticdetProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    DPU_STATICDETPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_StaticDetProcDSP_Handle DPU_StaticDetProcDSP_init
(
    int32_t                       *errCode
)
{
    DPU_StaticDetProcDSP_Obj  *obj = NULL;

    *errCode       = 0;
    
    /* Allocate memory */
    obj = MemoryP_ctrlAlloc(sizeof(DPU_StaticDetProcDSP_Obj), 0U);
    if(obj == NULL)
    {
        *errCode = DPU_STATICDETPROCDSP_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)obj, 0U, sizeof(DPU_StaticDetProcDSP_Obj));
   
exit:    
   return ((DPU_StaticDetProcDSP_Handle)obj);
}

/**
  *  @b Description
  *  @n
  *   StaticDet DPU configuration 
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_STATICDETPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_STATICDETPROC_ERROR_CODE
  */
int32_t DPU_StaticDetProcDSP_config(
    DPU_StaticDetProcDSP_Handle    handle,
    DPU_StaticDetProcDSP_Config    *cfg
)
{
    DPU_StaticDetProcDSP_Obj   *obj;
    int32_t                  retVal = 0;

    obj = (DPU_StaticDetProcDSP_Obj *)handle;
    if(obj == NULL)
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
        goto exit;
    }
    
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.radarCube.data ||
       !cfg->hwRes.pingPongBuf 
      )
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
        goto exit;
    }
    
    if(obj->inProgress == true)
    {
        retVal = DPU_STATICDETPROCDSP_EINPROGRESS;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_1)
    {
        retVal = DPU_STATICDETPROCDSP_ECUBEFORMAT;
        goto exit;
    }

    /* Check if number of staticdet chirps is a multiple of 4*/
    if((cfg->staticCfg.numDopplerChirps & 3) != 0)
    {
        retVal = DPU_STATICDETPROCDSP_ENUMDOPCHIRPS;
        goto exit;
    }

      
    /* Check if DPU configuration is compatible with EDMA max jump size of (32K - 1)*/
    if(cfg->staticCfg.numRxAntennas * cfg->staticCfg.numRangeBins * sizeof(cmplx16ImRe_t) >= 32768)
    {
        retVal = DPU_STATICDETPROCDSP_EEXCEEDMAXEDMA;
        goto exit;
    }
    
    /* Check if buffers provided by application have the correct alignment*/
    if (MEM_IS_NOT_ALIGN(cfg->hwRes.pingPongBuf        , DPU_STATICDETPROCDSP_BUFFER_BYTE_ALIGNMENT))        
    {
        retVal = DPU_STATICDETPROCDSP_EBUFALIGNMENT;
        goto exit;
    }
    
    /* check sizes for the scratch buffers provided by application */
    if((cfg->hwRes.pingPongSize < 2 * sizeof(cmplx16ImRe_t) * cfg->staticCfg.numDopplerChirps   ) )
    {
            retVal = DPU_STATICDETPROCDSP_ESCRATCHSIZE;
            goto exit;
    }
    
    if(cfg->staticCfg.isBpmEnabled)
    {
        retVal = DPU_STATICDETPROCDSP_EBPMCFG;
        goto exit;
    }


    /* Generate tables */
    DPU_StaticDetProcDSP_steeringVecGen(cfg);

    /* Save necessary parameters to DPU object that will be used during Process time */
    memcpy((void *)&obj->cfg, (void *)cfg, sizeof(DPU_StaticDetProcDSP_Config));

    /* Configure EDMA */
    retVal = DPU_StaticDetProcDSP_configEdma(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }
    
    obj->inProgress = false;
    
exit:
    return retVal;
}


void  DPU_StaticDetProcDSP_rxChanPhaseBiasCompensation(cmplx16ImRe_t *rxChPhaseComp, uint16_t numRxAnt, cmplx16ImRe_t *inpBuf, cplxfImRe_t *outBuf)
{
	uint16_t idx;
	
	// Phase error compensation
	for (idx = 0; idx < numRxAnt; idx++)
	{
		outBuf[idx].real = (float)(inpBuf[idx].real * rxChPhaseComp[idx].real - inpBuf[idx].real * rxChPhaseComp[idx].real);
		outBuf[idx].imag = (float)(inpBuf[idx].imag * rxChPhaseComp[idx].real + inpBuf[idx].real * rxChPhaseComp[idx].imag);
	}
}

 /**
  *  @b Description
  *  @n StaticDet DPU process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[out]  outParams  Output parameters.
  *
  *  \ingroup    DPU_STATICDETPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_STATICDETPROC_ERROR_CODE
  */
int32_t DPU_StaticDetProcDSP_process
(
    DPU_StaticDetProcDSP_Handle    handle,
    DPU_StaticDetProcDSP_OutParams *outParams
)
{
    DPU_StaticDetProcDSP_Obj *obj;
    DPU_StaticDetProcDSP_Config *cfg;
    int16_t rxAntIdx, rangeIdx, txAntIdx,k, heatmapIdx, visualAntID;
    uint16_t nextTransferRxIdx, nextTransferRangeIdx, nextTransferTxIdx;     
    uint32_t nextTransferIdx, waitingTime, pingPongIdx;
    volatile uint32_t startTime;
    volatile uint32_t startTimeWait;
    int32_t  retVal;
    uint8_t  channel;
	uint32_t totalNumDet; 
    cmplx16ImRe_t  *inpDoppFftBuf;
    cmplx16ImRe_t  *radarCubeBase;
    uint32_t zeroDopplerIdx;
    
    retVal = 0;    
    waitingTime = 0;
    obj = (DPU_StaticDetProcDSP_Obj *)handle;
    
    if (obj == NULL)
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
        goto exit;
    }    
    
    if(obj->inProgress == true)
    {
        retVal = DPU_STATICDETPROCDSP_EINPROGRESS;
        goto exit;
    }
    else
    {
        obj->inProgress = true;
    }

    startTime = Cycleprofiler_getTimeStamp();
    
    radarCubeBase = (cmplx16ImRe_t *)obj->cfg.hwRes.radarCube.data; 
    cfg =  &obj->cfg;

    pingPongIdx = DPU_STATICDETPROCDSP_PING_IDX;
    
    /* trigger first DMA */
    EDMA_setSourceAddress(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.edmaIn.ping.channel, (uint32_t) &radarCubeBase[cfg->staticCfg.minRangeIndex]);

    EDMA_startDmaTransfer(cfg->hwRes.edmaCfg.edmaHandle, cfg->hwRes.edmaCfg.edmaIn.ping.channel);

    for (rangeIdx = cfg->staticCfg.minRangeIndex; rangeIdx < cfg->staticCfg.maxRangeIndex; rangeIdx++)
    {
        for (rxAntIdx = 0; rxAntIdx < cfg->staticCfg.numRxAntennas; rxAntIdx++)
        {
            for (txAntIdx = 0; txAntIdx < cfg->staticCfg.numTxAntennas; txAntIdx++)
            {
				visualAntID = (txAntIdx * cfg->staticCfg.numRxAntennas) + rxAntIdx; 
                /* verify that previous DMA has completed */
                startTimeWait = Cycleprofiler_getTimeStamp();
                DPU_StaticDetProcDSP_waitInData (obj, pingPongIdx);
                waitingTime += Cycleprofiler_getTimeStamp() - startTimeWait;
                
                /*Find index in radar cube for next EDMA. 
                  Order from bringing data from radar cube is:
                  1. Next TX antennas for same range/RxAnt (to assure BPM can be decoded).
                  2. Next virtual antenna(which means next RxAnt for same TxAnt) 
                     for same range (to assure sum of all virtual antennas can be computed).
                  3. Next range.
                */  
                nextTransferTxIdx    = txAntIdx + 1;
                nextTransferRxIdx    = rxAntIdx;
                nextTransferRangeIdx = rangeIdx;
                
                if(nextTransferTxIdx == cfg->staticCfg.numTxAntennas)
                {
                   nextTransferTxIdx = 0;
                   nextTransferRxIdx++;
                   if(nextTransferRxIdx == cfg->staticCfg.numRxAntennas)
                   {
                       nextTransferRxIdx = 0;
                       nextTransferRangeIdx++;
                   }
                }
                
                nextTransferIdx = (nextTransferTxIdx * cfg->staticCfg.numRxAntennas * cfg->staticCfg.numDopplerChirps + 
                                   nextTransferRxIdx) * cfg->staticCfg.numRangeBins + nextTransferRangeIdx;
                
                /*Last computation happens when nextTransferRangeIdx reaches numRangeBins. 
                  This indicates that, the current virtual antenna is the last one for (numRangeBins-1). 
                  Therefore, do not trigger next EDMA.*/
                if(nextTransferRangeIdx < cfg->staticCfg.numRangeBins)
                {
                    /* kick off next DMA */
                    if (pingPongIdx == DPU_STATICDETPROCDSP_PONG_IDX)
                    {
                        channel = cfg->hwRes.edmaCfg.edmaIn.ping.channel;
                    }
                    else
                    {
                        channel = cfg->hwRes.edmaCfg.edmaIn.pong.channel;
                    }
                    
                    EDMA_setSourceAddress(cfg->hwRes.edmaCfg.edmaHandle, channel,
                                         (uint32_t) &radarCubeBase[nextTransferIdx]);
                                         
                    EDMA_startDmaTransfer(cfg->hwRes.edmaCfg.edmaHandle, channel);
                }    
                
                inpDoppFftBuf = (cmplx16ImRe_t *) &cfg->hwRes.pingPongBuf[pingPongIdx * cfg->staticCfg.numDopplerChirps];
                

                /* calculate zero Doppler, and apply phase rotation, and do ReIm swap to get ready for the rest of the operation*/
                DPU_StaticDetProcDSP_zeroDopplerCalc(cfg, inpDoppFftBuf, 
				                        cfg->dynCfg.compRxChanCfg.rxChPhaseComp[visualAntID],
										&cfg->hwRes.zeroDopplerOut[visualAntID]);
				

				if (cfg->staticCfg.recordingMode == 1)
				{
	                if (rangeIdx == cfg->staticCfg.rangeBinForNoiseLevelCalc)
	                {    //calculate highest Doppler
	                     DPU_StaticDetProcDSP_highDopplerCalc(cfg, inpDoppFftBuf,
	                                         cfg->dynCfg.compRxChanCfg.rxChPhaseComp[visualAntID],
	                                        &cfg->hwRes.highDopplerOut[visualAntID]);
	                }

	                zeroDopplerIdx = (rangeIdx - cfg->staticCfg.minRangeIndex) * cfg->staticCfg.numVirtualAntennas;
					if (cfg->staticCfg.recordingFrameCnt == 0)
				        cfg->hwRes.zeroDopplerSum[zeroDopplerIdx + visualAntID] = cfg->hwRes.zeroDopplerOut[visualAntID];
				    else if (cfg->staticCfg.recordingFrameCnt < STATICDET_RECORDING_NUMFRAMETOAVG)
					{
				        cfg->hwRes.zeroDopplerSum[zeroDopplerIdx + visualAntID].real += cfg->hwRes.zeroDopplerOut[visualAntID].real;
				        cfg->hwRes.zeroDopplerSum[zeroDopplerIdx + visualAntID].imag += cfg->hwRes.zeroDopplerOut[visualAntID].imag;
					}
					if (cfg->staticCfg.recordingFrameCnt == (STATICDET_RECORDING_NUMFRAMETOAVG - 1))
					{
				        cfg->hwRes.zeroDopplerOut[visualAntID].real = cfg->hwRes.zeroDopplerSum[zeroDopplerIdx + visualAntID].real * STATICDET_RECORDING_NUMFRAMETOAVGINV;
				        cfg->hwRes.zeroDopplerOut[visualAntID].imag = cfg->hwRes.zeroDopplerSum[zeroDopplerIdx + visualAntID].imag * STATICDET_RECORDING_NUMFRAMETOAVGINV;
					}
				}
                 								               
                pingPongIdx ^= 1;
            }/*txAntIdx*/
        } /* rxAntIdx */
				

		// aHeatmap generation for each range bin
        if ((cfg->staticCfg.recordingMode == 0))
        {
            heatmapIdx = (rangeIdx - cfg->staticCfg.minRangeIndex) * cfg->hwRes.steeringVecSize;
            memset((void *)&cfg->hwRes.heatmapLocal[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            for (k = 0; k < STATICDET_NUM_ADOA_SET; k++)
                DPU_StaticDetProcDSP_heatmapGen(cfg, &aDOA_indList[k*(STATICDET_NUM_ADOA_ANT << STATICDET_HEATMAPGEN_NOTCH_ENABLE)], cfg->hwRes.zeroDopplerOut, STATICDET_NUM_ADOA_ANT, cfg->hwRes.heatmapLocal);
            memcpy((void*)&cfg->hwRes.rangeAzimuthHeatmap[heatmapIdx], (void*)&cfg->hwRes.heatmapLocal[0],  cfg->hwRes.steeringVecSize * sizeof(float));

            // eHeatmap generation for each range bin
            memset((void *)&cfg->hwRes.heatmapLocal[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            for (k = 0; k < STATICDET_NUM_EDOA_SET; k++)
                DPU_StaticDetProcDSP_heatmapGen(cfg, &eDOA_indList[k*(STATICDET_NUM_EDOA_ANT << STATICDET_HEATMAPGEN_NOTCH_ENABLE)], cfg->hwRes.zeroDopplerOut, STATICDET_NUM_EDOA_ANT, cfg->hwRes.heatmapLocal);
            memcpy((void*)&cfg->hwRes.rangeElevationHeatmap[heatmapIdx], (void*)&cfg->hwRes.heatmapLocal[0],  cfg->hwRes.steeringVecSize * sizeof(float));
        }

        if (((cfg->staticCfg.recordingMode == 1) && (cfg->staticCfg.recordingFrameCnt == (STATICDET_RECORDING_NUMFRAMETOAVG - 1))))
        {
            heatmapIdx = (rangeIdx - cfg->staticCfg.minRangeIndex) * cfg->hwRes.steeringVecSize;
            memset((void *)&cfg->hwRes.heatmapLocal[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            for (k = 0; k < STATICDET_NUM_ADOA_SET; k++)
                DPU_StaticDetProcDSP_heatmapGen(cfg, &aDOA_indList[k*(STATICDET_NUM_ADOA_ANT << STATICDET_HEATMAPGEN_NOTCH_ENABLE)], cfg->hwRes.zeroDopplerOut, STATICDET_NUM_ADOA_ANT, cfg->hwRes.heatmapLocal);
            memcpy((void*)&cfg->hwRes.rangeAzimuthHeatmap_record[heatmapIdx], (void*)&cfg->hwRes.heatmapLocal[0],  cfg->hwRes.steeringVecSize * sizeof(float));

            // eHeatmap generation for each range bin
            memset((void *)&cfg->hwRes.heatmapLocal[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            for (k = 0; k < STATICDET_NUM_EDOA_SET; k++)
                DPU_StaticDetProcDSP_heatmapGen(cfg, &eDOA_indList[k*(STATICDET_NUM_EDOA_ANT << STATICDET_HEATMAPGEN_NOTCH_ENABLE)], cfg->hwRes.zeroDopplerOut, STATICDET_NUM_EDOA_ANT, cfg->hwRes.heatmapLocal);
            memcpy((void*)&cfg->hwRes.rangeElevationHeatmap_record[heatmapIdx], (void*)&cfg->hwRes.heatmapLocal[0],  cfg->hwRes.steeringVecSize * sizeof(float));
         }

        if ((rangeIdx == cfg->staticCfg.rangeBinForNoiseLevelCalc) && (cfg->staticCfg.recordingMode == 1) && (cfg->staticCfg.recordingFrameCnt < STATICDET_RECORDING_NUMFRAMETOAVG))
        {
            // generate azimuth heatmap for highest Doppler bin
            memset((void *)&cfg->hwRes.heatmapLocal[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            for (k = 0; k < STATICDET_NUM_ADOA_SET; k++)
                DPU_StaticDetProcDSP_heatmapGen(cfg, &aDOA_indList[k*(STATICDET_NUM_ADOA_ANT << STATICDET_HEATMAPGEN_NOTCH_ENABLE)], cfg->hwRes.highDopplerOut, STATICDET_NUM_ADOA_ANT, cfg->hwRes.heatmapLocal);
            // average over STATICDET_RECORDING_NUMFRAMETOAVG frames
            if (cfg->staticCfg.recordingFrameCnt == 0)
            {
			    memset((void *)&cfg->hwRes.heatmapAvg[0], 0, cfg->hwRes.steeringVecSize*sizeof(float));
            }

            for (k = 0; k < cfg->hwRes.steeringVecSize; k++)
                cfg->hwRes.heatmapAvg[k] = cfg->hwRes.heatmapAvg[k] + (cfg->hwRes.heatmapLocal[k] * STATICDET_RECORDING_NUMFRAMETOAVGINV);
            
            // find the peak to use as heatmapNoiseLevel
            if (cfg->staticCfg.recordingFrameCnt == (STATICDET_RECORDING_NUMFRAMETOAVG - 1))
            {
                cfg->staticCfg.noiseLevel_record = 0;
                for (k = 0; k < cfg->hwRes.steeringVecSize; k++)
                    if (cfg->hwRes.heatmapAvg[k] > cfg->staticCfg.noiseLevel_record)
                        cfg->staticCfg.noiseLevel_record = cfg->hwRes.heatmapAvg[k];
            }


        }


    } /* for loop rangeIdx 0 to cfg->staticCfg.numRangeBins */
    

    if ((cfg->staticCfg.recordingMode == 1) )
	{
        cfg->staticCfg.recordingFrameCnt ++;
        outParams->numDetectedPoints = 0;
        cfg->staticCfg.recordingHeatmapReady = 0;
        if (cfg->staticCfg.recordingFrameCnt == STATICDET_RECORDING_NUMFRAMETOAVG)
            cfg->staticCfg.recordingHeatmapReady = 1;
	}
	else
	{
		totalNumDet = DPU_StaticDetProcDSP_detection(cfg);
		outParams->numDetectedPoints = totalNumDet; 
		
		/*Update stats*/
		outParams->stats.numProcess++;
		outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitingTime;
		outParams->stats.waitTime = waitingTime;
    }
    if (cfg->staticCfg.recordingHeatmapReady == 1)
        cfg->staticCfg.recordingMode = 0;

exit:
    if (obj != NULL)
    {
        obj->inProgress = false;
    }    
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      This the DPU control function. 
 *
 *
 *  @param[in]  handle           DPU handle
 *  @param[in]  cmd              DPU control command
 *  @param[in]  arg              DPU control argument pointer
 *  @param[in]  argSize          DPU control argument size
 *
 *  \ingroup    DPU_STATICDETPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_StaticDetProcDSP_control
(
    DPU_StaticDetProcDSP_Handle   handle,
    DPU_StaticDetProcDSP_Cmd      cmd,
    void*                       arg,
    uint32_t                    argSize
)
{
    int32_t                retVal = 0;
    DPU_StaticDetProcDSP_Obj *obj;

    obj = (DPU_StaticDetProcDSP_Obj *)handle;
    
    if (obj == NULL)
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
        goto exit;
    }    
    
    /* Check if control() is called during processing time */
    if(obj->inProgress == true)
    {
        retVal = DPU_STATICDETPROCDSP_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        default:
            retVal = DPU_STATICDETPROCDSP_ECMD;
            break;
    }
    
exit:
    return (retVal);
}

/**
  *  @b Description
  *  @n
  *  StaticDet DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_STATICDETPROC_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_STATICDETPROC_ERROR_CODE
  */
int32_t DPU_StaticDetProcDSP_deinit(DPU_StaticDetProcDSP_Handle handle)
{
    int32_t     retVal = 0;
    
    /* Free memory */
    if(handle == NULL)
    {
        retVal = DPU_STATICDETPROCDSP_EINVAL;
    }
    else
    {
        MemoryP_ctrlFree(handle, sizeof(DPU_StaticDetProcDSP_Obj));
    }
    
    return retVal;
}
