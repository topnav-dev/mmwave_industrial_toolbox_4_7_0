/**
 *   @file  aoaprocdsp.c
 *
 *   @brief
 *      Implements AoA DPU on DSP.
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

/* mmWave SDK drivers/common Include Files */
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/MemoryP.h>

/* mmWave lib*/
#include <ti/alg/mmwavelib/mmwavelib.h>

/* Data Path Include Files */
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/utils/mathutils/mathutils.h>

/* Industrial Lab Include */
#include <common/src/dpu/aoaproc/include/aoaprocdsp_internal.h>

/* C64P dsplib (fixed point part for C674X) */
#include "DSP_fft32x32.h"


/*===========================================================
 *                    Internal Functions
 *===========================================================*/
/**
 *   @b Description
 *   @n
 *      AoAProc Rx channel phase/gain compensation. The function performs phase/gain
 *      compensation on the Rx virtual antenna symbols of the detected object. The
 *      function is also used to perform the phase/gain compensation on the two dimensional
 *      array of only azimuth antennas which is used for the azimuth heat map display
 *      on the host. In this case the two dimensional array is laid out as
 *      X[number of range bins][number of azimuth antennas].
 *
 * @param[in]      rxChComp Rx channel phase/gain compensation coefficients
 *
 * @param[in]      numAnt number of antennas
 *
 * @param[in]      symbolsIn Pointer to array of input symbols
 *
 * @param[out]     symbolsOut Pointer to array of output symbols
 *
 * @return         void
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static void AoAProcDSP_rxChanPhaseBiasCompensation(cmplx16ImRe_t *rxChComp,
                                                   uint32_t numAnt,
                                                   cmplx32ReIm_t *symbolsIn,
                                                   cmplx32ReIm_t *symbolsOut)
{  
    /*Note that input/output symbols are in ReIm format and rxChComp coefficients are in ImRe format*/
    mmwavelib_vecmul16x32_anylen((uint32_t *)rxChComp, /*ImRe*/  
                                 (int64_t *)symbolsIn, /*ReIm*/ 
                                 (int64_t *)symbolsOut,/*ReIm*/ 
                                 numAnt);
}

/**
 *  @b Description
 *  @n
 *      This function implements static clutter removal.
 *
 *  @param[in]  cfg             DPU static configuration.
 *  @param[in]  inputBuf        input buffer. Clutter removal algorithm will work on this data.
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval N/A
 */
static void AoAProcDSP_clutterRemoval
(
    DPU_AoAProcDSP_StaticConfig *cfg,
    cmplx16ImRe_t               *inputBuf
)
{
    uint32_t sumVal[2]={0};
    cmplx32ImRe_t *pSumVal  = (cmplx32ImRe_t *) sumVal;
    uint32_t meanVal = 0;
    cmplx16ImRe_t *pMeanVal = (cmplx16ImRe_t *) &meanVal;
    
    /*Mmwavelib API requires size to be multiple of 4, which is fine because DPU allows 
      only numDopplerChirps that is multiple of 4*/
    mmwavelib_vecsum((int16_t *) inputBuf,
                     (int32_t *) sumVal,
                     (int32_t)   cfg->numDopplerChirps);

    /*Divide by numDopplerChirps as the scratch buffer was padded to zero to make it a 
      multiple of 4 so it does not affect the average*/    
    pMeanVal->real = pSumVal->real / cfg->numDopplerChirps ;
    pMeanVal->imag = pSumVal->imag / cfg->numDopplerChirps ;
    
    /*Lib requires size to be multiple of 4*/
    mmwavelib_vecsubc((int16_t *) inputBuf,
                      (int16_t *) inputBuf,
                      (uint32_t)  meanVal,
                      (int32_t)   cfg->numDopplerChirps);
}

/**
 *  @b Description
 *  @n
 *      The function configures input EDMA to transfer data pertinent to one virtual antenna
 *      at a time. The same EDMA programing is used for both heatmap computation and 
 *      AoA estimation.
 *
 *  @param[in]  aoaDspCfg     Pointer to DPU configuration
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static inline int32_t AoAProcDSP_configEDMA
(
    DPU_AoAProcDSP_Config    *aoaDspCfg
)
{
    int32_t             retVal = EDMA_NO_ERROR;
    uint16_t            sampleLenInBytes = sizeof(cmplx16ImRe_t);
    DPEDMA_syncABCfg    syncABCfg;
    cmplx16ImRe_t      *radarCubeBase;
    DPU_AoAProcDSP_HW_Resources *res;
    DPU_AoAProcDSP_StaticConfig *DPParams;

    if(aoaDspCfg == NULL)
    {
        retVal = DPU_AOAPROCDSP_EINVAL;
        goto exit;
    }

    res = &aoaDspCfg->res;
    DPParams = &aoaDspCfg->staticCfg;    
    radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;

    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer data from Radar cube to input buffer (ping)
    ******************************************************************************************/   
    syncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    syncABCfg.destAddress = (uint32_t)(&res->pingPongBuf[0]);
    syncABCfg.aCount      = sampleLenInBytes;
    syncABCfg.bCount      = DPParams->numDopplerChirps;
    syncABCfg.cCount      = 1;/*EDMA Source address is re-programmed for every virtual antenna*/
    syncABCfg.srcBIdx     = DPParams->numRxAntennas * DPParams->numRangeBins * sampleLenInBytes;
    syncABCfg.dstBIdx     = sampleLenInBytes;
    syncABCfg.srcCIdx     = 0U;    
    syncABCfg.dstCIdx     = 0U;

    retVal = DPEDMA_configSyncAB(res->edmaHandle,
                                 &res->edmaPing,
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
    syncABCfg.destAddress = (uint32_t)(&res->pingPongBuf[DPParams->numDopplerChirps]);

    retVal = DPEDMA_configSyncAB(res->edmaHandle,
                                 &res->edmaPong,
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

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      This function waits for an input (from radar cube to scratch buffer)
 *      EDMA transfer to complete.
 *
 *  @param[in]  res         DPU resources configuration.
 *  @param[in]  pingPongId  Ping/pong ID
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       !=0
 */
static inline int32_t AoAProcDSP_waitInData(DPU_AoAProcDSP_HW_Resources *res, uint32_t pingPongId)
{
    /* wait until transfer done */
    volatile bool isTransferDone;
    uint8_t       chId;
    int32_t       retVal = 0;
    
    if(pingPongId == DPU_AOAPROCDSP_PING_IDX)
    {
        chId = res->edmaPing.channel;
    }
    else
    {
        chId = res->edmaPong.channel;
    }
    do 
    {
        retVal = EDMA_isTransferComplete(res->edmaHandle,
                                        chId,
                                        (bool *)&isTransferDone);
        if(retVal != EDMA_NO_ERROR)
        {
            return retVal;
        }
    } while (isTransferDone == false);
    
    return 0;
}
#if !defined (AOA_TYPE_2D)
/**
 *  @b Description
 *  @n
 *      This function is called per object and it calculates its x/y/z coordinates
 *      based on Azimuth/elevation FFT outputs. It stores the coordinates
 *      to the output list of type @ref DPIF_PointCloudCartesian_t and it also
 *      calculates object's SNR and the noise level and writes into the side
 *      information list of type @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] aoaDspObj Pointer to AoA DPU internal object
 *
 *  @param[in] objOutIdx Index of the current object in the output lists @ref DPIF_PointCloudCartesian_t and @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] objInCfarIdx Index of the current object in CFAR output list @ref DPIF_CFARDetList_t
 *
 *  @param[in] maxIdx Peak position in the FFT output of azimuth antennas
 *
 *  @param[in] peakVal Value of the FFT output of azimuth antennas
 *
 *  @retval objOutIdx Index for the next object in the output list
 *
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static uint32_t AoAProcDSP_XYZestimation
(
    AOADspObj   *aoaDspObj,
    uint32_t    objOutIdx,
    uint32_t    objInCfarIdx,
    uint32_t    maxIdx,
    uint32_t    peakVal
)
{
    int32_t         sMaxIdx;
    float           temp;
    float           Wx, Wz;
    float           phi, theta;
    float           range;
    float           limitScale;
    float           x, y, z;
    float           peakAzimRe, peakAzimIm, peakElevRe, peakElevIm;
    DPU_AoAProcDSP_HW_Resources  *res = &aoaDspObj->res;
    DPIF_CFARDetList  *objIn = res->cfarRngDopSnrList;
    DPIF_PointCloudCartesian *objOut = res->detObjOut;
    DPIF_PointCloudSpherical *objOutSph = res->detObjOutSph;
    DPIF_PointCloudSideInfo *objOutSideInfo = res->detObjOutSideInfo;

    DPU_AoAProcDSP_StaticConfig *params = &aoaDspObj->params;
    uint16_t numDopplerBins = params->numDopplerBins;
    cmplx32ReIm_t *azimuthFftOut = (cmplx32ReIm_t *)res->scratch1Buff;

    range = objIn[objInCfarIdx].rangeIdx * params->rangeStep;

    /* Compensate for range bias */
    range -= aoaDspObj->dynLocalCfg.compRxChanCfg.rangeBias;
    if (range < 0)
    {
        range = 0;
    }

    if(maxIdx > (DPU_AOAPROCDSP_NUM_ANGLE_BINS/2 -1))
    {
        sMaxIdx = maxIdx - DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    }
    else
    {
        sMaxIdx = maxIdx;
    }

    Wx = 2 * (float) sMaxIdx / DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    /* Check if it is within configured field of view */

    x = range * Wx;

    if (params->numVirtualAntElev > 0)
    {
        peakAzimIm = (float) azimuthFftOut[maxIdx].imag;
        peakAzimRe = (float) azimuthFftOut[maxIdx].real;
        peakElevIm = (float) res->elevationFftOut[maxIdx].imag;
        peakElevRe = (float) res->elevationFftOut[maxIdx].real;
        
        Wz = atan2(peakAzimIm * peakElevRe - peakAzimRe * peakElevIm,
                   peakAzimRe * peakElevRe + peakAzimIm * peakElevIm)/PI_ + (2 * Wx);

        if (Wz > 1)
        {
            Wz = Wz - 2;
        }
        else if (Wz < -1)
        {
            Wz = Wz + 2;
        }
        /* Check if it is within configured field of view */
        if((Wz < aoaDspObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal) || (Wz > aoaDspObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal))
        {
            goto exit;
        }

        z = range * Wz;
        /*record wz for debugging/testing*/
        res->detObjElevationAngle[objOutIdx] = Wz;
        limitScale = sqrt(1 - Wz*Wz);
    }
    else
    {
        z = 0;
        limitScale = 1;
    }

    if((Wx < (limitScale * aoaDspObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal)) ||
       (Wx > (limitScale * aoaDspObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal)))
    {
        goto exit;
    }

    /*@TODO MMWAVESDK-1868:BEGIN */

    /* Elevation angle */
    phi = asin(Wz);

    /* Azimuth angle */
    theta = asin(Wx/(cos(phi)));
    /*@TODO MMWAVESDK-1868:END */

    temp = range*range -x*x -z*z;
    if (temp > 0)
    {
        y = sqrt(temp);
    }
    else
    {
        goto exit;
    }

    objOut[objOutIdx].x = x;
    objOut[objOutIdx].y = y;
    objOut[objOutIdx].z = z;

    objOut[objOutIdx].velocity = params->dopplerStep *
        AOA_DOPPLER_IDX_TO_SIGNED(objIn[objInCfarIdx].dopplerIdx, numDopplerBins);
    objOutSideInfo[objOutIdx].noise = objIn[objInCfarIdx].noise;
    objOutSideInfo[objOutIdx].snr = objIn[objInCfarIdx].snr;
    res->detObj2dAzimIdx[objOutIdx] = maxIdx;

   /*@TODO MMWAVESDK-1868:BEGIN */
    objOutSph[objOutIdx].range = range;
    objOutSph[objOutIdx].azimuthAngle = theta;
    objOutSph[objOutIdx].elevAngle = phi;
    objOutSph[objOutIdx].velocity = objOut[objOutIdx].velocity;
    /*@TODO MMWAVESDK-1868:END */

    objOutIdx++;

exit:
    return (objOutIdx);
}
#endif
/**
 *   @b Description
 *   @n
 *      this function calculates doppler compensation index for a given Doppler
 *      index of the object. It first converts Doppler index to the signed value
 *      and then divides by number of Tx antennas (number TDM MIMO Tx antennas).
 *      The type of return value is float.
 *
 *  @param[in]  dopplerIdx           Doppler index 
 *
 *  @param[in]  numDopplerBins       Number of Doppler bins
 *
 *  @param[in]  numTxAnt             Number of Tx Antennas
 *
 *  @return Doppler compensation index
 *
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static float AoAProcDSP_calcCompIdx
(
    uint16_t dopplerIdx,
    uint32_t numDopplerBins,
    uint32_t numTxAnt
)
{
    float      dopplerCompensationIdx;

    /* Doppler compensation index calculation */
    if (dopplerIdx >= numDopplerBins/2)
    {
        dopplerCompensationIdx =  ((int32_t) dopplerIdx - (int32_t) numDopplerBins);
    }
    else
    {
        dopplerCompensationIdx =  dopplerIdx;
    }
    
    /* Doppler phase correction is 1/2 or (1/3 in elevation case) of the phase between two chirps of the same antenna */
    dopplerCompensationIdx = dopplerCompensationIdx / (float) numTxAnt;
    if (dopplerCompensationIdx < 0)
    {
        dopplerCompensationIdx +=  (float) numDopplerBins;
    }

    return dopplerCompensationIdx;
}

/*!
 *  @b Description
 *  @n
 *      This function performs Doppler compensation on a single antenna symbol.
 *
 *  @param[in]  in           Pointer to the Input Symbol 
 *
 *  @param[in]  out          Pointer to the Output Symbol
 *
 *  @param[in]  Cos          Cosine value depending on doppler index
 *
 *  @param[in]  Sin          Sine value depending on doppler index
 *
 *  @retval None
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static void AoAProcDSP_dopplerComp
(
    cmplx32ReIm_t *in,
    cmplx32ReIm_t *out,
    float  Cos,
    float  Sin
)
{
    float           yRe, yIm;

    /* Rotate symbol (correct the phase) */
    yRe = in->real * Cos + in->imag * Sin;
    yIm = in->imag * Cos - in->real * Sin;
    out->real = (int32_t) yRe;
    out->imag = (int32_t) yIm;
}

/**
 *   @b Description
 *   @n
 *      Function performs Doppler compensation on antenna symbols.
 *
 *
 *  @param[in]  numDetectedObjects  Number of detected objects
 *
 *  @param[in]  srcPtr              Input pointer to antenna symbols
 *
 *  @param[in]  cfarOutList         CFAR detection list
 *
 *  @param[in]  dstPtr              Output pointer to antenna symbols
 *
 *  @param[in]  numTxAnt            Number of Tx antennas
 *
 *  @param[in]  numRxAnt            Number of physical Rx antennas
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual azimuth Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual elevation Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @return None
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static void AoAProcDSP_dopplerCompensation
(
    uint32_t numDetectedObjects,
    cmplx32ReIm_t *srcPtr,
    DPIF_CFARDetList *cfarOutList,
    cmplx32ReIm_t *dstPtr,
    uint32_t numTxAnt,
    uint32_t numRxAnt,
    uint32_t numVirtualAntAzim,
    uint32_t numVirtualAntElev,
    uint32_t numDopplerBins
)
{
    uint32_t   index;
    uint32_t   j;
    uint16_t   dopplerIdx;
    float      dopplerCompensationIdx;
    
    for(index = 0; index < numDetectedObjects; index ++)
    {
        dopplerIdx = cfarOutList->dopplerIdx;

        /* First set of virtual antennas (corresponding to chirp of antenna TX0), nothing needs to be done,
           just advance the pointers to the next set of symbols for TX1. */
        dstPtr += numRxAnt;
        srcPtr += numRxAnt;
        
        /* Do we have more virtual antennas? If so need to do doppler compensation
           for TX1 and TX2. */
        if (numVirtualAntAzim > numRxAnt)
        {
            float Cos,Sin;
            
            dopplerCompensationIdx = AoAProcDSP_calcCompIdx(dopplerIdx, numDopplerBins, numTxAnt);

            Cos = cos(2 * PI_ * dopplerCompensationIdx / numDopplerBins);
            Sin = sin(2 * PI_ * dopplerCompensationIdx / numDopplerBins);

            /* Compensate for virtual antennas corresponding to  Tx1*/ 
            for(j = numRxAnt; j < numVirtualAntAzim; j++)
            {
                AoAProcDSP_dopplerComp(srcPtr++, dstPtr++, Cos, Sin);
            }

            if (numVirtualAntElev > 0)
            {
                /* Compensate for virtual antennas corresponding to Tx2 (elevation)*/ 
                float Cos2, Sin2;
                /* Doppler phase shift is 2/3 */
                Cos2 = Cos * Cos - Sin * Sin;
                Sin2 = 2 * Cos * Sin;
                for(j = 0; j < numVirtualAntElev; j++)
                {
                    AoAProcDSP_dopplerComp(srcPtr++, dstPtr++, Cos2, Sin2);
                }
            }
        }
        cfarOutList++;
    }
}            

#if !defined (AOA_TYPE_2D)
/**
 *  @b  Description
 *  @n
 *      Function calculates x/y/z coordinates of the objects detected by CFAR DPU.
 *      The number of output objects may be larger than input if multiobject beam
 *      forming is enabled, and more than one objects are detected at the
 *      same range/doppler bin.
 *
 *  @param[in]  aoaDspObj          Pointer to AoAProc DPU internal data Object
 *
 *  @param[in]  objInCfarIdx       Index of the current object in CFAR output list @ref DPIF_CFARDetList_t
 *
 *  @param[in]  objOutIdx          Index to the next object in the output list
 *
 *  @retval     objOutIdx          Number of detected objects in the output list
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static inline uint32_t AoAProcDSP_angleEstimationAzimElev
(
    AOADspObj       *aoaDspObj,
    uint32_t        objInCfarIdx,
    uint32_t        objOutIdx
)
{
    DPU_AoAProcDSP_HW_Resources *res = &aoaDspObj->res;
    uint16_t maxNumObj = res->detObjOutMaxSize;
    float maxVal;
    int32_t maxIdx;
    cmplx32ReIm_t *azimuthFftOut = (cmplx32ReIm_t *)res->scratch1Buff;
    float *azimuthMagSqr = (float *)res->scratch2Buff;


    /* Calculate power(Magnitude Squared) of complex samples of the azimuth FFT output,
    and also find azimuth index which is the position with the maximum power. */
    maxIdx = mmwavelib_powerAndMax((int32_t*)&azimuthFftOut[0],
                                        DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                                        &azimuthMagSqr[0],
                                        &maxVal);

    /* Estimate x,y,z */
    objOutIdx = AoAProcDSP_XYZestimation(aoaDspObj,
                                         objOutIdx,
                                         objInCfarIdx,
                                         maxIdx,
                                         maxVal);
                                         
    if (objOutIdx >= maxNumObj)
    {
        return(objOutIdx);
    }
    
    /* Multi peak azimuzth search?*/
    if (aoaDspObj->dynLocalCfg.multiObjBeamFormingCfg.enabled)
    {
        float maxVal2;
        uint16_t azimIdx;
        float* azimuthMag = (float *)res->scratch2Buff;
        
        azimIdx = mmwavelib_secondPeakSearch(azimuthMag,
                                             DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                                             maxIdx);
                                             
        maxVal2 = azimuthMag[azimIdx];
                
        /* Is second peak greater than threshold? */
        if ( (maxVal2 >( ((maxVal * aoaDspObj->dynLocalCfg.multiObjBeamFormingCfg.multiPeakThrsScal)))) && (objOutIdx < maxNumObj) )
        {

            /* Estimate x,y,z for second peak */
            objOutIdx = AoAProcDSP_XYZestimation(aoaDspObj,
                                                 objOutIdx,
                                                 objInCfarIdx,
                                                 azimIdx,
                                                 maxVal2);
        }
    }
    
    return(objOutIdx);
}
#else
/**
 *  @b  Description
 *  @n
 *      Function calculates x/y/z coordinates of the objects detected by CFAR DPU
 *      using 2D DoA method.
 *
 *  @param[in]  aoaDspObj          Pointer to AoAProc DPU internal data Object
 *
 *  @param[in]  objInCfarIdx       Index of the current object in CFAR output list @ref DPIF_CFARDetList_t
 *
 *  @param[in]  objOutIdx          Index to the next object in the output list
 *
 *  @retval     objOutIdx          Number of detected objects in the output list
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static inline uint32_t AoAProcDSP_angleEstimationAzimElev_2dDoa
(
    AOADspObj       *aoaDspObj,
    uint32_t        objInCfarIdx,
    uint32_t        objOutIdx
)
{
    DPU_AoAProcDSP_HW_Resources *res = &aoaDspObj->res;
    cmplx32ReIm_t *azimuthFftOut = (cmplx32ReIm_t *)res->scratch1Buff;
	DOA_2D_t *doa_2D =  (DOA_2D_t *)res->scratch3Buff;

    float mag_sqr;
    float range;
    float maxVal = 0;
    double theta,phi,az_freq,el_freq;
    float x,y,z;
    int16_t antIndx;
    int32_t fft_2D_peak_row_idx,fft_2D_peak_col_idx;
    int16_t row_idx,col_idx;
    cmplx32ReIm_t temp_rearrange[12];
    cmplx32ReIm_t cmplx_zero;

    DPIF_CFARDetList  *objIn = res->cfarRngDopSnrList;
    DPIF_PointCloudCartesian *objOut = res->detObjOut;
    DPIF_PointCloudSpherical *objOutSph = res->detObjOutSph;
    DPIF_PointCloudSideInfo *objOutSideInfo = res->detObjOutSideInfo;

    DPU_AoAProcDSP_StaticConfig *params = &aoaDspObj->params;
    uint16_t numDopplerBins = params->numDopplerBins;

    range = objIn[objInCfarIdx].rangeIdx * params->rangeStep;

    /* Compensate for range bias */
    range -= aoaDspObj->dynLocalCfg.compRxChanCfg.rangeBias;
    if (range < 0)
    {
        range = 0;
    }
    
    cmplx_zero.real = 0;
    cmplx_zero.imag = 0;

    /* Initialize the 2D-DOA complex array */
    for (row_idx = 0; row_idx < DPU_AOAPROCDSP_NUM_ANGLE_BINS; row_idx++)
    {
      for (col_idx = 0; col_idx < DPU_AOAPROCDSP_NUM_ANGLE_BINS; col_idx++)
      {
          doa_2D->DOA_2D_storage[row_idx][col_idx].real = cmplx_zero.real; 
          doa_2D->DOA_2D_storage[row_idx][col_idx].imag = cmplx_zero.imag; 
      }
    }

    /* Store the 2D-FFT output for the detected object across the virtual antennas */
    for (antIndx = 0; antIndx < (params->numTxAntennas * params->numRxAntennas); antIndx++)
    {
        temp_rearrange[antIndx]= res->angleFftIn[antIndx];
    }

#if defined (ANT_TYPE_ODS)
    /*Arrnage the complex output across virtual antennas in 2D grid based on 6843_ODS antenna placement*/
    /* Channels are arranged as (because the chirps are sent in this order: TX1, TX2 and TX3)
     * ch-0   ch-3   ch-4   ch-7
     * ch-1   ch-2   ch-5   ch-6
     *               ch-8   ch-11
     *               ch-9   ch-10
     */
    doa_2D->DOA_2D_storage[0][0]= temp_rearrange[0];
    doa_2D->DOA_2D_storage[0][1]= temp_rearrange[3];
    doa_2D->DOA_2D_storage[0][2]= temp_rearrange[4];
    doa_2D->DOA_2D_storage[0][3]= temp_rearrange[7];
    
    doa_2D->DOA_2D_storage[1][0]= temp_rearrange[1];
    doa_2D->DOA_2D_storage[1][1]= temp_rearrange[2];
    doa_2D->DOA_2D_storage[1][2]= temp_rearrange[5];
    doa_2D->DOA_2D_storage[1][3]= temp_rearrange[6];

    doa_2D->DOA_2D_storage[2][0]= cmplx_zero;
    doa_2D->DOA_2D_storage[2][1]= cmplx_zero;
    doa_2D->DOA_2D_storage[2][2]= temp_rearrange[8];
    doa_2D->DOA_2D_storage[2][3]= temp_rearrange[11];

    doa_2D->DOA_2D_storage[3][0]= cmplx_zero;
    doa_2D->DOA_2D_storage[3][1]= cmplx_zero;
    doa_2D->DOA_2D_storage[3][2]= temp_rearrange[9];
    doa_2D->DOA_2D_storage[3][3]= temp_rearrange[10];
#elif defined(ANT_TYPE_AOP)
    /*Arrange the complex output across virtual antennas in 2D grid based on 6843_AOP ES2 antenna placement*/
    /* Channels are arranged as (because the chirps are sent in this order: TX1, TX2 and TX3)
     *
     * 
     * ch-3   ch-1
     * ch-2   ch-0
     * ch-11  ch-9   ch-7   ch-5
     * ch-10  ch-8   ch-6   ch-4
     */
    doa_2D->DOA_2D_storage[0][0]= temp_rearrange[3];
    doa_2D->DOA_2D_storage[0][1]= temp_rearrange[1];
    doa_2D->DOA_2D_storage[0][2]= cmplx_zero;
    doa_2D->DOA_2D_storage[0][3]= cmplx_zero;
    
    doa_2D->DOA_2D_storage[1][0]= temp_rearrange[2];
    doa_2D->DOA_2D_storage[1][1]= temp_rearrange[0];
    doa_2D->DOA_2D_storage[1][2]= cmplx_zero;
    doa_2D->DOA_2D_storage[1][3]= cmplx_zero;

    doa_2D->DOA_2D_storage[2][0]= temp_rearrange[11];
    doa_2D->DOA_2D_storage[2][1]= temp_rearrange[9];
    doa_2D->DOA_2D_storage[2][2]= temp_rearrange[7];
    doa_2D->DOA_2D_storage[2][3]= temp_rearrange[5];

    doa_2D->DOA_2D_storage[3][0]= temp_rearrange[10];
    doa_2D->DOA_2D_storage[3][1]= temp_rearrange[8];
    doa_2D->DOA_2D_storage[3][2]= temp_rearrange[6];
    doa_2D->DOA_2D_storage[3][3]= temp_rearrange[4];    
#else
#error "Antenna type define is needed with AOA_TYPE_2D. Valid options: ANT_TYPE_ODS or ANT_TYPE_AOP"
#endif 
    /* 1D FFT on azimuth array of virtual antennas */
    for(row_idx=0; row_idx<4; row_idx++)
    {
       for (col_idx=0;col_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;col_idx++)
       {
           res->angleFftIn[col_idx]= doa_2D->DOA_2D_storage[row_idx][col_idx];
       }

       DSP_fft32x32((int32_t *)res->angleTwiddle32x32,
                    DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                    (int32_t *) &res->angleFftIn[0],
                    (int32_t *) &azimuthFftOut[0]);

       for (col_idx=0;col_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;col_idx++)
       {
           doa_2D->DOA_2D_storage[row_idx][col_idx] = azimuthFftOut[col_idx];
       }
   }

    /* 1D FFT on eleavtion array of virtual antennas */
    for (col_idx=0;col_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;col_idx++)
    {
        for (row_idx=0;row_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;row_idx++)
        {
            res->angleFftIn[row_idx]= doa_2D->DOA_2D_storage[row_idx][col_idx];
        }

        DSP_fft32x32((int32_t *)res->angleTwiddle32x32,
                    DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                    (int32_t *) &res->angleFftIn[0],
                    (int32_t *) &azimuthFftOut[0]);

        for (row_idx=0;row_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;row_idx++)
        {
            doa_2D->DOA_2D_storage[row_idx][col_idx] = azimuthFftOut[row_idx];
        }

   }
   
   /*Find the peak value in the 2D-DOA */
    for (row_idx=0;row_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;row_idx++)
    {
        for (col_idx=0;col_idx<DPU_AOAPROCDSP_NUM_ANGLE_BINS;col_idx++)
        {
            mag_sqr = (float) (doa_2D->DOA_2D_storage[row_idx][col_idx]).real * (float) (doa_2D->DOA_2D_storage[row_idx][col_idx]).real +
                (float) (doa_2D->DOA_2D_storage[row_idx][col_idx]).imag * (float) (doa_2D->DOA_2D_storage[row_idx][col_idx]).imag;
                
            if (mag_sqr > maxVal)
            {
                fft_2D_peak_row_idx = row_idx;
                fft_2D_peak_col_idx = col_idx;
                maxVal = mag_sqr;
            }
        }
    }
    
    /* convert the peak indices b/w [-Fs/2, Fs/2]*/
    if (fft_2D_peak_row_idx > (DPU_AOAPROCDSP_NUM_ANGLE_BINS >> 1))
    {
        fft_2D_peak_row_idx -= DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    }

    if (fft_2D_peak_col_idx > (DPU_AOAPROCDSP_NUM_ANGLE_BINS >> 1))
    {
        fft_2D_peak_col_idx -= DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    }
    
    /* Based on detected peak indices,  compute the azimuth and elevation frequency corresponding to peak value */
    az_freq = ((double) fft_2D_peak_col_idx) * 2.0f * (PI_/DPU_AOAPROCDSP_NUM_ANGLE_BINS);
    el_freq = ((double) fft_2D_peak_row_idx) * 2.0f * (PI_/DPU_AOAPROCDSP_NUM_ANGLE_BINS);

    /*Compute the elevation angle */
    phi= asin(el_freq/PI_);
 
    /*Check if azimuth angle can be computed or not */
    if (abs(az_freq/(cos(phi))) <= PI_)
    {
        theta = asin(az_freq/(PI_ * cos(phi)));
    }
    else
    {
        goto exit;
    }

    /* Compute (x,y,z) cordinates of the detected object */
    x = range*sin(theta)*cos(phi);
    y = range*cos(theta)*cos(phi);
    z = range*sin(phi);

    objOut[objOutIdx].x = x;
    objOut[objOutIdx].y = y;
    objOut[objOutIdx].z = z;
    objOut[objOutIdx].velocity = params->dopplerStep *
        AOA_DOPPLER_IDX_TO_SIGNED(objIn[objInCfarIdx].dopplerIdx, numDopplerBins);
    
    objOutSideInfo[objOutIdx].noise = objIn[objInCfarIdx].noise;
    objOutSideInfo[objOutIdx].snr = objIn[objInCfarIdx].snr;

   /*@TODO MMWAVESDK-1868:BEGIN */
    objOutSph[objOutIdx].range = range;
    objOutSph[objOutIdx].azimuthAngle = theta;
    objOutSph[objOutIdx].elevAngle = phi;
    objOutSph[objOutIdx].velocity = objOut[objOutIdx].velocity;
    /*@TODO MMWAVESDK-1868:END */

    objOutIdx++;

exit:
    return(objOutIdx);
}
#endif    
/**
 *  @b  Description
 *  @n
 *      Function decodes BPM during the heatmap computation. Decoding is done in place.
 *
 *  @param[in]  bpmAPtr         Pointer to input ping buffer with TX0+TX1 BPM signal
 *
 *  @param[in]  bpmBPtr         Pointer to input pong buffer with TX0-TX1 BPM signal
 *
 *  @param[in]  numSymbols      Number of symbols to be decoded
 *
 *  @retval     None. 
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static inline void AoAProcDSP_azimuthHeatMapDecodeBPM
(
    cmplx16ImRe_t   *bpmAPtr,/*ping*/
    cmplx16ImRe_t   *bpmBPtr,/*pong*/
    uint16_t         numSymbols
)
{
    int16_t       real, imag;
    uint16_t      idx;

    /*We now have Ping and Pong available.
      Do BPM decoding in place:
      A=S1+S2 (PING) 
      B=S1-S2 (PONG)
      Compute S1=(A+B)/2 and overwrite A with this value 
      Compute S2=(A-B)/2 and overwrite B with this value
    */                    
    for(idx = 0; idx < numSymbols; idx++)
    { 
        /* Note there is no need to do doppler correction because 
           heatmap is computed using doppler bin zero.*/

        /*first make a copy of A*/
        real = bpmAPtr[idx].real;
        imag = bpmAPtr[idx].imag;
        /*compute S1 and store in A*/
        bpmAPtr[idx].real = (bpmAPtr[idx].real + bpmBPtr[idx].real)/2;
        bpmAPtr[idx].imag = (bpmAPtr[idx].imag + bpmBPtr[idx].imag)/2;
        /*compute S2 and store in B*/
        bpmBPtr[idx].real = (real - bpmBPtr[idx].real)/2;
        bpmBPtr[idx].imag = (imag - bpmBPtr[idx].imag)/2;
    }
}

/**
 *  @b Description
 *  @n
 *      The function computes the heatmap value (Doppler bin zero) from the input
 *      radar cube symbols.
 *
 *  @param[in]    aoaObj        Pointer to internal AoAProc data object
 *  @param[in]    inputBuffer   Input symbols
 *  @param[in]    rxAntIdx      RX antenna index for the input symbols
 *  @param[in]    txAntIdx      TX antenna index for the input symbols
 *  @param[in]    rangeIdx      Range index for the input symbols
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval none
 */
static void AoAProcDSP_computeHeatMapVal
(
    AOADspObj     *aoaObj,
    cmplx16ImRe_t *inputBuffer,
    uint16_t      rxAntIdx,
    uint16_t      txAntIdx,
    uint16_t      rangeIdx
)
{
    cmplx32ImRe_t sumValImRe; 
    cmplx32ReIm_t sumValReIm; 
    uint16_t virtAntIdx, heatMapIdx;
        
    /* Sum all symbols to compute Doppler bin zero*/
    /* Lib requires size (numDopplerChirps) to be multiple of 4*/
    mmwavelib_vecsum((int16_t *) inputBuffer,
                     (int32_t *) &sumValImRe,
                     (int32_t)   aoaObj->params.numDopplerChirps);
    
    /*First swap from ImRe to ReIm*/                                                                      
    sumValReIm.real = sumValImRe.real;
    sumValReIm.imag = sumValImRe.imag;
    
    /*Index in bias compensation coefficient array: rxChPhaseComp[tx][rx]*/
    virtAntIdx = txAntIdx * SYS_COMMON_NUM_RX_CHANNEL + rxAntIdx; 
    
    /* RX channel compensation */
    AoAProcDSP_rxChanPhaseBiasCompensation(&aoaObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[virtAntIdx],
                                           1,
                                           &sumValReIm,
                                           &sumValReIm);
                                           
    /* Populate azimuthHeatMapArray = azimuthStaticHeatMap[numRangeBins][numVirtualAntAzim]*/
    heatMapIdx = virtAntIdx + rangeIdx * aoaObj->params.numVirtualAntAzim;    
    aoaObj->res.azimuthStaticHeatMap[heatMapIdx].real = (int16_t) (sumValReIm.real / aoaObj->params.numDopplerChirps);
    aoaObj->res.azimuthStaticHeatMap[heatMapIdx].imag = (int16_t) (sumValReIm.imag / aoaObj->params.numDopplerChirps);
}


/**
 *  @b Description
 *  @n
 *      The function generates the range-azimuth heat-map. 
 *      It computes bin zero of 2D-FFT for all azimuth virtual antennas  
 *      for all range bins.
 *
 *  @param[in]    aoaDspObj     Pointer to internal AoAProc data object
 *  @param[inout] waitingTime   Pointer to CPU waiting time
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static inline int32_t  AoAProcDSP_RangeAzimuthHeatmap(AOADspObj *aoaDspObj,
                                        volatile uint32_t *waitingTime)
{
    volatile uint32_t startTimeWait;
    int32_t retVal = 0;
    cmplx16ImRe_t *radarCubeBase;
    DPU_AoAProcDSP_HW_Resources *res;
    DPU_AoAProcDSP_StaticConfig *DPParams;
    uint16_t rxAntIdx, rangeIdx, txAntIdx, rxAntIdxPing, txAntIdxPing, numAzimTxAnt; 
    uint16_t nextTransferRxIdx, nextTransferRangeIdx, nextTransferTxIdx; 
    uint32_t pingPongIdx, nextTransferIdx;
    uint8_t  channel;
    cmplx16ImRe_t  *inpBuf;
    cmplx16ImRe_t *bpmPingBuff;

    res = &aoaDspObj->res;
    DPParams = &aoaDspObj->params;
    radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    
    /*Use scratch1Buff to temporarily store ping data until pong data is available.
      Note that scratch1Buff is always bigger than pingBuf and scratch1Buff is only used on the
      angle estimation so it is safe to use it here. This is needed for BPM only.*/
    bpmPingBuff = (cmplx16ImRe_t *)res->scratch1Buff;
    
    /*Compute number of azimuth antennas. */
    numAzimTxAnt = DPParams->numTxAntennas;
    if(DPParams->numVirtualAntElev > 0 )
    {
        numAzimTxAnt--;    
    }    
    
    /* Reset ping/pong index */
    pingPongIdx = DPU_AOAPROCDSP_PING_IDX;
    
    /* Trigger first DMA.
       Note: EDMA ping/pong scheme should cover only azimuth antennas and must bring antenna data
       for consecutive TX antennas such that BPM can be decoded with one set of ping/pong buffers
       without the need to store extra ping/pong buffers. 
       It should also support combinations of [1,2] TX antennas and [2,4] RX antennas.
       The scheme used here satisfies the requirements above and do not require reconfiguring the EDMA
       channels (except for source address). Therefore, once the DPU configures the EDMA channel, (one ping
       and one pong channel), the same channels can be used by the azimuth computation and AoA estimation
       by just changing the source addresses.       */
    EDMA_setSourceAddress(res->edmaHandle, 
                          res->edmaPing.channel, 
                          (uint32_t) &radarCubeBase[0]);
    
    EDMA_startDmaTransfer(res->edmaHandle, res->edmaPing.channel);
    
    for (rangeIdx = 0; rangeIdx < DPParams->numRangeBins; rangeIdx++)
    {
        for (rxAntIdx = 0; rxAntIdx < DPParams->numRxAntennas; rxAntIdx++)
        {
            for (txAntIdx = 0; txAntIdx < numAzimTxAnt; txAntIdx++)
            {
                /* verify that previous DMA has completed */
                startTimeWait = Cycleprofiler_getTimeStamp();
                retVal = AoAProcDSP_waitInData (res, pingPongIdx);
                if(retVal != 0)
                {
                    goto exit;
                }    
                *waitingTime += Cycleprofiler_getTimeStamp() - startTimeWait;
                
                /*Find index in radar cube for next EDMA.*/ 
                nextTransferTxIdx    = txAntIdx + 1;
                nextTransferRxIdx    = rxAntIdx;
                nextTransferRangeIdx = rangeIdx;
                
                if(nextTransferTxIdx == numAzimTxAnt)
                {
                   nextTransferTxIdx = 0;
                   nextTransferRxIdx++;
                   if(nextTransferRxIdx == DPParams->numRxAntennas)
                   {
                       nextTransferRxIdx = 0;
                       nextTransferRangeIdx++;
                   }
                }
                
                nextTransferIdx = (nextTransferTxIdx * DPParams->numRxAntennas * DPParams->numDopplerChirps + 
                                   nextTransferRxIdx) * DPParams->numRangeBins + nextTransferRangeIdx;
                
                /*Last computation happens when nextTransferRangeIdx reaches numRangeBins. 
                  This indicates that, the current virtual antenna is the last one for (numRangeBins-1). 
                  Therefore, do not trigger next EDMA.*/
                if(nextTransferRangeIdx < DPParams->numRangeBins)
                {
                    /* kick off next DMA */
                    if (pingPongIdx == DPU_AOAPROCDSP_PONG_IDX)
                    {
                        channel = res->edmaPing.channel;
                    }
                    else
                    {
                        channel = res->edmaPong.channel;
                    }
                    
                    EDMA_setSourceAddress(res->edmaHandle, channel,
                                         (uint32_t) &radarCubeBase[nextTransferIdx]);
                                         
                    EDMA_startDmaTransfer(res->edmaHandle, channel);
                }    
                
                inpBuf = (cmplx16ImRe_t *) &res->pingPongBuf[pingPongIdx * DPParams->numDopplerChirps];

                /* Remove static clutter? */
                if (aoaDspObj->dynLocalCfg.staticClutterCfg.isEnabled)
                {
                    AoAProcDSP_clutterRemoval(DPParams, inpBuf);
                }
                
                if(DPParams->isBpmEnabled)
                {
                    /*If BPM is enabled, need to store 2 sets of 
                      doppler bins (1 ping + 1 pong) so that BPM decoding can be done later on.
                      The order of the EDMA assures that consecutive ping/pong buffers have the correct
                      data for BPM decoding.*/
                    
                    if(pingPongIdx == DPU_AOAPROCDSP_PING_IDX)  
                    {
                        memcpy((void*)&bpmPingBuff[0], inpBuf, DPParams->numDopplerChirps * sizeof(cmplx16ImRe_t));
                                            
                        /*Store ping indexes to be used later when both ping and pong are available.*/
                        rxAntIdxPing = rxAntIdx;
                        txAntIdxPing = txAntIdx;
                    }                    
                    else
                    {
                        /*Decode BPM*/
                        AoAProcDSP_azimuthHeatMapDecodeBPM(bpmPingBuff,/*Ping data*/
                                                           inpBuf,     /*Pong data*/
                                                           DPParams->numDopplerChirps);
                                                               
                        /* Compute heatmap value from ping data*/
                        AoAProcDSP_computeHeatMapVal(aoaDspObj, 
                                                     bpmPingBuff,
                                                     rxAntIdxPing,
                                                     txAntIdxPing,
                                                     rangeIdx);
                        
                        /* Compute heatmap value from pong data*/
                        AoAProcDSP_computeHeatMapVal(aoaDspObj, 
                                                     inpBuf,
                                                     rxAntIdx,
                                                     txAntIdx,
                                                     rangeIdx);
                    }                    
                }
                else
                {/*BPM not enabled*/
                    
                        AoAProcDSP_computeHeatMapVal(aoaDspObj, 
                                                     inpBuf,
                                                     rxAntIdx,
                                                     txAntIdx,
                                                     rangeIdx);
                }
                        
                pingPongIdx ^= 1;
            }/*txAntIdx*/
        } /* rxAntIdx */
    }/*rangeIdx*/
    
exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function converts angle of arrival field of view values specified
 *      in degrees to the values appropriate for internal DPU comparison.
 *
 *  @param[in]  aoaDspObj    Pointer to internal AoAProc data object
 *
 *  @param[in]  fovAoaCfg    Pointer to field of view configuration
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval  None
 *
 */
static void AoAProcDSP_ConvertFov(AOADspObj *aoaDspObj,
                                  DPU_AoAProc_FovAoaCfg *fovAoaCfg)
{
    aoaDspObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal = sin(fovAoaCfg->minAzimuthDeg / 180. * PI_);
    aoaDspObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal = sin(fovAoaCfg->maxAzimuthDeg / 180. * PI_);
    aoaDspObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal = sin(fovAoaCfg->minElevationDeg / 180. * PI_);
    aoaDspObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal = sin(fovAoaCfg->maxElevationDeg  / 180. * PI_);
}

/**
 *  @b Description
 *  @n
 *      Generate Twiddle tables used by the DPU.
 *
 *  @param[in]  aoaDspCfg       DPU configuration.
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Not Applicable.
 */ 
static inline void AoAProcDSP_generateTables  
(
    DPU_AoAProcDSP_Config    *aoaDspCfg
)
{
    /* Generate twiddle factors for Doppler FFT. This is in ReIm format.*/
    mmwavelib_gen_twiddle_smallfft_32x32((int32_t *)aoaDspCfg->res.twiddle32x32, 
                                         aoaDspCfg->staticCfg.numDopplerBins);

    /* Generate twiddle factors for the angle FFT. This is in ReIm format.*/
    mmwavelib_gen_twiddle_smallfft_32x32((int32_t *)aoaDspCfg->res.angleTwiddle32x32, 
                                         DPU_AOAPROCDSP_NUM_ANGLE_BINS);
}

/**
 *  @b Description
 *  @n
 *    Decodes BPM 
 *
 * @param[inout]  virtAntennaBuf : Pointer to antenna symbols 
 *            
 * @param[in]  numRxAnt  : Number of RX antennas
 
 *  @retval
 *      Not Applicable.
 */
static inline void AoAProcDSP_decodeBPM(cmplx32ReIm_t *virtAntennaBuf, uint16_t numRxAnt)
{
    cmplx32ReIm_t *bpmAPtr;
    cmplx32ReIm_t *bpmBPtr;
    int32_t real, imag, idx;

    bpmAPtr  = &virtAntennaBuf[0];
    bpmBPtr  = &virtAntennaBuf[numRxAnt];
    
    for(idx = 0; idx < numRxAnt; idx++)
    {        
        /*BPM decoding*/                        
        /*store A*/
        real = bpmAPtr[idx].real;
        imag = bpmAPtr[idx].imag;
        /*compute S1 and store in place*/
        bpmAPtr[idx].real = (bpmAPtr[idx].real + bpmBPtr[idx].real)/2;
        bpmAPtr[idx].imag = (bpmAPtr[idx].imag + bpmBPtr[idx].imag)/2;
        /*compute S2 and store in place*/
        bpmBPtr[idx].real = (real - bpmBPtr[idx].real)/2;
        bpmBPtr[idx].imag = (imag - bpmBPtr[idx].imag)/2;
    }
}

/**
 *  @b Description
 *  @n
 *    AoA processing when there are more than one virtual antennas 
 *
 *  @param[in]    aoaDspObj   DPU object
 *  @param[in]    numObjsIn   Number of objects in CFAR list
 *  @param[out]   numObjsOut  Number of detected objects in the output list
 *  @param[inout] waitTime    CPU waiting time
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 *            
 */
static inline uint32_t AoAProcDSP_processMultipleVirtualAntennas
(
    AOADspObj                 *aoaDspObj,
    uint32_t                  numObjsIn,
    uint32_t                  *numObjsOut,
    volatile uint32_t         *waitTime
)
{
    uint16_t idx, rxAntIdx, rangeIdx, txAntIdx, dopplerIdx; 
    uint16_t nextTransferRxIdx, nextTransferRangeIdx, nextTransferTxIdx; 
    DPU_AoAProcDSP_HW_Resources *res;
    DPU_AoAProcDSP_StaticConfig *DPParams;
    cmplx16ImRe_t *inpDoppFftBuf;

    uint32_t  numObjsOutLocal = 0;
    volatile uint32_t startTimeWait;
    int32_t  retVal = 0;
    cmplx16ImRe_t  *radarCubeBase;
    uint32_t pingPongIdx, nextTransferIdx;
    uint8_t channel;
    cmplx32ReIm_t *windowingBuf;
    cmplx32ReIm_t *twoDfftOut; 

    res = &aoaDspObj->res;

#if !defined (AOA_TYPE_2D)
    cmplx32ReIm_t *azimuthFftOut;
    cmplx32ReIm_t  elevationSymbols[SYS_COMMON_NUM_RX_CHANNEL];
    azimuthFftOut = (cmplx32ReIm_t *)res->scratch1Buff;
#endif

    DPParams = &aoaDspObj->params;
    radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    /* Setup overlayed buffers */
    windowingBuf  = (cmplx32ReIm_t *)res->scratch1Buff;
    twoDfftOut    = (cmplx32ReIm_t *)res->scratch2Buff;

    /* Angle estimation for each object */
    for (idx = 0; idx < numObjsIn; idx++)
    {               
        rangeIdx   = res->cfarRngDopSnrList[idx].rangeIdx;
        dopplerIdx = res->cfarRngDopSnrList[idx].dopplerIdx;
        
        /* Reset ping/pong index */
        pingPongIdx = DPU_AOAPROCDSP_PING_IDX;
        
        /* Trigger first DMA. First transfer is for [txAntIdx=0, rxAntIdx=0, rangeIdx=(Obj range from CFAR list)].
           Note: EDMA ping/pong scheme must support #TX antennas = 1,2,3 #RX antennas = 2,4 */
        EDMA_setSourceAddress(res->edmaHandle, 
                              res->edmaPing.channel, 
                              (uint32_t) &radarCubeBase[rangeIdx]);

        EDMA_startDmaTransfer(res->edmaHandle, res->edmaPing.channel);

        for (rxAntIdx = 0; rxAntIdx < DPParams->numRxAntennas; rxAntIdx++)
        {
            for (txAntIdx = 0; txAntIdx < DPParams->numTxAntennas; txAntIdx++)
            {
                /* verify that previous DMA has completed */
                startTimeWait = Cycleprofiler_getTimeStamp();
                retVal = AoAProcDSP_waitInData (res, pingPongIdx);
                if(retVal != 0)
                {
                    goto exit;
                }    

                *waitTime += Cycleprofiler_getTimeStamp() - startTimeWait;
                
                /*Find index in radar cube for next EDMA.*/ 
                nextTransferTxIdx    = txAntIdx + 1;
                nextTransferRxIdx    = rxAntIdx;
                nextTransferRangeIdx = rangeIdx;
                
                if(nextTransferTxIdx == DPParams->numTxAntennas)
                {
                   nextTransferTxIdx = 0;
                   nextTransferRxIdx++;
                   if(nextTransferRxIdx == DPParams->numRxAntennas)
                   {
                       nextTransferRxIdx = 0;
                       nextTransferRangeIdx++;
                   }
                }
                
                nextTransferIdx = (nextTransferTxIdx * DPParams->numRxAntennas * DPParams->numDopplerChirps + 
                                   nextTransferRxIdx) * DPParams->numRangeBins + nextTransferRangeIdx;
                
                /*Last computation happens when nextTransferRangeIdx reaches (rangeIdx + 1) was we are processing
                  only one range index. Therefore, do not trigger next EDMA.*/
                if(nextTransferRangeIdx == rangeIdx)
                {
                    /* kick off next DMA */
                    if (pingPongIdx == DPU_AOAPROCDSP_PONG_IDX)
                    {
                        channel = res->edmaPing.channel;
                    }
                    else
                    {
                        channel = res->edmaPong.channel;
                    }
                    
                    EDMA_setSourceAddress(res->edmaHandle, channel,
                                         (uint32_t) &radarCubeBase[nextTransferIdx]);
                                         
                    EDMA_startDmaTransfer(res->edmaHandle, channel);
                }    
                
                inpDoppFftBuf = (cmplx16ImRe_t *) &res->pingPongBuf[pingPongIdx * DPParams->numDopplerChirps];

                /* Remove static clutter? */
                if (aoaDspObj->dynLocalCfg.staticClutterCfg.isEnabled)
                {
                    AoAProcDSP_clutterRemoval(DPParams, inpDoppFftBuf);
                }
                
                /* The windowing output buffer is the input buffer for the Doppler FFT. 
                   It needs to be padded with zeroes if number of Doppler chirps is less than 
                   the number of Doppler bins.*/
                if(DPParams->numDopplerBins > DPParams->numDopplerChirps)
                {
                    memset((void*)(&windowingBuf[DPParams->numDopplerChirps]), 0,
                           (DPParams->numDopplerBins - DPParams->numDopplerChirps) * sizeof(cmplx32ReIm_t));
                }

                /*Up to this point samples are in ImRe format, which is the radar cube format.
                  The windowing function will do IQ swap and the Doppler FFT is computed in
                  ReIm format.*/                                
                mmwavelib_windowing16x32_IQswap((int16_t *) inpDoppFftBuf,
                                                res->windowCoeff,
                                                (int32_t *) windowingBuf,
                                                DPParams->numDopplerChirps); 

                /*From this point forward samples are in ReIm format. In particular, FFT, twiddle are all in ReIm format.*/                                
        
                /*DSPLIB notes for DSP_fft32x32():
                  Minimum FFT size is 16. Size must be power of 2. 
                  No scaling done in FFT computation.*/
                DSP_fft32x32((int32_t *)res->twiddle32x32,
                             DPParams->numDopplerBins,
                             (int32_t *)windowingBuf,
                             (int32_t *)twoDfftOut);
                             
                /* Save the 2D doppler FFT output for the required doppler bin */  
                res->angleFftIn[txAntIdx * DPParams->numRxAntennas + rxAntIdx] = twoDfftOut[dopplerIdx];
                        
                pingPongIdx ^= 1;
            }/*txAntIdx*/
        } /* rxAntIdx */
                
        /* When BPM is enabled, order must be:
           2D FFT -> Doppler compensation -> BPM decoding -> RX phase compensation */

        /* Doppler compensation. */           
        AoAProcDSP_dopplerCompensation(1,
                                   &res->angleFftIn[0],
                                   &res->cfarRngDopSnrList[idx],
                                   &res->angleFftIn[0],
                                   DPParams->numTxAntennas,
                                   DPParams->numRxAntennas,
                                   DPParams->numVirtualAntAzim,
                                   DPParams->numVirtualAntElev,
                                   DPParams->numDopplerBins);

        /* BPM decoding*/
        if(DPParams->isBpmEnabled)
        {
            AoAProcDSP_decodeBPM(&res->angleFftIn[0], DPParams->numRxAntennas);
        }                       
        
        /* RX channel gain/phase offset compensation. */
        AoAProcDSP_rxChanPhaseBiasCompensation(&aoaDspObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[0],
                                               DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev,
                                               &res->angleFftIn[0],
                                               &res->angleFftIn[0]);
#if defined (AOA_TYPE_2D)              
        /* Angle estimation*/
        numObjsOutLocal = AoAProcDSP_angleEstimationAzimElev_2dDoa(aoaDspObj,
                                                             idx,
                                                             numObjsOutLocal);
#else
        /* Save elevation symbols before doing Azimuth FFT */
        if(DPParams->numVirtualAntElev > 0)
        {
            memcpy((void*)&elevationSymbols[0],
                   (void*)&res->angleFftIn[DPParams->numVirtualAntAzim],
                   DPParams->numVirtualAntElev * sizeof(cmplx32ReIm_t));
        }    
        
        /* Zero padding */
        memset((void *) &res->angleFftIn[DPParams->numVirtualAntAzim], 0,
               (DPU_AOAPROCDSP_NUM_ANGLE_BINS - DPParams->numVirtualAntAzim) * sizeof(cmplx32ReIm_t));
               
        /* 3D-FFT (Azimuth FFT) */
        DSP_fft32x32((int32_t *)res->angleTwiddle32x32,
                     DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                     (int32_t *) &res->angleFftIn[0],
                     (int32_t *) &azimuthFftOut[0]);
        
        /* Perform 3D elevation FFT if elevation virtual antennas are present*/
        if(DPParams->numVirtualAntElev > 0)
        {
            /* Restore elevation symbols to FFT input buffer */
            memcpy((void*)&res->angleFftIn[0],
                   (void*)&elevationSymbols[0],
                   DPParams->numVirtualAntElev * sizeof(cmplx32ReIm_t));
                   
            /* Zero padding */
            memset((void *) &res->angleFftIn[DPParams->numVirtualAntElev], 0,
                   (DPU_AOAPROCDSP_NUM_ANGLE_BINS - DPParams->numVirtualAntElev) * sizeof(cmplx32ReIm_t));

            /* 3D-FFT (Elevation FFT) */
            DSP_fft32x32((int32_t *)res->angleTwiddle32x32,
                         DPU_AOAPROCDSP_NUM_ANGLE_BINS,
                         (int32_t *) &res->angleFftIn[0],
                         (int32_t *) &res->elevationFftOut[0]);
        }    

        /* Angle estimation*/
        numObjsOutLocal = AoAProcDSP_angleEstimationAzimElev(aoaDspObj,
                                                             idx,
                                                             numObjsOutLocal);

#endif

        if(numObjsOutLocal >= res->detObjOutMaxSize)
        {
            /*Stop processing CFAR list as the limit for number of detected objects was reached*/
            goto exit;
        }
        
    }/* for (idx = 0; idx < numObjsIn; idx++) */

exit:
    *numObjsOut = numObjsOutLocal;
    return retVal;
}

/*===========================================================
 *                    Exported APIs
 *===========================================================*/


DPU_AoAProcDSP_Handle DPU_AoAProcDSP_init
(
    int32_t*            errCode
)
{
    AOADspObj     *aoaDspObj = NULL;

    aoaDspObj = MemoryP_ctrlAlloc(sizeof(AOADspObj), 0);
    if(aoaDspObj == NULL)
    {
        *errCode = DPU_AOAPROCDSP_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)aoaDspObj, 0, sizeof(AOADspObj));

exit:
    return ((DPU_AoAProcDSP_Handle)aoaDspObj);
}


int32_t DPU_AoAProcDSP_config
(
    DPU_AoAProcDSP_Handle    handle,
    DPU_AoAProcDSP_Config    *aoaDspCfg
)
{
    int32_t   retVal = 0;
    AOADspObj *aoaDspObj = (AOADspObj *)handle;

    if((aoaDspObj == NULL) || (aoaDspCfg == NULL)) 
    {
        retVal = DPU_AOAPROCDSP_EINVAL;
        goto exit;
    }

    /* Check if radar cube formats are supported. */
    if (!(aoaDspCfg->res.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1))
    {
        retVal = DPU_AOAPROCDSP_EINVAL__RADARCUBE_DATAFORMAT;
        goto exit;
    }
    
    /* Check if number of doppler bins is at least 16 (due to DSPLIB which requires size of FFT to be
       at least 16*/
    if(aoaDspCfg->staticCfg.numDopplerBins < 16)
    {
        retVal = DPU_AOAPROCDSP_ENUMDOPBINS;
        goto exit;
    }

    /* Check if number of doppler chirps is a multiple of 4. Limitation of mmwavelib */
    if((aoaDspCfg->staticCfg.numDopplerChirps & 3) != 0)
    {
        retVal = DPU_AOAPROCDSP_ENUMDOPCHIRPS;
        goto exit;
    }

    /* Check if DPU configuration is compatible with EDMA max jump size of (32K - 1)*/
    if(aoaDspCfg->staticCfg.numRxAntennas * aoaDspCfg->staticCfg.numRangeBins * sizeof(cmplx16ImRe_t) >= 32768)
    {
        retVal = DPU_AOAPROCDSP_EEXCEEDMAXEDMA;
        goto exit;
    }
    
    /* Validate buffers and params */
    if(!aoaDspCfg->res.edmaHandle ||
       !aoaDspCfg->res.cfarRngDopSnrList ||
       !aoaDspCfg->res.detObjOut  ||
       !aoaDspCfg->res.detObjOutSideInfo ||
       !aoaDspCfg->res.detObj2dAzimIdx ||
       (!aoaDspCfg->res.detObjElevationAngle && (aoaDspCfg->staticCfg.numVirtualAntElev > 0)) ||
       (!aoaDspCfg->res.azimuthStaticHeatMap && aoaDspCfg->dynCfg.prepareRangeAzimuthHeatMap) ||
       !aoaDspCfg->res.windowCoeff ||
       !aoaDspCfg->res.pingPongBuf || 
       !aoaDspCfg->res.angleTwiddle32x32 ||
       !aoaDspCfg->res.twiddle32x32 ||
       !aoaDspCfg->res.angleFftIn ||
       (!aoaDspCfg->res.elevationFftOut && (aoaDspCfg->staticCfg.numVirtualAntElev > 0)) ||
       !aoaDspCfg->res.scratch1Buff ||
       !aoaDspCfg->res.scratch2Buff
      )
    {
        retVal= DPU_AOAPROCDSP_EINVAL;
        goto exit;
    }
    
    if (MEM_IS_NOT_ALIGN(aoaDspCfg->res.cfarRngDopSnrList,
                        DPU_AOAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT))
    {
        retVal = DPU_AOAPROCDSP_ENOMEMALIGN_CFAR_DET_LIST;
        goto exit;
    }

    if (MEM_IS_NOT_ALIGN(aoaDspCfg->res.detObjOut,
                        DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT))
    {
        retVal = DPU_AOAPROCDSP_ENOMEMALIGN_POINT_CLOUD_CARTESIAN;
        goto exit;
    }

    /* @TODO MMWAVESDK-1868:BEGIN, used same error code */    
    if MEM_IS_NOT_ALIGN(aoaDspCfg->res.detObjOutSph,
                        DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDSP_ENOMEMALIGN_POINT_CLOUD_CARTESIAN;
        goto exit;
    }
    /* @TODO MMWAVESDK-1868:END */  

    if (MEM_IS_NOT_ALIGN(aoaDspCfg->res.detObjOutSideInfo,
                        DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT))
    {
        retVal = DPU_AOAPROCDSP_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO;
        goto exit;
    }

    /* Check if buffers provided by application have the correct alignment*/
    if (MEM_IS_NOT_ALIGN(aoaDspCfg->res.detObjElevationAngle, DPU_AOAPROCDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.azimuthStaticHeatMap, DPU_AOAPROCDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.windowCoeff,          DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.pingPongBuf ,         DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.angleTwiddle32x32,    DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.twiddle32x32,         DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.angleFftIn,           DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.elevationFftOut,      DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.scratch1Buff,         DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT) ||
        MEM_IS_NOT_ALIGN(aoaDspCfg->res.scratch2Buff,         DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT))
    {
        retVal = DPU_AOAPROCDSP_ENOMEMALIGN_BUFF;
        goto exit;
    }
    
    /* check sizes for the scratch buffers provided by application */
    if((aoaDspCfg->res.windowSize          < (sizeof(int32_t) * aoaDspCfg->staticCfg.numDopplerChirps) / 2) ||
       (aoaDspCfg->res.pingPongSize        < 2 * sizeof(cmplx16ImRe_t) * aoaDspCfg->staticCfg.numDopplerChirps) ||
       (aoaDspCfg->res.angleTwiddleSize    < sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS) ||
       (aoaDspCfg->res.twiddleSize         < sizeof(cmplx32ReIm_t) * aoaDspCfg->staticCfg.numDopplerBins) ||
       (aoaDspCfg->res.angleFftInSize      < sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS) ||
       (aoaDspCfg->res.elevationFftOutSize < sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS) ||
       (aoaDspCfg->res.scratch1Size        < MAX(sizeof(cmplx32ReIm_t) * aoaDspCfg->staticCfg.numDopplerBins,
                                                     sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS)) ||
       (aoaDspCfg->res.scratch2Size        < MAX(sizeof(cmplx32ReIm_t) * aoaDspCfg->staticCfg.numDopplerBins,
                                                     sizeof(float) * DPU_AOAPROCDSP_NUM_ANGLE_BINS)))
    {
            retVal = DPU_AOAPROCDSP_ESCRATCHSIZE;
            goto exit;
    }

    /* Check Heatmap configuration */
    if (aoaDspCfg->dynCfg.prepareRangeAzimuthHeatMap)
    {
        if (aoaDspCfg->res.azimuthStaticHeatMapSize !=
          (aoaDspCfg->staticCfg.numRangeBins * aoaDspCfg->staticCfg.numVirtualAntAzim))
        {
            retVal = DPU_AOAPROCDSP_ENOMEM__AZIMUTH_STATIC_HEAT_MAP;
            goto exit;
        }
    }
    
    if (aoaDspCfg->dynCfg.prepareRangeAzimuthHeatMap && (aoaDspCfg->staticCfg.numVirtualAntAzim == 1))
    {
        retVal = DPU_AOAPROCDSP_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP;
        goto exit;
    }
    
    aoaDspObj->res = aoaDspCfg->res;
    aoaDspObj->params = aoaDspCfg->staticCfg;

    aoaDspObj->dynLocalCfg.compRxChanCfg  = *aoaDspCfg->dynCfg.compRxChanCfg;
    aoaDspObj->dynLocalCfg.multiObjBeamFormingCfg = *aoaDspCfg->dynCfg.multiObjBeamFormingCfg;
    aoaDspObj->dynLocalCfg.prepareRangeAzimuthHeatMap  = aoaDspCfg->dynCfg.prepareRangeAzimuthHeatMap;
    aoaDspObj->dynLocalCfg.staticClutterCfg = aoaDspCfg->dynCfg.staticClutterCfg;
    AoAProcDSP_ConvertFov(aoaDspObj, aoaDspCfg->dynCfg.fovAoaCfg);
    
    /* Generate twiddle tables */
    AoAProcDSP_generateTables(aoaDspCfg);
    
    /* Configure EDMA */
    retVal = AoAProcDSP_configEDMA(aoaDspCfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}


uint32_t DPU_AoAProcDSP_process
(
    DPU_AoAProcDSP_Handle     handle,
    uint32_t                  numObjsIn,
    DPU_AoAProcDSP_OutParams  *outParams
)
{
    volatile uint32_t   startTime;
    volatile uint32_t   waitTime = 0;
    int32_t             retVal = 0;
    uint16_t            idx;
    float               range;
    uint32_t            numObjsOut = 0;

    AOADspObj *aoaDspObj;
    DPU_AoAProcDSP_HW_Resources *res;
    DPU_AoAProcDSP_StaticConfig *DPParams;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCDSP_EINVAL;
        goto exit;
    }
    aoaDspObj = (AOADspObj *)handle;
    res = &aoaDspObj->res;
    DPParams = &aoaDspObj->params;

    startTime = Cycleprofiler_getTimeStamp();

    if (numObjsIn == 0U)
    {
        outParams->numAoADetectedPoints = numObjsIn;
        outParams->stats.waitTime = 0;
        outParams->stats.processingTime = 0;
        retVal = 0;
        goto exit;
    }

    /* Azimuth heap map */
    if ((aoaDspObj->dynLocalCfg.prepareRangeAzimuthHeatMap) && (DPParams->numVirtualAntAzim > 1))
    {
        retVal = AoAProcDSP_RangeAzimuthHeatmap(aoaDspObj,
                                                &waitTime);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    
    /* Limit number of input samples */
    if (numObjsIn > res->detObjOutMaxSize)
    {
        numObjsIn = res->detObjOutMaxSize;
    }

    /* Angle estimation */
    if(DPParams->numVirtualAntAzim == 1)
    {
       /*If there is only one virtual antenna, there is no
         need of azimuth FFT as azimuth can not be estimated.*/

        /* Fill the output list */
        for(idx=0; idx < numObjsIn; idx++)
        {
            range = res->cfarRngDopSnrList[idx].rangeIdx * DPParams->rangeStep;
            res->detObjOut[idx].y = range;
            res->detObjOut[idx].x = 0.;
            res->detObjOut[idx].z = 0.;

            res->detObjOut[idx].velocity = DPParams->dopplerStep *
                 AOA_DOPPLER_IDX_TO_SIGNED(res->cfarRngDopSnrList[idx].dopplerIdx,
                                       DPParams->numDopplerBins);
            res->detObjOutSideInfo[idx].snr = res->cfarRngDopSnrList[idx].snr;
            res->detObjOutSideInfo[idx].noise = res->cfarRngDopSnrList[idx].noise;
        }
        numObjsOut = numObjsIn;
    }
    else
    {
        retVal = AoAProcDSP_processMultipleVirtualAntennas(aoaDspObj, numObjsIn, &numObjsOut, &waitTime);        
        if (retVal != 0)
        {
            goto exit;
        }
    }

    outParams->numAoADetectedPoints = numObjsOut;
    outParams->stats.waitTime = waitTime;
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitTime;

exit:
    return retVal;
}

int32_t DPU_AoAProcDSP_control
(
   DPU_AoAProcDSP_Handle handle,
   DPU_AoAProcDSP_Cmd cmd,
   void *arg,
   uint32_t argSize
)
{
   int32_t retVal = 0;
   AOADspObj *aoaDspObj = (AOADspObj *)handle;

   if (aoaDspObj == NULL)
   {
       retVal = DPU_AOAPROCDSP_EINVAL;
       goto exit;
   }

   switch(cmd)
   {
       case DPU_AoAProcDSP_Cmd_FovAoACfg:
       {
           if((argSize != sizeof(DPU_AoAProc_FovAoaCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDSP_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               AoAProcDSP_ConvertFov(aoaDspObj, (DPU_AoAProc_FovAoaCfg *) arg);
           }
       }
       break;
       case DPU_AoAProcDSP_Cmd_MultiObjBeamFormingCfg:
       {
           if((argSize != sizeof(DPU_AoAProc_MultiObjBeamFormingCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDSP_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaDspObj->dynLocalCfg.multiObjBeamFormingCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcDSP_Cmd_CompRxChannelBiasCfg:
       {
           if((argSize != sizeof(DPU_AoAProc_compRxChannelBiasCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDSP_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaDspObj->dynLocalCfg.compRxChanCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcDSP_Cmd_PrepareRangeAzimuthHeatMap:
       {
           if((argSize != sizeof(bool)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDSP_EINVAL;
               goto exit;
           }
           else
           {
                /* Sanity check on saved heapmap ptr and size */
                if ((aoaDspObj->res.azimuthStaticHeatMapSize !=
                    (aoaDspObj->params.numRangeBins * aoaDspObj->params.numVirtualAntAzim)) ||
                    (aoaDspObj->res.azimuthStaticHeatMap == NULL))
                {
                   retVal = DPU_AOAPROCDSP_EINVAL;
                   goto exit;
                }

                /* Save configuration */
                memcpy((void *)&aoaDspObj->dynLocalCfg.prepareRangeAzimuthHeatMap, arg, argSize);
           }
       }
       break;
       
       case DPU_AoAProcDSP_Cmd_staticClutterCfg:
       {            
           if((argSize != sizeof(DPU_AoAProc_StaticClutterRemovalCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDSP_EINVAL;
               goto exit;
           }
           else
           {
               /* Update the static clutter configuration. */
               memcpy((void *)&aoaDspObj->dynLocalCfg.staticClutterCfg, (void *)arg, argSize);
           }
       }
       break;
       
       default:
           retVal = DPU_AOAPROCDSP_EINVAL;
           break;
   }
exit:
   return (retVal);
}

int32_t DPU_AoAProcDSP_deinit(DPU_AoAProcDSP_Handle handle)
{
    int32_t retVal = 0;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCDSP_EINVAL;
        goto exit;
    }
    MemoryP_ctrlFree(handle, sizeof(AOADspObj));
exit:
    return (retVal);
}

