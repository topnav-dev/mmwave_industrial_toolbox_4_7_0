/**
 *   @file  aoaprochwa.c
 *
 *   @brief
 *      Implements Data path processing Unit using HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018-2019 Texas Instruments, Inc.
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
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/osal/MemoryP.h>

/* Data Path Include Files */
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpedma/dpedmahwa.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/utils/mathutils/mathutils.h>

/* Industrial Lab Include */
#include <common/src/dpu/aoaproc/include/aoaprochwa_internal.h>

//#define DBG_AOA_HWA_OBJ_DPU

#ifdef DBG_AOA_HWA_OBJ_DPU
volatile AOAHwaObj *gAoaHwaObj[RL_MAX_SUBFRAMES] = {NULL};
volatile uint32_t gAoaHwaObjInd = 0;
#endif

//#define PROFILE_AOA_HWA_OBJ_DPU

#ifdef PROFILE_AOA_HWA_OBJ_DPU
DPU_AoAProcHWA_OutParams       gAoAProcStats[16];
volatile uint32_t gAoAProcStatsIdx = 0;
#endif

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval     None
 */
static void AOAProcHWADoneIsrCallback(void * arg)
{
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Handle)arg);
    }
}

static uint32_t AoAProcHWA_angleEstimationAzimElev
(
    AOAHwaObj       *aoaHwaObj,
    uint32_t        objInIdx,
    uint32_t        pingPongIdx,
    uint32_t        objOutIdx
);


static int32_t HWAutil_configHWA_extendedVelocityScheme
(
    HWA_Handle      hwaHandle,
    DPU_AoAProcHWA_HW_Resources *res,
    DPU_AoAProcHWA_StaticConfig *DPParams,
    uint32_t        *hwaMemBankAddr,
    uint8_t         numHypotheses,
    uint32_t        numAzimuthBins
);

/**
 *   @b Description
 *   @n
 *      AoAProcHWA Rx channel phase/gain compensation. The function performs phase/gain
 *      compensation on the Rx virtual antenna symbols of the detected object. The
 *      function is also used to perform the phase/gain compensation on the two dimensional
 *      array of only azimuth antennas which is used for the azimuth heat map display
 *      on the host. In this case the two dimensional array is laid out as
 *      X[number of range bins][number of azimuth antennas].
 *
 * @param[in]      rxChComp Rx channel phase/gain compensation coefficients
 *
 * @param[in]      numObj Number of detected points
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
static void AoAProcHWA_rxChanPhaseBiasCompensation(cmplx16ImRe_t *rxChComp,
                                                   uint32_t numObj,
                                                   uint32_t numAnt,
                                                   cmplx16ImRe_t *symbolsIn,
                                                   cmplx16ImRe_t *symbolsOut)
{
    int32_t Re, Im;
    uint32_t i, j;
    uint32_t antIndx;
    uint32_t objIdx;

    for (objIdx = 0; objIdx < numObj; objIdx++)
    {
        j = 0;
        /* Compensation of azimuth antennas */
        for (antIndx = 0; antIndx < numAnt; antIndx++)
        {
            i = objIdx*numAnt + antIndx;
            Re = (int32_t) symbolsIn[i].real * (int32_t) rxChComp[j].real -
                 (int32_t) symbolsIn[i].imag * (int32_t) rxChComp[j].imag;

            Im = (int32_t) symbolsIn[i].real * (int32_t) rxChComp[j].imag +
                 (int32_t) symbolsIn[i].imag * (int32_t) rxChComp[j].real;
            MATHUTILS_ROUND_AND_SATURATE_Q15(Re);
            MATHUTILS_ROUND_AND_SATURATE_Q15(Im);

            symbolsOut[i].real = (int16_t) Re;
            symbolsOut[i].imag = (int16_t) Im;
            j++;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function configures EDMA for data transfers to/from HWA in a two stage
 *      data processing where the first stage is 2D-FFT (Doppler FFT) computation,
 *      and the second stage is 3D-FFT (azimuth FFT) computation.
 *      The 2D + 3D processing is organized in a ping/pong manner to run in parallel
 *      with input/output data transfers. Each ping/pong path processes one
 *      detected object at a time. Total 4 physical EDMA channels are used, two  per ping/pong path.
 *      In each path one channel is used for input data transfers to both first and second stage,
 *      and the other channel is used for output data transfers from both first and second stage.
 *
 *  @param[in]  hwaHandle           Handle to HWA
 *  @param[in]  res                 Pointer to hardware resources structure
 *  @param[in]  DPParams            Pointer to profile parameters
 *  @param[in]  srcIn2DFFTBuffAddr  Source address of the 1D-FFT radar cube
 *                                  matrix, the address is set in every iteration
 *                                  by local core per detected object according
 *                                  to its index.
 *  @param[in]  dstIn2DFFTBuffAddr  Destination addresses in HWA memory of
 *                                  ping/pong paths, 0-ping, 1-pong, for 2D-FFT calculation.
 *  @param[in]  srcOut2DFFTBuffAddr Source 2DFFT output addresses in HWA memory
 *                                  of ping/pong paths, 0-ping, 1-pong, the address
 *                                  is set in every iteration by local core per
 *                                  detected object according to its doppler index
 *  @param[in]  dstOut2DFFTBuffAddr Destination 2DFFT output address in local
 *                                  core memory of ping/pong paths. Output data
 *                                  consists of array of 2D antenna symbols corresponding
 *                                  to detected object range/doppler index. The
 *                                  2D array format is X[numTxAntenna][numRxAntenna]
 *  @param[in]  srcIn3DFFTBuffAddr  Address of 2DFFT antenna symbols, (Doppler and
 *                                  phase/gain compensated), of ping/pong paths,
 *                                  0-ping, 1-pong. In each path, it is 3D array
 *                                  in the form X[numberHypotheses][numTxAntenna][numRxAntenna],
 *                                  where the number of hypotheses is equal to number
 *                                  of Tx antennas in TDM MIMO scheme when extended
 *                                  maximum velocity scheme is enabled, otherwise
 *                                  number of hypotheses is equal to one.
 *  @param[in]  dstIn3DFFTBuffAddr  Destination addresses in HWA memory of ping/pong
 *                                  paths, 0-ping, 1-pong, for 3D FFT calculation.
 *  @param[in]  srcOut3DFFTBuffAddr Source 3DFFT output addresses in HWA memory of
 *                                  ping/pong paths, 0-ping, 1-pong.
 *  @param[in]  dstOut3DFFTBuffAddr Destination 3DFFT output address in local core
 *                                  memory, 0-ping, 1-pong.
 *  @param[in]  extMaxVelEnabled    0 - extended maximum velocity feature enabled,
 *                                  1 - disabled
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static int32_t AoAProcHWA_config_EDMA
(
    HWA_Handle  hwaHandle,
    DPU_AoAProcHWA_HW_Resources        *res,
    DPU_AoAProcHWA_StaticConfig  *DPParams,
    uint32_t           srcIn2DFFTBuffAddr,
    uint32_t           *dstIn2DFFTBuffAddr,
    uint32_t           *srcOut2DFFTBuffAddr,
    uint32_t           *dstOut2DFFTBuffAddr,

    uint32_t           *srcIn3DFFTBuffAddr,
    uint32_t           *dstIn3DFFTBuffAddr,
    uint32_t           *srcOut3DFFTBuffAddr,
    uint32_t           *dstOut3DFFTBuffAddr,
    uint8_t            extMaxVelEnabled

)
{
    volatile bool       edmaTransComplete = false ;
    uint16_t numBlocks;
    uint32_t pingPongIdx;
    uint16_t numIterations = 1; //EDMA will be reloaded per target
    uint16_t numHypotheses;

    EDMA_channelConfig_t    config;
    EDMA_paramConfig_t      paramCfg;
    int32_t errorCode = EDMA_NO_ERROR;
    bool isEventTriggered;
    HWA_SrcDMAConfig        dmaConfig;
    EDMA_Handle             handle = res->edmaHandle;


    if(extMaxVelEnabled)
    {
        numHypotheses = DPParams->numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }

    /* Set common fields for Params configuration */
    paramCfg.transferCompletionCallbackFxn = NULL;
    paramCfg.transferCompletionCallbackFxnArg = 0;
    paramCfg.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    paramCfg.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    paramCfg.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    paramCfg.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;
    paramCfg.paramSetConfig.isStaticSet = false;
    paramCfg.paramSetConfig.isEarlyCompletion = false;
    paramCfg.transferCompletionCallbackFxn = NULL;
    paramCfg.transferCompletionCallbackFxnArg = 0;

    /* Set a common configuration for physical input/output channels - initially set to dummy channel */
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = 0;
    memset(&config.paramSetConfig, 0, sizeof(EDMA_paramSetConfig_t));
    config.paramSetConfig.aCount = 1;
    config.paramSetConfig.bCount = 0;
    config.paramSetConfig.cCount = 0;


    numBlocks = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * DPParams->numDopplerChirps;
    for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
    {


        /************************************************************/
        /* Physical Input channel is initially set to dummy channel */
        /************************************************************/
        config.channelId = res->edmaHwaExt[pingPongIdx].chIn.channel;
        config.paramId = res->edmaHwaExt[pingPongIdx].chIn.channel;
        config.eventQueueId = res->edmaHwaExt[pingPongIdx].eventQueue;
        isEventTriggered = false;
        if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /************************************************************/
        /* Physical Output channel is initially set to dummy channel */
        /************************************************************/
        config.channelId = res->edmaHwaExt[pingPongIdx].chOut.channel;
        config.paramId = res->edmaHwaExt[pingPongIdx].chOut.channel;
        config.eventQueueId = res->edmaHwaExt[pingPongIdx].eventQueue;
        isEventTriggered = true;
        if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /* Set EDMA Transfer type to AB type */
        paramCfg.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
        /************************/
        /* Param set: 2D-FFT In */
        /************************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcIn2DFFTBuffAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstIn2DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.bCount = numBlocks;
        paramCfg.paramSetConfig.cCount = numIterations;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = DPParams->numRangeBins * sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.destinationBindex = sizeof(cmplx16ImRe_t);

        paramCfg.paramSetConfig.sourceCindex = sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chIn.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = true;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = true;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        /* Set EDMA Transfer type to A type */
        paramCfg.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;

        /**********************************/
        /* Param set: 2D-FFT Signature In */
        /**********************************/
        HWA_getDMAconfig(hwaHandle,
                         res->hwaCfg.paramSetStartIdx + (pingPongIdx * DPParams->numTxAntennas),
                         &dmaConfig);

        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(dmaConfig.srcAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dmaConfig.destAddr,
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = dmaConfig.aCnt;
        paramCfg.paramSetConfig.bCount = dmaConfig.bCnt;
        paramCfg.paramSetConfig.cCount = dmaConfig.cCnt;
        paramCfg.paramSetConfig.bCountReload = dmaConfig.bCnt;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = 0;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /************************/
        /* Param set: 3D-FFT In */
        /************************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcIn3DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);

        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstIn3DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);

        paramCfg.paramSetConfig.aCount = numHypotheses * (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chIn.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = true;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = true;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /**********************************/
        /* Param set: 3D-FFT Signature In */
        /**********************************/
        uint32_t num3DFFTParamSets = ((DPParams->numVirtualAntElev>0)*1 + 2);
        HWA_getDMAconfig(hwaHandle,
                         res->hwaCfg.paramSetStartIdx + 2 * DPParams->numTxAntennas + (pingPongIdx * num3DFFTParamSets),
                         &dmaConfig);


        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(dmaConfig.srcAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dmaConfig.destAddr,
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = dmaConfig.aCnt;
        paramCfg.paramSetConfig.bCount = dmaConfig.bCnt;
        paramCfg.paramSetConfig.cCount = dmaConfig.cCnt;
        paramCfg.paramSetConfig.bCountReload = dmaConfig.bCnt;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = 0;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle,
                             res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature,
                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /**********************************************/
        /* Link Input physical channel and param sets */
        /**********************************************/
        if ((errorCode = EDMA_linkParamSets(handle,
                            (uint16_t) res->edmaHwaExt[pingPongIdx].chIn.channel,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /********************************/
        /* Bring in the first param set */
        /********************************/
        errorCode = EDMA_startDmaTransfer(handle,
                                       res->edmaHwaExt[pingPongIdx].chIn.channel);
        if (errorCode != 0)
        {
            goto exit;
        }

        /*********************/
        /* 2D-FFT EDMA Out:  */
        /*********************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcOut2DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstOut2DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chOut.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /*********************/
        /* 3D-FFT EDMA Out:  */
        /*********************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcOut3DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstOut3DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = numHypotheses * DPU_AOAPROCHWA_NUM_ANGLE_BINS * sizeof(uint16_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chOut.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /***********************************************/
        /* Link Output physical channel and param sets */
        /***********************************************/
        if ((errorCode = EDMA_linkParamSets(handle,
                            (uint16_t) res->edmaHwaExt[pingPongIdx].chOut.channel,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /********************************/
        /* Bring in the first param set */
        /********************************/
        errorCode = EDMA_startDmaTransfer(handle,
                                       res->edmaHwaExt[pingPongIdx].chOut.channel);
        if (errorCode != 0)
        {
            goto exit;
        }
    }

exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      The function configures EDMA for single bin 2D-FFT computation using HWA.
 *      The output result is 2D array of zero Doppler 2D-FFT symbols,
 *      (X[numVirtualAntAzim][numRangeBins]), used for azimuth heatmap display.
 *      It configures EDMA to copy data from 1D-FFT radar cube matrix to HWA memory for
 *      single bin 2D-FFT calculation. The radar cube matrix format must be defined
 *      as @ref DPIF_RADARCUBE_FORMAT_1. The 2D-FFT processing is organized in a
 *      ping/pong manner to run parallel with input/output data transfer.
 *      Two input EDMA sets are configured to copy 1D-FFT antenna symbols, ping
 *      set reading even and pong set reading odd range bins, into HWA memory and
 *      trigger HWA to execute 2D-FFT computation. The output EDMA sets are configured
 *      to copy zero Doppler symbols, ping set copying even and pong set copying
 *      odd range bins.
 *
 *  @param[in]  hwaHandle            Handle to HWA
 *  @param[in]  res                  Pointer to hardware resources structure
 *  @param[in]  DPParams             Pointer to profile parameters
 *  @param[in]  srcIn2DFFTBuffAddr   Address of the 1D-FFT radar cube matrix
 *  @param[in]  dstIn2DFFTBuffAddr   FFT input addresses in HWA memory of ping/pong
 *                                   paths, 0-ping, 1-pong
 *  @param[in]  srcOut2DFFTBuffAddr  FFT output addresses in HWA memory of ping/pong
 *                                   paths, 0-ping, 1-pong
 *  @param[in]  dstOut2DFFTBuffAddr  Final result address of zero Doppler array in
 *                                   local core memory, 2D array:
 *                                   X[numVirtualAntAzim][numRangeBins]
 *  @param[in]  numTxAnt             Number of Tx Azimuth antennas (this is equal
 *                                   to number of HWA param sets per ping/pong path)
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static int32_t AoAProcHWA_config_azimuthHeatMap_EDMA
(
    HWA_Handle  hwaHandle,
    DPU_AoAProcHWA_HW_Resources        *res,
    DPU_AoAProcHWA_StaticConfig  *DPParams,
    uint32_t           srcIn2DFFTBuffAddr,
    uint32_t           *dstIn2DFFTBuffAddr,
    uint32_t           *srcOut2DFFTBuffAddr,
    uint32_t           dstOut2DFFTBuffAddr,
    uint8_t            numTxAnt
)
{
    int32_t     retVal = 0;
    volatile bool       edmaTransComplete = false ;
    uint16_t numBlocks;
    DPEDMA_ChainingCfg  chainingCfg;
    DPEDMA_syncABCfg    syncABCfg;
    uint32_t pingPongIdx;
    uint16_t numIterations = DPParams->numRangeBins/AOAHWA_NUM_PING_PONG_BUF;
    bool isIntermediateTransferCompletionEnabled = false;
    bool isFinalTransferCompletionEnabled = true;

    numBlocks = (DPParams->numVirtualAntAzim) * DPParams->numDopplerChirps;
    for (pingPongIdx = 0; pingPongIdx < AOAHWA_NUM_PING_PONG_BUF; pingPongIdx++)
    {
        /************************************************************/
        /* EDMA In: load to HWA all antennas, azimuth and elevation */
        /************************************************************/

        /* Copy one Range bin to HWA memory                         */
        chainingCfg.chainingChan                  = res->edmaHwa[pingPongIdx].inSignature.channel;
        chainingCfg.isIntermediateChainingEnabled = true;
        chainingCfg.isFinalChainingEnabled        = true;

        syncABCfg.srcAddress  = srcIn2DFFTBuffAddr + pingPongIdx * sizeof(cmplx16ImRe_t);
        syncABCfg.destAddress = dstIn2DFFTBuffAddr[pingPongIdx];
        syncABCfg.aCount      = sizeof(cmplx16ImRe_t);
        syncABCfg.bCount      = numBlocks;
        syncABCfg.cCount      = numIterations;
        syncABCfg.srcBIdx     = DPParams->numRangeBins * sizeof(cmplx16ImRe_t);
        syncABCfg.dstBIdx     = sizeof(cmplx16ImRe_t);
        syncABCfg.srcCIdx     = sizeof(cmplx16ImRe_t) * AOAHWA_NUM_PING_PONG_BUF;
        syncABCfg.dstCIdx     = 0;

        retVal = DPEDMA_configSyncAB(res->edmaHandle,
                                     &res->edmaHwa[pingPongIdx].in,
                                     &chainingCfg,
                                     &syncABCfg,
                                     false,//isEventTriggered
                                     false, //isIntermediateTransferCompletionEnabled
                                     false,//isTransferCompletionEnabled
                                     NULL, //transferCompletionCallbackFxn
                                     NULL);//transferCompletionCallbackFxnArg
        if (retVal != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /* Copy one hot signature to trigger HWA 2D FFT */
        retVal = DPEDMAHWA_configOneHotSignature(res->edmaHandle,
                                     &res->edmaHwa[pingPongIdx].inSignature,
                                     hwaHandle,
                                     res->hwaCfg.paramSetStartIdx + (pingPongIdx * numTxAnt),
                                     false);
        if (retVal != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /**************/
        /* EDMA Out:  */
        /**************/

        /* Copy one Range bin to HWA memory */
        chainingCfg.chainingChan                  = res->edmaHwa[pingPongIdx].in.channel;
        chainingCfg.isIntermediateChainingEnabled = true;
        chainingCfg.isFinalChainingEnabled        = false;

        syncABCfg.srcAddress  = srcOut2DFFTBuffAddr[pingPongIdx];
        syncABCfg.destAddress = dstOut2DFFTBuffAddr + pingPongIdx * DPParams->numVirtualAntAzim * sizeof(cmplx16ImRe_t) ;
        syncABCfg.aCount      = DPParams->numVirtualAntAzim  * sizeof(cmplx16ImRe_t);
        syncABCfg.bCount      = 1;
        syncABCfg.cCount      = numIterations;
        syncABCfg.srcBIdx     = 0;
        syncABCfg.dstBIdx     = 0;
        syncABCfg.srcCIdx     = 0;
        syncABCfg.dstCIdx     = DPParams->numVirtualAntAzim * sizeof(cmplx16ImRe_t) * AOAHWA_NUM_PING_PONG_BUF;

        retVal = DPEDMA_configSyncAB(res->edmaHandle,
                                     &res->edmaHwa[pingPongIdx].out,
                                     &chainingCfg,
                                     &syncABCfg,
                                     true,//isEventTriggered
                                     isIntermediateTransferCompletionEnabled,
                                     isFinalTransferCompletionEnabled,
                                     NULL, //transferCompletionCallbackFxn
                                     NULL);//transferCompletionCallbackFxnArg

        if (retVal != EDMA_NO_ERROR)
        {
            goto exit;
        }
    }

exit:
    return(retVal);
}



/**
 *  @b Description
 *  @n
 *      The function configures source address of the input EDMA (based on object's
 *      range index), and source address of the output EDMA (based on object's Doppler index).
 *      The function then triggers input EDMA to transfer data to HWA. This trigger initiates the
 *      first stage of cloud point processing, 2D-FFT.
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] detObjIdx    Object index in detected object list
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 */
int32_t AoAProcHWA_cfgAndTrigger_EDMA_2DFFT(AOAHwaObj *aoaHwaObj,
                                 uint32_t detObjIdx,
                                 uint8_t pingPongIdx)
{
    int32_t retVal;
    DPU_AoAProcHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    cmplx16ImRe_t *radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    uint32_t dopplerBinSize = sizeof(cmplx16ImRe_t) * (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev);
    uint32_t srcBuffAddr;

    /* EDMA In Source address in radar cube */
    srcBuffAddr = (uint32_t) (&radarCubeBase[res->cfarRngDopSnrList[detObjIdx].rangeIdx]);
    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chIn.channel,
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* EDMA Out Source address in M2 or M3 */
    srcBuffAddr =  aoaHwaObj->edmaSrcOut2DFFTBuffAddr[pingPongIdx] +
                   res->cfarRngDopSnrList[detObjIdx].dopplerIdx *  dopplerBinSize;
    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chOut.channel,
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger single bin 2D FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                    res->edmaHwaExt[pingPongIdx].chIn.channel);
    if (retVal != 0)
    {
        goto exit;
    }
exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function triggers input EDMA to transfer data to HWA. This trigger initiates the
 *      second stage of cloud point processing, 3D-FFTs.
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 */
int32_t AoAProcHWA_trigger_EDMA_3DFFT(AOAHwaObj *aoaHwaObj,
                                 uint8_t pingPongIdx)
{
    int32_t retVal;
    DPU_AoAProcHWA_HW_Resources *res = &aoaHwaObj->res;

    /* Trigger 3D-FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                   res->edmaHwaExt[pingPongIdx].chIn.channel);

    return retVal;
}

/**
 *   @b Description
 *   @n
 *      The function configures HWA Param sets for 2D-FFT and 3D-FFT calculation.
 *      Processing is organized in a ping pong manner, to run in parallel with
 *      EDMA in/out transfers. THe PARAM sets are split in two groups, one for ping
 *      path and the other for pong path. Processing in each path is divided in
 *      two stages. In the first stage 2D-FFT is calculated, and in the second stage
 *      3D-FFT is calculated. One HWA loop iteration processes 2 cloud points.
 *      (If the number of cloud-points is odd, a dummy one is appended at the end
 *      of the list of detected points.) One loop runs in the following order: ping
 *      path 2D-FFT, pong path 2D-FFT, ping path 3D-FFTs. pong path 3D-FFTs. The
 *      number of HWA Param sets for 2D-FFT calculation is equal to number of Tx
 *      antennas in TDM MIMO scheme. The number of Param sets for 3D-FFT calculation
 *      depends on whether the elevation Tx antenna is enabled or disabled. If the
 *      Tx elevation antenna is disabled, the number of param sets is equal to 2, the
 *      first one calculates 3D-FFT complex output samples, the second one calculates
 *      magnitude square of the complex samples. If the Tx elevation antenna is enabled,
 *      the number of param sets is equal to 3, the additional param set calculates 3D-FFT
 *      complex samples of the elevation antennas.
 *
 *   @param[in] hwaHandle           HWA driver handle
 *   @param[in] res                 Hardware resources
 *   @param[in] DPParams            Static parameters
 *   @param[in] hwaMemBankAddr      Array of 4 HWA bank addresses
 *   @param[in] extMaxVelEnabled    0 - Extended maximum velocity disabled, 1 - enabled
 *   @param[in] numAzimuthBins      Size of 3D-FFT (angle FFT)
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *   \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static int32_t HWAutil_configHWA_extendedVelocityScheme
(
    HWA_Handle      hwaHandle,
    DPU_AoAProcHWA_HW_Resources *res,
    DPU_AoAProcHWA_StaticConfig *DPParams,
    uint32_t        *hwaMemBankAddr,
    uint8_t         extMaxVelEnabled,
    uint32_t        numAzimuthBins
)
{
    HWA_ParamConfig hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    int32_t retVal = 0;
    int32_t i;
    uint32_t pingPongidx;
    int32_t paramSetCurrentIdx;
    uint8_t         dmaTriggerDest;
    uint8_t         numHypotheses;

    if(extMaxVelEnabled)
    {
        numHypotheses = DPParams->numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }

    uint8_t         paramSetStartIdx = res->hwaCfg.paramSetStartIdx;
    uint32_t        windowOffset = res->hwaCfg.winRamOffset;
    uint8_t         winSym = res->hwaCfg.winSym;
    uint32_t        numDopplerChirps = DPParams->numDopplerChirps;
    uint32_t        numDopplerBins = DPParams->numDopplerBins;
    uint8_t         numRxAnt = DPParams->numRxAntennas;
    uint8_t         numTxAnt = DPParams->numTxAntennas;
    uint8_t         numVirtualAntAzim = DPParams->numVirtualAntAzim;
    uint8_t         numVirtualAntElev = DPParams->numVirtualAntElev;

    uint16_t        hwaMemAzimSource[2];
    uint16_t        hwaMemAzimDest[2];


    hwaMemAzimSource[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[0]);
    hwaMemAzimSource[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[1]);
    hwaMemAzimDest[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[2]);
    hwaMemAzimDest[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[3]);



    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************************/
    /******************* Configure 2D-FFT *********************/
    /**********************************************************/
    paramSetCurrentIdx = paramSetStartIdx;
    for (pingPongidx =0; pingPongidx < 2; pingPongidx++)
    {
        memset( (void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));


        /* Program 2D FFT */
        hwaParamCfg.dmaTriggerSrc = paramSetCurrentIdx;
        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT; //do FFT

        hwaParamCfg.source.srcAcnt = numDopplerChirps - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = numRxAnt * sizeof(cmplx16ImRe_t); //
        hwaParamCfg.source.srcBcnt = numRxAnt - 1;
        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t); //should be dont care
        hwaParamCfg.source.srcShift = 0; //no shift
        hwaParamCfg.source.srcCircShiftWrap = 0; //no shift
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED; //signed
        hwaParamCfg.source.srcConjugate = 0; //no conjugate
        hwaParamCfg.source.srcScale = 0;
        hwaParamCfg.source.bpmEnable = 0; //bpm removal not enabled
        hwaParamCfg.source.bpmPhase = 0; //dont care

        hwaParamCfg.dest.dstAcnt = numDopplerBins -1;
        hwaParamCfg.dest.dstAIdx = (numVirtualAntAzim + numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx16ImRe_t) ; //should be dont care
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstScale = 8 ;
        hwaParamCfg.dest.dstSkipInit = 0; // no skipping

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_floorLog2(numDopplerBins);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; //enabled
        hwaParamCfg.accelModeArgs.fftMode.windowStart = windowOffset; //start of window RAM
        hwaParamCfg.accelModeArgs.fftMode.winSymm = winSym;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

        for (i = 0; i < numTxAnt; i++)
        {
            if (i == 0)
            {
                hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
            }
            else
            {
                hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            }
            hwaParamCfg.source.srcAddr = hwaMemAzimSource[pingPongidx] + i * numDopplerChirps * numRxAnt * sizeof(cmplx16ImRe_t); // address is relative to start of MEM0
            hwaParamCfg.dest.dstAddr = hwaMemAzimDest[pingPongidx] + i * numRxAnt * sizeof(cmplx16ImRe_t); // address is relative to start of MEM0
            retVal = HWA_configParamSet(hwaHandle, (uint8_t) paramSetCurrentIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Disable DMA/interrupt hookup to all except the last one */
            retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                                  paramSetCurrentIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
             {
                  goto exit;
             }
             /* Output interrupt to DMA only by last param set */
            if (i == (numTxAnt-1))
            {
                /* Last param set: enable the DMA hookup to this paramset so that data gets copied out */
                retVal = HWA_getDMAChanIndex(hwaHandle,
                                             res->edmaHwaExt[pingPongidx].chOut.channel,
                                             &dmaTriggerDest);
                if (retVal != 0)
                {
                    goto exit;
                }
                paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
                paramISRConfig.dma.dstChannel = dmaTriggerDest;  /* EDMA channel to trigger to copy the data out */
                paramISRConfig.cpu.callbackArg = NULL;
                retVal = HWA_enableParamSetInterrupt(hwaHandle,
                                                     paramSetCurrentIdx,
                                                     &paramISRConfig);
                if (retVal != 0)
                 {
                      goto exit;
                 }
            }
            paramSetCurrentIdx++;
        }
    }

    /**********************************************************/
    /******************* Configure 3D-FFT *********************/
    /**********************************************************/
    for (pingPongidx =0; pingPongidx < 2; pingPongidx++)
    {
        /************** Param set computes the complex values of the azimuth-FFT **********/
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
        hwaParamCfg.dmaTriggerSrc = paramSetCurrentIdx;
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;

        hwaParamCfg.source.srcAddr = (uint16_t) hwaMemAzimSource[pingPongidx];
        hwaParamCfg.source.srcAcnt = numVirtualAntAzim-1;
        hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBIdx = (numVirtualAntAzim + numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = numHypotheses-1;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcScale = 8;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

        hwaParamCfg.dest.dstAddr =  (uint16_t) hwaMemAzimDest[pingPongidx];
        hwaParamCfg.dest.dstAcnt = numAzimuthBins-1;
        hwaParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.dest.dstScale = 3;

        retVal = HWA_configParamSet(hwaHandle, paramSetCurrentIdx, &hwaParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                           paramSetCurrentIdx,
                                           HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
          goto exit;
        }

        paramSetCurrentIdx++;

        if (numVirtualAntElev > 0)
        {
            /************** Param set computes the complex values of the elevation-FFT **********/
            memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
            hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

            hwaParamCfg.source.srcAddr = (uint16_t) (hwaMemAzimSource[pingPongidx] +
                                                     (numVirtualAntAzim * sizeof(cmplx16ImRe_t)));
            hwaParamCfg.source.srcAcnt = numVirtualAntElev-1;
            hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
            hwaParamCfg.source.srcBIdx = (numVirtualAntAzim + numVirtualAntElev) * sizeof(cmplx16ImRe_t);
            hwaParamCfg.source.srcBcnt = numHypotheses-1;
            hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.source.srcScale = 8;


            hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
            hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
            hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
            hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

            hwaParamCfg.dest.dstAddr =  (uint16_t) (hwaMemAzimDest[pingPongidx] +
                                                    (numHypotheses * numAzimuthBins * sizeof(cmplx16ImRe_t)));
            hwaParamCfg.dest.dstAcnt = numAzimuthBins-1;
            hwaParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
            hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
            hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
            hwaParamCfg.dest.dstScale = 3;

            retVal = HWA_configParamSet(hwaHandle, paramSetCurrentIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }
            retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                               paramSetCurrentIdx,
                                               HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
              goto exit;
            }

            paramSetCurrentIdx++;
        }

        //This calculates magnitude
        /************** Param set computes magnitude of input complex values of the azimuth-FFT **********/
        memset( (void*) &hwaParamCfg, 0, sizeof(hwaParamCfg));
        hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

        hwaParamCfg.source.srcAddr = (uint16_t) hwaMemAzimDest[pingPongidx];
        hwaParamCfg.source.srcAcnt = numAzimuthBins - 1;
        hwaParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
        hwaParamCfg.source.srcBcnt = numHypotheses-1;
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwaParamCfg.source.srcScale = 8;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwaParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//Ignored
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;

        hwaParamCfg.dest.dstAddr =  (uint16_t) hwaMemAzimSource[pingPongidx] + DPU_AOAPROCHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;
        hwaParamCfg.dest.dstAcnt = numAzimuthBins-1;
        hwaParamCfg.dest.dstAIdx = sizeof(uint16_t);
        hwaParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(uint16_t);
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwaParamCfg.dest.dstScale = 0;
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;

        retVal = HWA_configParamSet(hwaHandle, paramSetCurrentIdx, &hwaParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }

        /* Enable the DMA hookup to EDMA */
        retVal = HWA_getDMAChanIndex(hwaHandle,
                                     res->edmaHwaExt[pingPongidx].chOut.channel,
                                     &dmaTriggerDest);
        if (retVal != 0)
        {
             goto exit;
        }
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = dmaTriggerDest;  /* EDMA channel to trigger to copy the data out */
        paramISRConfig.cpu.callbackArg = NULL;
        retVal = HWA_enableParamSetInterrupt(hwaHandle,
                                             paramSetCurrentIdx,
                                             &paramISRConfig);
        if (retVal != 0)
        {
             goto exit;
        }

        paramSetCurrentIdx++;

    }
exit:
    return (retVal);
}

/**
 *   @b Description
 *   @n
 *      The function configures HWA Param sets for 2D-FFT single range bin
 *      calculation from 1D-FFT data from radarCube. The 1D FFT symbols are
 *      arranged in HWA memory as three dimensional array as
 *      X[Number of Tx antennas][Number of virtual chirps][Number of Rx antennas]
 *      THe output symbols are arrange as three dimensional array as
 *      Y[Number of Doppler bins][Number of Tx antnnas][Number of Rx antennas]
 *
 *   @param[in] hwaHandle       HWA driver handle
 *   @param[in] res             Hardware resources
 *   @param[in] DPParams        Static parameters
 *   @param[in] hwaMemBankAddr  Array of 4 HWA bank addresses
 *   @param[in] numAzimTxAnt    Number of azimuth Tx antennas
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *   \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static int32_t HWAutil_configHWA_azimuthHeatMap
(
    HWA_Handle      hwaHandle,
    DPU_AoAProcHWA_HW_Resources *res,
    DPU_AoAProcHWA_StaticConfig *DPParams,
    uint32_t        *hwaMemBankAddr,
    uint8_t        numAzimTxAnt
)
{
    HWA_ParamConfig hwaParamCfg;
    HWA_InterruptConfig     paramISRConfig;
    int32_t retVal = 0;
    int32_t i;
    uint32_t pingPongidx;
    int32_t paramSetCurrentIdx;
    uint8_t         dmaTriggerDest;

    uint8_t         paramSetStartIdx = res->hwaCfg.paramSetStartIdx;
    uint32_t        windowOffset = res->hwaCfg.winRamOffset;
    uint8_t         winSym = res->hwaCfg.winSym;
    uint32_t        numDopplerChirps = DPParams->numDopplerChirps;
    uint32_t        numDopplerBins = DPParams->numDopplerBins;
    uint8_t         numRxAnt = DPParams->numRxAntennas;
    uint8_t         numVirtualAntAzim = DPParams->numVirtualAntAzim;

    uint16_t        hwaMemAzimSource[2];
    uint16_t        hwaMemAzimDest[2];


    hwaMemAzimSource[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[0]);
    hwaMemAzimSource[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[1]);
    hwaMemAzimDest[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[2]);
    hwaMemAzimDest[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[3]);

    uint32_t        numOutSamplesPerFft = 1;



    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************************/
    /******************* Configure 2D-FFT *********************/
    /**********************************************************/
    paramSetCurrentIdx = paramSetStartIdx;
    for (pingPongidx =0; pingPongidx < 2; pingPongidx++)
    {
        memset( (void*) &hwaParamCfg, 0, sizeof(HWA_ParamConfig));


        /* Program 2D FFT */
        hwaParamCfg.dmaTriggerSrc = paramSetCurrentIdx;
        hwaParamCfg.accelMode = HWA_ACCELMODE_FFT; //do FFT

        hwaParamCfg.source.srcAcnt = numDopplerChirps - 1; //size in samples - 1

        hwaParamCfg.source.srcAIdx = numRxAnt * sizeof(cmplx16ImRe_t); //
        hwaParamCfg.source.srcBcnt = numRxAnt - 1;
        hwaParamCfg.source.srcBIdx = sizeof(cmplx16ImRe_t); //should be dont care
        hwaParamCfg.source.srcShift = 0; //no shift
        hwaParamCfg.source.srcCircShiftWrap = 0; //no shift
        hwaParamCfg.source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
        hwaParamCfg.source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
        hwaParamCfg.source.srcSign = HWA_SAMPLES_SIGNED; //signed
        hwaParamCfg.source.srcConjugate = 0; //no conjugate
        hwaParamCfg.source.srcScale = 0;
        hwaParamCfg.source.bpmEnable = 0; //bpm removal not enabled
        hwaParamCfg.source.bpmPhase = 0; //dont care

        hwaParamCfg.dest.dstAcnt = numOutSamplesPerFft -1;
        hwaParamCfg.dest.dstAIdx = (numVirtualAntAzim) * sizeof(cmplx16ImRe_t);
        hwaParamCfg.dest.dstBIdx = sizeof(cmplx16ImRe_t) ; //should be dont care
        hwaParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
        hwaParamCfg.dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
        hwaParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
        hwaParamCfg.dest.dstConjugate = 0; //no conjugate
        hwaParamCfg.dest.dstScale = 8 ;
        hwaParamCfg.dest.dstSkipInit = 0; // no skipping

        hwaParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwaParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_floorLog2(numDopplerBins);
        hwaParamCfg.accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant
        hwaParamCfg.accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
        hwaParamCfg.accelModeArgs.fftMode.windowEn = 1; //enabled
        hwaParamCfg.accelModeArgs.fftMode.windowStart = windowOffset; //start of window RAM
        hwaParamCfg.accelModeArgs.fftMode.winSymm = winSym;
        hwaParamCfg.accelModeArgs.fftMode.winInterpolateMode = 0;
        hwaParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
        hwaParamCfg.accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

        hwaParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

        for (i = 0; i < numAzimTxAnt; i++)
        {
            if (i == 0)
            {
                hwaParamCfg.triggerMode = HWA_TRIG_MODE_DMA;
            }
            else
            {
                hwaParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;
            }

            hwaParamCfg.source.srcAddr = hwaMemAzimSource[pingPongidx] + i * numDopplerChirps * numRxAnt * sizeof(cmplx16ImRe_t); // address is relative to start of MEM0
            hwaParamCfg.dest.dstAddr = hwaMemAzimDest[pingPongidx] + i * numRxAnt * sizeof(cmplx16ImRe_t); // address is relative to start of MEM0
            retVal = HWA_configParamSet(hwaHandle, (uint8_t) paramSetCurrentIdx, &hwaParamCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }

            /* Disable DMA/interrupt hookup to all except the last one */
            retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                                  paramSetCurrentIdx,
                                                  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
             {
                  goto exit;
             }
             /* Output interrupt to DMA only by last param set */
            if (i == (numAzimTxAnt-1))
            {
                /* Last param set: enable the DMA hookup to this paramset so that data gets copied out */
                retVal = HWA_getDMAChanIndex(hwaHandle,
                                             res->edmaHwa[pingPongidx].out.channel,
                                             &dmaTriggerDest);
                if (retVal != 0)
                {
                    goto exit;
                }
                paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
                paramISRConfig.dma.dstChannel = dmaTriggerDest;  /* EDMA channel to trigger to copy the data out */
                paramISRConfig.cpu.callbackArg = NULL;
                retVal = HWA_enableParamSetInterrupt(hwaHandle,
                                                     paramSetCurrentIdx,
                                                     &paramISRConfig);
                if (retVal != 0)
                 {
                      goto exit;
                 }
            }
            paramSetCurrentIdx++;
        }
    }

exit:
    return (retVal);
}



/**
 *  @b Description
 *  @n
 *      Configures HWA common register. It sets the Params start index and
 *      number of iteration loops.
 *
 *   @param[in] hwaHandle               HWA driver handle
 *   @param[in] numLoops                Number of HWA loops
 *   @param[in] paramStartIdx           Param start index
 *   @param[in] numParams               Number of params
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static int32_t AoAProc_HWAutil_configCommon (HWA_Handle hwaHandle,
                                                uint16_t numLoops,
                                                uint16_t paramStartIdx,
                                                uint16_t numParams)
{
    int32_t errCode = 0;
    HWA_CommonConfig    hwaCommonConfig;

    /* Disable the HWA */
     errCode = HWA_enable(hwaHandle, 0);
     if (errCode != 0)
     {
         goto exit;
     }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;

    hwaCommonConfig.numLoops = numLoops;
    hwaCommonConfig.paramStartIdx = paramStartIdx;
    hwaCommonConfig.paramStopIdx = paramStartIdx + (numParams-1);

    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    errCode = HWA_configCommon(hwaHandle, &hwaCommonConfig);
exit:
    return (errCode);
}

/**
 *  @b Description
 *  @n
 *      Wait for angle estimation to complete. This is a blocking function.
 *
 *  @param[in] semaHandle  Semaphore handle
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *
 */
int32_t AoAProcHWA_dataPathWait2DFFTDetObj(SemaphoreP_Handle semaHandle)
{
    return((int32_t) SemaphoreP_pend(semaHandle, SemaphoreP_WAIT_FOREVER));
}

/**
 *  @b Description
 *  @n
 *      This function is called per object and it calculates its x/y/z coordinates
 *      based on Azimuth FFT output. It stores the coordinates
 *      to the output list of type @ref DPIF_PointCloudCartesian_t and it also
 *      calculates object's SNR and the noise level and writes into the side
 *      information list of type @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] aoaHwaObj Pointer to AoA DPU internal object
 *
 *  @param[in] pingPongIdx ping/pong index
 *
 *  @param[in] objInIdx Index of the current object in CFAR output list @ref DPIF_CFARDetList_t
 *
 *  @param[in] objOutIdx Index of the current object in the output lists @ref DPIF_PointCloudCartesian_t and @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] maxIdx Peak position in the FFT output of azimuth antennas
 *
 *  @param[in] dopplerIdx Doppler index of detected object
 *
 *  @param[in] hypothesisIdx Hypothesis index related to extended maximum velocity feature.
 *              (If the feature is disabled, hypothesisIdx is passed as zero)
 *
 *  @retval objOutIdx Index for the next object in the output list
 *
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
uint32_t AoAProcHWA_XYZestimation
(
    AOAHwaObj   *aoaHwaObj,
    uint32_t    pingPongIdx,
    uint32_t    objInIdx,
    uint32_t    objOutIdx,
    uint32_t    maxIdx,
    int32_t     dopplerIdx,
    uint32_t    hypothesisIdx
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
    uint32_t numAngleBins = DPU_AOAPROCHWA_NUM_ANGLE_BINS;
    DPU_AoAProcHWA_HW_Resources            *res = &aoaHwaObj->res;
    DPIF_CFARDetList        *objIn = res->cfarRngDopSnrList;
    DPIF_PointCloudCartesian *objOut = res->detObjOut;
    DPIF_PointCloudSpherical *objOutSph = res->detObjOutSph;
    DPIF_PointCloudSideInfo *objOutSideInfo = res->detObjOutSideInfo;

    DPU_AoAProcHWA_StaticConfig *params = &aoaHwaObj->params;
    //uint16_t numDopplerBins = params->numDopplerBins;

    cmplx16ImRe_t *azimFFTPtr = (cmplx16ImRe_t *) (aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[pingPongIdx] + (hypothesisIdx*numAngleBins)*sizeof(cmplx16ImRe_t));

    uint32_t numHypotheses;

    if (aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled)
    {
        numHypotheses = aoaHwaObj->params.numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }




    range = objIn[objInIdx].rangeIdx * params->rangeStep;

    /* Compensate for range bias */
    range -= aoaHwaObj->dynLocalCfg.compRxChanCfg.rangeBias;
    if (range < 0)
    {
        range = 0;
    }

    if(maxIdx > (numAngleBins/2 -1))
    {
        sMaxIdx = maxIdx - numAngleBins;
    }
    else
    {
        sMaxIdx = maxIdx;
    }

    Wx = 2 * (float) sMaxIdx / numAngleBins;
    /* Check if it is within configured field of view */

    x = range * Wx;

    if (params->numVirtualAntElev > 0)
    {
        peakAzimIm = (float) azimFFTPtr[maxIdx].imag;
        peakAzimRe = (float) azimFFTPtr[maxIdx].real;
        peakElevIm = (float) azimFFTPtr[maxIdx + numHypotheses*numAngleBins].imag;
        peakElevRe = (float) azimFFTPtr[maxIdx + numHypotheses*numAngleBins].real;

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
        if((Wz < aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal) || (Wz > aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal))
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

    if((Wx < (limitScale * aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal)) ||
       (Wx > (limitScale * aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal)))
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

    objOut[objOutIdx].velocity = params->dopplerStep * dopplerIdx;
    objOutSideInfo[objOutIdx].noise = objIn[objInIdx].noise;
    objOutSideInfo[objOutIdx].snr = objIn[objInIdx].snr;
    res->detObj2dAzimIdx[objOutIdx] = maxIdx;

    /*@TODO MMWAVESDK-1868:BEGIN */
    objOutSph[objOutIdx].range = range;
    objOutSph[objOutIdx].azimuthAngle = theta;
    objOutSph[objOutIdx].elevAngle = phi;
    objOutSph[objOutIdx].velocity = params->dopplerStep * dopplerIdx;
    /*@TODO MMWAVESDK-1868:END */

    objOutIdx++;

exit:
    return (objOutIdx);
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
static void aoaHwa_dopplerComp
(
    cmplx16ImRe_t *in,
    cmplx16ImRe_t *out,
    float  Cos,
    float  Sin
)
{
    float           yRe, yIm;

    /* Rotate symbol (correct the phase) */
    yRe = in->real * Cos + in->imag * Sin;
    yIm = in->imag * Cos - in->real * Sin;

    out->real = (int16_t) yRe;
    out->imag = (int16_t) yIm;
}

/**
 *   @b Description
 *   @n
 *      Function performs Doppler compensation on antenna symbols.
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
 *  @param[in]  extendedMaxVelEnabled      0: extended maximum velocity feature disabled, 1: enabled
 *
 *  @return None
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static void aoaHwa_dopplerCompensation
(
    uint32_t *srcPtr,
    DPIF_CFARDetList *cfarOutList,
    uint32_t *dstPtr,
    uint32_t numTxAnt,
    uint32_t numRxAnt,
    uint32_t numVirtualAntAzim,
    uint32_t numVirtualAntElev,
    uint32_t numDopplerBins,
    uint32_t extendedMaxVelEnabled
)
{
    uint32_t    index;
    uint32_t    j;
    uint32_t    virtAntIdx;
    uint32_t    txAntIdx;
    uint16_t    dopplerIdx;
    int32_t     dopplerSignIdx;
    float       dopplerCompensationIdx;
    int32_t     wrapInd;
    uint32_t    numHypotheses;
    float       Cos,Sin,temp;
    uint32_t    numVirtualAnt;

    dopplerIdx = cfarOutList->dopplerIdx;
    dopplerSignIdx = (int32_t) dopplerIdx;
    if(dopplerIdx >= (numDopplerBins/2))
    {
        dopplerSignIdx = dopplerSignIdx - (int32_t) numDopplerBins;
    }

    if(extendedMaxVelEnabled)
    {
        numHypotheses = numTxAnt;
        wrapInd = - ((int32_t) (numHypotheses >> 1));
        if (!(numTxAnt & 0x1) && (dopplerSignIdx < 0))
        {
            wrapInd++;
        }
    }
    else
    {
        numHypotheses = 1;
        wrapInd = 0;
    }

    numVirtualAnt = numVirtualAntAzim + numVirtualAntElev;
    
    for(index = 0; index < numHypotheses; index++)
    {
        virtAntIdx = 0;
        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx0) */
        for(j = 0; j < numRxAnt; j++)
        {
            dstPtr[virtAntIdx + index*numVirtualAnt] = srcPtr[virtAntIdx];
            virtAntIdx++;
        }

        if(numTxAnt > 1)
        {
            dopplerCompensationIdx = (dopplerSignIdx + (wrapInd * (int32_t)numDopplerBins)) / (float)numTxAnt;
            Cos = cos(2*PI_*dopplerCompensationIdx/numDopplerBins);
            Sin = sin(2*PI_*dopplerCompensationIdx/numDopplerBins);

            for(txAntIdx=1; txAntIdx < numTxAnt; txAntIdx++)
            {
                for(j = 0; j < numRxAnt; j++)
                {
                    aoaHwa_dopplerComp((cmplx16ImRe_t *)&srcPtr[virtAntIdx],
                                       (cmplx16ImRe_t *)&dstPtr[virtAntIdx + index*numVirtualAnt],
                                       Cos,
                                       Sin);
                    virtAntIdx++;
                }

                if (txAntIdx < (numTxAnt-1))
                {
                    /* Increment Doppler phase shift */
                    temp = Cos * Cos - Sin * Sin;
                    Sin = 2 * Cos * Sin;
                    Cos = temp;
                }

            }
        }
        wrapInd++;
    }
}


/**
 *  @b  Description
 *  @n
 *      Function calculates x/y/z coordinates of the objects detected by CFAR DPU.
 *      The number of output objects may be larger than input if multiobject beam
 *      forming is enabled, and more than one objects are detected at the
 *      same range/doppler bin.
 *
 *  @param[in]  aoaHwaObj          Pointer to AoAProc DPU internal data Object
 *  @param[in]  objInIdx           Index to the next object in the output list
 *  @param[in]  pingPongIdx        Ping/pong index
 *  @param[in]  objOutIdx          Index to the next object in the output list
 *
 *  @retval    objOutIdx           Number of detected objects in the output list
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 */
static uint32_t AoAProcHWA_angleEstimationAzimElev
(
    AOAHwaObj       *aoaHwaObj,
    uint32_t        objInIdx,
    uint32_t        pingPongIdx,
    uint32_t        objOutIdx
)
{
    uint32_t j, maxVal = 0,maxIdx = 0, tempVal;
    //uint32_t objInHwaIdx;
    DPU_AoAProcHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    DPIF_CFARDetList        *objIn = res->cfarRngDopSnrList;


    uint32_t numAngleBins = DPU_AOAPROCHWA_NUM_ANGLE_BINS;
    uint16_t maxNumObj = res->detObjOutMaxSize;
    uint32_t numHypotheses;
    int32_t  hypothesisMaxIdx;
    uint16_t *azimFFTAbsPtr = (uint16_t *)aoaHwaObj->azimuthFftOutMagBuf[pingPongIdx];
    int16_t  dopplerSignIdx;
    int32_t  wrapStartInd;
    uint8_t numTxAnt = DPParams->numTxAntennas;
    uint16_t numDopplerBins = DPParams->numDopplerBins;

    if (objOutIdx >= maxNumObj)
    {
        return objOutIdx;
    }

    dopplerSignIdx = AOA_DOPPLER_IDX_TO_SIGNED(objIn[objInIdx].dopplerIdx, numDopplerBins);

    if(aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled)
    {
        numHypotheses = aoaHwaObj->params.numTxAntennas;
        wrapStartInd = - ((int32_t) (numHypotheses >> 1));
        if (!(numTxAnt & 0x1) && (dopplerSignIdx < 0))
        {
            wrapStartInd++;
        }
    }
    else
    {
        numHypotheses = 1;
        wrapStartInd = 0;
    }



    maxVal = 0;
    for(j=0; j < numHypotheses * numAngleBins; j++)
    {
        tempVal = azimFFTAbsPtr[j];
        if(tempVal > maxVal)
        {
            maxVal = tempVal;
            maxIdx = j;
        }
    }

    hypothesisMaxIdx = maxIdx >> DPU_AOAPROCHWA_LOG2_NUM_ANGLE_BINS;
    maxIdx = maxIdx & (numAngleBins - 1);

    dopplerSignIdx += (wrapStartInd + hypothesisMaxIdx) * (int16_t) numDopplerBins;

    /* Estimate x,y,z */
    objOutIdx = AoAProcHWA_XYZestimation(aoaHwaObj,
                                         pingPongIdx,
                                         objInIdx,
                                         objOutIdx,
                                         maxIdx,
                                         dopplerSignIdx,
                                         hypothesisMaxIdx);

    if (objOutIdx >= maxNumObj)
    {
        return objOutIdx;
    }

    /* Multi peak azimuzth search?*/
    if (aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg.enabled)
    {
        uint32_t leftSearchIdx;
        uint32_t rightSearchIdx;
        uint32_t secondSearchLen;
        uint32_t iModAzimLen;
        uint32_t maxVal2;
        int32_t k;
        uint32_t t;
        uint16_t azimIdx = maxIdx;
        uint16_t* azimuthMag = &azimFFTAbsPtr[hypothesisMaxIdx * numAngleBins];

        /* Find right edge of the first peak */
        t = azimIdx;
        leftSearchIdx = (t + 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMag[t] >= azimuthMag[leftSearchIdx]) && (k > 0))
        {
            t = (t + 1) & (numAngleBins-1);
            leftSearchIdx = (leftSearchIdx + 1) & (numAngleBins-1);
            k--;
        }

        /* Find left edge of the first peak */
        t = azimIdx;
        rightSearchIdx = (t - 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMag[t] >= azimuthMag[rightSearchIdx]) && (k > 0))
        {
            t = (t - 1) & (numAngleBins-1);
            rightSearchIdx = (rightSearchIdx - 1) & (numAngleBins-1);
            k--;
        }

        secondSearchLen = ((rightSearchIdx - leftSearchIdx) & (numAngleBins-1)) + 1;
        /* Find second peak */
        maxVal2 = azimuthMag[leftSearchIdx];
        azimIdx = leftSearchIdx;
        for (t = leftSearchIdx; t < (leftSearchIdx + secondSearchLen); t++)
        {
            iModAzimLen = t & (numAngleBins-1);
            if (azimuthMag[iModAzimLen] > maxVal2)
            {
                azimIdx = iModAzimLen;
                maxVal2 = azimuthMag[iModAzimLen];
            }
        }
        /* Is second peak greater than threshold? */
        if ( (maxVal2 >( ((uint32_t)(maxVal * aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg.multiPeakThrsScal)))) && (objOutIdx < maxNumObj) )
        {

            /* Estimate x,y,z for second peak */
            objOutIdx = AoAProcHWA_XYZestimation(aoaHwaObj,
                                                 pingPongIdx,
                                                 objInIdx,
                                                 objOutIdx,
                                                 azimIdx,
                                                 dopplerSignIdx,
                                                 hypothesisMaxIdx);
        }
    }
    return(objOutIdx);
}

/**
 *  @b Description
 *  @n
 *      The function waits for 2D FFT single bin data to be transfered to the output buffer.
 *      This is a blocking function. The function polls for EDMA transfer completion.
 *
 *  @param[in]  aoaHwaObj    Pointer to internal AoAProcHWA data object
 *  @param[in]  channel      EDMA channel number
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t  AOAHWA_waitEdma(AOAHwaObj * aoaHwaObj,
                         uint8_t channel)
{
    DPU_AoAProcHWA_HW_Resources *res = &aoaHwaObj->res;
    int32_t errCode = EDMA_NO_ERROR;

    /* wait until transfer done */
    volatile bool isTransferDone;
    do {
        if ((errCode = EDMA_isTransferComplete(res->edmaHandle,
                                    channel,
                                    (bool *)&isTransferDone)) != EDMA_NO_ERROR)
        {
            break;
        }
    } while (isTransferDone == false);

    return errCode;
}


/**
 *  @b Description
 *  @n
 *      The function prepares data for range-azimuth heat-map. 
 *      It calculates 2D-FFT for all virtual antennas and saves 
 *      zero Doppler values for all range bins.
 *
 *  @param[in]    aoaHwaObj     Pointer to internal AoAProcHWA data object
 *
 *  @param[inout] waitTimeLocal CPU waiting time
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t  AoAProcHWA_PrepareRangeAzimuthHeatmap(AOAHwaObj *aoaHwaObj,
                                               volatile uint32_t   *waitTimeLocal)
{
    volatile uint32_t   startTime1;
    int32_t retVal = 0;
    uint16_t numParams;
    HWA_Handle hwaHandle;
    cmplx16ImRe_t     *radarCubeBase;
    DPU_AoAProcHWA_HW_Resources *res;
    DPU_AoAProcHWA_StaticConfig *DPParams;
    uint8_t numTxAzimAnt;
    uint8_t pingPongIdx;

    res = &aoaHwaObj->res;
    DPParams = &aoaHwaObj->params;
    radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    hwaHandle = aoaHwaObj->hwaHandle;

    numTxAzimAnt = (uint8_t) (DPParams->numVirtualAntAzim / DPParams->numRxAntennas);

    retVal = HWAutil_configHWA_azimuthHeatMap(hwaHandle,
                                              res,
                                              DPParams,
                                              aoaHwaObj->hwaMemBankAddr,
                                              numTxAzimAnt);
    if (retVal != 0)
    {
        goto exit;
    }

    numParams = numTxAzimAnt * AOAHWA_NUM_PING_PONG_BUF;
    retVal =  AoAProc_HWAutil_configCommon (hwaHandle,
                                            DPParams->numRangeBins / AOAHWA_NUM_PING_PONG_BUF, //numLoops,
                                            res->hwaCfg.paramSetStartIdx, //paramStartIdx,
                                            numParams);
    if (retVal != 0)
    {
        goto exit;
    }

    retVal = AoAProcHWA_config_azimuthHeatMap_EDMA(hwaHandle,
                                                   res,
                                                   DPParams,
                                                   (uint32_t) radarCubeBase,
                                                   aoaHwaObj->edmaDstIn2DFFTBuffAddr,
                                                   aoaHwaObj->edmaSrcOut2DFFTBuffAddr,
                                                   (uint32_t) res->azimuthStaticHeatMap,
                                                   numTxAzimAnt);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(hwaHandle, 1);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger first two range bins 2D FFT */
    pingPongIdx = 0;
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                    res->edmaHwa[pingPongIdx].in.channel);
    pingPongIdx = 1;
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                            res->edmaHwa[pingPongIdx].in.channel);

    if (retVal != 0)
    {
        goto exit;
    }

    /* Wait until HWA done with FFT */
    startTime1 = Cycleprofiler_getTimeStamp();
    AoAProcHWA_dataPathWait2DFFTDetObj(aoaHwaObj->hwaDone_semaHandle);
    *waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;

    /* Wait until EDMA output is done */
    startTime1 = Cycleprofiler_getTimeStamp();
    pingPongIdx = 0;
    retVal = AOAHWA_waitEdma(aoaHwaObj,
                             res->edmaHwa[pingPongIdx].in.channel);
    pingPongIdx = 1;
    retVal = AOAHWA_waitEdma(aoaHwaObj,
                             res->edmaHwa[pingPongIdx].in.channel);
    *waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
    if (retVal != 0)
    {
          goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Rx channel gain/phase offset compensation */
    AoAProcHWA_rxChanPhaseBiasCompensation(&aoaHwaObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[0],
                                        DPParams->numRangeBins,
                                        DPParams->numVirtualAntAzim,
                                        res->azimuthStaticHeatMap,
                                        res->azimuthStaticHeatMap);
exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function converts angle of arrival field of view values specified
 *      in degrees to the values appropriate for internal DPU comparison.
 *
 *  @param[in]  aoaHwaObj    Pointer to internal AoAProcHWA data object
 *
 *  @param[in]  fovAoaCfg    Pointer to field of view configuration
 *
 *  \ingroup    DPU_AOAPROC_INTERNAL_FUNCTION
 *
 *  @retval  None
 *
 */
void AoAProcHWA_ConvertFov(AOAHwaObj *aoaHwaObj,
                           DPU_AoAProc_FovAoaCfg *fovAoaCfg)
{
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal = sin(fovAoaCfg->minAzimuthDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal = sin(fovAoaCfg->maxAzimuthDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal = sin(fovAoaCfg->minElevationDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal = sin(fovAoaCfg->maxElevationDeg  / 180. * PI_);
}

DPU_AoAProcHWA_Handle DPU_AoAProcHWA_init
(
    DPU_AoAProcHWA_InitParams *initParams,
    int32_t*            errCode
)
{
    AOAHwaObj     *aoaHwaObj = NULL;
    HWA_MemInfo   hwaMemInfo;
    uint8_t       index;
    SemaphoreP_Params  semParams;

    if ((initParams == NULL) || (initParams->hwaHandle == NULL))
    {
        *errCode = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }

    aoaHwaObj = MemoryP_ctrlAlloc(sizeof(AOAHwaObj), 0);
    if(aoaHwaObj == NULL)
    {
        *errCode = DPU_AOAPROCHWA_ENOMEM;
        goto exit;
    }

    /* Save for debugging */
#ifdef DBG_AOA_HWA_OBJ_DPU
    if (gAoaHwaObjInd < RL_MAX_SUBFRAMES)
    {
        gAoaHwaObj[gAoaHwaObjInd++] = aoaHwaObj;
    }
#endif

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    /* Initialize memory */
    memset((void *)aoaHwaObj, 0, sizeof(AOAHwaObj));

    /* Save init config params */
    aoaHwaObj->hwaHandle   = initParams->hwaHandle;

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        aoaHwaObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for HWA done */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    aoaHwaObj->hwaDone_semaHandle = SemaphoreP_create(0, &semParams);
    if(aoaHwaObj->hwaDone_semaHandle == NULL)
    {
        *errCode = DPU_AOAPROCHWA_ESEMA;
        goto exit;
    }


    aoaHwaObj->edmaDstIn2DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0];  //HWA M0
    aoaHwaObj->edmaDstIn2DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1];  //HWA M1

    aoaHwaObj->edmaSrcOut2DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[2];  //HWA M2
    aoaHwaObj->edmaSrcOut2DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[3];  //HWA M3

    aoaHwaObj->edmaDstIn3DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0];  //HWA M0
    aoaHwaObj->edmaDstIn3DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1];  //HWA M1

    aoaHwaObj->edmaSrcOut3DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0] + DPU_AOAPROCHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;  //HWA M0
    aoaHwaObj->edmaSrcOut3DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1] + DPU_AOAPROCHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;  //HWA M1

    aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[2];  //HWA M2
    aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[3];  //HWA M3

exit:
    return ((DPU_AoAProcHWA_Handle)aoaHwaObj);
}

int32_t DPU_AoAProcHWA_config
(
    DPU_AoAProcHWA_Handle    handle,
    DPU_AoAProcHWA_Config    *aoaHwaCfg
)
{
    int32_t   retVal = 0;
    AOAHwaObj *aoaHwaObj = (AOAHwaObj *)handle;
    int32_t i;

    if(aoaHwaObj == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }

    /* Check if radar cube formats are supported. */
    if (!(aoaHwaCfg->res.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1))
    {
        retVal = DPU_AOAPROCHWA_EINVAL__RADARCUBE_DATAFORMAT;
        goto exit;
    }


    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.cfarRngDopSnrList,
                        DPU_AOAPROCHWA_CFAR_DET_LIST_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_CFAR_DET_LIST;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjOut,
                        DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_POINT_CLOUD_CARTESIAN;
        goto exit;
    }
    /* @TODO MMWAVESDK-1868:BEGIN, used same error code */    
    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjOutSph,
                        DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_POINT_CLOUD_CARTESIAN;
        goto exit;
    }
    /* @TODO MMWAVESDK-1868:END */    
    
    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjOutSideInfo,
                        DPU_AOAPROCHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.azimuthStaticHeatMap,
                        DPU_AOAPROCHWA_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_AZIMUTH_STATIC_HEAT_MAP;
        goto exit;
    }

    /* Check if radar cube range column fits into one HWA memory bank */
    if((aoaHwaCfg->staticCfg.numTxAntennas * aoaHwaCfg->staticCfg.numRxAntennas *
            aoaHwaCfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if complex values of 2D FFT fit in one HWA memory bank (16B) */
    /* HWA is running in ping/pong manner, so only one bank is available */
    if((aoaHwaCfg->staticCfg.numTxAntennas * aoaHwaCfg->staticCfg.numRxAntennas *
            aoaHwaCfg->staticCfg.numDopplerBins * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check Heapmap configuration */
    if (aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap)
    {
        if (aoaHwaCfg->res.azimuthStaticHeatMapSize !=
          (aoaHwaCfg->staticCfg.numRangeBins * aoaHwaCfg->staticCfg.numVirtualAntAzim))
        {
            retVal = DPU_AOAPROCHWA_ENOMEM__AZIMUTH_STATIC_HEAT_MAP;
            goto exit;
        }

        if (aoaHwaCfg->res.azimuthStaticHeatMap == NULL)
        {
            retVal = DPU_AOAPROCHWA_EINVAL;
            goto exit;
        }
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjElevationAngle,
                        DPU_AOAPROCHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_DET_OBJ_ELEVATION_ANGLE;
        goto exit;
    }

    if (aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap && (aoaHwaCfg->staticCfg.numVirtualAntAzim == 1))
    {
        retVal = DPU_AOAPROCHWA_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP;
        goto exit;
    }

    if (aoaHwaCfg->res.detObjOutMaxSize & 0x1)
    {
        retVal = DPU_AOAPROCHWA_EDETECTED_OBJECT_LIST_SIZE_ODD_NUMBER;
        goto exit;
    }

    for(i = 0; i < DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFERS; i++)
    {
        if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.localScratchBuffer[i],
                            DPU_AOAPROCHWA_LOCAL_SCRATCH_BYTE_ALIGNMENT)
        {
            retVal = DPU_AOAPROCHWA_ENOMEMALIGN_LOCAL_SCRATCH_BUF;
            goto exit;
        }
        if (aoaHwaCfg->res.localScratchBuffer[i] == NULL)
        {
            retVal = DPU_AOAPROCHWA_EINVAL;
            goto exit;
        }
    }
    if (aoaHwaCfg->res.localScratchBufferSizeBytes !=
      (DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES(aoaHwaCfg->staticCfg.numTxAntennas)))
    {
        retVal = DPU_AOAPROCHWA_ENOMEM_LOCAL_SCRATCH_BUF;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.radarCube.data,
                        DPU_AOAPROCHWA_RADAR_CUBE_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCHWA_ENOMEMALIGN_RADAR_CUBE;
        goto exit;
    }


#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!aoaHwaCfg->edmaHandle ||
      !aoaHwaCfg->hwaHandle ||
      !aoaHwaCfg->params ||
      !aoaHwaCfg->radarCube ||
      !aoaHwaCfg->detObjOut ||
      (aoaHwaCfg->numParamSet != 1)
      )
    {
        retVal= DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
#endif
    aoaHwaObj->res = aoaHwaCfg->res;
    aoaHwaObj->params = aoaHwaCfg->staticCfg;

    aoaHwaObj->dynLocalCfg.compRxChanCfg  = *aoaHwaCfg->dynCfg.compRxChanCfg;
    aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg = *aoaHwaCfg->dynCfg.multiObjBeamFormingCfg;
    aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap  = aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap;
    AoAProcHWA_ConvertFov(aoaHwaObj, aoaHwaCfg->dynCfg.fovAoaCfg);

    aoaHwaObj->dynLocalCfg.extMaxVelCfg = *aoaHwaCfg->dynCfg.extMaxVelCfg;

    /* Allocate buffers for ping and pong paths: */
    for (i = 0; i < DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFERS; i++)
    {
        uint32_t byteAlignment;

        aoaHwaObj->azimuthFftOutMagBuf[i] = (uint16_t *) aoaHwaCfg->res.localScratchBuffer[i];
        aoaHwaObj->azimElevLocalBuf[i] = (uint32_t *) aoaHwaCfg->res.localScratchBuffer[i];

#ifndef SOC_XWR68XX
        /* Speculative workaround for an issue where EDMA is not completing transfer. */
        if (((uint32_t)aoaHwaCfg->staticCfg.numTxAntennas * (uint32_t)aoaHwaCfg->staticCfg.numTxAntennas *
             (uint32_t)aoaHwaCfg->staticCfg.numRxAntennas * sizeof(cmplx16ImRe_t)) == 64U)
        {
            /* Note this will only happen when numTxAntennas = 2 and numRxAntennas = 4 */
            byteAlignment = 64U;
        }
        else
#endif
        {
            byteAlignment = 1U;
        }
        aoaHwaObj->azimElevLocalHypothesesBuf[i] = 
            (uint32_t *) MEM_ALIGN((uint32_t)&aoaHwaObj->azimElevLocalBuf[i][aoaHwaCfg->staticCfg.numTxAntennas *
                                                                             aoaHwaCfg->staticCfg.numRxAntennas],
                                    byteAlignment);
    }

    aoaHwaObj->edmaDstOut2DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimElevLocalBuf[0][0];
    aoaHwaObj->edmaDstOut2DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimElevLocalBuf[1][0];
    aoaHwaObj->edmaSrcIn3DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimElevLocalHypothesesBuf[0][0];
    aoaHwaObj->edmaSrcIn3DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimElevLocalHypothesesBuf[1][0];
    aoaHwaObj->edmaDstOut3DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimuthFftOutMagBuf[0][0];
    aoaHwaObj->edmaDstOut3DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimuthFftOutMagBuf[1][0];


    /* Windowing configuraiton in HWA */
    retVal = HWA_configRam(aoaHwaObj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)aoaHwaCfg->res.hwaCfg.window,
                           aoaHwaCfg->res.hwaCfg.windowSize, /* size in bytes */
                           aoaHwaCfg->res.hwaCfg.winRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}


int32_t DPU_AoAProcHWA_process
(
    DPU_AoAProcHWA_Handle    handle,
    uint32_t        numObjsIn,
    DPU_AoAProcHWA_OutParams  *outParams
)
{
    volatile uint32_t   startTime;
    volatile uint32_t   startTime1;
    volatile uint32_t   waitTimeLocal = 0;
    int32_t             retVal = 0;
    uint16_t            idx;
    uint16_t            detObjIdxIn;
    uint16_t            detObjIdxProc;
    uint8_t             pingPongIdx;
    uint32_t            trueNumObjsIn;

    float               range;
    uint32_t            numObjsOut = 0;
    uint16_t numParams;
    HWA_Handle hwaHandle;

    AOAHwaObj *aoaHwaObj;
    DPU_AoAProcHWA_HW_Resources *res;
    DPU_AoAProcHWA_StaticConfig *DPParams;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    aoaHwaObj = (AOAHwaObj *)handle;
    res = &aoaHwaObj->res;
    DPParams = &aoaHwaObj->params;
    hwaHandle = aoaHwaObj->hwaHandle;

    startTime = Cycleprofiler_getTimeStamp();

    if (numObjsIn == 0U)
    {
        outParams->numAoADetectedPoints = numObjsIn;
        outParams->stats.waitTime = 0;
        outParams->stats.processingTime = 0;
        retVal = 0;
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(hwaHandle,
                                     AOAProcHWADoneIsrCallback,
                                     aoaHwaObj->hwaDone_semaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Azimuth heap doppler compensation */
    if (aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap)
    {
        /* Perform 2D FFT for all range bins and save zero Doppler bin */
        retVal = AoAProcHWA_PrepareRangeAzimuthHeatmap(aoaHwaObj,
                                                       &waitTimeLocal);
        if (retVal != 0)
        {
            goto exit;
        }
    }

    /* Angle estimation */
    if(DPParams->numVirtualAntAzim == 1)
    {
       /*If there is only one virtual antenna, there is no
         need of azimuth FFT as azimuth can not be estimated.*/

        /* Limit number of input samples */
        if (numObjsIn > res->detObjOutMaxSize)
        {
          numObjsIn = res->detObjOutMaxSize;
        }

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
        /* Limit number of input samples */
        if (numObjsIn > res->detObjOutMaxSize)
        {
          numObjsIn = res->detObjOutMaxSize;
        }
        trueNumObjsIn = numObjsIn;
        /* Make number of detected objects EVEN */
        if (numObjsIn & 0x1)
        {
            /* If odd number of input objects, add dummy as duplicated last object */
            res->cfarRngDopSnrList[numObjsIn] = res->cfarRngDopSnrList[numObjsIn-1];
            numObjsIn++;
        }


        /* Configure HWA Params*/
        retVal = HWAutil_configHWA_extendedVelocityScheme(hwaHandle,
                                        res,
                                        DPParams,
                                        aoaHwaObj->hwaMemBankAddr,
                                        aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled,
                                        DPU_AOAPROCHWA_NUM_ANGLE_BINS);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Configure Common HWA registers */
        numParams = 2*(DPParams->numTxAntennas + 2 + ((DPParams->numVirtualAntElev > 0) & 0x1));
        retVal =  AoAProc_HWAutil_configCommon (hwaHandle,
                                                numObjsIn/2, //numLoops,
                                                res->hwaCfg.paramSetStartIdx, //paramStartIdx,
                                                numParams);
        if (retVal != 0)
        {
            goto exit;
        }


        /* Configure EDMA */
        retVal = AoAProcHWA_config_EDMA(hwaHandle,
                                        res,
                                        DPParams,
                                        0,//srcIn2DFFTBuffAddr
                                        aoaHwaObj->edmaDstIn2DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcOut2DFFTBuffAddr,
                                        aoaHwaObj->edmaDstOut2DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcIn3DFFTBuffAddr,
                                        aoaHwaObj->edmaDstIn3DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcOut3DFFTBuffAddr,
                                        aoaHwaObj->edmaDstOut3DFFTBuffAddr,
                                        aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Enable the HWA */
        retVal = HWA_enable(hwaHandle, 1);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Trigger first two objects */
        detObjIdxIn = 0;
        pingPongIdx = 0;
        AoAProcHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
                                      detObjIdxIn,
                                      pingPongIdx);
        detObjIdxIn = 1;
        pingPongIdx = 1;
        AoAProcHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
                                      detObjIdxIn,
                                      pingPongIdx);
        /* Loop through the list two objects per loop */
        detObjIdxIn = 2;
        detObjIdxProc = 0;
        while(detObjIdxProc < numObjsIn)
        {
            for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
            {
                /* Wait until EDMA output is done */
                startTime1 = Cycleprofiler_getTimeStamp();
                retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
                waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
                if (retVal != 0)
                {
                      goto exit;
                }
                /* Rx channel gain/phase offset compensation */
                AoAProcHWA_rxChanPhaseBiasCompensation(&aoaHwaObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[0],
                                                    1,
                                                    DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev,
                                                    (cmplx16ImRe_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx],
                                                    (cmplx16ImRe_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx]);
                /* Doppler Compensation - */
                aoaHwa_dopplerCompensation((uint32_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx],
                                           &res->cfarRngDopSnrList[detObjIdxProc + pingPongIdx],
                                           (uint32_t *) aoaHwaObj->azimElevLocalHypothesesBuf[pingPongIdx],
                                           DPParams->numTxAntennas,
                                           DPParams->numRxAntennas,
                                           DPParams->numVirtualAntAzim,
                                           DPParams->numVirtualAntElev,
                                           DPParams->numDopplerBins,
                                           aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled);
                /* Trigger 3D-FFT */
                AoAProcHWA_trigger_EDMA_3DFFT(aoaHwaObj, pingPongIdx);
            }

            for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
            {
                /* Wait until EDMA outputs 3D-FFTs and Maximum peaks */
                startTime1 = Cycleprofiler_getTimeStamp();
                retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
                waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
                if (retVal != 0)
                {
                      goto exit;
                }
                if (detObjIdxProc < trueNumObjsIn)
                {
                    numObjsOut = AoAProcHWA_angleEstimationAzimElev(aoaHwaObj,
                                                                    detObjIdxProc,
                                                                    pingPongIdx,
                                                                    numObjsOut);
                }
                if (detObjIdxIn < numObjsIn)
                {
                    /* Trigger 2D-FFT - next target */
                    AoAProcHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
                                                         detObjIdxIn,
                                                         pingPongIdx);
                }
                detObjIdxProc++;
                detObjIdxIn++;
            }

        } /* Loop per two detected points */



        /* Disable the HWA */
        retVal = HWA_enable(hwaHandle, 0);
        if (retVal != 0)
        {
            goto exit;
        }

    }

    outParams->numAoADetectedPoints = numObjsOut;
    outParams->stats.waitTime = waitTimeLocal;
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitTimeLocal;
#ifdef PROFILE_AOA_HWA_OBJ_DPU
    gAoAProcStats[gAoAProcStatsIdx] = *outParams;
    gAoAProcStatsIdx = (gAoAProcStatsIdx +1) & 0xf;
#endif

    /* Process is done, disable Done interrupt */
    HWA_disableDoneInterrupt(hwaHandle);
exit:
    return retVal;
}

int32_t DPU_AoAProcHWA_control
(
   DPU_AoAProcHWA_Handle handle,
   DPU_AoAProcHWA_Cmd cmd,
   void *arg,
   uint32_t argSize
)
{
   int32_t retVal = 0;
   AOAHwaObj *aoaHwaObj = (AOAHwaObj *)handle;

   /* Get rangeProc data object */
   if (aoaHwaObj == NULL)
   {
       retVal = DPU_AOAPROCHWA_EINVAL;
       goto exit;
   }

   switch(cmd)
   {
       case DPU_AoAProcHWA_Cmd_FovAoACfg:
       {
           if((argSize != sizeof(DPU_AoAProc_FovAoaCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               AoAProcHWA_ConvertFov(aoaHwaObj, (DPU_AoAProc_FovAoaCfg *) arg);

           }
       }
       break;
       case DPU_AoAProcHWA_Cmd_MultiObjBeamFormingCfg:
       {
           if((argSize != sizeof(DPU_AoAProc_MultiObjBeamFormingCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcHWA_Cmd_ExtMaxVelocityCfg:
       {
           if((argSize != sizeof(DPU_AoAProc_ExtendedMaxVelocityCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.extMaxVelCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcHWA_Cmd_CompRxChannelBiasCfg:
       {
           if((argSize != sizeof(DPU_AoAProc_compRxChannelBiasCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.compRxChanCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcHWA_Cmd_PrepareRangeAzimuthHeatMap:
       {
           if((argSize != sizeof(bool)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCHWA_EINVAL;
               goto exit;
           }
           else
           {
                /* Sanity check on saved heapmap ptr and size */
                if ((aoaHwaObj->res.azimuthStaticHeatMapSize !=
                    (aoaHwaObj->params.numRangeBins * aoaHwaObj->params.numVirtualAntAzim)) ||
                    (aoaHwaObj->res.azimuthStaticHeatMap == NULL))
                {
                   retVal = DPU_AOAPROCHWA_EINVAL;
                   goto exit;
                }

                /* Save configuration */
                memcpy((void *)&aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap, arg, argSize);
           }
       }
       break;
       default:
           retVal = DPU_AOAPROCHWA_EINVAL;
           break;
   }
exit:
   return (retVal);
}

int32_t DPU_AoAProcHWA_deinit(DPU_AoAProcHWA_Handle handle)
{
    int32_t retVal = 0;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCHWA_EINVAL;
        goto exit;
    }
    MemoryP_ctrlFree(handle, sizeof(AOAHwaObj));
exit:
    return (retVal);
}

