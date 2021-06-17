/*
 *   @file  objdetrangehwa.c
 *
 *   @brief
 *      Object Detection DPC implementation with range HWA DPU only
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
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define DBG_DPC_OBJDETRANGEHWA

/* mmWave SDK Include Files: */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>

/* DPUs */
#include <common/src/dpu/rangeprochwa/rangeprochwa.h>

 /** @addtogroup DPC_OBJDETRANGEHWA_IOCTL__INTERNAL_DEFINITIONS
  @{ */

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this object detection DPC are prefixed by DPC_OBJDETRANGEHWA */
#include APP_RESOURCE_FILE

/* Obj Det instance etc */
#include <common/src/dpc/objectdetectionandtracking/objdetrangehwa/include/objdetrangehwainternal.h>
#include <common/src/dpc/objectdetectionandtracking/objdetrangehwa/objdetrangehwa.h>

#ifdef DBG_DPC_OBJDETRANGEHWA
ObjDetObj     *gObjDetObj;
#endif

/*! Radar cube data buffer alignment in bytes. No DPU module specifying alignment
 *  need (through a \#define) implies that no alignment is needed i.e 1 byte alignment.
 *  But we do the natural data type alignment which is 2 bytes (as radar cube is complex16-bit type)
 *  because radar cube is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing.
 */
#define DPC_OBJDETRANGEHWA_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT       (sizeof(int16_t))


/**
@}
*/

#define DPC_OBJDETRANGEHWA_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES    SOC_HWA_WINDOW_RAM_SIZE_IN_SAMPLES

/******************************************************************************/
/* Local definitions */
#define DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE            MATHUTILS_WIN_BLACKMAN


#define DPC_OBJDET_QFORMAT_RANGE_FFT 17

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/

static DPM_DPCHandle DPC_ObjectDetection_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
);

static int32_t DPC_ObjectDetection_execute
(
    DPM_DPCHandle handle,
    DPM_Buffer*       ptrResult
);

static int32_t DPC_ObjectDetection_ioctl
(
    DPM_DPCHandle   handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
);

static int32_t DPC_ObjectDetection_start  (DPM_DPCHandle handle);
static int32_t DPC_ObjectDetection_stop   (DPM_DPCHandle handle);
static int32_t DPC_ObjectDetection_deinit (DPM_DPCHandle handle);
static void DPC_ObjectDetection_frameStart (DPM_DPCHandle handle);

/**************************************************************************
 ************************* Global Declarations ****************************
 **************************************************************************/

/** @addtogroup DPC_OBJDETRANGEHWA__GLOBAL
 @{ */

/**
 * @brief   Global used to register Object Detection DPC in DPM
 */
DPM_ProcChainCfg gDPC_ObjDetRangeHWACfg =
{
    DPC_ObjectDetection_init,            /* Initialization Function:         */
    DPC_ObjectDetection_start,           /* Start Function:                  */
    DPC_ObjectDetection_execute,         /* Execute Function:                */
    DPC_ObjectDetection_ioctl,           /* Configuration Function:          */
    DPC_ObjectDetection_stop,            /* Stop Function:                   */
    DPC_ObjectDetection_deinit,          /* Deinitialization Function:       */
    NULL,                                /* Inject Data Function:            */
    NULL,                                /* Chirp Available Function:        */
    DPC_ObjectDetection_frameStart       /* Frame Start Function:            */
};

/* @} */

/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDETRANGEHWA_INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
static void DPC_ObjDetRangeHwa_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}


/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDETRANGEHWA_INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
static uint32_t DPC_ObjDetRangeHwa_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_OBJDETRANGEHWA_INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
static void *DPC_ObjDetRangeHwa_MemPoolAlloc(MemPoolObj *pool,
                              uint32_t size,
                              uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
}


/**
 *  @b Description
 *  @n
 *      Sends Assert
 *
 *  @retval
 *      Not Applicable.
 */
void _DPC_Objdet_Assert(DPM_Handle handle, int32_t expression,
                        const char *file, int32_t line)
{
    DPM_DPCAssert       fault;

    if (!expression)
    {
        fault.lineNum = (uint32_t)line;
        fault.arg0    = 0U;
        fault.arg1    = 0U;
        strncpy (fault.fileName, file, (DPM_MAX_FILE_NAME_LEN-1));

        /* Report the fault to the DPM entities */
        DPM_ioctl (handle,
                   DPM_CMD_DPC_ASSERT,
                   (void*)&fault,
                   sizeof(DPM_DPCAssert));
    }
}


/**
 *  @b Description
 *  @n
 *      Computes the length of window to generate for range DPU.
 *
 *  @param[in]  numAdcSamples   Number of ADC samples
 *
 *  @retval   Length of window to generate
 *
 * \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetRangeHwa_GetRangeWinGenLen(uint16_t numAdcSamples)
{
    uint32_t winGenLen;

#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    winGenLen = (numAdcSamples + 1)/2;
#else
    winGenLen = numAdcSamples;
#endif
    return(winGenLen);
}

/**
 *  @b Description
 *  @n
 *      Generate the range DPU window using mathutils API.
 *
 *  @param[in]  cfg     Range DPU configuration, output window is generated in window
 *                      pointer in the staticCfg of this.
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 */
static void DPC_ObjDetRangeHwa_GenRangeWindow(DPU_RangeProcHWA_Config *cfg)
{
    mathUtils_genWindow((uint32_t *)cfg->staticCfg.window,
                        cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                        DPC_ObjDetRangeHwa_GetRangeWinGenLen(cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples),
                        DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                        DPC_OBJDET_QFORMAT_RANGE_FFT);
}


/**
 *  @b Description
 *  @n
 *      Sub-frame reconfiguration, used when switching sub-frames. Invokes the
 *      DPU configuration using the configuration that was stored during the
 *      pre-start configuration so reconstruction time is saved  because this will
 *      happen in real-time.
 *  @param[in]  objDetObj       Pointer to DPC object
 *  @param[in]  subFrameIndx    Sub-frame index.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 * \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetRangeHwa_reconfigSubFrame(ObjDetObj *objDetObj, uint8_t subFrameIndx)
{
    int32_t retVal = 0;
    SubFrameObj *subFrmObj;

    subFrmObj = &objDetObj->subFrameObj[subFrameIndx];

    if(objDetObj->commonCfg.numSubFrames > 1)
    {
        DPC_ObjDetRangeHwa_GenRangeWindow(&subFrmObj->rangeCfg);
        retVal = DPU_RangeProcHWA_config(subFrmObj->dpuRangeObj, &subFrmObj->rangeCfg);
        if (retVal != 0)
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
 *     Configure range DPU.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg    Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube Pointer to DPIF radar cube, which is output of range
 *                        processing.
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, only for scratch purposes
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetRangeHwa_rangeConfig
(
    DPU_RangeProcHWA_Handle dpuHandle,
    DPC_ObjectDetectionRangeHWA_StaticCfg *staticCfg,
    DPC_ObjectDetectionRangeHWA_DynCfg *dynCfg,
    EDMA_Handle                   edmaHandle,
    DPIF_RadarCube                *radarCube,
    MemPoolObj                    *CoreLocalRamObj,
    DPU_RangeProcHWA_Config       *cfgSave
)
{
    int32_t retVal = 0;
    DPU_RangeProcHWA_Config rangeCfg;
    DPU_RangeProcHWA_HW_Resources *hwRes = &rangeCfg.hwRes;
    DPU_RangeProcHWA_EDMAInputConfig *edmaIn = &hwRes->edmaInCfg;
    DPU_RangeProcHWA_EDMAOutputConfig *edmaOut = &hwRes->edmaOutCfg;
    DPU_RangeProcHWA_HwaConfig *hwaCfg = &hwRes->hwaCfg;
    uint32_t numRxAntennas, winGenLen;

    memset(&rangeCfg, 0, sizeof(rangeCfg));

    numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;

    /* Even though Range DPU supports both modes,
     * object detection DPC only supports non-interleaved at present */
    DebugP_assert(staticCfg->ADCBufData.dataProperty.interleave == DPIF_RXCHAN_NON_INTERLEAVE_MODE);

    /* dynamic configuration */
    rangeCfg.dynCfg.calibDcRangeSigCfg = &dynCfg->calibDcRangeSigCfg;

    /* static configuration */
    rangeCfg.staticCfg.ADCBufData         = staticCfg->ADCBufData;
    rangeCfg.staticCfg.numChirpsPerFrame  = staticCfg->numChirpsPerFrame;
    rangeCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    rangeCfg.staticCfg.numTxAntennas      = staticCfg->numTxAntennas;
    rangeCfg.staticCfg.numVirtualAntennas = staticCfg->numVirtualAntennas;
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 1;
    rangeCfg.staticCfg.rangeFFTtuning.fftOutputDivShift = staticCfg->rangeFFTtuning.fftOutputDivShift;
    rangeCfg.staticCfg.rangeFFTtuning.numLastButterflyStagesToScale = staticCfg->rangeFFTtuning.numLastButterflyStagesToScale;
  
    /* radarCube */
    rangeCfg.hwRes.radarCube = *radarCube;

    /* static configuration - window */
    /* Generating 1D window, allocate first */
    winGenLen = DPC_ObjDetRangeHwa_GetRangeWinGenLen(rangeCfg.staticCfg.ADCBufData.dataProperty.numAdcSamples);
    rangeCfg.staticCfg.windowSize = winGenLen * sizeof(uint32_t);
    rangeCfg.staticCfg.window = (int32_t *)DPC_ObjDetRangeHwa_MemPoolAlloc(CoreLocalRamObj, rangeCfg.staticCfg.windowSize, sizeof(uint32_t));
    if (rangeCfg.staticCfg.window == NULL)
    {
        retVal = DPC_OBJDETRANGEHWA_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }
    DPC_ObjDetRangeHwa_GenRangeWindow(&rangeCfg);

    /* hwres - dcRangeSig, allocate from heap, this needs to persist within sub-frame/frame
     * processing and across sub-frames */
    hwRes->dcRangeSigMeanSize = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
               staticCfg->numTxAntennas * numRxAntennas * sizeof(cmplx32ImRe_t);
    hwRes->dcRangeSigMean = (cmplx32ImRe_t *) MemoryP_ctrlAlloc (hwRes->dcRangeSigMeanSize,
                                       (uint8_t) 0 /*sizeof(cmplx32ImRe_t)*/);
    DebugP_assert(rangeCfg.hwRes.dcRangeSigMeanSize == hwRes->dcRangeSigMeanSize);

    /* hwres - edma */
    hwRes->edmaHandle = edmaHandle;
    /* We have choosen ISOLATE mode, so we have to fill in dataIn */
    edmaIn->dataIn.channel                  = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    edmaIn->dataIn.channelShadow            = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW;
    edmaIn->dataIn.eventQueue               = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    edmaIn->dataInSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    edmaIn->dataInSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    edmaIn->dataInSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;

    /* We are radar Cube FORMAT1 and non-interleaved ADC, so for 3 tx antenna case, we have to
     * fill format2, otherwise format1
     */
    if ((staticCfg->numTxAntennas == 3) && (radarCube->datafmt == DPIF_RADARCUBE_FORMAT_1))
    {
        /* Ping */
        /* Ping - dataOutPing */
        edmaOut->u.fmt2.dataOutPing.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PING_CH;
        edmaOut->u.fmt2.dataOutPing.channelShadow[0]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PING_SHADOW_0;
        edmaOut->u.fmt2.dataOutPing.channelShadow[1]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PING_SHADOW_1;
        edmaOut->u.fmt2.dataOutPing.channelShadow[2]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PING_SHADOW_2;
        edmaOut->u.fmt2.dataOutPing.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PING_EVENT_QUE;
        /* Ping - dataOutPingData */
        edmaOut->u.fmt2.dataOutPingData[0].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_0_CH;
        edmaOut->u.fmt2.dataOutPingData[0].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_0_SHADOW;
        edmaOut->u.fmt2.dataOutPingData[0].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_0_EVENT_QUE;
        edmaOut->u.fmt2.dataOutPingData[1].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_1_CH;
        edmaOut->u.fmt2.dataOutPingData[1].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_1_SHADOW;
        edmaOut->u.fmt2.dataOutPingData[1].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_1_EVENT_QUE;
        edmaOut->u.fmt2.dataOutPingData[2].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_2_CH;
        edmaOut->u.fmt2.dataOutPingData[2].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_2_SHADOW;
        edmaOut->u.fmt2.dataOutPingData[2].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PINGDATA_2_EVENT_QUE;

        /* Pong */
        /* Pong - dataOutPong */
        edmaOut->u.fmt2.dataOutPong.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONG_CH;
        edmaOut->u.fmt2.dataOutPong.channelShadow[0]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONG_SHADOW_0;
        edmaOut->u.fmt2.dataOutPong.channelShadow[1]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONG_SHADOW_1;
        edmaOut->u.fmt2.dataOutPong.channelShadow[2]     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONG_SHADOW_2;
        edmaOut->u.fmt2.dataOutPong.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONG_EVENT_QUE;
        /* Pong - dataOutPongData */
        edmaOut->u.fmt2.dataOutPongData[0].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_0_CH;
        edmaOut->u.fmt2.dataOutPongData[0].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_0_SHADOW;
        edmaOut->u.fmt2.dataOutPongData[0].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_0_EVENT_QUE;
        edmaOut->u.fmt2.dataOutPongData[1].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_1_CH;
        edmaOut->u.fmt2.dataOutPongData[1].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_1_SHADOW;
        edmaOut->u.fmt2.dataOutPongData[1].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_1_EVENT_QUE;
        edmaOut->u.fmt2.dataOutPongData[2].channel       = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_2_CH;
        edmaOut->u.fmt2.dataOutPongData[2].channelShadow = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_2_SHADOW;
        edmaOut->u.fmt2.dataOutPongData[2].eventQueue    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT2_PONGDATA_2_EVENT_QUE;
    }
    else
    {
        /* Ping */
        edmaOut->u.fmt1.dataOutPing.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_CH;
        edmaOut->u.fmt1.dataOutPing.channelShadow        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_SHADOW;
        edmaOut->u.fmt1.dataOutPing.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_EVENT_QUE;

        /* Pong */
        edmaOut->u.fmt1.dataOutPong.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_CH;
        edmaOut->u.fmt1.dataOutPong.channelShadow        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_SHADOW;
        edmaOut->u.fmt1.dataOutPong.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_EVENT_QUE;
    }

    edmaOut->dataOutSignature.channel                    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_CH;
    edmaOut->dataOutSignature.channelShadow              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_SHADOW;
    edmaOut->dataOutSignature.eventQueue                 = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_EVENT_QUE;

    /* hwres - hwa */
    /* Use ISOLATED mode to support CBUFF in future */
    hwaCfg->dataInputMode = DPU_RangeProcHWA_InputMode_ISOLATED;

#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_SYMMETRIC;
#else
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_NONSYMMETRIC;
#endif
    hwaCfg->hwaWinRamOffset = (uint16_t) DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;
    if ((hwaCfg->hwaWinRamOffset + winGenLen) > DPC_OBJDETRANGEHWA_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES)
    {
        retVal = DPC_OBJDETRANGEHWA_ENOMEM_HWA_WINDOW_RAM;
        goto exit;
    }

    hwaCfg->numParamSet = DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS;
    hwaCfg->paramSetStartIdx = DPC_OBJDET_DPU_RANGEPROC_PARAMSET_START_IDX;
        
    *cfgSave = rangeCfg;

    retVal = DPU_RangeProcHWA_config(dpuHandle, &rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated and
     * dc range sig should not be reset. */
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 0;
    *cfgSave = rangeCfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Performs processing related to pre-start configuration, which is per sub-frame,
 *     by configuring each of the DPUs involved in the processing chain.
 *  Memory management notes:
 *  1. Core Local Memory that needs to be preserved across sub-frames (such as range DPU's calib DC buffer)
 *     will be allocated using MemoryP_alloc.
 *  2. Core Local Memory that needs to be preserved within a sub-frame across DPU calls
 *     (the DPIF * type memory) or for intermediate private scratch memory for
 *     DPU (i.e no preservation is required from process call to process call of the DPUs
 *     within the sub-frame) will be allocated from the Core Local RAM configuration supplied in
 *     @ref DPC_ObjectDetectionRangeHWA_InitParams given to @ref DPC_ObjectDetection_init API
 *  3. L3 memory will only be allocated from the L3 RAM configuration supplied in
 *     @ref DPC_ObjectDetectionRangeHWA_InitParams given to @ref DPC_ObjectDetection_init API
 *     No L3 buffers are presently required that need to be preserved across sub-frames
 *     (type described in #1 above), neither are L3 scratch buffers required for
 *     intermediate processing within DPU process call.
 *
 *  @param[in]  subFrmObj   Pointer to sub-frame object
 *  @param[in]  objDetObj   Pointer to objDetObj
 *  @param[in]  staticCfg   Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg      Pointer to dynamic configuration of the sub-frame
 *  @param[out]  memUsage   Pointer to memory usage structure to report mmemory 
                            usage in this DPC
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetRangeHwa_preStartConfig
(
    SubFrameObj *subFrmObj,
    ObjDetObj   *objDetObj,
    DPC_ObjectDetectionRangeHWA_StaticCfg *staticCfg,
    DPC_ObjectDetectionRangeHWA_DynCfg *dynCfg,
    DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage *memUsage
)
{
    int32_t             retVal = 0;
    DPIF_RadarCube      radarCube;
    MemPoolObj          *L3RamObj;
    MemPoolObj          *CoreLocalRamObj;

    L3RamObj = &objDetObj->L3RamObj;
    CoreLocalRamObj = &objDetObj->CoreLocalRamObj;

    DPC_ObjDetRangeHwa_MemPoolReset(L3RamObj);
    DPC_ObjDetRangeHwa_MemPoolReset(CoreLocalRamObj);

    /* L3 allocations */
    /* L3 - radar cube -- This has be synced with the DSP chain */
    radarCube.dataSize = staticCfg->numRangeBins * staticCfg->numDopplerChirps *
                         staticCfg->numVirtualAntennas * sizeof(cmplx16ReIm_t);
    radarCube.data = DPC_ObjDetRangeHwa_MemPoolAlloc(L3RamObj, radarCube.dataSize,
                         DPC_OBJDETRANGEHWA_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);
    if (radarCube.data == NULL)
    {
        retVal = DPC_OBJDETRANGEHWA_ENOMEM__L3_RAM_RADAR_CUBE;
        goto exit;
    }
    radarCube.datafmt = staticCfg->radarCubeFormat;

    retVal = DPC_ObjDetRangeHwa_rangeConfig(subFrmObj->dpuRangeObj,
                 staticCfg,
                 dynCfg,
                 objDetObj->edmaHandle,
                 &radarCube, CoreLocalRamObj,
                 &subFrmObj->rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Report RAM usage */
    memUsage->CoreLocalRamUsage = DPC_ObjDetRangeHwa_MemPoolGetMaxUsage(CoreLocalRamObj);
    memUsage->L3RamUsage = DPC_ObjDetRangeHwa_MemPoolGetMaxUsage(L3RamObj);

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      DPC frame start function registered with DPM. This is invoked on reception
 *      of the frame start ISR from the RF front-end. 
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
static void DPC_ObjectDetection_frameStart (DPM_DPCHandle handle)
{
    ObjDetObj     *objDetObj = (ObjDetObj *) handle;

    objDetObj->stats.frameStartTimeStamp = Cycleprofiler_getTimeStamp();

    DebugP_log2("ObjDet DPC: Frame Start, frameIndx = %d, subFrameIndx = %d\n",
                objDetObj->stats.frameStartIntCounter, objDetObj->subFrameIndx);

    /* Check if previous frame (sub-frame) processing has completed */
    DPC_Objdet_Assert(objDetObj->dpmHandle, (objDetObj->interSubFrameProcToken == 0));
    objDetObj->interSubFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */
    if (objDetObj->subFrameIndx == 0)
    {
        objDetObj->stats.frameStartIntCounter++;
    }

    /* Notify the DPM Module that the DPC is ready for execution */
    DebugP_assert (DPM_notifyExecute (objDetObj->dpmHandle, handle, true) == 0);
    return;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) execute function which is invoked by the application
 *      in the DPM's execute context when the DPC issues DPM_notifyExecute API from
 *      its registered @ref DPC_ObjectDetection_frameStart API that is invoked every
 *      frame interrupt.
 *
 *  @param[in]  handle       DPM's DPC handle
 *  @param[out]  ptrResult   Pointer to the result
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
int32_t DPC_ObjectDetection_execute
(
    DPM_DPCHandle   handle,
    DPM_Buffer*     ptrResult
)
{
    ObjDetObj   *objDetObj;
    SubFrameObj *subFrmObj;
    DPU_RangeProcHWA_OutParams outRangeProc;
    int32_t retVal;
    DPM_Buffer      rangeProcResult;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);
    DebugP_assert (ptrResult != NULL);

    DebugP_log1("ObjDet DPC: Processing sub-frame %d\n", objDetObj->subFrameIndx);

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];
    if (objDetObj->processCallBackFxn.processFrameBeginCallBackFxn != NULL)
    {
        (*objDetObj->processCallBackFxn.processFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
    }

    retVal = DPU_RangeProcHWA_process(subFrmObj->dpuRangeObj, &outRangeProc);
    if (retVal != 0)
    {
        goto exit;
    }
    DebugP_assert(outRangeProc.endOfChirp == true);

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];
    if (objDetObj->processCallBackFxn.processInterFrameBeginCallBackFxn != NULL)
    {
        (*objDetObj->processCallBackFxn.processInterFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
    }

    /********************************************************************************
     * Range Processing is finished, now it can send data to inter frame processing DPC.
     ********************************************************************************/
    rangeProcResult.ptrBuffer[0] = objDetObj->subFrameObj[objDetObj->subFrameIndx].rangeCfg.hwRes.radarCube.data;
    rangeProcResult.size[0] = objDetObj->subFrameObj[objDetObj->subFrameIndx].rangeCfg.hwRes.radarCube.dataSize;

    /* Relay the results: */
    retVal = DPM_relayResult(objDetObj->dpmHandle, handle, &rangeProcResult);
    DebugP_assert (retVal == 0);

    /********************************************************************************
     * This DPC does not report results to the application so for the
     * sake of clarity reset the result buffer
     ********************************************************************************/
    memset ((void *)ptrResult, 0, sizeof(DPM_Buffer));
    objDetObj->interSubFrameProcToken--;

    /********************************************************************************
     * Prepare for next Frame
     ********************************************************************************/

    /* Update subFrame index */
    objDetObj->subFrameIndx++;
    objDetObj->subFrameIndx = objDetObj->subFrameIndx % objDetObj->commonCfg.numSubFrames;

    if(objDetObj->commonCfg.numSubFrames > 1U)
    {
        /* Re-configure Range DPU for next subFrame */
        retVal = DPC_ObjDetRangeHwa_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
        DebugP_assert (retVal == 0);
    }

    /* Trigger Range DPU for next frame */
    retVal = DPU_RangeProcHWA_control(objDetObj->subFrameObj[objDetObj->subFrameIndx].dpuRangeObj,
                 DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    DPC_Objdet_Assert(objDetObj->dpmHandle, (retVal == 0));

    /* For rangeProcHwa, interChirpProcessingMargin is not available */
    objDetObj->stats.interChirpProcessingMargin = 0;
    objDetObj->stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();

    DebugP_log0("ObjDet DPC: Range Proc Done\n");

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) start function which is invoked by the
 *      application using DPM_start API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_start (DPM_DPCHandle handle)
{
    ObjDetObj   *objDetObj;
    SubFrameObj *subFrmObj;
    int32_t retVal = 0;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);

    objDetObj->stats.frameStartIntCounter = 0;

    /* Start marks consumption of all pre-start configs, reset the flag to check
     * if pre-starts were issued only after common config was issued for the next
     * time full configuration happens between stop and start */
    objDetObj->isCommonCfgReceived = false;

    /* App must issue export of last frame after stop which will switch to sub-frame 0,
     * so start should always see sub-frame indx of 0, check */
    DebugP_assert(objDetObj->subFrameIndx == 0);

    if(objDetObj->commonCfg.numSubFrames > 1U)
    {
        /* Pre-start cfgs for sub-frames may have come in any order, so need
         * to ensure we reconfig for the current (0) sub-frame before starting */
        DPC_ObjDetRangeHwa_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
    }

    /* Trigger Range DPU, related to reconfig above */
    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];
    retVal = DPU_RangeProcHWA_control(subFrmObj->dpuRangeObj,
                 DPU_RangeProcHWA_Cmd_triggerProc, NULL, 0);
    if(retVal < 0)
    {
        goto exit;
    }

    DebugP_log0("ObjDet DPC: Start done\n");
exit:
    return(retVal);
}


/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) stop function which is invoked by the
 *      application using DPM_stop API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_stop (DPM_DPCHandle handle)
{
    ObjDetObj       *objDetObj;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);

    /* We can be here only after complete frame processing is done, which means
     * processing token must be 0 and subFrameIndx also 0  */
    DebugP_assert(objDetObj->subFrameIndx == 0);

    DebugP_log0("ObjDet DPC: Stop done\n");
    return(0);
}

/**
 *  @b Description
 *  @n
 *      DPC IOCTL commands configuration API which will be invoked by the
 *      application using DPM_ioctl API
 *
 *  @param[in]  handle   DPM's DPC handle
 *  @param[in]  cmd      Capture DPC specific commands
 *  @param[in]  arg      Command specific arguments
 *  @param[in]  argLen   Length of the arguments which is also command specific
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_ioctl
(
    DPM_DPCHandle   handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
)
{
    ObjDetObj   *objDetObj;
    SubFrameObj *subFrmObj;
    int32_t      retVal = 0;

    /* Get the DSS MCB: */
    objDetObj = (ObjDetObj *) handle;
    DebugP_assert(objDetObj != NULL);
    DebugP_assert(arg != NULL);

    /* Process the commands. Process non sub-frame specific ones first
     * so the sub-frame specific ones can share some code. */
    if((cmd < DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG) ||
        (cmd > DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG))
    {
        retVal = DPM_EINVCMD;
    }
    else if (cmd == DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
        uint8_t     index;
        DPC_ObjectDetectionRangeHWA_PreStartCommonCfg *cfg;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetectionRangeHWA_PreStartCommonCfg));

        cfg = (DPC_ObjectDetectionRangeHWA_PreStartCommonCfg*)arg;

        /* Release DPU DC range Signal mean memory */
        for(index = 0; index < objDetObj->commonCfg.numSubFrames; index ++)
        {
            subFrmObj = &objDetObj->subFrameObj[index];

            if (subFrmObj->rangeCfg.hwRes.dcRangeSigMean)
            {
                MemoryP_ctrlFree(subFrmObj->rangeCfg.hwRes.dcRangeSigMean,
                                 subFrmObj->rangeCfg.hwRes.dcRangeSigMeanSize);
            }
        }

        objDetObj->commonCfg = *cfg;
        objDetObj->isCommonCfgReceived = true;

        DebugP_log0("ObjDet DPC: Pre-start Common Config IOCTL processed\n");
    }
#ifndef TRACKERPROC_ON_DSP
	else if (cmd == DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG)
    {
        DPC_ObjectDetection_TrackerConfig *cfg;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_TrackerConfig));

        cfg = (DPC_ObjectDetection_TrackerConfig*)arg;

        retVal = DPU_TrackerProc_config(objDetObj->dpuTrackerObj, &(cfg->trackerDpuCfg));
        if (retVal != 0)
        {
            goto exit;
        }
    }
#endif
    else
    {
        uint8_t     subFrameNum;
        subFrameNum = *(uint8_t *)arg;

        switch (cmd)
        {
            /* Range DPU related */
            case DPC_OBJDETRANGEHWA_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG:
            {
                DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg));

                cfg = (DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg*)arg;

                subFrmObj = &objDetObj->subFrameObj[subFrameNum];

                retVal = DPU_RangeProcHWA_control(subFrmObj->dpuRangeObj,
                             DPU_RangeProcHWA_Cmd_dcRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_RangeProc_CalibDcRangeSigCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.calibDcRangeSigCfg = cfg->cfg;

                break;
            }

            /* Related to pre-start configuration */
            case DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG:
            {
                DPC_ObjectDetectionRangeHWA_PreStartCfg *cfg;
                DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage *memUsage;
                MemoryP_Stats statsStart;
                MemoryP_Stats statsEnd;

                /* Pre-start common config must be received before pre-start configs are received. */
                if (objDetObj->isCommonCfgReceived == false)
                {
                    retVal = DPC_OBJDETRANGEHWA_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;
                    goto exit;
                }

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionRangeHWA_PreStartCfg));

                /* Get system heap size before preStart configuration */
                MemoryP_getStats(&statsStart);

                cfg = (DPC_ObjectDetectionRangeHWA_PreStartCfg*)arg;
                subFrmObj = &objDetObj->subFrameObj[subFrameNum];

                memUsage = &cfg->memUsage;
                memUsage->L3RamTotal = objDetObj->L3RamObj.cfg.size;
                memUsage->CoreLocalRamTotal = objDetObj->CoreLocalRamObj.cfg.size;
                retVal = DPC_ObjDetRangeHwa_preStartConfig(subFrmObj,
                             objDetObj,
                             &cfg->staticCfg,
                             &cfg->dynCfg,
                             memUsage);
                if (retVal != 0)
                {
                    goto exit;
                }

                /* Populate radarCube memory information */
                cfg->radarCubeMem.addr = subFrmObj->rangeCfg.hwRes.radarCube.data;
                cfg->radarCubeMem.size = subFrmObj->rangeCfg.hwRes.radarCube.dataSize;

                /* Get system heap size after preStart configuration */
                MemoryP_getStats(&statsEnd);

                /* Populate system heap usage */
                memUsage->SystemHeapTotal = statsEnd.totalSize;
                memUsage->SystemHeapUsed = statsEnd.totalSize -statsEnd.totalFreeSize;
                memUsage->SystemHeapDPCUsed = statsStart.totalFreeSize - statsEnd.totalFreeSize;

                DebugP_log1("ObjDet DPC: Pre-start Config IOCTL processed (subFrameIndx = %d)\n", cfg->subFrameNum);
                break;
            }

            default:
            {
                /* Error: This is an unsupported command */
                retVal = DPM_EINVCMD;
                break;
            }
        }
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) initialization function which is invoked by the
 *      application using DPM_init API. Among other things, this API allocates DPC instance
 *      and DPU instances (by calling DPU's init APIs) from the MemoryP osal
 *      heap. If this API returns an error of any type, the heap is not guaranteed
 *      to be in the same state as before calling the API (i.e any allocations
 *      from the heap while executing the API are not guaranteed to be deallocated
 *      in case of error), so any error from this API should be considered fatal and
 *      if the error is of _ENOMEM type, the application will
 *      have to be built again with a bigger heap size to address the problem.
 *
 *  @param[in]  dpmHandle   DPM's DPC handle
 *  @param[in]  ptrInitCfg  Handle to the framework semaphore
 *  @param[out] errCode     Error code populated on error
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static DPM_DPCHandle DPC_ObjectDetection_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
)
{
    int32_t i;
    ObjDetObj     *objDetObj = NULL;
    DPC_ObjectDetectionRangeHWA_InitParams *dpcInitParams;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;

    *errCode = 0;

    if ((ptrInitCfg == NULL) || (ptrInitCfg->arg == NULL))
    {
        *errCode = DPC_OBJDETRANGEHWA_EINVAL;
        goto exit;
    }

    if (ptrInitCfg->argSize != sizeof(DPC_ObjectDetectionRangeHWA_InitParams))
    {
        *errCode = DPC_OBJDETRANGEHWA_EINVAL__INIT_CFG_ARGSIZE;
        goto exit;
    }

    dpcInitParams = (DPC_ObjectDetectionRangeHWA_InitParams *) ptrInitCfg->arg;

    objDetObj = MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);

#ifdef DBG_DPC_OBJDETRANGEHWA
    gObjDetObj = objDetObj;
#endif

    DebugP_log1("ObjDet DPC: objDetObj address = %d\n", (uint32_t) objDetObj);

    if(objDetObj == NULL)
    {
        *errCode = DPC_OBJDETRANGEHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)objDetObj, 0, sizeof(ObjDetObj));

    /* Copy over the DPM configuration: */
    memcpy ((void*)&objDetObj->dpmInitCfg, (void*)ptrInitCfg, sizeof(DPM_InitCfg));

    objDetObj->dpmHandle = dpmHandle;
    objDetObj->socHandle = ptrInitCfg->socHandle;
    objDetObj->L3RamObj.cfg = dpcInitParams->L3ramCfg;
    objDetObj->CoreLocalRamObj.cfg = dpcInitParams->CoreLocalRamCfg;

    /* Validate EDMA handle */
    DebugP_assert(dpcInitParams->edmaHandle!= NULL);
    objDetObj->edmaHandle = dpcInitParams->edmaHandle;

    /* Validate HWA handle */
    DebugP_assert(dpcInitParams->hwaHandle!= NULL);
    objDetObj->hwaHandle = dpcInitParams->hwaHandle;
    objDetObj->processCallBackFxn = dpcInitParams->processCallBackFxn;

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(dpcInitParams->hwaHandle, &hwaMemInfo);
    if (*errCode != 0)
    {
        goto exit;
    }

    objDetObj->hwaMemBankSize = hwaMemInfo.bankSize;

    for (i = 0; i < hwaMemInfo.numBanks; i++)
    {
        objDetObj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress +
            i * hwaMemInfo.bankSize;
    }

    for(index = 0; index < RL_MAX_SUBFRAMES; index++)
    {
        DPU_RangeProcHWA_InitParams rangeInitParams;

        rangeInitParams.hwaHandle = objDetObj->hwaHandle;

        objDetObj->subFrameObj[index].dpuRangeObj = DPU_RangeProcHWA_init(&rangeInitParams, errCode);
    }
#ifndef TRACKERPROC_ON_DSP
    if (*errCode != 0)
    {
        goto exit;
    }

    /* Initialize the tracker DPU - this is not subframe based */
    objDetObj->dpuTrackerObj = DPU_TrackerProc_init(errCode);
    
    if (*errCode != 0)
    {
        goto exit;
    }
#endif
exit:
    if(*errCode != 0)
    {
        if(objDetObj != NULL)
        {
            MemoryP_ctrlFree(objDetObj, sizeof(ObjDetObj));
            objDetObj = NULL;
        }
    }

    return ((DPM_DPCHandle)objDetObj);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) de-initialization function which is invoked by the
 *      application using DPM_deinit API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_deinit (DPM_DPCHandle handle)
{
    int32_t retVal = 0;
    ObjDetObj     *objDetObj = NULL;
    uint8_t       index;

    if (handle == NULL)
    {
        retVal = DPC_OBJDETRANGEHWA_EINVAL;
        goto exit;
    }

    objDetObj = handle;

    for(index = 0; index < RL_MAX_SUBFRAMES; index++)
    {
        if(objDetObj->subFrameObj[index].dpuRangeObj != NULL)
        {
            retVal = DPU_RangeProcHWA_deinit(objDetObj->subFrameObj[index].dpuRangeObj);
            if (retVal != 0)
            {
                goto exit;
            }
        }
    }

    /* Free DPC memory */
    MemoryP_ctrlFree(handle, sizeof(ObjDetObj));

exit:
    return (retVal);
}
