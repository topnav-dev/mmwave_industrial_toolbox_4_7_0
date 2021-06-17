/*
 *   @file  objectdetection.c
 *
 *   @brief
 *      Object Detection DPC implementation using DSP.
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

/* mmWave SDK Include Files: */
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this object detection DPC are prefixed by DPC_OBJDET_ */
#include APP_RESOURCE_FILE

/* Obj Det instance etc */
#include <objectdetectioninternal.h>
#include <objectdetection.h>

#include <ti/alg/mmwavelib/mmwavelib.h>

#define DBG_DPC_OBJDET

#ifdef DBG_DPC_OBJDET
ObjDetObj     *gObjDetObj;
#endif

/**************************************************************************
 ************************** Local Definitions **********************************
 **************************************************************************/

 /** @addtogroup DPC_OBJDET_IOCTL__INTERNAL_DEFINITIONS
  @{ */

/*! Radar cube data buffer alignment in bytes. No DPU module specifying alignment
 *  need (through a \#define) implies that no alignment is needed i.e 1 byte alignment.
 *  But we do the natural data type alignment which is 2 bytes (as radar cube is complex16-bit type)
 *  because radar cube is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing.
 */
#define DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT       (sizeof(int16_t))

/*! Detection matrix alignment is declared by CFAR dpu, we size to
 *  the max of this and CPU alignment for accessing detection matrix
 *  it is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing. Note currently the CFAR alignment is the same as
 *  CPU alignment so this max is redundant but it is more to illustrate the
 *  generality of alignments should be done.
 */
#define DPC_OBJDET_DET_MATRIX_DATABUF_BYTE_ALIGNMENT       (MAX(sizeof(uint16_t), \
                                                                DPU_CFARCAPROCDSP_DET_MATRIX_BYTE_ALIGNMENT))

/*! cfar list alignment is declared by cfar and AoA dpu, for debug purposes we size to
 *  the max of these and CPU alignment for accessing cfar list for debug purposes.
 *  Note currently the dpu alignments are the same as CPU alignment so these max are
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_CFAR_DET_LIST_BYTE_ALIGNMENT            (MAX(MAX(DPU_CFARCAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT,  \
                                                                    DPU_AOAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT),\
                                                                DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT))


/*! Point cloud cartesian alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT))

/*! Point cloud side info alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT))

/*! Azimuth static heat map alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT, \
                                                                   sizeof(int16_t)))

/*! Elevation angle alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT, \
                                                                   sizeof(float)))

/**
@}
*/
/*! Maximum Number of objects that can be detected in a frame */
#ifdef TRACKERPROC_ON_DSP
#define DPC_OBJDET_MAX_NUM_OBJECTS                       250U
#else
#define DPC_OBJDET_MAX_NUM_OBJECTS                       500U
#endif

/* Window definition for range and doppler */
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE            MMWAVELIB_WIN_BLACKMAN
#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE          MMWAVELIB_WIN_HANNING

/* QFORMAT for Range and Doppler Windowing */
#define DPC_OBJDET_QFORMAT_RANGE_FFT 15
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 19

/**************************************************************************
 ************************** Local Functions Prototype **************************
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
#ifdef OBJDET_NO_RANGE
int32_t DPC_ObjectDetection_dataInjection(DPM_DPCHandle handle, DPM_Buffer* ptrBuffer);
#else
void DPC_ObjectDetection_chirpEvent (DPM_DPCHandle handle);
#endif

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
static void DPC_ObjDetDSP_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}

/**
 *  @b Description
 *  @n
 *      Utility function for setting memory pool to desired address in the pool.
 *      Helps to rewind for example.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  addr Address to assign to the pool's current address.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
static void DPC_ObjDetDSP_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr = (uintptr_t)addr;
    pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting memory pool current address.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to current address of the pool (from which next allocation will
 *      allocate to the desired alignment).
 */
static void *DPC_ObjDetDSP_MemPoolGet(MemPoolObj *pool)
{
    return((void *)pool->currAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
static uint32_t DPC_ObjDetDSP_MemPoolGetMaxUsage(MemPoolObj *pool)
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
static void *DPC_ObjDetDSP_MemPoolAlloc(MemPoolObj *pool,
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
 *      Inline function generates a single FFT window samples. It calls single precision
 *      sine and cosine functions from mathlib library for the first sample only, and then call
 *      mmwavelib_windowCoef_gen function to recursively generate cosine values 
 *      for other samples.
 *
 *  @param[out] win             Pointer to calculated window samples.
 *  @param[in]  windowDatumType Window samples data format. For windowDatumType = MMWAVELIB_WINDOW_INT16,
 *              the samples format is int16_t. For windowDatumType = MMWAVELIB_WINDOW_INT32,
 *              the samples format is int32_t.
 *  @param[in]  winLen          Nominal window length
 *  @param[in]  winGenLen       Number of generated samples
 *  @param[in]  oneQformat      Q format of samples.
 *  @param[in]  winType         Type of window, one of MMWAVELIB_WIN_HANNING, MMWAVELIB_WIN_BLACKMAN,
 *              MMWAVELIB_WIN_RECT, or MMWAVELIB_WIN_HAMMING.
 *  @retval none.
 */
static inline void DPC_ObjDetDSP_genWindow(void *win,
                        uint32_t windowDatumType,
                        uint32_t winLen,
                        uint32_t winGenLen,
                        int32_t oneQformat,
                        uint32_t winType)
{
    
    float ephyR, ephyI;
    float phi = 2 * PI_ / ((float) winLen - 1);

    ephyR  = cossp(phi);
    ephyI  = sinsp(phi);

    mmwavelib_windowCoef_gen(win,
                        windowDatumType,
                        ephyR,
                        ephyI,
                        winGenLen,
                        oneQformat,
                        winType);

}

#ifndef OBJDET_NO_RANGE
/**
 *  @b Description
 *  @n
 *      DPC chirp event function registered with DPM. This is invoked on reception
 *      of the chirp data available ISR from the RF front-end. This API is also invoked
 *      when application issues @ref DPC_OBJDET_IOCTL__TRIGGER_CHIRP to simulate
 *      a chirp event trigger (e.g for unit testing purpose).
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
void DPC_ObjectDetection_chirpEvent (DPM_DPCHandle handle)
{
    ObjDetObj       *objDetObj = (ObjDetObj *) handle;
    uint32_t        margin;

    if(objDetObj->chirpIndex != 0)
    {
        margin = Cycleprofiler_getTimeStamp() - objDetObj->chirpEndTime;
        if(margin < objDetObj->chirpMargin)
        {
            /* Find the smallest margin to report as chirp margin */
            objDetObj->chirpMargin = margin;
        }
    }
    else
    {
        /* Reset margin min */
        objDetObj->chirpMargin = 0xffffffff;
    }

    /* Notify the DPM Module that the DPC is ready for execution */
    DebugP_assert (DPM_notifyExecute (objDetObj->dpmHandle, handle, true) == 0);

    return;
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
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetDSP_GetRangeWinGenLen(uint16_t numAdcSamples)
{
    return((numAdcSamples + 1)/2);
}

/**
 *  @b Description
 *  @n
 *      Generate the range DPU window using mathutils API.
 *
 *  @param[in]  rangeCfg    Range DPU configuration, output window is generated in window
 *                          pointer in the staticCfg of this.
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetDSP_GenRangeWindow
(
    DPU_RangeProcDSP_Config       *rangeCfg
)
{
    DPC_ObjDetDSP_genWindow((void *)rangeCfg->hwRes.window,
                        MMWAVELIB_WINDOW_INT16,
                        rangeCfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                        DPC_ObjDetDSP_GetRangeWinGenLen(rangeCfg->staticCfg.ADCBufData.dataProperty.numAdcSamples),
                        DPC_OBJDET_QFORMAT_RANGE_FFT,
                        DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE);
}

/**
 *  @b Description
 *  @n
 *     Configure range DPU.
 *
 *  @param[in]  dpuHandle       Handle to Range DPU
 *  @param[in]  staticCfg       Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg          Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle      Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube       Pointer to DPIF radar cube, which is output of range
 *                              processing.
 *  @param[in]  CoreL2RamObj    Pointer to core local L2 RAM object to allocate local memory
 *                              for the DPU, only for scratch purposes
 *  @param[in]  CoreL1RamObj    Pointer to core local L1 RAM object to allocate local memory
 *                              for the DPU, only for scratch purposes
 *  @param[in]  window          Pointer to range FFT window coefficients buffer
 *  @param[in]  windowSize      Range FFT window size in bytes. See Range DPU
 *                              configuration for more information.
 *  @param[in]  twiddle         Pointer to range FFT twiddle buffer
 *  @param[in]  twiddleSize     Range FFT twiddle buffer size in bytes. See Range DPU
 *                              configuration for more information.
 *  @param[out] cfgSave         Configuration that is built in local
 *                              (stack) variable is saved here. This is for facilitating
 *                              quick reconfiguration later without having to go through
 *                              the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_rangeConfig
(
    DPU_RangeProcDSP_Handle dpuHandle,
    DPC_ObjectDetection_StaticCfg *staticCfg,
    DPC_ObjectDetection_DynCfg    *dynCfg,
    EDMA_Handle                   edmaHandle,
    DPIF_RadarCube                *radarCube,
    MemPoolObj                    *CoreL2RamObj,
    MemPoolObj                    *CoreL1RamObj,
    int16_t                       *window,
    uint32_t                      windowSize,
    cmplx16ImRe_t                 *twiddle,
    uint32_t                      twiddleSize,
    DPU_RangeProcDSP_Config       *cfgSave
)
{
    int32_t retVal = 0;
    DPU_RangeProcDSP_Config rangeCfg;
    DPU_RangeProcDSP_HW_Resources *hwRes = &rangeCfg.hwRes;
    DPU_RangeProcDSP_EDMAConfig *edmaCfg = &hwRes->edmaCfg;
    uint32_t numRxAntennas;

    memset(&rangeCfg, 0, sizeof(rangeCfg));

    numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;

    /* rangeProcDSP only supports non-interleaved at present */
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

    /* radarCube */
    hwRes->radarCube = *radarCube;
    hwRes->windowSize = windowSize;
    hwRes->window  = window;
    DPC_ObjDetDSP_GenRangeWindow(&rangeCfg);

    hwRes->twiddleSize = twiddleSize;
    hwRes->twiddle16x16 = twiddle;

    /* hwres - adcDataInSize */
    hwRes->adcDataInSize = 2U * staticCfg->numRangeBins * sizeof(cmplx16ImRe_t);
    hwRes->adcDataIn = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  hwRes->adcDataInSize,
                                                  DPU_RANGEPROCDSP_ADCDATAIN_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)hwRes->adcDataIn);

    /* hwres - fftOut1D */
    hwRes->fftOut1DSize = 2* numRxAntennas * staticCfg->numRangeBins * sizeof(cmplx16ImRe_t);
    hwRes->fftOut1D = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  hwRes->fftOut1DSize,
                                                  DPU_RANGEPROCDSP_FFTOUT_BYTE_ALIGNMENT_DSP);
    DebugP_assert((int)hwRes->fftOut1D);

    /* hwres - dcRangeSig, allocate from heap, this needs to persist within sub-frame/frame
     * processing and across sub-frames */
    hwRes->dcRangeSigMeanSize = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
                                staticCfg->numTxAntennas * numRxAntennas * sizeof(cmplx32ImRe_t);
    hwRes->dcRangeSigMean = (cmplx32ImRe_t *) MemoryP_ctrlAlloc (hwRes->dcRangeSigMeanSize, 0);
    DebugP_assert(hwRes->dcRangeSigMean != NULL);

    /* hwres - edma */
    edmaCfg->edmaHandle = edmaHandle;
    edmaCfg->dataInPing.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PING_CH;
    edmaCfg->dataInPing.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PING_SHADOW;
    edmaCfg->dataInPing.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PING_EVENT_QUE;
    edmaCfg->dataInPong.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PONG_CH;
    edmaCfg->dataInPong.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PONG_SHADOW;
    edmaCfg->dataInPong.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_PONG_EVENT_QUE;

    /* Ping */
    edmaCfg->dataOutPing.channel        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PING_CH;
    edmaCfg->dataOutPing.channelShadow  = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PING_SHADOW;
    edmaCfg->dataOutPing.eventQueue     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PING_EVENT_QUE;

    /* Pong */
    edmaCfg->dataOutPong.channel        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PONG_CH;
    edmaCfg->dataOutPong.channelShadow  = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PONG_SHADOW;
    edmaCfg->dataOutPong.eventQueue     = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_PONG_EVENT_QUE;

    retVal = DPU_RangeProcDSP_config(dpuHandle, &rangeCfg);
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

#else

/**
 *  @b Description
 *  @n
 *      DPC data injection function registered with DPM. This is invoked on reception
 *      of the data injection from DPM. 
 *
 *  @param[in]  handle      DPM's DPC handle
 *  @param[in]  ptrBuffer   Buffer for data injected
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
int32_t DPC_ObjectDetection_dataInjection(DPM_DPCHandle handle, DPM_Buffer* ptrBuffer)
{
    ObjDetObj     *objDetObj = (ObjDetObj *) handle;

    /* Notify the DPM Module that the DPC is ready for execution */
    DebugP_assert (DPM_notifyExecute (objDetObj->dpmHandle, handle, true) == 0);

    return 0;
}

#endif
#if 0
/**
 *  @b Description
 *  @n
 *      Utility function to do a parabolic/quadratic fit on 3 input points
 *      and return the coordinates of the peak. This is used to accurately estimate
 *      range bias.
 *
 *  @param[in]  x Pointer to array of 3 elements representing the x-coordinate
 *              of the points to fit
 *  @param[in]  y Pointer to array of 3 elements representing the y-coordinate
 *              of the points to fit
 *  @param[out] xv Pointer to output x-coordinate of the peak value
 *  @param[out] yv Pointer to output y-coordinate of the peak value
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetDSP_quadFit(float *x, float*y, float *xv, float *yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];

    denom = (x0 - x1)*(x0 - x2)*(x1 - x2);
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
    b = (x2*x2 * (y0 - y1) + x1*x1 * (y2 - y0) + x0*x0 * (y1 - y2)) / denom;
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;

    *xv = -b/(2 * a);
    *yv = c - b*b/(4 * a);
}

/**
 *  @b Description
 *  @n
 *      Computes the range bias and rx phase compensation from the detection matrix
 *      during calibration measurement procedure of these parameters.
 *
 *  @param[in]  staticCfg Pointer to static configuration
 *  @param[in]  targetDistance Target distance in meters
 *  @param[in]  searchWinSize Search window size in meters
 *  @param[in] detMatrix Pointer to detection matrix
 *  @param[in] symbolMatrix Pointer to symbol matrix
 *  @param[out] compRxChanCfg computed output range bias and rx phase comp vector
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetDSP_rangeBiasRxChPhaseMeasure
(
    DPC_ObjectDetection_StaticCfg       *staticCfg,
    float                   targetDistance,
    float                   searchWinSize,
    uint16_t                *detMatrix,
    uint32_t                *symbolMatrix,
    DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg
)
{
    cmplx16ImRe_t rxSym[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    cmplx16ImRe_t *tempPtr;
    float sumSqr;
    uint32_t * rxSymPtr = (uint32_t * ) rxSym;
    float xMagSq[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    int32_t iMax;
    float xMagSqMin;
    float scal;
    float truePosition;
    int32_t truePositionIndex;
    float y[3];
    float x[3];
    int32_t halfWinSize ;
    float estPeakPos;
    float estPeakVal;
    int32_t i, ind;
    int32_t txIdx, rxIdx;

    uint32_t numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    uint32_t numTxAntennas = staticCfg->numTxAntennas;
    uint32_t numRangeBins = staticCfg->numRangeBins;
    uint32_t numDopplerChirps = staticCfg->numDopplerChirps;
    uint32_t numSymPerTxAnt = numDopplerChirps * numRxAntennas * numRangeBins;
    uint32_t symbolMatrixIndx;

    uint16_t maxVal = 0;

    truePosition = targetDistance / staticCfg->rangeStep;
    truePositionIndex = (int32_t) (truePosition + 0.5);

    halfWinSize = (int32_t) (0.5 * searchWinSize / staticCfg->rangeStep + 0.5);

    /**** Range calibration ****/
    iMax = truePositionIndex;
    for (i = truePositionIndex - halfWinSize; i <= truePositionIndex + halfWinSize; i++)
    {
        if (detMatrix[i * staticCfg->numDopplerBins] > maxVal)
        {
            maxVal = detMatrix[i * staticCfg->numDopplerBins];
            iMax = i;
        }
    }

    /* Fine estimate of the peak position using quadratic fit */
    ind = 0;
    for (i = iMax - 1; i <= iMax + 1; i++)
    {
        sumSqr = 0.0;
        for (txIdx=0; txIdx < numTxAntennas; txIdx++)
        {
            for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
            {
                symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + i;
                tempPtr = (cmplx16ImRe_t *) &symbolMatrix[symbolMatrixIndx];
                sumSqr += (float) tempPtr->real * (float) tempPtr->real +
                          (float) tempPtr->imag * (float) tempPtr->imag;
            }
        }
#ifdef SUBSYS_DSS
        y[ind] = sqrtsp(sumSqr);
#else
        y[ind] = sqrt(sumSqr);
#endif
        x[ind] = (float)i;
        ind++;
    }
    DPC_ObjDetDSP_quadFit(x, y, &estPeakPos, &estPeakVal);
    compRxChanCfg->rangeBias = (estPeakPos - truePosition) * staticCfg->rangeStep;

    /*** Calculate Rx channel phase/gain compensation coefficients ***/
    for (txIdx = 0; txIdx < numTxAntennas; txIdx++)
    {
        for (rxIdx = 0; rxIdx < numRxAntennas; rxIdx++)
        {
            i = txIdx * numRxAntennas + rxIdx;
            symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + iMax;
            rxSymPtr[i] = symbolMatrix[symbolMatrixIndx];
            xMagSq[i] = (float) rxSym[i].real * (float) rxSym[i].real +
                        (float) rxSym[i].imag * (float) rxSym[i].imag;
        }
    }
    xMagSqMin = xMagSq[0];
    for (i = 1; i < staticCfg->numVirtualAntennas; i++)
    {
        if (xMagSq[i] < xMagSqMin)
        {
            xMagSqMin = xMagSq[i];
        }
    }

    for (txIdx=0; txIdx < staticCfg->numTxAntennas; txIdx++)
    {
        for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
        {
            int32_t temp;
            i = txIdx * numRxAntennas + rxIdx;
            scal = 32768./ xMagSq[i] * sqrt(xMagSqMin);

            temp = (int32_t) MATHUTILS_ROUND_FLOAT(scal * rxSym[i].real);
            MATHUTILS_SATURATE16(temp);
            compRxChanCfg->rxChPhaseComp[staticCfg->txAntOrder[txIdx] * numRxAntennas +
                                         rxIdx].real = (int16_t) (temp);

            temp = (int32_t) MATHUTILS_ROUND_FLOAT(-scal * rxSym[i].imag);
            MATHUTILS_SATURATE16(temp);
            compRxChanCfg->rxChPhaseComp[staticCfg->txAntOrder[txIdx] * numRxAntennas
                                         + rxIdx].imag = (int16_t) (temp);
        }
    }
}
#endif
/**
 *  @b Description
 *  @n
 *      Computes the length of window to generate for doppler DPU.
 *
 *  @param[in]  numDopplerChirps    Number of Doppler chirps
 *
 *  @retval   Length of window to generate
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetDSP_GetDopplerWinGenLen(uint16_t numDopplerChirps)
{
    return((numDopplerChirps + 1)/2);
}

/**
 *  @b Description
 *  @n
 *      Generate the doppler DPU window using mmwavelib API.
 *
 *  @param[in]  cfg Doppler DPU configuration, output window is generated in window
 *                  pointer embedded in this configuration.
 *
 *  @retval   winType window type, see mathutils.h
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetDSP_GenDopplerWindow
(
    DPU_DopplerProcDSP_Config *cfg
)
{
    uint32_t winType;

    /* For too small window, force rectangular window to avoid loss of information
     * due to small window values (e.g. hanning has first and last coefficients 0) */
    if (cfg->staticCfg.numDopplerChirps <= 4)
    {
        winType = MMWAVELIB_WIN_RECT;
    }
    else
    {
        winType = DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE;
    }
    DPC_ObjDetDSP_genWindow((void *)cfg->hwRes.windowCoeff,
                        MMWAVELIB_WINDOW_INT32,
                        cfg->staticCfg.numDopplerChirps,
                        DPC_ObjDetDSP_GetDopplerWinGenLen(cfg->staticCfg.numDopplerChirps),
                        DPC_OBJDET_QFORMAT_DOPPLER_FFT,
                        winType);
    return(winType);
}

/**
 *  @b Description
 *  @n
 *      Extracts the sub-frame specific vector from the common (full vector for all antennnas)
 *      input vector of the range bias and rx phase compensation. Uses the antenna order of
 *      the sub-frame.
 *
 *  @param[in]  staticCfg   Static configuration of the sub-frame
 *  @param[in]  inpCfg      The full vector.
 *  @param[out] outCfg      Sub-frame specific compensation vector that will be used during processing
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetDSP_GetRxChPhaseComp(DPC_ObjectDetection_StaticCfg *staticCfg,
                                 DPU_AoAProc_compRxChannelBiasCfg *inpCfg,
                                 DPU_AoAProc_compRxChannelBiasCfg *outCfg)
{
    uint32_t tx, rx, numTxAnt, numRxAnt;
    uint8_t *txAntOrder, *rxAntOrder;
    cmplx16ImRe_t one;

    one.imag = 0;
    one.real = 0x7fff;

    numTxAnt = staticCfg->numTxAntennas;
    numRxAnt = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    txAntOrder = staticCfg->txAntOrder;
    rxAntOrder = staticCfg->rxAntOrder;
    outCfg->rangeBias = inpCfg->rangeBias;

    for(tx = 0; tx < numTxAnt; tx++)
    {
        for(rx = 0; rx < numRxAnt; rx++)
        {
            if (staticCfg->isValidProfileHasOneTxPerChirp == 1U)
            {
                outCfg->rxChPhaseComp[tx * numRxAnt + rx] =
                    inpCfg->rxChPhaseComp[txAntOrder[tx] * SYS_COMMON_NUM_RX_CHANNEL +
                                          rxAntOrder[rx]];
            }
            else
            {
                outCfg->rxChPhaseComp[tx * numRxAnt + rx] = one;
            }
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Sub-frame reconfiguration, used when switching sub-frames. Invokes the
 *      DPU configuration using the configuration that was stored during the
 *      pre-start configuration so reconstruction time is saved  because this will
 *      happen in real-time.
 *  @param[in]  objDetObj Pointer to DPC object
 *  @param[in]  subFrameIndx Sub-frame index.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_reconfigSubFrame(ObjDetObj *objDetObj, uint8_t subFrameIndx)
{
    int32_t retVal = 0;
    DPU_AoAProc_compRxChannelBiasCfg outCompRxCfg;
    SubFrameObj *subFrmObj;

    subFrmObj = &objDetObj->subFrameObj[subFrameIndx];

#ifndef OBJDET_NO_RANGE
    /* Generate windowing buffer */
    DPC_ObjDetDSP_GenRangeWindow(&subFrmObj->dpuCfg.rangeCfg);
    retVal = DPU_RangeProcDSP_config(subFrmObj->dpuRangeObj, &subFrmObj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }
#endif

    retVal = DPU_CFARCAProcDSP_config(subFrmObj->dpuCFARCAObj, &subFrmObj->dpuCfg.cfarCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Generate 2D window coefficients.*/
    DPC_ObjDetDSP_GenDopplerWindow(&subFrmObj->dpuCfg.dopplerCfg);
    retVal = DPU_DopplerProcDSP_config(subFrmObj->dpuDopplerObj, &subFrmObj->dpuCfg.dopplerCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Note doppler window will be used for AoA, so maintain the sequence as in
     * pre-start config. We need to regenerate the rxChPhaseComp because it was
     * temporary (note DPUs get pointers to dynamic configs) */
    DPC_ObjDetDSP_GetRxChPhaseComp(&subFrmObj->staticCfg,
                                &objDetObj->commonCfg.compRxChanCfg, &outCompRxCfg);
    subFrmObj->dpuCfg.aoaCfg.dynCfg.compRxChanCfg = &outCompRxCfg;
    retVal = DPU_AoAProcDSP_config(subFrmObj->dpuAoAObj, &subFrmObj->dpuCfg.aoaCfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Function to initialize all DPUs used in the DPC chain
 *
 *  @param[in] objDetObj        Pointer to sub-frame object
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t DPC_ObjDetDSP_initDPU
(
    ObjDetObj     *objDetObj,
    uint8_t       numSubFrames
)
{
    SubFrameObj     *subFrmObj;
    int32_t         retVal = 0;
    int32_t         idx;

    for(idx = 0; idx < numSubFrames; idx++)
    {
        subFrmObj = &objDetObj->subFrameObj[idx];

#ifndef OBJDET_NO_RANGE
        subFrmObj->dpuRangeObj = DPU_RangeProcDSP_init(&retVal);
        if (retVal != 0)
        {
            goto exit;
        }
#endif

        subFrmObj->dpuCFARCAObj = DPU_CFARCAProcDSP_init(&retVal);
        if (retVal != 0)
        {
            goto exit;
        }

        subFrmObj->dpuDopplerObj = DPU_DopplerProcDSP_init(&retVal);
        if (retVal != 0)
        {
            goto exit;
        }

        subFrmObj->dpuAoAObj = DPU_AoAProcDSP_init(&retVal);
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
 *      Function to de-initialize all DPUs used in the DPC chain
 *
 *  @param[in] objDetObj        Pointer to sub-frame object
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static inline int32_t DPC_ObjDetDSP_deinitDPU
(
    ObjDetObj     *objDetObj,
    uint8_t       numSubFrames
)
{
    SubFrameObj     *subFrmObj;
    int32_t         retVal = 0;
    int32_t         idx;

    for(idx = 0; idx < numSubFrames; idx++)
    {
        subFrmObj = &objDetObj->subFrameObj[idx];

#ifndef OBJDET_NO_RANGE
        retVal = DPU_RangeProcDSP_deinit(subFrmObj->dpuRangeObj);

        if (retVal != 0)
        {
            goto exit;
        }
#endif
        retVal = DPU_DopplerProcDSP_deinit(subFrmObj->dpuDopplerObj);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = DPU_CFARCAProcDSP_deinit(subFrmObj->dpuCFARCAObj);
        if (retVal != 0)
        {
            goto exit;
        }

        retVal = DPU_AoAProcDSP_deinit(subFrmObj->dpuAoAObj);
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
 *      Configures DPC for Range Bias and Phase Comp measurement.
 *
 *  @param[in] obj       Pointer to DPC object
 *  @param[in] cfg       Pointer to Range Bias and Phase Comp measurement configuration
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjDetDSP_Config_MeasureRxChannelBiasCfg(ObjDetObj *obj,
                   DPC_ObjectDetection_MeasureRxChannelBiasCfg *cfg)
{
    int32_t retVal = 0;

    if (cfg->enabled == 1)
    {
        if ((-cfg->searchWinSize/2.0f + cfg->targetDistance) <= 0.0f)
        {
            retVal = DPC_OBJECTDETECTION_EINVAL__MEASURE_RX_CHANNEL_BIAS_CFG;
            goto exit;
        }
    }
    obj->commonCfg.measureRxChannelBiasCfg = *cfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure Doppler DPU.
 *
 *  @param[in]  dpuHandle           Handle to Doppler DPU
 *  @param[in]  staticCfg           Pointer to static configuration of the sub-frame
 *  @param[in]  log2NumDopplerBins  log2 of numDopplerBins of the static config.
 *  @param[in]  dynCfg              Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle          Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube           Pointer to DPIF radar cube, which will be the input
 *                                  to doppler processing
 *  @param[in]  detMatrix           Pointer to DPIF detection matrix, which will be the output
 *                                  of doppler processing
 *  @param[in]  CoreL2RamObj        Pointer to core local L2 RAM object to allocate local memory
 *                                  for the DPU, only for scratch purposes
 *  @param[in]  CoreL1RamObj        Pointer to core local L1 RAM object to allocate local memory
 *                                  for the DPU, only for scratch purposes
 *  @param[in]  window              Pointer to doppler FFT window coefficients buffer
 *  @param[in]  windowSize          Doppler FFT window size in bytes. See Doppler DPU
 *                                  configuration for more information.
 *  @param[in]  twiddleBuf          Pointer to doppler FFT twiddle buffer
 *  @param[in]  twiddleSize         Doppler FFT twiddle buffer size in bytes. See Doppler DPU
 *                                  configuration for more information.
 *  @param[in]  dftSinCosTable      Pointer to DFT Sin/Cos table buffer
 *  @param[in]  dftSinCosTableSize  DFT Sin/Cos Table size in bytes. See Doppler DPU
 *                                  configuration for more information.
 *  @param[out] cfgSave             Configuration that is built in local
 *                                  (stack) variable is saved here. This is for facilitating
 *                                  quick reconfiguration later without having to go through
 *                                  the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_dopplerConfig
(
    DPU_DopplerProcDSP_Handle dpuHandle,
    DPC_ObjectDetection_StaticCfg *staticCfg,
    uint8_t                       log2NumDopplerBins,
    DPC_ObjectDetection_DynCfg    *dynCfg,
    EDMA_Handle                   edmaHandle,
    DPIF_RadarCube                *radarCube,
    DPIF_DetMatrix                *detMatrix,
    MemPoolObj                    *CoreL2RamObj,
    MemPoolObj                    *CoreL1RamObj,
    int32_t                       *window,
    uint32_t                      windowSize,
    cmplx32ReIm_t                 *twiddleBuf,
    uint32_t                      twiddleSize,
    cmplx16ImRe_t                 *dftSinCosTable,
    uint32_t                      dftSinCosTableSize,
    DPU_DopplerProcDSP_Config     *cfgSave
)
{
    int32_t retVal = 0;
    DPU_DopplerProcDSP_Config dopCfg;
    DPU_DopplerProcDSP_HW_Resources  *hwRes;
    DPU_DopplerProcDSP_StaticConfig  *dopStaticCfg;
    DPU_DopplerProcDSP_EdmaCfg *edmaCfg;
    DPU_DopplerProcDSP_DynamicConfig  *dopDynCfg;

    hwRes = &dopCfg.hwRes;
    dopStaticCfg = &dopCfg.staticCfg;
    dopDynCfg = &dopCfg.dynCfg;
    edmaCfg = &hwRes->edmaCfg;

    memset(&dopCfg, 0, sizeof(dopCfg));

    /* Static Configurations */
    dopStaticCfg->numDopplerChirps   = staticCfg->numDopplerChirps;
    dopStaticCfg->numDopplerBins     = staticCfg->numDopplerBins;
    dopStaticCfg->numRangeBins       = staticCfg->numRangeBins;
    dopStaticCfg->numRxAntennas      = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    dopStaticCfg->numVirtualAntennas = staticCfg->numVirtualAntennas;
    dopStaticCfg->log2NumDopplerBins = log2NumDopplerBins;
    dopStaticCfg->numTxAntennas      = staticCfg->numTxAntennas;
    dopStaticCfg->isBpmEnabled       = staticCfg->isBpmEnabled;

    /* Dynamic Configuration */
    dopDynCfg->staticClutterCfg.isEnabled = dynCfg->staticClutterRemovalCfg.enabled;

    /* hwRes */
    hwRes->radarCube = *radarCube;
    hwRes->detMatrix = *detMatrix;

    /* hwRes - edmaCfg */
    edmaCfg->edmaHandle = edmaHandle;

    /* edmaIn - ping */
    edmaCfg->edmaIn.ping.channel =            DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_CH;
    edmaCfg->edmaIn.ping.channelShadow =      DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_SHADOW;
    edmaCfg->edmaIn.ping.eventQueue =         DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_EVENT_QUE;

    /* edmaIn - pong */
    edmaCfg->edmaIn.pong.channel =            DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_CH;
    edmaCfg->edmaIn.pong.channelShadow =      DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_SHADOW;
    edmaCfg->edmaIn.pong.eventQueue =         DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_EVENT_QUE;

    /* edmaOut */
    edmaCfg->edmaOut.channel =           DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_CH;
    edmaCfg->edmaOut.channelShadow =     DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_SHADOW;
    edmaCfg->edmaOut.eventQueue =        DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_EVENT_QUE;

    /* hwres window */
    hwRes->windowSize = windowSize;
    hwRes->windowCoeff = window;
    DPC_ObjDetDSP_GenDopplerWindow(&dopCfg);

    /* hwres twiddle */
    hwRes->twiddle32x32 = twiddleBuf;
    hwRes->twiddleSize = twiddleSize;

    /* hwres dftSinCosTable */
    hwRes->dftSinCosSize  = dftSinCosTableSize;
    hwRes->dftSinCosTable = dftSinCosTable;

    /* hwres - pingPong buffer*/
    hwRes->pingPongSize = 2U * staticCfg->numDopplerChirps * sizeof(cmplx16ImRe_t);
    hwRes->pingPongBuf  = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  hwRes->pingPongSize,
                                                  DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert((int)hwRes->pingPongBuf);

    /* hwres - windowing output and log2Abs output overlaid buffer*/
    hwRes->windowingLog2AbsSize = staticCfg->numDopplerBins * sizeof(cmplx32ReIm_t);
    hwRes->windowingLog2AbsBuf  = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                            hwRes->windowingLog2AbsSize,
                                            DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert((int)hwRes->windowingLog2AbsBuf);

    /* hwres - fftOutBuf */
    hwRes->fftOutSize = staticCfg->numDopplerBins * sizeof(cmplx32ReIm_t);
    if (dopStaticCfg->isBpmEnabled)
    {
        /* BPM only supports azimuth antenna */
        hwRes->fftOutSize *= 2U;
    }

    hwRes->fftOutBuf = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  hwRes->fftOutSize,
                                                  DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert((int)hwRes->fftOutBuf);

    /* hwres - sumAbsBuf */
    hwRes->sumAbsSize = staticCfg->numDopplerBins * sizeof(uint16_t);
    hwRes->sumAbsBuf  = (uint16_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  hwRes->sumAbsSize,
                                                  DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert((int)hwRes->sumAbsBuf);

    retVal = DPU_DopplerProcDSP_config(dpuHandle, &dopCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated */
    *cfgSave = dopCfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure CFARCA DPU.
 *
 *  @param[in]  dpuHandle               Handle to CFARCA DPU
 *  @param[in]  subFrameObj             Pointer to subFrame data object
 *  @param[in]  edmaHandle              Handle to edma driver to be used for the DPU
 *  @param[in]  detMatrix               Pointer to DPIF detection matrix, which will be the input
 *                                      to the CFARCA
 *  @param[in]  cfarRngDopSnrList       Pointer to range-doppler SNR list, which will be
 *                                      the output of CFARCA
 *  @param[in]  cfarRngDopSnrListSize   Range-doppler SNR List Size to which the list will be
 *                                      capped.
 *  @param[in]  CoreL2RamObj            Pointer to core local L2 RAM object to allocate local memory
 *                                      for the DPU, only for scratch purposes
 *  @param[in]  CoreL1RamObj            Pointer to core local L1 RAM object to allocate local memory
 *                                      for the DPU, only for scratch purposes
 *  @param[in]  rangeBias               Range Bias which will be used to adjust fov min value.
 *  @param[out] cfgSave                 Configuration that is built in local
 *                                      (stack) variable is saved here. This is for facilitating
 *                                      quick reconfiguration later without having to go through
 *                                      the construction of the configuration.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_CFARCAconfig
(
    DPU_CFARCAProcDSP_Handle dpuHandle,
    SubFrameObj              *subFrameObj,
    EDMA_Handle              edmaHandle,
    DPIF_DetMatrix           *detMatrix,
    DPIF_CFARDetList         *cfarRngDopSnrList,
    uint32_t                 cfarRngDopSnrListSize,
    MemPoolObj               *CoreL2RamObj,
    MemPoolObj               *CoreL1RamObj,
    float                    rangeBias,
    DPU_CFARCAProcDSP_Config *cfgSave
)
{
    int32_t retVal = 0;
    DPU_CFARCAProcDSP_Config cfarCfg;
    DPU_CFARCAProcDSP_HW_Resources *hwRes;
    uint32_t bitMaskCoreLocalRamSize;
    DPC_ObjectDetection_StaticCfg *staticCfg;
    DPC_ObjectDetection_DynCfg  *dynCfg;

    hwRes = &cfarCfg.res;
    staticCfg = &subFrameObj->staticCfg;
    dynCfg = &subFrameObj->dynCfg;

    memset(&cfarCfg, 0, sizeof(cfarCfg));

    /* Static config */
    cfarCfg.staticCfg.log2NumDopplerBins = subFrameObj->log2NumDopplerBins;
    cfarCfg.staticCfg.numDopplerBins     = staticCfg->numDopplerBins;
    cfarCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    cfarCfg.staticCfg.rangeStep          = staticCfg->rangeStep;
    cfarCfg.staticCfg.dopplerStep        = staticCfg->dopplerStep;

    /* Dynamic config */
    cfarCfg.dynCfg.fovDoppler     = &dynCfg->fovDoppler;
    cfarCfg.dynCfg.fovRange       = &dynCfg->fovRange;
    cfarCfg.dynCfg.cfarCfgDoppler = &dynCfg->cfarCfgDoppler;
    cfarCfg.dynCfg.cfarCfgRange   = &dynCfg->cfarCfgRange;

    /* Although CFARCA sopports both schemes, in DPC we fixed into one scheme */
    cfarCfg.dynCfg.cfarCfgDoppler->peakGroupingScheme = DPU_CFAR_PEAK_GROUPING_DET_MATRIX_BASED;
    cfarCfg.dynCfg.cfarCfgRange->peakGroupingScheme = DPU_CFAR_PEAK_GROUPING_DET_MATRIX_BASED;

    /* Need to adjust min by range bias */
    cfarCfg.dynCfg.fovRange->min  += rangeBias;

    /* Handware ressource config */
    hwRes->detMatrix = *detMatrix;
    hwRes->edmaHandle = edmaHandle;
    hwRes->edmaInPing.channel           = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PING_CH;
    hwRes->edmaInPing.channelShadow     = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PING_SHADOW;
    hwRes->edmaInPing.eventQueue        = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PING_EVENT_QUE;

    hwRes->edmaInPong.channel           = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PONG_CH;
    hwRes->edmaInPong.channelShadow     = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PONG_SHADOW;
    hwRes->edmaInPong.eventQueue        = DPC_OBJDET_DPU_CFARCA_PROC_EDMA_PONG_EVENT_QUE;

    hwRes->cfarDopplerDetOutBitMaskSize = (staticCfg->numRangeBins * staticCfg->numDopplerBins) / 32;
    bitMaskCoreLocalRamSize = hwRes->cfarDopplerDetOutBitMaskSize * sizeof(uint32_t);
    hwRes->cfarDopplerDetOutBitMask = (uint32_t *) DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                       bitMaskCoreLocalRamSize,
                                                       DPU_CFARCAPROCDSP_DOPPLER_DET_OUT_BIT_MASK_BYTE_ALIGNMENT);
    DebugP_assert(hwRes->cfarDopplerDetOutBitMask != NULL);

    hwRes->cfarRngDopSnrList = cfarRngDopSnrList;
    hwRes->cfarRngDopSnrListSize = cfarRngDopSnrListSize;

    hwRes->localDetMatrixBufferSize = MAX(staticCfg->numRangeBins,staticCfg->numDopplerBins ) * sizeof(uint16_t) * 2;
    hwRes->localDetMatrixBuffer = (uint16_t *) DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                       hwRes->localDetMatrixBufferSize,
                                                       DPU_CFARCAPROCDSP_DET_MATRIX_LOCAL_BYTE_ALIGNMENT);
    DebugP_assert(hwRes->localDetMatrixBuffer != NULL);

    hwRes->cfarDetOutBufferSize = MAX(staticCfg->numRangeBins,staticCfg->numDopplerBins ) * sizeof(uint16_t) * 2;
    hwRes->cfarDetOutBuffer = (uint16_t *) DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                               hwRes->cfarDetOutBufferSize,
                                                               DPU_CFARCAPROCDSP_CFAR_DETOUT_BYTE_ALIGNMENT);
    DebugP_assert(hwRes->cfarDetOutBuffer != NULL);

    /* Scratch buffers for CFAR. */
    hwRes->cfarScrachBufferSize = DPC_OBJDET_MAX_NUM_OBJECTS * sizeof(uint16_t) * 4;
    hwRes->cfarScrachBuffer = (uint16_t *) DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                               hwRes->cfarScrachBufferSize,
                                                               DPU_CFARCAPROCDSP_SCRACHBUF_BYTE_ALIGNMENT);
    DebugP_assert(hwRes->cfarScrachBuffer != NULL);

    retVal = DPU_CFARCAProcDSP_config(dpuHandle, &cfarCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated */
    *cfgSave = cfarCfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure AoA DPU. Note window information is passed to this function that
 *     is expected to be the same that was used in doppler processing because
 *     the AoA recomputes the doppler (2D) FFT. The reason for this recompute
 *     is because doppler does not update the radar cube (radar cube is range (1D) output).
 *     The window is used in AoA only for doppler FFT recompute, no window
 *     (in other words rectangular) is used for angle (3D) FFT computation.
 *     Note no DPIF info is passed here for AoA output because these DPIF buffers
 *     are not required to be shared with any other DPUs (but only at the exit
 *     f DPC's processing chain which will be consumed by the app) hence they are
 *     allocated within this function, this way more allocation remains localized
 *     in this function instead of being managed by the caller.
 *
 *  @param[in]  dpuHandle               Handle to DPU
 *  @param[in]  inpCommonCompRxCfg      Range bias and rx phase compensation common (across sub-frames)
 *                                      configuration, this will be used to extract the rx phase for sub-frame specific
 *                                      configuration based on antenna order.
 *  @param[in]  staticCfg               Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg                  Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle              Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube               Pointer to DPIF radar cube, which will be the
 *                                      input for AoA processing
 *  @param[in]  cfarRngDopSnrList       Pointer to range-doppler SNR list, which will be
 *                                      input for AoA processing
 *  @param[in]  cfarRngDopSnrListSize   Range-doppler SNR List Size to which the list
 *                                      was capped by cfar processing
 *  @param[in]  CoreL1RamObj            Pointer to core local L1 RAM object to allocate local memory
 *                                      for the DPU, all allocated memory will be permanent (within frame/sub-frame)
 *  @param[in]  CoreL2RamObj            Pointer to core local L2 RAM object to allocate local memory
 *                                      for the DPU, all allocated memory will be permanent (within frame/sub-frame)
 *  @param[in]  L3RamObj                Pointer to L3 RAM object to allocate L3RAM memory
 *                                      for the DPU, all allocated memory will be permanent (within frame/sub-frame)
 *  @param[in]  window                  Pointer to AoA doppler FFT window coefficients buffer
 *  @param[in]  windowSize              AoA Doppler FFT window size in bytes. See AoA DPU
 *                                      configuration for more information.
 *  @param[in]  twiddleBuf              Pointer to AoA doppler FFT twiddle buffer
 *  @param[in]  twiddleSize             Doppler FFT twiddle buffer size in bytes. See AoA DPU
 *                                      configuration for more information.
 *  @param[in]  angleTwiddle32x32       Pointer to angle 3D FFT twiddle buffer
 *  @param[in]  angleTwiddle32x32Size   3D (Azimuth/Elevation) FFT twiddle buffer size in bytes. See AoA DPU
 *                                      configuration for more information.
 *  @param[out] cfgSave                 Configuration that is built in local
 *                                      (stack) variable is saved here. This is for facilitating
 *                                      quick reconfiguration later without having to go through
 *                                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_AoAconfig
(
    DPU_AoAProcDSP_Handle            dpuHandle,
    DPU_AoAProc_compRxChannelBiasCfg *inpCommonCompRxCfg,
    DPC_ObjectDetection_StaticCfg    *staticCfg,
    DPC_ObjectDetection_DynCfg       *dynCfg,
    EDMA_Handle                      edmaHandle,
    DPIF_RadarCube                   *radarCube,
    DPIF_CFARDetList                 *cfarRngDopSnrList,
    uint32_t                         cfarRngDopSnrListSize,
    MemPoolObj                       *CoreL1RamObj,
    MemPoolObj                       *CoreL2RamObj,
    MemPoolObj                       *L3RamObj,
    int32_t                          *window,
    uint32_t                         windowSize,
    cmplx32ReIm_t                    *twiddleBuf,
    uint32_t                         twiddleSize,
    cmplx32ReIm_t                    *angleTwiddle32x32,
    uint32_t                         angleTwiddle32x32Size,
    DPU_AoAProcDSP_Config            *cfgSave
)
{
    int32_t retVal = 0;
    DPU_AoAProcDSP_Config aoaCfg;
    DPU_AoAProcDSP_HW_Resources *res;
    DPU_AoAProc_compRxChannelBiasCfg outCompRxCfg;

    res = &aoaCfg.res;
    memset(&aoaCfg, 0, sizeof(aoaCfg));

    /* Static config */
    aoaCfg.staticCfg.numDopplerChirps   = staticCfg->numDopplerChirps;
    aoaCfg.staticCfg.numDopplerBins     = staticCfg->numDopplerBins;
    aoaCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    aoaCfg.staticCfg.numRxAntennas      = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    aoaCfg.staticCfg.dopplerStep        = staticCfg->dopplerStep;
    aoaCfg.staticCfg.rangeStep          = staticCfg->rangeStep;
    aoaCfg.staticCfg.numTxAntennas      = staticCfg->numTxAntennas;
    aoaCfg.staticCfg.numVirtualAntAzim  = staticCfg->numVirtualAntAzim;
    aoaCfg.staticCfg.numVirtualAntElev  = staticCfg->numVirtualAntElev;
    aoaCfg.staticCfg.isBpmEnabled       = staticCfg->isBpmEnabled;

    /* Dynamic config */
    DPC_ObjDetDSP_GetRxChPhaseComp(staticCfg, inpCommonCompRxCfg, &outCompRxCfg);
    aoaCfg.dynCfg.compRxChanCfg              = &outCompRxCfg;
    aoaCfg.dynCfg.fovAoaCfg                  = &dynCfg->fovAoaCfg;
    aoaCfg.dynCfg.multiObjBeamFormingCfg     = &dynCfg->multiObjBeamFormingCfg;
    aoaCfg.dynCfg.prepareRangeAzimuthHeatMap = dynCfg->prepareRangeAzimuthHeatMap;
    aoaCfg.dynCfg.extMaxVelCfg               = &dynCfg->extMaxVelCfg;
    aoaCfg.dynCfg.staticClutterCfg.isEnabled = dynCfg->staticClutterRemovalCfg.enabled;

    /* Hardware Resources */
    res->radarCube = *radarCube;
    res->cfarRngDopSnrList = cfarRngDopSnrList;
    res->cfarRngDopSnrListSize = cfarRngDopSnrListSize;

    res->detObjOutMaxSize = DPC_OBJDET_MAX_NUM_OBJECTS;

    res->detObjOut = DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                         res->detObjOutMaxSize *sizeof(DPIF_PointCloudCartesian),
                         DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT);
    DebugP_assert(res->detObjOut != NULL);

    /* @TODO MMWAVESDK-1868:BEGIN */
    res->detObjOutSph = DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
            res->detObjOutMaxSize *sizeof(DPIF_PointCloudSpherical),
            DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT);
    
    DebugP_assert(res->detObjOutSph != NULL);
    /* @TODO MMWAVESDK-1868:END */
    res->detObjOutSideInfo = DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                 res->detObjOutMaxSize *sizeof(DPIF_PointCloudSideInfo),
                                 DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT);
    DebugP_assert(res->detObjOutSideInfo != NULL);

    res->detObj2dAzimIdx = DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                               res->detObjOutMaxSize *sizeof(uint8_t),
                               1U);
    DebugP_assert(res->detObj2dAzimIdx != NULL);

    res->detObjElevationAngle = DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                    res->detObjOutMaxSize *sizeof(float),
                                    DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT);
    DebugP_assert(res->detObjElevationAngle != NULL);

    if(aoaCfg.dynCfg.prepareRangeAzimuthHeatMap)
    {
        res->azimuthStaticHeatMapSize = staticCfg->numRangeBins * staticCfg->numVirtualAntAzim;
        res->azimuthStaticHeatMap = DPC_ObjDetDSP_MemPoolAlloc(L3RamObj,
                                         res->azimuthStaticHeatMapSize *sizeof(cmplx16ImRe_t),
                                         DPC_OBJDET_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT);
        DebugP_assert(res->azimuthStaticHeatMap != NULL);
    }

    /* 3D processing buffers */
    res->angleFftInSize = sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    res->angleFftIn = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  res->angleFftInSize,
                                                  DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(res->angleFftIn != NULL);

    /* AOA scratch buffers */
    res->scratch1Size = MAX(sizeof(cmplx32ReIm_t) * staticCfg->numDopplerBins,
                          (sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS));

    res->scratch1Buff = (void *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  res->scratch1Size,
                                                  DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(res->scratch1Buff != NULL);


    res->scratch2Size = MAX(sizeof(cmplx32ReIm_t) * staticCfg->numDopplerBins,
                          sizeof(float) * DPU_AOAPROCDSP_NUM_ANGLE_BINS);

    res->scratch2Buff = (void *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  res->scratch2Size,
                                                  DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(res->scratch2Buff != NULL);

    res->elevationFftOutSize = sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    res->elevationFftOut = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                                  res->elevationFftOutSize,
                                                  DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(res->elevationFftOut != NULL);

    /* Doppler calculation related buffers */
    res->pingPongSize      = 2U * sizeof(cmplx16ImRe_t) * staticCfg->numDopplerChirps;
    res->pingPongBuf       = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                                  res->pingPongSize,
                                                  DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(res->pingPongBuf != NULL);

    /* Coeff buffers */
    res->windowCoeff       = window;
    res->windowSize        = windowSize;

    res->twiddle32x32      = twiddleBuf;
    res->twiddleSize       = twiddleSize;
    
    res->angleTwiddleSize  = angleTwiddle32x32Size;
    res->angleTwiddle32x32  = angleTwiddle32x32;

    res->edmaHandle = edmaHandle;
    res->edmaPing.channel       = DPC_OBJDET_DPU_AOA_PROC_EDMA_PING_CH   ;
    res->edmaPing.channelShadow = DPC_OBJDET_DPU_AOA_PROC_EDMA_PING_SHADOW   ;
    res->edmaPing.eventQueue    = DPC_OBJDET_DPU_AOA_PROC_EDMA_PING_EVENT_QUE;

    res->edmaPong.channel       = DPC_OBJDET_DPU_AOA_PROC_EDMA_PONG_CH ;
    res->edmaPong.channelShadow = DPC_OBJDET_DPU_AOA_PROC_EDMA_PONG_SHADOW     ;
    res->edmaPong.eventQueue    = DPC_OBJDET_DPU_AOA_PROC_EDMA_PONG_EVENT_QUE  ;

    retVal = DPU_AoAProcDSP_config(dpuHandle, &aoaCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window and compRx will need to be regenerated */
    *cfgSave = aoaCfg;

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
 *     @ref DPC_ObjectDetection_InitParams given to @ref DPC_ObjectDetection_init API
 *  3. L3 memory will only be allocated from the L3 RAM configuration supplied in
 *     @ref DPC_ObjectDetection_InitParams given to @ref DPC_ObjectDetection_init API
 *     No L3 buffers are presently required that need to be preserved across sub-frames
 *     (type described in #1 above), neither are L3 scratch buffers required for
 *     intermediate processing within DPU process call.
 *
 *  @param[in]  subFrameObj     Pointer to sub-frame object
 *  @param[in]  commonCfg       Pointer to pre-start common configuration
 *  @param[in]  preStartCfg     Pointer to pre-start configuration of the sub-frame
 *  @param[in]  edmaHandle      Pointer to array of EDMA handles for the device, this
 *                              can be distributed among the DPUs, the actual EDMA handle used
 *                              in DPC is determined by definition in application resource file
 *  @param[in]  L3ramObj        Pointer to L3 RAM memory pool object
 *  @param[in]  CoreL2RamObj    Pointer to Core Local L2 memory pool object
 *  @param[in]  CoreL1RamObj    Pointer to Core Local L1 memory pool object
 *  @param[out] L3RamUsage      Net L3 RAM memory usage in bytes as a result of allocation
 *                              by the DPUs.
 *  @param[out] CoreL2RamUsage  Net Local L2 RAM memory usage in bytes
 *  @param[out] CoreL1RamUsage  Net Core L1 RAM memory usage in bytes
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetDSP_preStartConfig
(
    SubFrameObj                    *subFrameObj,
    DPC_ObjectDetection_PreStartCommonCfg *commonCfg,
    DPC_ObjectDetection_PreStartCfg  *preStartCfg,
    EDMA_Handle                    edmaHandle[EDMA_NUM_CC],
    MemPoolObj                     *L3ramObj,
    MemPoolObj                     *CoreL2RamObj,
    MemPoolObj                     *CoreL1RamObj,
    uint32_t                       *L3RamUsage,
    uint32_t                       *CoreL2RamUsage,
    uint32_t                       *CoreL1RamUsage
)
{
    int32_t retVal = 0;
    DPC_ObjectDetection_StaticCfg  *staticCfg;
    DPC_ObjectDetection_DynCfg     *dynCfg;
    DPIF_RadarCube radarCube;
    DPIF_DetMatrix detMatrix;
#ifndef OBJDET_NO_RANGE
    int16_t *rangeWindow;
    uint32_t rangeWindowSize;
    cmplx16ImRe_t *rangeTwiddle16x16;
    uint32_t rangeTwiddleSize;
#endif
    DPIF_CFARDetList *cfarRngDopSnrList;
    uint32_t cfarRngDopSnrListSize;
    void *CoreL2ScratchStartPoolAddr;
    void *CoreL1ScratchStartPoolAddr;

    int32_t         *dopplerWindow;
    uint32_t        dopplerWindowSize;
    cmplx32ReIm_t   *dopplerTwiddle;
    uint32_t        dopplerTwiddleSize;
    cmplx16ImRe_t   *dftSinCosTable;
    uint32_t        dftSinCosTableSize;
    uint32_t        angleTwiddle32x32Size;
    cmplx32ReIm_t   *angleTwiddle32x32;

    staticCfg = &preStartCfg->staticCfg;
    dynCfg = &preStartCfg->dynCfg;

    /* Save configs to object. We need to pass this stored config (instead of
       the input arguments to this function which will be in stack) to
       the DPU config functions inside of this function because the DPUs
       have pointers to dynamic configurations which are later going to be
       reused during re-configuration (intra sub-frame or inter sub-frame)
     */
    subFrameObj->staticCfg = *staticCfg;
    subFrameObj->dynCfg = *dynCfg;

    /* Run-time L3RAM memory configuration
     * By default, L3RAM memory configuration is initialized during DPC init time.
     * L3 RAM and radar cube memory configuration can also be overwritten at run time
     * in case the range DPU is not part of the DPC chain.
     */
    if(preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        /* L3 RAM configuration */
        if(preStartCfg->shareMemCfg.L3Ram.addr != NULL)
        {
            L3ramObj->cfg.addr = preStartCfg->shareMemCfg.L3Ram.addr;
            L3ramObj->cfg.size = preStartCfg->shareMemCfg.L3Ram.size;
        }
        else
        {
            retVal = DPC_OBJECTDETECTION_EINVAL__COMMAND;
            goto exit;
        }
    }

    /* Derived config */
    subFrameObj->log2NumDopplerBins = mathUtils_floorLog2(staticCfg->numDopplerBins);

    DPC_ObjDetDSP_MemPoolReset(L3ramObj);
    DPC_ObjDetDSP_MemPoolReset(CoreL2RamObj);
    DPC_ObjDetDSP_MemPoolReset(CoreL1RamObj);

    /* L3 allocations */
    /* L3 - radar cube */
    radarCube.dataSize = staticCfg->numRangeBins * staticCfg->numDopplerChirps *
                         staticCfg->numVirtualAntennas * sizeof(cmplx16ReIm_t);
    if(preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        if((preStartCfg->shareMemCfg.radarCubeMem.addr != NULL) &&
          (preStartCfg->shareMemCfg.radarCubeMem.size == radarCube.dataSize))
        {
            /* Use assigned radar cube address */
            radarCube.data = preStartCfg->shareMemCfg.radarCubeMem.addr;
        }
        else
        {
            retVal = DPC_OBJECTDETECTION_EINVAL__COMMAND;
            goto exit;
        }
    }
    else
    {
        /* Allocate from memory */
        radarCube.data = DPC_ObjDetDSP_MemPoolAlloc(L3ramObj, radarCube.dataSize,
                                                 DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);

        if (radarCube.data == NULL)
        {
            retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE;
            goto exit;
        }
    }

    /* Only supported radar Cube format in this DPC */
    radarCube.datafmt = DPIF_RADARCUBE_FORMAT_1;

    /* L3 - detection matrix */
    detMatrix.dataSize = staticCfg->numRangeBins * staticCfg->numDopplerBins * sizeof(uint16_t);
    detMatrix.data = DPC_ObjDetDSP_MemPoolAlloc(L3ramObj, detMatrix.dataSize,
                                             DPC_OBJDET_DET_MATRIX_DATABUF_BYTE_ALIGNMENT);
    if (detMatrix.data == NULL)
    {
        retVal = DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX;
        goto exit;
    }
    detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;

    /* L1 or L2 Local memory allocaiton that are not shared between DPUs.
      It includes windowing coeffecients buffer, twiddle buffer, sin/cos table and CFAR detection list etc.
     */

    /* Core Local - CFAR output list */
    cfarRngDopSnrListSize = DPC_OBJDET_MAX_NUM_OBJECTS;

    cfarRngDopSnrList = DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                            cfarRngDopSnrListSize * sizeof(DPIF_CFARDetList),
                            DPC_OBJDET_CFAR_DET_LIST_BYTE_ALIGNMENT);
    DebugP_assert(cfarRngDopSnrList != NULL);

#ifndef OBJDET_NO_RANGE
    /* hwres - range DPU window allocate and generate */
    rangeWindowSize = DPC_ObjDetDSP_GetRangeWinGenLen(staticCfg->ADCBufData.dataProperty.numAdcSamples) * sizeof(uint16_t);
    rangeWindow = (int16_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj, rangeWindowSize,
                                                      DPU_RANGEPROCDSP_WINDOW_BYTE_ALIGNMENT_DSP);
    DebugP_assert(rangeWindow != NULL);

    /* hwres - range DPU twiddle allocate */
    rangeTwiddleSize = staticCfg->numRangeBins * sizeof(cmplx16ImRe_t);
    rangeTwiddle16x16 = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                              rangeTwiddleSize,
                                              DPU_RANGEPROCDSP_TWIDDLEBUF_BYTE_ALIGNMENT_DSP);
    DebugP_assert(rangeTwiddle16x16 != NULL);
#endif

    /* Doppler/AoA DPU window buffer allocation and generation */
    dopplerWindowSize = DPC_ObjDetDSP_GetDopplerWinGenLen(staticCfg->numDopplerChirps) * sizeof(cmplx16ImRe_t);
    dopplerWindow = DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj, dopplerWindowSize, DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert(dopplerWindow != NULL);

    /* Doppler/AoA DPU Twiddle buffer */
    dopplerTwiddleSize = staticCfg->numDopplerBins * sizeof(cmplx32ReIm_t);
    dopplerTwiddle = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL2RamObj,
                                              dopplerTwiddleSize,
                                              DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert(dopplerTwiddle != NULL);

    /* Doppler/AoA DPU DFT Sin/Cos table */
    dftSinCosTableSize  = staticCfg->numDopplerBins * sizeof(cmplx16ImRe_t);
    dftSinCosTable = (cmplx16ImRe_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                              dftSinCosTableSize,
                                              DPU_DOPPLERPROCDSP_BUFFER_BYTE_ALIGNMENT);
    DebugP_assert(dftSinCosTable != NULL);

    /* AoA DPU Twiddle buffer */
    angleTwiddle32x32Size = sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS;
    angleTwiddle32x32 = (cmplx32ReIm_t *)DPC_ObjDetDSP_MemPoolAlloc(CoreL1RamObj,
                                          angleTwiddle32x32Size,
                                          DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT);
    DebugP_assert(angleTwiddle32x32 != NULL);

    /* Remember pool (L1 & L2) position. These addresses will be the start address for scratch buffers */
    CoreL2ScratchStartPoolAddr = DPC_ObjDetDSP_MemPoolGet(CoreL2RamObj);
    CoreL1ScratchStartPoolAddr = DPC_ObjDetDSP_MemPoolGet(CoreL1RamObj);

#ifndef OBJDET_NO_RANGE
    DebugP_assert(edmaHandle[DPC_OBJDET_DPU_RANGEPROC_EDMA_INST_ID] != NULL);
    retVal = DPC_ObjDetDSP_rangeConfig(subFrameObj->dpuRangeObj, &subFrameObj->staticCfg, &subFrameObj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_RANGEPROC_EDMA_INST_ID],
                 &radarCube, CoreL2RamObj, CoreL1RamObj,
                 rangeWindow, rangeWindowSize,
                 rangeTwiddle16x16,rangeTwiddleSize,
                 &subFrameObj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }
#endif

    /* Rewind to the scratch beginning */
    DPC_ObjDetDSP_MemPoolSet(CoreL2RamObj, CoreL2ScratchStartPoolAddr);
    DPC_ObjDetDSP_MemPoolSet(CoreL1RamObj, CoreL1ScratchStartPoolAddr);

    DebugP_assert(edmaHandle[DPC_OBJDET_DPU_CFARCA_PROC_EDMA_INST_ID] != NULL);
    retVal = DPC_ObjDetDSP_CFARCAconfig(subFrameObj->dpuCFARCAObj,
                 subFrameObj,
                 edmaHandle[DPC_OBJDET_DPU_CFARCA_PROC_EDMA_INST_ID],
                 &detMatrix,
                 cfarRngDopSnrList,
                 cfarRngDopSnrListSize,
                 CoreL2RamObj,
                 CoreL1RamObj,
                 commonCfg->compRxChanCfg.rangeBias,
                 &subFrameObj->dpuCfg.cfarCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Rewind to the scratch beginning */
    DPC_ObjDetDSP_MemPoolSet(CoreL2RamObj, CoreL2ScratchStartPoolAddr);
    DPC_ObjDetDSP_MemPoolSet(CoreL1RamObj, CoreL1ScratchStartPoolAddr);

    retVal = DPC_ObjDetDSP_dopplerConfig(subFrameObj->dpuDopplerObj, &subFrameObj->staticCfg,
                 subFrameObj->log2NumDopplerBins, &subFrameObj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_DOPPLERPROC_EDMA_INST_ID],
                 &radarCube, &detMatrix, 
                 CoreL2RamObj,
                 CoreL1RamObj,
                 dopplerWindow, dopplerWindowSize,
                 dopplerTwiddle, dopplerTwiddleSize,
                 dftSinCosTable,dftSinCosTableSize,
                 &subFrameObj->dpuCfg.dopplerCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Rewind to the scratch beginning */
    DPC_ObjDetDSP_MemPoolSet(CoreL2RamObj, CoreL2ScratchStartPoolAddr);
    DPC_ObjDetDSP_MemPoolSet(CoreL1RamObj, CoreL1ScratchStartPoolAddr);

    DebugP_assert(edmaHandle[DPC_OBJDET_DPU_AOA_PROC_EDMA_INST_ID] != NULL);
    retVal = DPC_ObjDetDSP_AoAconfig(subFrameObj->dpuAoAObj, &commonCfg->compRxChanCfg,
                 &subFrameObj->staticCfg, &subFrameObj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_AOA_PROC_EDMA_INST_ID],
                 &radarCube,
                 cfarRngDopSnrList, cfarRngDopSnrListSize,
                 CoreL1RamObj,
                 CoreL2RamObj,
                 L3ramObj,
                 dopplerWindow, dopplerWindowSize,
                 dopplerTwiddle, dopplerTwiddleSize,
                 angleTwiddle32x32, angleTwiddle32x32Size,
                 &subFrameObj->dpuCfg.aoaCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Report RAM usage */
    *CoreL2RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(CoreL2RamObj);
    *CoreL1RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(CoreL1RamObj);
    *L3RamUsage = DPC_ObjDetDSP_MemPoolGetMaxUsage(L3ramObj);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC frame start function registered with DPM. This is invoked on reception
 *      of the frame start ISR from the RF front-end. This API is also invoked
 *      when application issues @ref DPC_OBJDET_IOCTL__TRIGGER_FRAME to simulate
 *      a frame trigger (e.g for unit testing purpose).
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
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

    return;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) start function which is invoked by the
 *      application using DPM_start API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_start (DPM_DPCHandle handle)
{
    ObjDetObj   *objDetObj;
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
        DPC_ObjDetDSP_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
    }
    DebugP_log0("ObjDet DPC: Start done\n");
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_stop (DPM_DPCHandle handle)
{
    ObjDetObj   *objDetObj;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);

    /* We can be here only after complete frame processing is done, which means
     * processing token must be 0 and subFrameIndx also 0  */
    DebugP_assert((objDetObj->interSubFrameProcToken == 0) && (objDetObj->subFrameIndx == 0));

    DebugP_log0("ObjDet DPC: Stop done\n");
    return(0);
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
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
#ifndef OBJDET_NO_RANGE
    DPU_RangeProcDSP_OutParams outRangeProc;
#endif
    DPU_DopplerProcDSP_OutParams outDopplerProc;
    DPU_CFARCAProcDSP_OutParams outCfarcaProc;
    DPU_AoAProcDSP_OutParams outAoaProc;
#ifdef TRACKERPROC_ON_DSP
    DPU_TrackerProc_OutParams outTrackerProc;
#endif
    DPC_ObjectDetection_ProcessCallBackCfg *processCallBack;
    DPC_ObjectDetection_ExecuteResult *result;
    int32_t retVal;
    uint8_t numChirpsPerChirpEvent;
    int32_t i;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);
    DebugP_assert (ptrResult != NULL);

    DebugP_log1("ObjDet DPC: Processing sub-frame %d\n", objDetObj->subFrameIndx);

    processCallBack = &objDetObj->processCallBackCfg;
    result = &objDetObj->executeResult;

    subFrmObj = &objDetObj->subFrameObj[objDetObj->subFrameIndx];
    numChirpsPerChirpEvent = subFrmObj->staticCfg.ADCBufData.dataProperty.numChirpsPerChirpEvent;

    /*==============================================
                   Chirp Processing
     ===============================================*/
#ifndef OBJDET_NO_RANGE
    if (((objDetObj->chirpIndex % subFrmObj->staticCfg.numChirpsPerFrame)== 0) &&
       (processCallBack->processFrameBeginCallBackFxn != NULL))
    {
        (*processCallBack->processFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
    }
    retVal = DPU_RangeProcDSP_process(subFrmObj->dpuRangeObj, &outRangeProc);
    if (retVal != 0)
    {
        goto exit;
    }

    objDetObj->chirpEndTime = Cycleprofiler_getTimeStamp();

    /* Chirp is processed, increse chirpIndex */
    objDetObj->chirpIndex += numChirpsPerChirpEvent;

    if (outRangeProc.endOfChirp == true)
#endif

    {
        if (processCallBack->processInterFrameBeginCallBackFxn != NULL)
        {
            (*processCallBack->processInterFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
        }
        objDetObj->stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();

        DebugP_log0("ObjDet DPC: Range Proc Done\n");

        retVal = DPU_DopplerProcDSP_process(subFrmObj->dpuDopplerObj, &outDopplerProc);
        if (retVal != 0)
        {
            goto exit;
        }
#if 0
        /* Procedure for range bias measurement and Rx channels gain/phase offset measurement */
        if(objDetObj->commonCfg.measureRxChannelBiasCfg.enabled)
        {
            DPC_ObjDetDSP_rangeBiasRxChPhaseMeasure(&subFrmObj->staticCfg,
                objDetObj->commonCfg.measureRxChannelBiasCfg.targetDistance,
                objDetObj->commonCfg.measureRxChannelBiasCfg.searchWinSize,
                subFrmObj->dpuCfg.dopplerCfg.hwRes.detMatrix.data,
                (uint32_t *) subFrmObj->dpuCfg.dopplerCfg.hwRes.radarCube.data,
                &objDetObj->compRxChanCfgMeasureOut);
        }
#endif
        retVal = DPU_CFARCAProcDSP_process(subFrmObj->dpuCFARCAObj, &outCfarcaProc);
        if (retVal != 0)
        {
            goto exit;
        }

        DebugP_log1("ObjDet DPC: number of detected objects after CFAR = %d\n",
                    outCfarcaProc.numCfarDetectedPoints);

        retVal = DPU_AoAProcDSP_process(subFrmObj->dpuAoAObj,
                     outCfarcaProc.numCfarDetectedPoints, &outAoaProc);
        if (retVal != 0)
        {
            goto exit;
        }

#ifdef TRACKERPROC_ON_DSP
        retVal = DPU_TrackerProc_process(objDetObj->dpuTrackerObj, outAoaProc.numAoADetectedPoints,
                subFrmObj->dpuCfg.aoaCfg.res.detObjOutSph, subFrmObj->dpuCfg.aoaCfg.res.detObjOutSideInfo,
                &outTrackerProc);

        if (retVal != 0)
        {
            goto exit;
        }
#endif
        /* Set DPM result with measure (bias, phase) and detection info */
        result->numObjOut = outAoaProc.numAoADetectedPoints;
        result->subFrameIdx = objDetObj->subFrameIndx;
        result->objOut               = subFrmObj->dpuCfg.aoaCfg.res.detObjOut;

        result->objOutSph            = subFrmObj->dpuCfg.aoaCfg.res.detObjOutSph;
#ifdef TRACKERPROC_ON_DSP
        result->trackerCurrDesc      = outTrackerProc.currentTargetDesc; 
        result->numTargets           = outTrackerProc.numTargets; 
        result->tList                = ((trackerProcObjType*)objDetObj->dpuTrackerObj)->targetDescrHandle->tList[result->trackerCurrDesc];   
        result->numIndices           = outTrackerProc.numIndices; 
        result->tIndex               = ((trackerProcObjType*)objDetObj->dpuTrackerObj)->targetDescrHandle->tIndex[result->trackerCurrDesc];   
#endif
        result->objOutSideInfo       = subFrmObj->dpuCfg.aoaCfg.res.detObjOutSideInfo;
#if 0
        result->azimuthStaticHeatMap = subFrmObj->dpuCfg.aoaCfg.res.azimuthStaticHeatMap;
        result->azimuthStaticHeatMapSize = subFrmObj->dpuCfg.aoaCfg.res.azimuthStaticHeatMapSize;
        result->radarCube            = subFrmObj->dpuCfg.aoaCfg.res.radarCube;
        result->detMatrix            = subFrmObj->dpuCfg.dopplerCfg.hwRes.detMatrix;
#endif
        if (objDetObj->commonCfg.measureRxChannelBiasCfg.enabled == 1)
        {
            result->compRxChanBiasMeasurement = &objDetObj->compRxChanCfgMeasureOut;
        }
        else
        {
            result->compRxChanBiasMeasurement = NULL;
        }

        /* interChirpProcessingMargin is the smallest margin in the frame */
        objDetObj->stats.interChirpProcessingMargin = objDetObj->chirpMargin / numChirpsPerChirpEvent;

        objDetObj->stats.interFrameEndTimeStamp = Cycleprofiler_getTimeStamp();
        result->stats = &objDetObj->stats;

        /* populate DPM_resultBuf - first pointer and size are for results of the processing */
        ptrResult->ptrBuffer[0] = (uint8_t *)result;
        ptrResult->size[0] = sizeof(DPC_ObjectDetection_ExecuteResult);

        /* clear rest of the result */
        for (i = 1; i < DPM_MAX_BUFFER; i++)
        {
            ptrResult->ptrBuffer[i] = NULL;
            ptrResult->size[i] = 0;
        }

        /* Frame is done, reset chirp index */
        objDetObj->chirpIndex = 0;
    }

exit:
    return retVal;
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_ioctl
(
    DPM_DPCHandle       handle,
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

    /* Process the commands. Process non sub-frame specific ones first
     * so the sub-frame specific ones can share some code. */
    if((cmd < DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG) || (cmd > DPC_OBJDET_IOCTL__MAX))
    {
        retVal = DPM_EINVCMD;
    }
    else if (cmd == DPC_OBJDET_IOCTL__TRIGGER_FRAME)
    {
        DPC_ObjectDetection_frameStart(handle);
    }
    else if(cmd == DPC_OBJDET_IOCTL__TRIGGER_CHIRP)
    {
#ifndef OBJDET_NO_RANGE
        DPC_ObjectDetection_chirpEvent(handle);
#else
        DPC_ObjectDetection_dataInjection(handle, NULL);
#endif
    }
    else if (cmd == DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
        DPC_ObjectDetection_PreStartCommonCfg *cfg;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_PreStartCommonCfg));

        cfg = (DPC_ObjectDetection_PreStartCommonCfg*)arg;

#ifndef OBJDET_NO_RANGE
        int32_t indx;

        /* Free all buffers that were allocated from system (MemoryP) heap.
         * Note we cannot free buffers during allocation time
         * for new config during the pre-start config processing because the heap is not capable
         * of defragmentation. This means pre-start common config must precede
         * all pre-start configs. */
        for(indx = 0; indx < objDetObj->commonCfg.numSubFrames; indx++)
        {
            subFrmObj = &objDetObj->subFrameObj[indx];

            if (subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMean)
            {
                MemoryP_ctrlFree(subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMean,
                                 subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMeanSize);
            }
        }
#endif

        objDetObj->commonCfg = *cfg;
        objDetObj->isCommonCfgReceived = true;

        DebugP_log0("ObjDet DPC: Pre-start Common Config IOCTL processed\n");
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE)
    {
        DPC_ObjectDetection_MeasureRxChannelBiasCfg *cfg;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_MeasureRxChannelBiasCfg));

        cfg = (DPC_ObjectDetection_MeasureRxChannelBiasCfg*)arg;

        retVal = DPC_ObjDetDSP_Config_MeasureRxChannelBiasCfg(objDetObj, cfg);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE)
    {
        DPU_AoAProc_compRxChannelBiasCfg *inpCfg;
        DPU_AoAProc_compRxChannelBiasCfg outCfg;
        int32_t i;

        DebugP_assert(argLen == sizeof(DPU_AoAProc_compRxChannelBiasCfg));

        inpCfg = (DPU_AoAProc_compRxChannelBiasCfg*)arg;

        for(i = 0; i < objDetObj->commonCfg.numSubFrames; i++)
        {
            subFrmObj = &objDetObj->subFrameObj[i];

            DPC_ObjDetDSP_GetRxChPhaseComp(&subFrmObj->staticCfg, inpCfg, &outCfg);

            retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                     DPU_AoAProcDSP_Cmd_CompRxChannelBiasCfg,
                     &outCfg,
                     sizeof(DPU_AoAProc_compRxChannelBiasCfg));
            if (retVal != 0)
            {
                goto exit;
            }
        }

        /* save into object */
        objDetObj->commonCfg.compRxChanCfg = *inpCfg;
    }
#ifdef TRACKERPROC_ON_DSP
    else if (cmd == DPC_OBJDET_IOCTL__STATIC_TRACKER_CFG)
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
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED)
    {
        DPC_ObjectDetection_ExecuteResultExportedInfo *inp;
        volatile uint32_t startTime;

        startTime = Cycleprofiler_getTimeStamp();

        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_ExecuteResultExportedInfo));

        inp = (DPC_ObjectDetection_ExecuteResultExportedInfo *)arg;

        /* input sub-frame index must match current sub-frame index */
        DebugP_assert(inp->subFrameIdx == objDetObj->subFrameIndx);

        /* Reconfigure all DPUs resources for next sub-frame as EDMA and scrach buffer
         * resources overlap across sub-frames */
        if (objDetObj->commonCfg.numSubFrames > 1)
        {
            /* Next sub-frame */
            objDetObj->subFrameIndx++;
            if (objDetObj->subFrameIndx == objDetObj->commonCfg.numSubFrames)
            {
                objDetObj->subFrameIndx = 0;
            }

            DPC_ObjDetDSP_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
        }
        DebugP_log0("ObjDet DPC: Range Proc Triggered in export IOCTL\n");

        objDetObj->stats.subFramePreparationCycles =
            Cycleprofiler_getTimeStamp() - startTime;

        /* mark end of processing of the frame/sub-frame by the DPC and the app */
        objDetObj->interSubFrameProcToken--;
    }
    else
    {
        uint8_t subFrameNum;

        /* First argument is sub-frame number */
        DebugP_assert(arg != NULL);
        subFrameNum = *(uint8_t *)arg;
        subFrmObj = &objDetObj->subFrameObj[subFrameNum];

        switch (cmd)
        {

#ifndef OBJDET_NO_RANGE
            /* Range DPU related */
            case DPC_OBJDET_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG:
            {
                DPC_ObjectDetection_CalibDcRangeSigCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_CalibDcRangeSigCfg));

                cfg = (DPC_ObjectDetection_CalibDcRangeSigCfg*)arg;

                retVal = DPU_RangeProcDSP_control(subFrmObj->dpuRangeObj,
                             DPU_RangeProcDSP_Cmd_dcRangeCfg,
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
#endif
            /* CFAR DPU related */
            case DPC_OBJDET_IOCTL__DYNAMIC_CFAR_RANGE_CFG:
            {
                DPC_ObjectDetection_CfarCfg *cfg;
                DPU_CFARCAProc_CfarCfg *cfarCfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_CfarCfg));

                cfg = (DPC_ObjectDetection_CfarCfg*)arg;
                cfarCfg = &cfg->cfg;

                /* Although CFARCA sopports both schemes, in DPC we fixed into one scheme */
                cfarCfg->peakGroupingScheme = DPU_CFAR_PEAK_GROUPING_DET_MATRIX_BASED;

                retVal = DPU_CFARCAProcDSP_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcDSP_Cmd_CfarRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_CfarCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.cfarCfgRange = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_CFAR_DOPPLER_CFG:
            {
                DPC_ObjectDetection_CfarCfg *cfg;
                DPU_CFARCAProc_CfarCfg *cfarCfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_CfarCfg));

                cfg = (DPC_ObjectDetection_CfarCfg*)arg;
                cfarCfg = &cfg->cfg;

                /* Although CFARCA sopports both schemes, in DPC we fixed into one scheme */
                cfarCfg->peakGroupingScheme = DPU_CFAR_PEAK_GROUPING_DET_MATRIX_BASED;

                retVal = DPU_CFARCAProcDSP_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcDSP_Cmd_CfarDopplerCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_CfarCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.cfarCfgDoppler = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_RANGE:
            {
                DPC_ObjectDetection_fovRangeCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_fovRangeCfg));

                cfg = (DPC_ObjectDetection_fovRangeCfg*)arg;

                /* Add range bias to the minimum/maximum */
                cfg->cfg.min += objDetObj->commonCfg.compRxChanCfg.rangeBias;
                cfg->cfg.max += objDetObj->commonCfg.compRxChanCfg.rangeBias;

                retVal = DPU_CFARCAProcDSP_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcDSP_Cmd_FovRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_FovCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovRange = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_DOPPLER:
            {
                DPC_ObjectDetection_fovDopplerCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_fovDopplerCfg));

                cfg = (DPC_ObjectDetection_fovDopplerCfg*)arg;

                retVal = DPU_CFARCAProcDSP_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcDSP_Cmd_FovDopplerCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_FovCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovDoppler = cfg->cfg;

                break;
            }

            /* AoA DPU related */
            case DPC_OBJDET_IOCTL__DYNAMIC_MULTI_OBJ_BEAM_FORM_CFG:
            {
                DPC_ObjectDetection_MultiObjBeamFormingCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_MultiObjBeamFormingCfg));

                cfg = (DPC_ObjectDetection_MultiObjBeamFormingCfg*)arg;

                retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDSP_Cmd_MultiObjBeamFormingCfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProc_MultiObjBeamFormingCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.multiObjBeamFormingCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_EXT_MAX_VELOCITY:
            {
                DPC_ObjectDetection_extMaxVelCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_extMaxVelCfg));

                cfg = (DPC_ObjectDetection_extMaxVelCfg*)arg;

                retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDSP_Cmd_ExtMaxVelocityCfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProc_ExtendedMaxVelocityCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.extMaxVelCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_AOA:
            {
                DPC_ObjectDetection_fovAoaCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_fovAoaCfg));

                cfg = (DPC_ObjectDetection_fovAoaCfg*)arg;

                retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDSP_Cmd_FovAoACfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProc_FovAoaCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovAoaCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_RANGE_AZIMUTH_HEAT_MAP:
            {
                DPC_ObjectDetection_RangeAzimuthHeatMapCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_RangeAzimuthHeatMapCfg));

                cfg = (DPC_ObjectDetection_RangeAzimuthHeatMapCfg*)arg;

                retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDSP_Cmd_PrepareRangeAzimuthHeatMap,
                             &cfg->prepareRangeAzimuthHeatMap,
                             sizeof(bool));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.prepareRangeAzimuthHeatMap = cfg->prepareRangeAzimuthHeatMap;

                break;
            }

            /* Static clutter related */
            case DPC_OBJDET_IOCTL__DYNAMIC_STATICCLUTTER_REMOVAL_CFG:
            {
                DPC_ObjectDetection_StaticClutterRemovalCfg *cfg;
                DPU_AoAProc_StaticClutterRemovalCfg aoaStaticClutterCfg;
                DPU_DopplerProc_StaticClutterRemovalCfg dopStaticClutterCfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_StaticClutterRemovalCfg));

                cfg = (DPC_ObjectDetection_StaticClutterRemovalCfg*)arg;

                /* Send control command to Doppler */
                dopStaticClutterCfg.isEnabled = cfg->cfg.enabled;
                retVal = DPU_DopplerProcDSP_control(subFrmObj->dpuDopplerObj,
                             DPU_DopplerProcDSP_Cmd_staticClutterCfg,
                             &dopStaticClutterCfg,
                             sizeof(DPU_DopplerProc_StaticClutterRemovalCfg));
                if (retVal != 0)
                {
                    goto exit;
                }
                subFrmObj->dpuCfg.dopplerCfg.dynCfg.staticClutterCfg.isEnabled = cfg->cfg.enabled;

                /* Send control command to AoA */
                aoaStaticClutterCfg.isEnabled = cfg->cfg.enabled;
                retVal = DPU_AoAProcDSP_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDSP_Cmd_staticClutterCfg,
                             &aoaStaticClutterCfg,
                             sizeof(DPU_AoAProc_StaticClutterRemovalCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                subFrmObj->dpuCfg.aoaCfg.dynCfg.staticClutterCfg.isEnabled= cfg->cfg.enabled;
                subFrmObj->dynCfg.staticClutterRemovalCfg.enabled = cfg->cfg.enabled;

                break;
            }

            /* Related to pre-start configuration */
            case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            {
                DPC_ObjectDetection_PreStartCfg *cfg;
                DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage *memUsage;
                MemoryP_Stats statsStart;
                MemoryP_Stats statsEnd;

                /* Pre-start common config must be received before pre-start configs
                 * are received. */
                if (objDetObj->isCommonCfgReceived == false)
                {
                    retVal = DPC_OBJECTDETECTION_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;
                    goto exit;
                }

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_PreStartCfg));

                /* Get system heap size before preStart configuration */
                MemoryP_getStats(&statsStart);

                cfg = (DPC_ObjectDetection_PreStartCfg*)arg;

                memUsage = &cfg->memUsage;
                memUsage->L3RamTotal = objDetObj->L3RamObj.cfg.size;
                memUsage->CoreL2RamTotal = objDetObj->CoreL2RamObj.cfg.size;
                memUsage->CoreL1RamTotal = objDetObj->CoreL1RamObj.cfg.size;
                retVal = DPC_ObjDetDSP_preStartConfig(subFrmObj,
                             &objDetObj->commonCfg, 
                             cfg,
                             &objDetObj->edmaHandle[0],
                             &objDetObj->L3RamObj,
                             &objDetObj->CoreL2RamObj,
                             &objDetObj->CoreL1RamObj,
                             &memUsage->L3RamUsage,
                             &memUsage->CoreL2RamUsage,
                             &memUsage->CoreL1RamUsage);
                if (retVal != 0)
                {
                    goto exit;
                }

                /* Get system heap size after preStart configuration */
                MemoryP_getStats(&statsEnd);

                /* Populate system heap usage */
                memUsage->SystemHeapTotal = statsEnd.totalSize;
                memUsage->SystemHeapUsed = statsEnd.totalSize -statsEnd.totalFreeSize;
                memUsage->SystemHeapDPCUsed = statsStart.totalFreeSize - statsEnd.totalFreeSize;

                DebugP_log1("ObjDet DPC: Pre-start Config IOCTL processed (subFrameIndx = %d)\n", subFrameNum);
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
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
    ObjDetObj     *objDetObj = NULL;
    DPC_ObjectDetection_InitParams *dpcInitParams;
    int32_t i;

    *errCode = 0;

    if ((ptrInitCfg == NULL) || (ptrInitCfg->arg == NULL))
    {
        *errCode = DPC_OBJECTDETECTION_EINVAL;
        goto exit;
    }

    if (ptrInitCfg->argSize != sizeof(DPC_ObjectDetection_InitParams))
    {
        *errCode = DPC_OBJECTDETECTION_EINVAL__INIT_CFG_ARGSIZE;
        goto exit;
    }

    dpcInitParams = (DPC_ObjectDetection_InitParams *) ptrInitCfg->arg;

    objDetObj = MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);

#ifdef DBG_DPC_OBJDET
    gObjDetObj = objDetObj;
#endif

    DebugP_log1("ObjDet DPC: objDetObj address = %d\n", (uint32_t) objDetObj);

    if(objDetObj == NULL)
    {
        *errCode = DPC_OBJECTDETECTION_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)objDetObj, 0, sizeof(ObjDetObj));

    /* Copy over the DPM configuration: */
    memcpy ((void*)&objDetObj->dpmInitCfg, (void*)ptrInitCfg, sizeof(DPM_InitCfg));

    objDetObj->dpmHandle = dpmHandle;
    objDetObj->socHandle = ptrInitCfg->socHandle;
    objDetObj->L3RamObj.cfg = dpcInitParams->L3ramCfg;
    objDetObj->CoreL2RamObj.cfg = dpcInitParams->CoreL2RamCfg;
    objDetObj->CoreL1RamObj.cfg = dpcInitParams->CoreL1RamCfg;

    for(i = 0; i < EDMA_NUM_CC; i++)
    {
        objDetObj->edmaHandle[i] = dpcInitParams->edmaHandle[i];
    }
    objDetObj->processCallBackCfg = dpcInitParams->processCallBackCfg;

    *errCode = DPC_ObjDetDSP_initDPU(objDetObj, RL_MAX_SUBFRAMES);
#ifdef TRACKERPROC_ON_DSP
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
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetection_deinit (DPM_DPCHandle handle)
{
    ObjDetObj *objDetObj = (ObjDetObj *) handle;
    int32_t retVal = 0;

    if (handle == NULL)
    {
        retVal = DPC_OBJECTDETECTION_EINVAL;
        goto exit;
    }

    retVal = DPC_ObjDetDSP_deinitDPU(objDetObj, RL_MAX_SUBFRAMES);

    MemoryP_ctrlFree(handle, sizeof(ObjDetObj));

exit:
    return (retVal);
}

/**************************************************************************
 ************************* Global Declarations ****************************
 **************************************************************************/

/** @addtogroup DPC_OBJDET__GLOBAL
 @{ */

/**
 * @brief   Global used to register Object Detection DPC in DPM
 */
DPM_ProcChainCfg gDPC_ObjectDetectionCfg =
{
    DPC_ObjectDetection_init,            /* Initialization Function:         */
    DPC_ObjectDetection_start,           /* Start Function:                  */
    DPC_ObjectDetection_execute,         /* Execute Function:                */
    DPC_ObjectDetection_ioctl,           /* Configuration Function:          */
    DPC_ObjectDetection_stop,            /* Stop Function:                   */
    DPC_ObjectDetection_deinit,          /* Deinitialization Function:       */
#ifdef OBJDET_NO_RANGE
    DPC_ObjectDetection_dataInjection,   /* Inject Data Function:            */
    NULL,                                /* Chirp Available Function:        */
#else
    NULL,                                /* Inject Data Function:            */
    DPC_ObjectDetection_chirpEvent,      /* Chirp Available Function:        */
#endif
    DPC_ObjectDetection_frameStart       /* Frame Start Function:            */
};

/* @} */

