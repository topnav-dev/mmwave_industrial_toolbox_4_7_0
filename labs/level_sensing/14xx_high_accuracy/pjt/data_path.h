/**
 *   @file  data_path.h
 *
 *   @brief
 *      This is the data path processing header.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
#ifndef DATA_PATH_H
#define DATA_PATH_H

#include <ti/sysbios/knl/Semaphore.h>

#include <ti/common/sys_common.h>
#include <ti/common/mmwave_error.h>
#include <ti/drivers/adcbuf/ADCBuf.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/hwa/hwa.h>
#include <ti/demo/io_interface/detected_obj.h>
#include "mmw_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PI_ 3.14159265

//#define INTERPTESTMULTITER

#define HWAUTIL_NUM_PARAM_SETS_1D 2
#define HWAUTIL_NUM_PARAM_SETS_PEAK 1
#define HWAUTIL_NUM_PARAM_SETS_INTERP 1

#define SAMPLES_TO_ZOOM_IN_ONE_SIDE 2


#define BYTES_PER_SAMP_1D (2*sizeof(int16_t))  /*16 bit real, 16 bit imaginary => 4 bytes */

#define ONE_Q15 (1 << 15)
#define ONE_Q19 (1 << 19)
#define ONE_Q8 (1 << 8)

/* EDMA resource partitioning */
#define EDMA_SHADOW_LNK_PARAM_BASE_ID       EDMA_NUM_DMA_CHANNELS

/* 1D */
#define MMW_HWA_1D_ADCBUF_INP              (&gMmwHwaMemBuf[0])
/* 1D -ping */
#define MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING  0
#define MMW_HWA_DMA_DEST_CHANNEL_1D_PING    0
#define MMW_HWA_1D_OUT_PING                (&gMmwHwaMemBuf[2])
#define MMW_EDMA_1D_PING_CH_ID              EDMA_TPCC0_REQ_HWACC_0
#define MMW_EDMA_1D_PING_SHADOW_LINK_CH_ID  EDMA_SHADOW_LNK_PARAM_BASE_ID
#define MMW_EDMA_1D_PING_CHAIN_CH_ID        EDMA_TPCC0_REQ_FREE_0
#define MMW_EDMA_1D_PING_ONE_HOT_SHADOW_LINK_CH_ID (EDMA_SHADOW_LNK_PARAM_BASE_ID + 2)


#define MMWDEMO_EDMA_TRANSFER_COMPLETION_CODE_1D_DONE   MMW_EDMA_1D_PING_CHAIN_CH_ID

/* interpolation */
#define MMW_HWA_INTERP_OUT                  (&gMmwHwaMemBuf[3])


#define MMW_HWA_START_POS_PARAMSETS_1D       0
#define MMW_HWA_START_POS_PARAMSETS_PEAK  (MMW_HWA_START_POS_PARAMSETS_1D + HWAUTIL_NUM_PARAM_SETS_1D)
#define MMW_HWA_START_POS_PARAMSETS_INTERP  (MMW_HWA_START_POS_PARAMSETS_1D + HWAUTIL_NUM_PARAM_SETS_1D + HWAUTIL_NUM_PARAM_SETS_PEAK)

#define MMW_HWA_WINDOWRAM_1D_OFFSET         0 //In samples

/* FFT Window */
/*! Hanning window */
#define MMW_WIN_HANNING  0
/*! Blackman window */
#define MMW_WIN_BLACKMAN 1
/*! Rectangular window */
#define MMW_WIN_RECT     2

#define ONE_Q17 (1U << 17)

/*! @brief Azimuth FFT size */
#define MMW_NUM_ANGLE_BINS 64

/* number of range gates processed by the HWACC per iteration (during 2D FFT processing) */
#define MMW_NUM_RANGE_BINS_PER_TRANSFER 2

#define MMWDEMO_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

/* CFAR tuning parameters */
#define MMW_HWA_NOISE_AVG_MODE             HWA_NOISE_AVG_MODE_CFAR_CASO
#define MMW_HWA_CFAR_THRESHOLD_SCALE       0x4b0
#define MMW_HWA_CFAR_WINDOW_LEN            8
#define MMW_HWA_CFAR_GUARD_LEN             4
#define MMW_HWA_CFAR_NOISE_DIVISION_RIGHT_SHIFT 3
#define MMW_HWA_CFAR_PEAK_GROUPING         HWA_FEATURE_BIT_DISABLE


/*! @brief Flag to enable/disable two peak detection in azimuth for same range and velocity */
#define MMWDEMO_AZIMUTH_TWO_PEAK_DETECTION_ENABLE 1
/*! @brief Threshold for two peak detection in azimuth for same range and velocity,
 *         if 2nd peak heigth > first peak height * this scale then declare
 *         2nd peak as detected. */
#define MMWDEMO_AZIMUTH_TWO_PEAK_THRESHOLD_SCALE  (0.5)



/*!
 *  @brief      Data path mode
 *
 *  This enum defines if the data path chain is running standalone (such as in a unit test)
 *  or if it is running as part of a demo (integrated with ADC buffer)
 */
typedef enum
{
    /*!
      *  Data path running standalone
      */
    DATA_PATH_STANDALONE,
    /*!
      *  Data path integrated with ADC buffer
      */
    DATA_PATH_WITH_ADCBUF
} DataPath_mode;

/*!
 *  @brief    Detected object parameters filled by HWA CFAR
 *
 */
typedef volatile struct cfarDetOutput
{
    uint32_t   noise;           /*!< Noise energy in CFAR cell */
    uint32_t   rangeIdx : 12;   /*!< Range index */
    uint32_t   dopplerIdx : 20; /*!< Doppler index */
} cfarDetOutput_t;

/*!
 *  @brief Timing information
 */
typedef struct MmwDemo_timingInfo
{
    /*! @brief number of processor cycles between frames excluding
           processing time to transmit output on UART */
    uint32_t interFrameProcCycles;

     /*! @brief number of processor cycles to transmit detected object information
           on the output UART port, which is not presently back-grounded. */
    uint32_t transmitOutputCycles;

    /*! @brief Inter frame processing end time */
    uint32_t interFrameProcessingEndTime;

    /*! @brief Inter frame processing end margin in number of cycles before
     * due time to start processing first chirp of the next frame */
    uint32_t interFrameProcessingEndMargin;

    /*! @brief CPU Load during active frame period - i.e. chirping */
    uint32_t activeFrameCPULoad;

    /*! @brief CPU Load during inter frame period - i.e. after chirps
     *  are done and before next frame starts */
    uint32_t interFrameCPULoad;

} MmwDemo_timingInfo_t;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_DataPathObj_t
{
    /*! @brief   Number of receive channels */
    uint32_t numRxAntennas;

    /*! Pointer to cliCfg */
    MmwDemo_CliCfg_t *cliCfg;

    /*! @brief Pointer to cli config common to all subframes*/
    MmwDemo_CliCommonCfg_t *cliCommonCfg;

    /*! @brief   ADCBUF handle. */
    ADCBuf_Handle adcbufHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle edmaHandle;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t  EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

    /*! @brief Semaphore handle for 1D EDMA completion. */
    Semaphore_Handle EDMA_1Ddone_semHandle;

    /*! @brief Handle to hardware accelerator driver. */
    HWA_Handle  hwaHandle;

    /*! @brief Semaphore handle for Hardware accelerator completion. */
    Semaphore_Handle HWA_done_semHandle;

    /*! @brief Hardware accelerator Completion Isr count for debug purposes. */
    uint32_t hwaDoneIsrCounter;

    /*! @brief Frame counter incremented in frame start interrupt handler*/
    uint32_t frameStartIntCounter;

    /*! @brief Semaphore handle for Frame start indication. */
    Semaphore_Handle frameStart_semHandle;

    /*! @brief Range processing HWA parameters. */
    HWA_ParamConfig rangeHwaParamCfg[HWAUTIL_NUM_PARAM_SETS_1D];

    /*! @brief Range processing common HWA parameters. */
    HWA_CommonConfig rangeHhwaCommonConfig;

    /*! @brief Peak search processing HWA parameters. */
    HWA_ParamConfig peakHwaParamCfg[HWAUTIL_NUM_PARAM_SETS_PEAK];

    /*! @brief Peak search processing common HWA parameters. */
    HWA_CommonConfig peakHwaCommonConfig;

    /*! @brief Interpolation processing HWA parameters. */
    HWA_ParamConfig interpHwaParamCfg[HWAUTIL_NUM_PARAM_SETS_INTERP];

    /*! @brief Interpolation processing common HWA parameters. */
    HWA_CommonConfig interpHhwaCommonConfig;

    /*! @brief valid Profile index */
    uint32_t validProfileIdx;

    /*! @brief number of transmit antennas */
    uint32_t numTxAntennas;

    /*! @brief max beat frequency */
    float maxBeatFreq;

    /*! @brief chirp BW */
    float chirpBandwidth;

    /*! @brief chirp ramp time */
    float chirpRampTime;

    /*! @brief number of ADC samples */
    uint32_t numAdcSamples;

    /*! @brief number of range bins */
    uint32_t numRangeBins;

    /*! @brief log2 of number of range bins */
    uint32_t log2RangeBins;

    /*! @brief zoom in fft size */
    uint32_t zoomInFFTSize;

    /*! @brief log2 of zoom in fft size */
    uint32_t log2ZoomInFFTSize;

    /*! @brief number of chirps per frame */
    uint32_t numChirpsPerFrame;

    /*! @brief range resolution in meters */
    float rangeResolution;

    /*! @brief Timing information */
    MmwDemo_timingInfo_t timingInfo;

    /*! @brief DC range signature calibration counter */
    uint32_t dcRangeSigCalibCntr;

    /*! @brief DC range signature calibration forced disable counter */
    uint32_t dcRangeForcedDisableCntr;

    /*! @brief Pointer to DC range signature compensation buffer */
    int32_t *dcRangeSigMean;

	/*! @brief Data path mode */
	DataPath_mode    dataPathMode;

	/* @brief output stats from HWA */
	HWA_Stats  rangeProcStats;

	/*! @brief Detected objects azimuth index for debugging */
	float rangeEst;

    /*! @brief Fine peak index */
	uint32_t finePeakIndex;

    /*! @brief interpolated index */
    float interpIndex;

	/*! @brief first pass of HWA done counter */
    uint32_t firstPassHWADoneCnt;

    /*! @brief second pass of HWA done counter */
    uint32_t secondPassHWADoneCnt;

    /*! @brief  Used for checking that inter frame processing finshed on time */
    int32_t interFrameProcToken;

    /*! @brief log2 of number of averaged chirps */
    uint32_t log2NumAvgChirps;

} MmwDemo_DataPathObj;

/*! simple memory pool structure for flexible partitioning of L3 RAM */
typedef struct {
    /*! base address of memory */
    uint8_t *base;

    /*! size of memory in bytes */
    uint32_t size;

    /*! index into the pool to track where is next free */
    uint32_t indx;
} MmwDemoMemPool_t;


/*! Partiion HWA memory into 4 equal parts - M0,M1,M2,M3. */
#define MMW_HWA_NUM_MEM_BUFS 4
typedef struct {
    uint8_t buf[SOC_XWR14XX_MSS_HWA_MEM_SIZE/MMW_HWA_NUM_MEM_BUFS];
} mmwHwaBuf_t;

void MmwDemo_hwaInit(MmwDemo_DataPathObj *obj);
void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj);
void MmwDemo_hwaOpen(MmwDemo_DataPathObj *obj, SOC_Handle socHandle);
void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj);
void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo);
void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo);

void MmwDemo_memPoolReset(MmwDemoMemPool_t *pool);
uint8_t *MmwDemo_memPoolAlloc(MmwDemoMemPool_t *pool, uint32_t size);
void MmwDemo_dataPathObjInit(MmwDemo_DataPathObj *obj,
                             MmwDemo_CliCfg_t *cliCfg,
                             MmwDemo_CliCommonCfg_t *cliCommonCfg);
void MmwDemo_dataPathCfgBuffers(MmwDemo_DataPathObj *obj, MmwDemoMemPool_t *pool);

void MmwDemo_dataPathConfigCommon(MmwDemo_DataPathObj *obj);
void MmwDemo_dcRangeSignatureCompensation(MmwDemo_DataPathObj *obj);
void MmwDemo_configInterp_HWA(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathTriggerInterp(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathWaitInterp(MmwDemo_DataPathObj *obj);
void MmwDemo_processInterpolation(MmwDemo_DataPathObj *obj);

void MmwDemo_configPeakSearch_HWA(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathTriggerPeakSearch(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathWaitPeakSearch(MmwDemo_DataPathObj *obj);
void MmwDemo_peakSearch(MmwDemo_DataPathObj *obj);
void MmwDemo_rangeLimit(MmwDemo_DataPathObj *obj);

void MmwDemo_config1D_HWA(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathTrigger1D(MmwDemo_DataPathObj *obj);
void MmwDemo_dataPathWait1D(MmwDemo_DataPathObj *obj);

uint32_t log2Approx(uint32_t x);

uint32_t MmwDemo_pow2roundup (uint32_t x);
#ifdef __cplusplus
}
#endif

#endif /* DATA_PATH_H */

