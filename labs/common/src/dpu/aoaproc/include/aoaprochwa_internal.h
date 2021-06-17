/**
 *   @file  aoaprochwa_internal.h
 *
 *   @brief
 *      Implements internal data structure for AoA Processing with HWA.
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
#ifndef AOAPROCHWA_INTERNAL_H
#define AOAPROCHWA_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Include Files */
#include <ti/common/sys_common.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Industrial Lab Include Files */
#include <common/src/dpu/aoaproc/aoaprochwa.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Field of view - AoA Configuration
 *
 * @details
 *  The structure contains the field of view - Angle of arrival configuration used in data path
 *
 *  \ingroup DPU_AOAPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_fovAoaLocalCfg_t
{
    /*! @brief    minimum azimuth angle (expressed as sine value) */
    float        minAzimuthSineVal;

    /*! @brief    maximum azimuth angle (expressed as sine value) */
    float        maxAzimuthSineVal;

    /*! @brief    minimum elevation angle (expressed as sine value) */
    float        minElevationSineVal;

    /*! @brief    maximum elevation angle (expressed as sine value) */
    float        maxElevationSineVal;
} DPU_AoAProc_fovAoaLocalCfg;


/**
 * @brief
 *  AoAProcHWA DPU dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration used for AoAProcHWA DPU
 *
 *  \ingroup DPU_AOAPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_DynamicLocalConfig_t
{

    /*! @brief     Multi object beam forming configuration */
    DPU_AoAProc_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

    /*! @brief     Flag indicates to prepare data for azimuth heat-map */
    bool  prepareRangeAzimuthHeatMap;

    /*! @brief     Pointer to Rx channel compensation configuration */
    DPU_AoAProc_compRxChannelBiasCfg compRxChanCfg;

    /*! @brief      Field of view configuration for AoA */
    DPU_AoAProc_fovAoaLocalCfg     fovAoaLocalCfg;

    /** @brief      Extended maximum velocity configuration */
    DPU_AoAProc_ExtendedMaxVelocityCfg extMaxVelCfg;

} DPU_AoAProc_DynamicLocalConfig;

/** @addtogroup DPU_AOAPROC_INTERNAL_DEFINITION
 *  HWA memory bank indices for various input/output operations
 *
 @{ */
/** @brief 2D-FFT single bin calculation - Input HWA memory bank index  */
#define AOAHWA_2DFFT_SINGLE_BIN_INP_HWA_MEM_BANK   1
/** @brief 2D-FFT single bin calculation - Output HWA memory bank index  */
#define AOAHWA_2DFFT_SINGLE_BIN_OUT_HWA_MEM_BANK   2
/** @brief Angle FFT calculation - Input HWA memory bank index  */
#define AOAHWA_ANGLE_INP_HWA_MEM_BANK              0
/** @brief Azimuth FFT calculation - Output (log2 magnitudes) HWA memory bank index  */
#define AOAHWA_ANGLE_AZIM_ABS_OUT_HWA_MEM_BANK     2
/** @brief Elevation FFT calculation - Output (Complex values) HWA memory bank index  */
#define AOAHWA_ANGLE_ELEV_CPLX_OUT_HWA_MEM_BANK    3
/** @brief Azimuth FFT calculation - Output (Complex values) HWA memory bank index  */
#define AOAHWA_ANGLE_AZIM_CPLX_OUT_HWA_MEM_BANK    1

/** @brief Maximum number of receive virtual antennas */
#define  DPU_AOAPROCHWA_MAX_NUM_RX_VIRTUAL_ANTENNAS (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL)

#define AOAHWA_2DFFT_STAGE 0
#define AOAHWA_3DFFT_STAGE 1
#define AOAHWA_NUM_FFT_STAGES 2
#define AOAHWA_NUM_PING_PONG_BUF 2

/** @brief 3D-FFFT magnitude square output address offset in HWA memory (in bytes) */
#define  DPU_AOAPROCHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL * sizeof(uint32_t))

/**
 @}
 */



/**
 * @brief
 *  AoAProcHWA DPU internal data Object
 *
 * @details
 *  The structure is used to hold AoAProcHWA DPU internal data object
 *
 *  \ingroup DPU_AOAPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct AOAHwaObj_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;

    /*! @brief     Data path common parameters */
    DPU_AoAProcHWA_StaticConfig   params;

    /*! @brief     AoA DPU hardware resources */
    DPU_AoAProcHWA_HW_Resources res;

    /*! @brief HWA Memory Bank addresses */
    uint32_t  hwaMemBankAddr[4];

    /*! @brief     DMA channel trigger after HWA processing is done */
    uint8_t     dmaDestChannel;

    /*! @brief     Dynamic configuration */
    DPU_AoAProc_DynamicLocalConfig dynLocalCfg;

    /*! @brief     HWA Processing Done semaphore Handle */
    SemaphoreP_Handle    hwaDone_semaHandle;

    uint32_t           edmaDstIn2DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF];  //HWA M0 and M1
    uint32_t           edmaSrcOut2DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF]; //HWA M2 and M3
    uint32_t           edmaDstOut2DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF]; //azimElevLocalBuf
    uint32_t           edmaSrcIn3DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF];  //azimElevLocalHypothesesBuf
    uint32_t           edmaDstIn3DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF];  //HWA M0 and M1
    uint32_t           edmaSrcOut3DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF]; //HWA M2 and M3
    uint32_t           edmaDstOut3DFFTBuffAddr[AOAHWA_NUM_PING_PONG_BUF]; //azimuthFftOutMagBuf
    uint32_t           hwaAzimuthFftCmplxOutBuffAddr[AOAHWA_NUM_PING_PONG_BUF]; //

    /*! @brief     Local ping/pong buffers with azimuth FFT input symbols */
    uint32_t    *azimElevLocalBuf[AOAHWA_NUM_PING_PONG_BUF];

    /*! @brief     Local ping/pong buffers with azimuth FFT doppler compensated input symbols for all hypotheses */
    uint32_t    *azimElevLocalHypothesesBuf[AOAHWA_NUM_PING_PONG_BUF];

    /*! @brief     Local buffer with azimuth FFT magnitude outputs for all hypotheses */
    uint16_t    *azimuthFftOutMagBuf[AOAHWA_NUM_PING_PONG_BUF];

}AOAHwaObj;


#ifdef __cplusplus
}
#endif

#endif /* DOPPLERPROC */
