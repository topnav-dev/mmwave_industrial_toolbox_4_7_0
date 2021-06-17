/**
 *   @file  aoaprocdsp_internal.h
 *
 *   @brief
 *      Internal header for DSP AoA DPU.
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
#ifndef AOAPROCDSP_INTERNAL_H
#define AOAPROCDSP_INTERNAL_H

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
#include <common/src/dpu/aoaproc/aoaproc_common.h>
#include <common/src/dpu/aoaproc/aoaprocdsp.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! Ping index used for EDMA-CPU processing parallelism */
#define DPU_AOAPROCDSP_PING_IDX 0

/*! Pong index used for EDMA-CPU processing parallelism */
#define DPU_AOAPROCDSP_PONG_IDX 1

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
 *  AoAProcDSP DPU dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration used for AoAProcDSP DPU
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

    /*! @brief Static clutter removal configuration. Valid only for DSP version of the AoA DPU.*/
    DPU_AoAProc_StaticClutterRemovalCfg  staticClutterCfg;

    /*! @brief      Extended maximum velocity configuration */
    DPU_AoAProc_ExtendedMaxVelocityCfg extMaxVelCfg;

} DPU_AoAProc_DynamicLocalConfig;


/**
 @}
 */

/**
 * @brief
 *  AoAProcDSP DPU internal data Object
 *
 * @details
 *  The structure is used to hold AoAProcDSP DPU internal data object
 *
 *  \ingroup DPU_AOAPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct AOADspObj_t
{
    /*! @brief     Data path common parameters */
    DPU_AoAProcDSP_StaticConfig   params;

    /*! @brief     AoA DPU hardware resources */
    DPU_AoAProcDSP_HW_Resources res;

    /*! @brief     Dynamic configuration */
    DPU_AoAProc_DynamicLocalConfig dynLocalCfg;

}AOADspObj;

#ifdef AOA_TYPE_2D
/**
 * @brief
 *  AoA2DProcDSP internal data storage
 *
 * @details
 *  The structure is used to hold AoA2DProcDSP DPU data storage
 *
 *  \ingroup DPU_AOA2DPROC_INTERNAL_DATA_STORAGE
 *
 */
typedef struct DOA_2D
{
	cmplx32ReIm_t DOA_2D_storage[DPU_AOAPROCDSP_NUM_ANGLE_BINS][DPU_AOAPROCDSP_NUM_ANGLE_BINS];
} DOA_2D_t;
#endif



#ifdef __cplusplus
}
#endif

#endif 
