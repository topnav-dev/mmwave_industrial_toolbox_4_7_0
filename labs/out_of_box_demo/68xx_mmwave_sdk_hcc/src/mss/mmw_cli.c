/*
 *   @file  mmw_cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO HCC Implementation
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 Texas Instruments, Inc.
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

/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <ti/demo/xwr68xx/mmw/include/mmw_config.h>
#include <ti/demo/xwr68xx/mmw/mss/mmw_mss.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>

/* Profile Header Include Files */
#include "mss/hc_config_defs.h"

/**************************************************************************
 *************************** Local function prototype****************************
 **************************************************************************/

/* HCC Extended Command Functions */
void mmwDemo_sensorConfig_task(UArg arg0, UArg arg1);
static int32_t MmwDemo_HCCGuiMonSel(uint8_t numProf);
static int32_t MmwDemo_HCCCfarCfg(uint8_t numProf);
static int32_t MmwDemo_HCCCfarFovCfg(uint8_t numProf);
static int32_t MmwDemo_HCCAoAFovCfg (void);
static int32_t MmwDemo_HCCExtendedMaxVelocity(void);
static int32_t MmwDemo_HCCMultiObjBeamForming (uint8_t numProf);
static int32_t MmwDemo_HCCCalibDcRangeSig (uint8_t numProf);
static int32_t MmwDemo_HCCClutterRemoval (void);
static int32_t MmwDemo_HCCADCBufCfg (void);
static int32_t MmwDemo_HCCCompRangeBiasAndRxChanPhaseCfg(void);
static int32_t MmwDemo_HCCMeasureRangeBiasAndRxChanPhaseCfg(void);
static int32_t MmwDemo_HCCBpmCfg(void);
static int32_t MmwDemo_HCCChirpQualityRxSatMonCfg(uint8_t numProf);
static int32_t MmwDemo_HCCChirpQualitySigImgMonCfg (uint8_t numProf);
static int32_t MmwDemo_HCCAnalogMonitorCfg (void);
static int32_t MmwDemo_HCCLvdsStreamCfg(void);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern MmwDemo_MSS_MCB    gMmwMssMCB;

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
#define MMWDEMO_DATAUART_MAX_BAUDRATE_SUPPORTED 3125000

/**************************************************************************
 *************************** HCC  Function Definitions **************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for gui monitoring configuration
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCGuiMonSel(uint8_t numProf)
{
    MmwDemo_GuiMonSel   guiMonSel[numProf];
    int8_t              subFrameIdx[numProf];
    uint8_t i;

    /* Initialize the guiMonSel configuration: */
    for (i = 0; i < numProf; i++)
        memset((void *)&guiMonSel[i], 0, sizeof(MmwDemo_GuiMonSel));

    /* Initialize Subframe Index */
    for (i = 0; i < numProf; i++)
        memset((void *)&subFrameIdx[i], 0, sizeof(int8_t));

    /* Populate configuration: */
    subFrameIdx[0]                         = GUI_HCC_0_SUBFRAME_IDX;
    guiMonSel[0].detectedObjects           = GUI_HCC_0_DETECTED_OBJECTS;
    guiMonSel[0].logMagRange               = GUI_HCC_0_LOG_MAGNITUDE_RANGE;
    guiMonSel[0].noiseProfile              = GUI_HCC_0_NOISE_PROFILE;
    guiMonSel[0].rangeAzimuthHeatMap       = GUI_HCC_0_RANGE_AZIMUTH_MAP;
    guiMonSel[0].rangeDopplerHeatMap       = GUI_HCC_0_RANGE_DOPPLER_MAP;
    guiMonSel[0].statsInfo                 = GUI_HCC_0_STATS_INFO;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 2){
        subFrameIdx[1]                         = GUI_HCC_1_SUBFRAME_IDX;
        guiMonSel[1].detectedObjects           = GUI_HCC_1_DETECTED_OBJECTS;
        guiMonSel[1].logMagRange               = GUI_HCC_1_LOG_MAGNITUDE_RANGE;
        guiMonSel[1].noiseProfile              = GUI_HCC_1_NOISE_PROFILE;
        guiMonSel[1].rangeAzimuthHeatMap       = GUI_HCC_1_RANGE_AZIMUTH_MAP;
        guiMonSel[1].rangeDopplerHeatMap       = GUI_HCC_1_RANGE_DOPPLER_MAP;
        guiMonSel[1].statsInfo                 = GUI_HCC_1_STATS_INFO;
    }
    if (numProf >= 3){
        subFrameIdx[2]                         = GUI_HCC_2_SUBFRAME_IDX;
        guiMonSel[2].detectedObjects           = GUI_HCC_2_DETECTED_OBJECTS;
        guiMonSel[2].logMagRange               = GUI_HCC_2_LOG_MAGNITUDE_RANGE;
        guiMonSel[2].noiseProfile              = GUI_HCC_2_NOISE_PROFILE;
        guiMonSel[2].rangeAzimuthHeatMap       = GUI_HCC_2_RANGE_AZIMUTH_MAP;
        guiMonSel[2].rangeDopplerHeatMap       = GUI_HCC_2_RANGE_DOPPLER_MAP;
        guiMonSel[2].statsInfo                 = GUI_HCC_2_STATS_INFO;
    }
    if (numProf == 4){
        subFrameIdx[3]                         = GUI_HCC_3_SUBFRAME_IDX;
        guiMonSel[3].detectedObjects           = GUI_HCC_3_DETECTED_OBJECTS;
        guiMonSel[3].logMagRange               = GUI_HCC_3_LOG_MAGNITUDE_RANGE;
        guiMonSel[3].noiseProfile              = GUI_HCC_3_NOISE_PROFILE;
        guiMonSel[3].rangeAzimuthHeatMap       = GUI_HCC_3_RANGE_AZIMUTH_MAP;
        guiMonSel[3].rangeDopplerHeatMap       = GUI_HCC_3_RANGE_DOPPLER_MAP;
        guiMonSel[3].statsInfo                 = GUI_HCC_3_STATS_INFO;
    }
#endif

    for (i = 0; i < numProf; i++)
        MmwDemo_CfgUpdate((void *)&guiMonSel[i], MMWDEMO_GUIMONSEL_OFFSET, sizeof(MmwDemo_GuiMonSel), subFrameIdx[i]);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for CFAR configuration
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCCfarCfg(uint8_t numProf)
{
    DPU_CFARCAProc_CfarCfg  cfarCfg[numProf];
    uint32_t                procDirection[numProf];
    int8_t                  subFrameIdx[numProf];
    uint8_t i;

    /* Initialize configuration: */
    for (i = 0; i < numProf; i++)
        memset ((void *)&cfarCfg[i], 0, sizeof(cfarCfg[i]));

    /* Initialize Processing Direction */
    for (i = 0; i < numProf; i++)
        memset ((void *)&procDirection[i], 0, sizeof(uint32_t));

    /* Initialize Subframe Index */
    for (i = 0; i < numProf; i++)
        memset((void *)&subFrameIdx[i], 0, sizeof(int8_t));

    /* Populate configuration: */
    subFrameIdx[0]               = CFAR_HCC_0_SUBFRAME_IDX;
    procDirection[0]             = CFAR_HCC_0_PROC_DIRECTION;
    cfarCfg[0].averageMode       = CFAR_HCC_0_MODE;
    cfarCfg[0].winLen            = CFAR_HCC_0_NOISE_WIN;
    cfarCfg[0].guardLen          = CFAR_HCC_0_GUARD_LEN;
    cfarCfg[0].noiseDivShift     = CFAR_HCC_0_DIV_SHIFT;
    cfarCfg[0].cyclicMode        = CFAR_HCC_0_CYCLIC_MODE;
    cfarCfg[0].thresholdScale    = (uint16_t) (CFAR_HCC_0_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
    cfarCfg[0].peakGroupingEn    = CFAR_HCC_0_PEAK_GROUPING;

    subFrameIdx[1]               = CFAR_HCC_1_SUBFRAME_IDX;
    procDirection[1]             = CFAR_HCC_1_PROC_DIRECTION;
    cfarCfg[1].averageMode       = CFAR_HCC_1_MODE;
    cfarCfg[1].winLen            = CFAR_HCC_1_NOISE_WIN;
    cfarCfg[1].guardLen          = CFAR_HCC_1_GUARD_LEN;
    cfarCfg[1].noiseDivShift     = CFAR_HCC_1_DIV_SHIFT;
    cfarCfg[1].cyclicMode        = CFAR_HCC_1_CYCLIC_MODE;
    cfarCfg[1].thresholdScale    = (uint16_t) (CFAR_HCC_1_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
    cfarCfg[1].peakGroupingEn    = CFAR_HCC_1_PEAK_GROUPING;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 3){
        subFrameIdx[2]               = CFAR_HCC_2_SUBFRAME_IDX;
        procDirection[2]             = CFAR_HCC_2_PROC_DIRECTION;
        cfarCfg[2].averageMode       = CFAR_HCC_2_MODE;
        cfarCfg[2].winLen            = CFAR_HCC_2_NOISE_WIN;
        cfarCfg[2].guardLen          = CFAR_HCC_2_GUARD_LEN;
        cfarCfg[2].noiseDivShift     = CFAR_HCC_2_DIV_SHIFT;
        cfarCfg[2].cyclicMode        = CFAR_HCC_2_CYCLIC_MODE;
        cfarCfg[2].thresholdScale    = (uint16_t) (CFAR_HCC_2_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[2].peakGroupingEn    = CFAR_HCC_2_PEAK_GROUPING;

        subFrameIdx[3]               = CFAR_HCC_3_SUBFRAME_IDX;
        procDirection[3]             = CFAR_HCC_3_PROC_DIRECTION;
        cfarCfg[3].averageMode       = CFAR_HCC_3_MODE;
        cfarCfg[3].winLen            = CFAR_HCC_3_NOISE_WIN;
        cfarCfg[3].guardLen          = CFAR_HCC_3_GUARD_LEN;
        cfarCfg[3].noiseDivShift     = CFAR_HCC_3_DIV_SHIFT;
        cfarCfg[3].cyclicMode        = CFAR_HCC_3_CYCLIC_MODE;
        cfarCfg[3].thresholdScale    = (uint16_t) (CFAR_HCC_3_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[3].peakGroupingEn    = CFAR_HCC_3_PEAK_GROUPING;
    }
    if (numProf >= 5){
        subFrameIdx[4]               = CFAR_HCC_4_SUBFRAME_IDX;
        procDirection[4]             = CFAR_HCC_4_PROC_DIRECTION;
        cfarCfg[4].averageMode       = CFAR_HCC_4_MODE;
        cfarCfg[4].winLen            = CFAR_HCC_4_NOISE_WIN;
        cfarCfg[4].guardLen          = CFAR_HCC_4_GUARD_LEN;
        cfarCfg[4].noiseDivShift     = CFAR_HCC_4_DIV_SHIFT;
        cfarCfg[4].cyclicMode        = CFAR_HCC_4_CYCLIC_MODE;
        cfarCfg[4].thresholdScale    = (uint16_t) (CFAR_HCC_4_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[4].peakGroupingEn    = CFAR_HCC_4_PEAK_GROUPING;

        subFrameIdx[5]               = CFAR_HCC_5_SUBFRAME_IDX;
        procDirection[5]             = CFAR_HCC_5_PROC_DIRECTION;
        cfarCfg[5].averageMode       = CFAR_HCC_5_MODE;
        cfarCfg[5].winLen            = CFAR_HCC_5_NOISE_WIN;
        cfarCfg[5].guardLen          = CFAR_HCC_5_GUARD_LEN;
        cfarCfg[5].noiseDivShift     = CFAR_HCC_5_DIV_SHIFT;
        cfarCfg[5].cyclicMode        = CFAR_HCC_5_CYCLIC_MODE;
        cfarCfg[5].thresholdScale    = (uint16_t) (CFAR_HCC_5_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[5].peakGroupingEn    = CFAR_HCC_5_PEAK_GROUPING;
    }
    if (numProf >= 7){
        subFrameIdx[6]               = CFAR_HCC_6_SUBFRAME_IDX;
        procDirection[6]             = CFAR_HCC_6_PROC_DIRECTION;
        cfarCfg[6].averageMode       = CFAR_HCC_6_MODE;
        cfarCfg[6].winLen            = CFAR_HCC_6_NOISE_WIN;
        cfarCfg[6].guardLen          = CFAR_HCC_6_GUARD_LEN;
        cfarCfg[6].noiseDivShift     = CFAR_HCC_6_DIV_SHIFT;
        cfarCfg[6].cyclicMode        = CFAR_HCC_6_CYCLIC_MODE;
        cfarCfg[6].thresholdScale    = (uint16_t) (CFAR_HCC_6_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[6].peakGroupingEn    = CFAR_HCC_6_PEAK_GROUPING;

        subFrameIdx[7]               = CFAR_HCC_7_SUBFRAME_IDX;
        procDirection[7]             = CFAR_HCC_7_PROC_DIRECTION;
        cfarCfg[7].averageMode       = CFAR_HCC_7_MODE;
        cfarCfg[7].winLen            = CFAR_HCC_7_NOISE_WIN;
        cfarCfg[7].guardLen          = CFAR_HCC_7_GUARD_LEN;
        cfarCfg[7].noiseDivShift     = CFAR_HCC_7_DIV_SHIFT;
        cfarCfg[7].cyclicMode        = CFAR_HCC_7_CYCLIC_MODE;
        cfarCfg[7].thresholdScale    = (uint16_t) (CFAR_HCC_7_THRESHOLD_SCALE * MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
        cfarCfg[7].peakGroupingEn    = CFAR_HCC_7_PEAK_GROUPING;
    }
#endif

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++){
        if (procDirection[i] == 0){
            MmwDemo_CfgUpdate((void *)&cfarCfg[i], MMWDEMO_CFARCFGRANGE_OFFSET, sizeof(cfarCfg[i]), subFrameIdx[i]);
        }
        else{
            MmwDemo_CfgUpdate((void *)&cfarCfg[i], MMWDEMO_CFARCFGDOPPLER_OFFSET, sizeof(cfarCfg[i]), subFrameIdx[i]);
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for CFAR FOV (Field Of View) configuration
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCCfarFovCfg(uint8_t numProf)
{
    DPU_CFARCAProc_FovCfg   fovCfg[numProf];
    uint32_t                procDirection[numProf];
    int8_t                  subFrameIdx[numProf];
    uint8_t i;

    /* Initialize configuration: */
    for (i = 0; i < numProf; i++)
        memset ((void *)&fovCfg[i], 0, sizeof(fovCfg[i]));

    /* Initialize Processing Direction */
    for (i = 0; i < numProf; i++)
        memset ((void *)&procDirection[i], 0, sizeof(uint32_t));

    /* Initialize Subframe Index */
    for (i = 0; i < numProf; i++)
        memset((void *)&subFrameIdx[i], 0, sizeof(int8_t));

    /* Populate configuration: */
    subFrameIdx[0]               = CFAR_FOV_HCC_0_SUBFRAME_IDX;
    procDirection[0]             = CFAR_FOV_HCC_0_PROC_DIRECTION;
    fovCfg[0].min                = CFAR_FOV_HCC_0_MINIMUM;
    fovCfg[0].max                = CFAR_FOV_HCC_0_MAXIMUM;

    subFrameIdx[1]               = CFAR_FOV_HCC_1_SUBFRAME_IDX;
    procDirection[1]             = CFAR_FOV_HCC_1_PROC_DIRECTION;
    fovCfg[1].min                = CFAR_FOV_HCC_1_MINIMUM;
    fovCfg[1].max                = CFAR_FOV_HCC_1_MAXIMUM;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 3){
        subFrameIdx[2]               = CFAR_FOV_HCC_2_SUBFRAME_IDX;
        procDirection[2]             = CFAR_FOV_HCC_2_PROC_DIRECTION;
        fovCfg[2].min                = CFAR_FOV_HCC_2_MINIMUM;
        fovCfg[2].max                = CFAR_FOV_HCC_2_MAXIMUM;

        subFrameIdx[3]               = CFAR_FOV_HCC_3_SUBFRAME_IDX;
        procDirection[3]             = CFAR_FOV_HCC_3_PROC_DIRECTION;
        fovCfg[3].min                = CFAR_FOV_HCC_3_MINIMUM;
        fovCfg[3].max                = CFAR_FOV_HCC_3_MAXIMUM;
    }
    if (numProf >= 5){
        subFrameIdx[4]               = CFAR_FOV_HCC_4_SUBFRAME_IDX;
        procDirection[4]             = CFAR_FOV_HCC_4_PROC_DIRECTION;
        fovCfg[4].min                = CFAR_FOV_HCC_4_MINIMUM;
        fovCfg[4].max                = CFAR_FOV_HCC_4_MAXIMUM;

        subFrameIdx[5]               = CFAR_FOV_HCC_5_SUBFRAME_IDX;
        procDirection[5]             = CFAR_FOV_HCC_5_PROC_DIRECTION;
        fovCfg[5].min                = CFAR_FOV_HCC_5_MINIMUM;
        fovCfg[5].max                = CFAR_FOV_HCC_5_MAXIMUM;
    }
    if (numProf >= 7){
        subFrameIdx[6]               = CFAR_FOV_HCC_6_SUBFRAME_IDX;
        procDirection[6]             = CFAR_FOV_HCC_6_PROC_DIRECTION;
        fovCfg[6].min                = CFAR_FOV_HCC_6_MINIMUM;
        fovCfg[6].max                = CFAR_FOV_HCC_6_MAXIMUM;

        subFrameIdx[7]               = CFAR_FOV_HCC_7_SUBFRAME_IDX;
        procDirection[7]             = CFAR_FOV_HCC_7_PROC_DIRECTION;
        fovCfg[7].min                = CFAR_FOV_HCC_7_MINIMUM;
        fovCfg[7].max                = CFAR_FOV_HCC_7_MAXIMUM;
    }
#endif

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++){
        if (procDirection[i] == 0){
            MmwDemo_CfgUpdate((void *)&fovCfg[i], MMWDEMO_FOVRANGE_OFFSET, sizeof(fovCfg[i]), subFrameIdx[i]);
        }
        else{
            MmwDemo_CfgUpdate((void *)&fovCfg[i], MMWDEMO_FOVDOPPLER_OFFSET, sizeof(fovCfg[i]), subFrameIdx[i]);
        }
    }
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for AoA FOV (Field Of View) configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCAoAFovCfg (void)
{
    DPU_AoAProc_FovAoaCfg   fovCfg;

    /* Initialize configuration: */
    memset ((void *)&fovCfg, 0, sizeof(fovCfg));

    /* Populate configuration: */
    fovCfg.minAzimuthDeg      = AOAFOV_HCC_MIN_AZIMUTH_DEG;
    fovCfg.maxAzimuthDeg      = AOAFOV_HCC_MAX_AZIMUTH_DEG;
    fovCfg.minElevationDeg    = AOAFOV_HCC_MIN_ELEVATION_DEG;
    fovCfg.maxElevationDeg    = AOAFOV_HCC_MAX_ELEVATION_DEG;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&fovCfg, MMWDEMO_FOVAOA_OFFSET, sizeof(fovCfg), AOAFOV_HCC_SUBFRAME_IDX);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for extended maximum velocity configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCExtendedMaxVelocity (void)
{
    DPU_AoAProc_ExtendedMaxVelocityCfg      cfg;

    /* Initialize configuration: */
    memset((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled      = EXTENDMAXVELOCITY_HCC_ENABLED;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_EXTMAXVEL_OFFSET, sizeof(cfg), EXTENDMAXVELOCITY_HCC_SUBFRAME_IDX);

    return 0;

}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for multi object beam forming configuration
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCMultiObjBeamForming (uint8_t numProf)
{
    DPU_AoAProc_MultiObjBeamFormingCfg  cfg[numProf];
    int8_t                              subFrameIdx[numProf];
    uint8_t                             i;

    /* Initialize configuration: */
    for (i = 0; i < numProf; i++)
        memset ((void *)&cfg[i], 0, sizeof(cfg[i]));

    /* Initialize Subframe Index */
    for (i = 0; i < numProf; i++)
        memset((void *)&subFrameIdx[i], 0, sizeof(int8_t));

    /* Populate configuration: */
    subFrameIdx[0]                     = MULTI_OBJ_BEAM_HCC_0_SUBFRAME_IDX;
    cfg[0].enabled                     = MULTI_OBJ_BEAM_HCC_0_ENABLED;
    cfg[0].multiPeakThrsScal           = MULTI_OBJ_BEAM_HCC_0_THRESHOLD;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 2){
        subFrameIdx[1]                     = MULTI_OBJ_BEAM_HCC_1_SUBFRAME_IDX;
        cfg[1].enabled                     = MULTI_OBJ_BEAM_HCC_1_ENABLED;
        cfg[1].multiPeakThrsScal           = MULTI_OBJ_BEAM_HCC_1_THRESHOLD;
    }
    if (numProf >= 3){
        subFrameIdx[2]                     = MULTI_OBJ_BEAM_HCC_2_SUBFRAME_IDX;
        cfg[2].enabled                     = MULTI_OBJ_BEAM_HCC_2_ENABLED;
        cfg[2].multiPeakThrsScal           = MULTI_OBJ_BEAM_HCC_2_THRESHOLD;
    }
    if (numProf == 4){
        subFrameIdx[3]                     = MULTI_OBJ_BEAM_HCC_3_SUBFRAME_IDX;
        cfg[3].enabled                     = MULTI_OBJ_BEAM_HCC_3_ENABLED;
        cfg[3].multiPeakThrsScal           = MULTI_OBJ_BEAM_HCC_3_THRESHOLD;
    }
#endif

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++)
        MmwDemo_CfgUpdate((void *)&cfg[i], MMWDEMO_MULTIOBJBEAMFORMING_OFFSET, sizeof(cfg[i]), subFrameIdx[i]);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for DC range calibration
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCCalibDcRangeSig (uint8_t numProf)
{
    DPU_RangeProc_CalibDcRangeSigCfg    cfg[numProf];
    uint32_t                            log2NumAvgChirps[numProf];
    int8_t                              subFrameIdx[numProf];
    uint8_t i;

    /* Initialize configuration for DC range signature calibration */
    for (i = 0; i < numProf; i++)
        memset ((void *)&cfg[i], 0, sizeof(cfg[i]));

    /* Initialize log2(Number Average Chirps) */
    for (i = 0; i < numProf; i++)
        memset((void *)&log2NumAvgChirps[i], 0, sizeof(uint32_t));

    /* Initialize Subframe Index */
    for (i = 0; i < numProf; i++)
        memset((void *)&subFrameIdx[i], 0, sizeof(int8_t));

    /* Populate configuration: */
    subFrameIdx[0]          = CALIB_DC_RANGE_HCC_0_SUBFRAME_IDX;
    cfg[0].enabled          = CALIB_DC_RANGE_HCC_0_ENABLED;
    cfg[0].negativeBinIdx   = CALIB_DC_RANGE_HCC_0_NEG_BIN_IDX;
    cfg[0].positiveBinIdx   = CALIB_DC_RANGE_HCC_0_POS_BIN_IDX;
    cfg[0].numAvgChirps     = CALIB_DC_RANGE_HCC_0_NUM_AVG;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 2){
        subFrameIdx[1]          = CALIB_DC_RANGE_HCC_1_SUBFRAME_IDX;
        cfg[1].enabled          = CALIB_DC_RANGE_HCC_1_ENABLED;
        cfg[1].negativeBinIdx   = CALIB_DC_RANGE_HCC_1_NEG_BIN_IDX;
        cfg[1].positiveBinIdx   = CALIB_DC_RANGE_HCC_1_POS_BIN_IDX;
        cfg[1].numAvgChirps     = CALIB_DC_RANGE_HCC_1_NUM_AVG;
    }
    if (numProf >= 3){
        subFrameIdx[2]          = CALIB_DC_RANGE_HCC_2_SUBFRAME_IDX;
        cfg[2].enabled          = CALIB_DC_RANGE_HCC_2_ENABLED;
        cfg[2].negativeBinIdx   = CALIB_DC_RANGE_HCC_2_NEG_BIN_IDX;
        cfg[2].positiveBinIdx   = CALIB_DC_RANGE_HCC_2_POS_BIN_IDX;
        cfg[2].numAvgChirps     = CALIB_DC_RANGE_HCC_2_NUM_AVG;
    }
    if (numProf == 4){
        subFrameIdx[3]          = CALIB_DC_RANGE_HCC_3_SUBFRAME_IDX;
        cfg[3].enabled          = CALIB_DC_RANGE_HCC_3_ENABLED;
        cfg[3].negativeBinIdx   = CALIB_DC_RANGE_HCC_3_NEG_BIN_IDX;
        cfg[3].positiveBinIdx   = CALIB_DC_RANGE_HCC_3_POS_BIN_IDX;
        cfg[3].numAvgChirps     = CALIB_DC_RANGE_HCC_3_NUM_AVG;
    }
#endif

    for (i = 0; i < numProf; i++){
        if (cfg[i].negativeBinIdx > 0)
        {
            printf("Error: Invalid negative bin index\n");
            return -1;
        }
        if ((cfg[i].positiveBinIdx - cfg[i].negativeBinIdx + 1) > DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE)
        {
            printf("Error: Number of bins exceeds the limit\n");
            return -1;
        }
        log2NumAvgChirps[i] = (uint32_t) mathUtils_ceilLog2(cfg[i].numAvgChirps);
        if (cfg[i].numAvgChirps != (1U << log2NumAvgChirps[i]))
        {
            printf("Error: Number of averaged chirps is not power of two\n");
            return -1;
        }
    }

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++)
        MmwDemo_CfgUpdate((void *)&cfg[i], MMWDEMO_CALIBDCRANGESIG_OFFSET, sizeof(cfg[i]), subFrameIdx[i]);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      Clutter removal Configuration
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCClutterRemoval (void)
{
    DPC_ObjectDetection_StaticClutterRemovalCfg_Base    cfg;

    /* Initialize configuration for clutter removal */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = CLUTTER_HCC_ENABLED;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_STATICCLUTTERREMOFVAL_OFFSET, sizeof(cfg), CLUTTER_HCC_SUBFRAME_IDX);

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for data logger set command
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCADCBufCfg (void)
{
    MmwDemo_ADCBufCfg   adcBufCfg;

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(adcBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = ADCBUF_HCC_OUTPUT_FMT;
    adcBufCfg.iqSwapSel       = ADCBUF_HCC_SAMPLE_SWAP;
    adcBufCfg.chInterleave    = ADCBUF_HCC_CHAN_INTERLEAVE;
    adcBufCfg.chirpThreshold  = ADCBUF_HCC_CHIRP_THRESHOLD;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&adcBufCfg, MMWDEMO_ADCBUFCFG_OFFSET, sizeof(adcBufCfg), ADCBUF_HCC_SUBFRAME_IDX);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for compensation of range bias and channel phase offsets
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCCompRangeBiasAndRxChanPhaseCfg(void)
{
    DPU_AoAProc_compRxChannelBiasCfg   cfg;
    int32_t Re, Im;
    int32_t argInd;
    int32_t i;

    char *compRange[] = {"compRangeBiasAndRxChanPhase", "0.0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0", "1", "0"};

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.rangeBias          = COMPRANGEBIASANDRXCHANPHASE;

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        Re = (int32_t) (atof (compRange[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Re);
        cfg.rxChPhaseComp[i].real = (int16_t) Re;

        Im = (int32_t) (atof (compRange[argInd++]) * 32768.);
        MATHUTILS_SATURATE16(Im);
        cfg.rxChPhaseComp[i].imag = (int16_t) Im;

    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.compRxChanCfg, &cfg, sizeof(cfg));

    gMmwMssMCB.objDetCommonCfg.isCompRxChannelBiasCfgPending = 1;

    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for measurement configuration of range bias
 *      and channel phase offsets
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCMeasureRangeBiasAndRxChanPhaseCfg(void)
{
    DPC_ObjectDetection_MeasureRxChannelBiasCfg   cfg;

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(cfg));

    /* Populate configuration: */
    cfg.enabled          = MEASURERANGEBIAS_HCC_ENABLED;
    cfg.targetDistance   = MEASURERANGEBIAS_HCC_TARGET_DIST;
    cfg.searchWinSize    = MEASURERANGEBIAS_HCC_SEARCH_WIN;

    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg, &cfg, sizeof(cfg));

    gMmwMssMCB.objDetCommonCfg.isMeasureRxChannelBiasCfgPending = 1;

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for BPM configuration supported by the mmw Demo
 *      Note that there is a generic BPM configuration command supported by
 *      utils/cli and mmwave. The generic BPM command is not supported by the
 *      demo as the mmw demo assumes a specific BPM pattern for the TX antennas.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCBpmCfg(void)
{
    MmwDemo_BpmCfg      bpmCfg;

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&bpmCfg, 0, sizeof(MmwDemo_BpmCfg));

    /* Populate configuration: */
    bpmCfg.isEnabled = BPM_HCC_ENABLED;
    bpmCfg.chirp0Idx = BPM_HCC_CHIRP_0_IDX;
    bpmCfg.chirp1Idx = BPM_HCC_CHIRP_1_IDX;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&bpmCfg, MMWDEMO_BPMCFG_OFFSET, sizeof(MmwDemo_BpmCfg), BPM_HCC_SUBFRAME_IDX);

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for configuring CQ RX Saturation monitor
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCChirpQualityRxSatMonCfg(uint8_t numProf)
{
    rlRxSatMonConf_t        cqSatMonCfg[numProf];
    uint8_t i;

    /* Initialize configuration: */
    for (i = 0; i < numProf; i++)
        memset ((void *)&cqSatMonCfg[i], 0, sizeof(rlRxSatMonConf_t));

    /* Populate configuration: */
    cqSatMonCfg[0].profileIndx                 = CQRXSAT_HCC_0_PROFILE_ID;
    cqSatMonCfg[0].satMonSel                   = CQRXSAT_HCC_0_SAT_MON_SEL;
    cqSatMonCfg[0].primarySliceDuration        = CQRXSAT_HCC_0_PRI_SLICE_DURATION;
    cqSatMonCfg[0].numSlices                   = CQRXSAT_HCC_0_NUM_SLICES;
    cqSatMonCfg[0].rxChannelMask               = CQRXSAT_HCC_0_RX_CHAN_MASK;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 2){
        cqSatMonCfg[1].profileIndx                 = CQRXSAT_HCC_1_PROFILE_ID;
        cqSatMonCfg[1].satMonSel                   = CQRXSAT_HCC_1_SAT_MON_SEL;
        cqSatMonCfg[1].primarySliceDuration        = CQRXSAT_HCC_1_PRI_SLICE_DURATION;
        cqSatMonCfg[1].numSlices                   = CQRXSAT_HCC_1_NUM_SLICES;
        cqSatMonCfg[1].rxChannelMask               = CQRXSAT_HCC_1_RX_CHAN_MASK;
    }
    if (numProf >= 3){
        cqSatMonCfg[2].profileIndx                 = CQRXSAT_HCC_2_PROFILE_ID;
        cqSatMonCfg[2].satMonSel                   = CQRXSAT_HCC_2_SAT_MON_SEL;
        cqSatMonCfg[2].primarySliceDuration        = CQRXSAT_HCC_2_PRI_SLICE_DURATION;
        cqSatMonCfg[2].numSlices                   = CQRXSAT_HCC_2_NUM_SLICES;
        cqSatMonCfg[2].rxChannelMask               = CQRXSAT_HCC_2_RX_CHAN_MASK;
    }
    if (numProf == 4){
        cqSatMonCfg[3].profileIndx                 = CQRXSAT_HCC_3_PROFILE_ID;
        cqSatMonCfg[3].satMonSel                   = CQRXSAT_HCC_3_SAT_MON_SEL;
        cqSatMonCfg[3].primarySliceDuration        = CQRXSAT_HCC_3_PRI_SLICE_DURATION;
        cqSatMonCfg[3].numSlices                   = CQRXSAT_HCC_3_NUM_SLICES;
        cqSatMonCfg[3].rxChannelMask               = CQRXSAT_HCC_3_RX_CHAN_MASK;
    }
#endif

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++)
        gMmwMssMCB.cqSatMonCfg[cqSatMonCfg[i].profileIndx] = cqSatMonCfg[i];

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for configuring CQ Signal & Image band monitor
 *
 *  @param[in] numProf
 *      Number of offset profiles
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCChirpQualitySigImgMonCfg (uint8_t numProf)
{
    rlSigImgMonConf_t       cqSigImgMonCfg[numProf];
    uint8_t i;

    /* Initialize configuration: */
    for (i = 0; i < numProf; i++)
        memset ((void *)&cqSigImgMonCfg[i], 0, sizeof(rlSigImgMonConf_t));

    /* Populate configuration: */
    cqSigImgMonCfg[0].profileIndx              = CQSIGIMG_HCC_0_PROFILE_ID;
    cqSigImgMonCfg[0].numSlices                = CQSIGIMG_HCC_0_NUM_SLICES;
    cqSigImgMonCfg[0].timeSliceNumSamples      = CQSIGIMG_HCC_0_NUM_SAMPLE_PER_SLICE;

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProf >= 2){
        cqSigImgMonCfg[1].profileIndx              = CQSIGIMG_HCC_1_PROFILE_ID;
        cqSigImgMonCfg[1].numSlices                = CQSIGIMG_HCC_1_NUM_SLICES;
        cqSigImgMonCfg[1].timeSliceNumSamples      = CQSIGIMG_HCC_1_NUM_SAMPLE_PER_SLICE;
    }
    if (numProf >= 3){
        cqSigImgMonCfg[2].profileIndx              = CQSIGIMG_HCC_2_PROFILE_ID;
        cqSigImgMonCfg[2].numSlices                = CQSIGIMG_HCC_2_NUM_SLICES;
        cqSigImgMonCfg[2].timeSliceNumSamples      = CQSIGIMG_HCC_2_NUM_SAMPLE_PER_SLICE;
    }
    if (numProf == 4){
        cqSigImgMonCfg[3].profileIndx              = CQSIGIMG_HCC_3_PROFILE_ID;
        cqSigImgMonCfg[3].numSlices                = CQSIGIMG_HCC_3_NUM_SLICES;
        cqSigImgMonCfg[3].timeSliceNumSamples      = CQSIGIMG_HCC_3_NUM_SAMPLE_PER_SLICE;
    }
#endif

    /* Save Configuration to use later */
    for (i = 0; i < numProf; i++)
        gMmwMssMCB.cqSigImgMonCfg[cqSigImgMonCfg[i].profileIndx] = cqSigImgMonCfg[i];

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for enabling analog monitors
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCAnalogMonitorCfg (void)
{

    /* Save Configuration to use later */
    gMmwMssMCB.anaMonCfg.rxSatMonEn  = ANALOGMONITOR_HCC_RX_SATURATION;
    gMmwMssMCB.anaMonCfg.sigImgMonEn = ANALOGMONITOR_HCC_SIG_IMG_BAND;

    gMmwMssMCB.isAnaMonCfgPending = 1;

    return 0;
}



/**
 *  @b Description
 *  @n
 *      This is the HCC Handler for the High Speed Interface
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_HCCLvdsStreamCfg()
{
    MmwDemo_LvdsStreamCfg   cfg;

    /* Initialize configuration for DC range signature calibration */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_LvdsStreamCfg));

    /* Populate configuration: */
    cfg.isHeaderEnabled = LVDSSTREAM_HCC_ENABLE_HEADER;
    cfg.dataFmt         = LVDSSTREAM_HCC_DATA_FMT;
    cfg.isSwEnabled     = LVDSSTREAM_HCC_ENABLE_SW;

    /* Save Configuration to use later */
    MmwDemo_CfgUpdate((void *)&cfg, MMWDEMO_LVDSSTREAMCFG_OFFSET, sizeof(MmwDemo_LvdsStreamCfg), LVDSSTREAM_HCC_SUBFRAME_IDX);

    return 0;
}



/**
 *  @b Description
 *  @n
 *      Sensor Configuration Task which initializes the
 *      hard-coded chirp configurations for the sensor.
 *
 *  @retval
 *      Not Applicable.
 */
void mmwDemo_sensorConfig_task(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    uint8_t             index, numAdvSubframes;

    memset ((void *)&errCode, 0, sizeof(int32_t));
    memset ((void *)&index, 0, sizeof(uint8_t));
    memset ((void *)&numAdvSubframes, 0, sizeof(uint8_t));

    /* Configure Varying Number of Subframes if Using Advanced Subframe Profile*/
#ifdef PROFILE_ADVANCED_SUBFRAME
    numAdvSubframes = ADVFRAME_HCC_NUM_SUBFRAMES;
#endif

    /* Get RF frequency scale factor */
    double              gHCC_mmwave_freq_scale_factor;
    memset ((void *)&gHCC_mmwave_freq_scale_factor, 0, sizeof(double));
    gHCC_mmwave_freq_scale_factor = SOC_getDeviceRFFreqScaleFactor(gMmwMssMCB.socHandle, &errCode);

    /*****************************************************************************
     * Configuration :: Open Sensor
     *****************************************************************************/

    /* Initialize the channel configuration */
    gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn             = CHANNEL_HCC_RX_CHANNEL_EN;
    gMmwMssMCB.cfg.openCfg.chCfg.txChannelEn             = CHANNEL_HCC_TX_CHANNEL_EN;
    gMmwMssMCB.cfg.openCfg.chCfg.cascading               = CHANNEL_HCC_CASCADING;

    /* Initialize the low power mode configuration */
    gMmwMssMCB.cfg.openCfg.lowPowerMode.lpAdcMode        = LP_HCC_LOW_POWER_MODE;

    /* Initialize the ADCOut configuration */
    gMmwMssMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcBits       = ADC_HCC_NUM_ADC_BITS;
    gMmwMssMCB.cfg.openCfg.adcOutCfg.fmt.b2AdcOutFmt     = ADC_HCC_OUTPUT_FMT;

    /* Call MMW Demo helper function to open sensor */
    if (MmwDemo_openSensor(true) < 0)
        printf("MmDemo_openSensor command returned an error.\n");
    else
        printf("Sensor was successfully opened.\n");

    /*****************************************************************************
     * Configuration :: Initializing and Adding Profiles
     *****************************************************************************/

    uint8_t             numProfiles;
    numProfiles = 1;

#ifdef PROFILE_ADVANCED_SUBFRAME
    numProfiles = numAdvSubframes;
#endif

    /* Initialize the profile configurations */
    rlProfileCfg_t      profileCfg[numProfiles];
    for (index = 0; index < numProfiles; index++)
        memset ((void *)&profileCfg[index], 0, sizeof(rlProfileCfg_t));

    /* Populate the profiles from hard-coded config */
    profileCfg[0].profileId                = PROFILE_HCC_0_PROFILE_ID;
    profileCfg[0].startFreqConst           = (uint32_t)((float)PROFILE_HCC_0_START_FREQ_GHZ * (1U << 26) / gHCC_mmwave_freq_scale_factor);
    profileCfg[0].idleTimeConst            = (uint32_t)((float)PROFILE_HCC_0_IDLE_TIME_VAL * 1000 / 10);
    profileCfg[0].adcStartTimeConst        = (uint32_t)((float)PROFILE_HCC_0_ADC_START_TIME_VAL * 1000 / 10);
    profileCfg[0].rampEndTime              = (uint32_t)((float)PROFILE_HCC_0_RAMP_END_TIME_VAL * 1000 / 10);
    profileCfg[0].txOutPowerBackoffCode    = PROFILE_HCC_0_TXOUT_POWER_BACKOFF;
    profileCfg[0].txPhaseShifter           = PROFILE_HCC_0_TXPHASESHIFTER_VAL;
    profileCfg[0].freqSlopeConst           = (int16_t)((float)PROFILE_HCC_0_FREQ_SLOPE_MHZ_PER_US * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e3) * 900.0));
    profileCfg[0].txStartTime              = (int32_t)((float)PROFILE_HCC_0_TX_START_TIME_VAL * 1000 / 10);
    profileCfg[0].numAdcSamples            = PROFILE_HCC_0_ADC_SAMPLE_VAL;
    profileCfg[0].digOutSampleRate         = PROFILE_HCC_0_DIGOUT_SAMPLERATE_VAL;
    profileCfg[0].hpfCornerFreq1           = PROFILE_HCC_0_HPFCORNER_FREQ1_VAL;
    profileCfg[0].hpfCornerFreq2           = PROFILE_HCC_0_HPFCORNER_FREQ2_VAL;
    profileCfg[0].rxGain                   = PROFILE_HCC_0_RX_GAIN_VAL;


#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numProfiles >= 2){
        profileCfg[1].profileId                = PROFILE_HCC_1_PROFILE_ID;
        profileCfg[1].startFreqConst           = (uint32_t)((float)PROFILE_HCC_1_START_FREQ_GHZ * (1U << 26) / gHCC_mmwave_freq_scale_factor);
        profileCfg[1].idleTimeConst            = (uint32_t)((float)PROFILE_HCC_1_IDLE_TIME_VAL * 1000 / 10);
        profileCfg[1].adcStartTimeConst        = (uint32_t)((float)PROFILE_HCC_1_ADC_START_TIME_VAL * 1000 / 10);
        profileCfg[1].rampEndTime              = (uint32_t)((float)PROFILE_HCC_1_RAMP_END_TIME_VAL * 1000 / 10);
        profileCfg[1].txOutPowerBackoffCode    = PROFILE_HCC_1_TXOUT_POWER_BACKOFF;
        profileCfg[1].txPhaseShifter           = PROFILE_HCC_1_TXPHASESHIFTER_VAL;
        profileCfg[1].freqSlopeConst           = (int16_t)((float)PROFILE_HCC_1_FREQ_SLOPE_MHZ_PER_US * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e3) * 900.0));
        profileCfg[1].txStartTime              = (int32_t)((float)PROFILE_HCC_1_TX_START_TIME_VAL * 1000 / 10);
        profileCfg[1].numAdcSamples            = PROFILE_HCC_1_ADC_SAMPLE_VAL;
        profileCfg[1].digOutSampleRate         = PROFILE_HCC_1_DIGOUT_SAMPLERATE_VAL;
        profileCfg[1].hpfCornerFreq1           = PROFILE_HCC_1_HPFCORNER_FREQ1_VAL;
        profileCfg[1].hpfCornerFreq2           = PROFILE_HCC_1_HPFCORNER_FREQ2_VAL;
        profileCfg[1].rxGain                   = PROFILE_HCC_1_RX_GAIN_VAL;
    }

    if (numProfiles >= 3){
        profileCfg[2].profileId                = PROFILE_HCC_2_PROFILE_ID;
        profileCfg[2].startFreqConst           = (uint32_t)((float)PROFILE_HCC_2_START_FREQ_GHZ * (1U << 26) / gHCC_mmwave_freq_scale_factor);
        profileCfg[2].idleTimeConst            = (uint32_t)((float)PROFILE_HCC_2_IDLE_TIME_VAL * 1000 / 10);
        profileCfg[2].adcStartTimeConst        = (uint32_t)((float)PROFILE_HCC_2_ADC_START_TIME_VAL * 1000 / 10);
        profileCfg[2].rampEndTime              = (uint32_t)((float)PROFILE_HCC_2_RAMP_END_TIME_VAL * 1000 / 10);
        profileCfg[2].txOutPowerBackoffCode    = PROFILE_HCC_2_TXOUT_POWER_BACKOFF;
        profileCfg[2].txPhaseShifter           = PROFILE_HCC_2_TXPHASESHIFTER_VAL;
        profileCfg[2].freqSlopeConst           = (int16_t)((float)PROFILE_HCC_2_FREQ_SLOPE_MHZ_PER_US * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e3) * 900.0));
        profileCfg[2].txStartTime              = (int32_t)((float)PROFILE_HCC_2_TX_START_TIME_VAL * 1000 / 10);
        profileCfg[2].numAdcSamples            = PROFILE_HCC_2_ADC_SAMPLE_VAL;
        profileCfg[2].digOutSampleRate         = PROFILE_HCC_2_DIGOUT_SAMPLERATE_VAL;
        profileCfg[2].hpfCornerFreq1           = PROFILE_HCC_2_HPFCORNER_FREQ1_VAL;
        profileCfg[2].hpfCornerFreq2           = PROFILE_HCC_2_HPFCORNER_FREQ2_VAL;
        profileCfg[2].rxGain                   = PROFILE_HCC_2_RX_GAIN_VAL;
    }

    if (numProfiles == 4){
        profileCfg[3].profileId                = PROFILE_HCC_3_PROFILE_ID;
        profileCfg[3].startFreqConst           = (uint32_t)((float)PROFILE_HCC_3_START_FREQ_GHZ * (1U << 26) / gHCC_mmwave_freq_scale_factor);
        profileCfg[3].idleTimeConst            = (uint32_t)((float)PROFILE_HCC_3_IDLE_TIME_VAL * 1000 / 10);
        profileCfg[3].adcStartTimeConst        = (uint32_t)((float)PROFILE_HCC_3_ADC_START_TIME_VAL * 1000 / 10);
        profileCfg[3].rampEndTime              = (uint32_t)((float)PROFILE_HCC_3_RAMP_END_TIME_VAL * 1000 / 10);
        profileCfg[3].txOutPowerBackoffCode    = PROFILE_HCC_3_TXOUT_POWER_BACKOFF;
        profileCfg[3].txPhaseShifter           = PROFILE_HCC_3_TXPHASESHIFTER_VAL;
        profileCfg[3].freqSlopeConst           = (int16_t)((float)PROFILE_HCC_3_FREQ_SLOPE_MHZ_PER_US * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e3) * 900.0));
        profileCfg[3].txStartTime              = (int32_t)((float)PROFILE_HCC_3_TX_START_TIME_VAL * 1000 / 10);
        profileCfg[3].numAdcSamples            = PROFILE_HCC_3_ADC_SAMPLE_VAL;
        profileCfg[3].digOutSampleRate         = PROFILE_HCC_3_DIGOUT_SAMPLERATE_VAL;
        profileCfg[3].hpfCornerFreq1           = PROFILE_HCC_3_HPFCORNER_FREQ1_VAL;
        profileCfg[3].hpfCornerFreq2           = PROFILE_HCC_3_HPFCORNER_FREQ2_VAL;
        profileCfg[3].rxGain                   = PROFILE_HCC_3_RX_GAIN_VAL;
    }
#endif

    /* Initialize the profile handles */
    MMWave_ProfileHandle    profileHandle[numProfiles];
    for (index = 0; index < numProfiles; index++)
        memset ((void *)&profileHandle[index], 0, sizeof(MMWave_ProfileHandle));

    /* Adding Profiles to Control Handle */
    for (index = 0; index < numProfiles; index++){
        profileHandle[index] = MMWave_addProfile(gMmwMssMCB.ctrlHandle, &profileCfg[index], &errCode);
    }

    /*****************************************************************************
     * Configuration :: Initializing and Adding Chirps
     *****************************************************************************/

    uint8_t             numChirps;
    numChirps = 2;

#if (defined(PROFILE_3d) || defined(PROFILE_CALIBRATION))
    numChirps = 3;
#elif defined(PROFILE_ADVANCED_SUBFRAME)
    numChirps = 2*numAdvSubframes;
#endif

    /* Initialize the chirp configurations */
    rlChirpCfg_t    chirpCfg[numChirps];
    for (index = 0; index < numChirps; index++)
        memset ((void *)&chirpCfg[index], 0, sizeof(rlChirpCfg_t));

    /* Populate the chirps from hard-coded configs */
    chirpCfg[0].chirpStartIdx           = CHIRP_HCC_0_START_INDEX;
    chirpCfg[0].chirpEndIdx             = CHIRP_HCC_0_END_INDEX;
    chirpCfg[0].profileId               = CHIRP_HCC_0_PROFILE_ID;
    chirpCfg[0].startFreqVar            = (uint32_t)((float)CHIRP_HCC_0_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
    chirpCfg[0].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_0_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
    chirpCfg[0].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_0_IDLE_TIME_VAL * 1000.0 / 10.0);
    chirpCfg[0].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_0_ADC_START_TIME_VAL * 1000.0 / 10.0);
    chirpCfg[0].txEnable                = CHIRP_HCC_0_TX_CHANNEL;

    chirpCfg[1].chirpStartIdx           = CHIRP_HCC_1_START_INDEX;
    chirpCfg[1].chirpEndIdx             = CHIRP_HCC_1_END_INDEX;
    chirpCfg[1].profileId               = CHIRP_HCC_1_PROFILE_ID;
    chirpCfg[1].startFreqVar            = (uint32_t)((float)CHIRP_HCC_1_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
    chirpCfg[1].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_1_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
    chirpCfg[1].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_1_IDLE_TIME_VAL * 1000.0 / 10.0);
    chirpCfg[1].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_1_ADC_START_TIME_VAL * 1000.0 / 10.0);
    chirpCfg[1].txEnable                = CHIRP_HCC_1_TX_CHANNEL;

#if (defined(PROFILE_3d) || defined(PROFILE_CALIBRATION) || defined(PROFILE_ADVANCED_SUBFRAME))

    if (numChirps >= 3){
        chirpCfg[2].chirpStartIdx           = CHIRP_HCC_2_START_INDEX;
        chirpCfg[2].chirpEndIdx             = CHIRP_HCC_2_END_INDEX;
        chirpCfg[2].profileId               = CHIRP_HCC_2_PROFILE_ID;
        chirpCfg[2].startFreqVar            = (uint32_t)((float)CHIRP_HCC_2_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[2].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_2_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[2].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_2_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[2].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_2_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[2].txEnable                = CHIRP_HCC_2_TX_CHANNEL;
    }
#endif

#ifdef PROFILE_ADVANCED_SUBFRAME
    if (numChirps >= 4){
        chirpCfg[3].chirpStartIdx           = CHIRP_HCC_3_START_INDEX;
        chirpCfg[3].chirpEndIdx             = CHIRP_HCC_3_END_INDEX;
        chirpCfg[3].profileId               = CHIRP_HCC_3_PROFILE_ID;
        chirpCfg[3].startFreqVar            = (uint32_t)((float)CHIRP_HCC_3_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[3].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_3_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[3].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_3_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[3].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_3_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[3].txEnable                = CHIRP_HCC_3_TX_CHANNEL;
    }
    if (numChirps >= 5){
        chirpCfg[4].chirpStartIdx           = CHIRP_HCC_4_START_INDEX;
        chirpCfg[4].chirpEndIdx             = CHIRP_HCC_4_END_INDEX;
        chirpCfg[4].profileId               = CHIRP_HCC_4_PROFILE_ID;
        chirpCfg[4].startFreqVar            = (uint32_t)((float)CHIRP_HCC_4_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[4].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_4_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[4].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_4_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[4].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_4_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[4].txEnable                = CHIRP_HCC_4_TX_CHANNEL;
    }
    if (numChirps >= 6){
        chirpCfg[5].chirpStartIdx           = CHIRP_HCC_5_START_INDEX;
        chirpCfg[5].chirpEndIdx             = CHIRP_HCC_5_END_INDEX;
        chirpCfg[5].profileId               = CHIRP_HCC_5_PROFILE_ID;
        chirpCfg[5].startFreqVar            = (uint32_t)((float)CHIRP_HCC_5_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[5].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_5_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[5].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_5_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[5].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_5_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[5].txEnable                = CHIRP_HCC_5_TX_CHANNEL;
    }
    if (numChirps >= 7){
        chirpCfg[6].chirpStartIdx           = CHIRP_HCC_6_START_INDEX;
        chirpCfg[6].chirpEndIdx             = CHIRP_HCC_6_END_INDEX;
        chirpCfg[6].profileId               = CHIRP_HCC_6_PROFILE_ID;
        chirpCfg[6].startFreqVar            = (uint32_t)((float)CHIRP_HCC_6_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[6].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_6_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[6].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_6_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[6].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_6_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[6].txEnable                = CHIRP_HCC_6_TX_CHANNEL;
    }
    if (numChirps >= 8){
        chirpCfg[7].chirpStartIdx           = CHIRP_HCC_7_START_INDEX;
        chirpCfg[7].chirpEndIdx             = CHIRP_HCC_7_END_INDEX;
        chirpCfg[7].profileId               = CHIRP_HCC_7_PROFILE_ID;
        chirpCfg[7].startFreqVar            = (uint32_t)((float)CHIRP_HCC_7_START_FREQ_VAL * (1U << 26) / (gHCC_mmwave_freq_scale_factor * 1e9));
        chirpCfg[7].freqSlopeVar            = (uint16_t)((float)CHIRP_HCC_7_FREQ_SLOPE_VAL * (1U << 26) / ((gHCC_mmwave_freq_scale_factor * 1e6) * 900.0));
        chirpCfg[7].idleTimeVar             = (uint32_t)((float)CHIRP_HCC_7_IDLE_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[7].adcStartTimeVar         = (uint32_t)((float)CHIRP_HCC_7_ADC_START_TIME_VAL * 1000.0 / 10.0);
        chirpCfg[7].txEnable                = CHIRP_HCC_7_TX_CHANNEL;
    }
#endif

    /* Initialize the chirp handles */
    MMWave_ChirpHandle      chirpHandle[numChirps];
    for (index = 0; index < numChirps; index++)
        memset ((void *)&chirpHandle[index], 0, sizeof(MMWave_ChirpHandle));

    /* Adding Chirps to profile handles */
    for (index = 0; index < numChirps; index++){
        chirpHandle[index] = MMWave_addChirp(profileHandle[chirpCfg[index].profileId], &chirpCfg[index], &errCode);
    }

    /*****************************************************************************
     * Configuration :: Initializing Output Mode and Frame Configurations
     *****************************************************************************/

    /* Determine Output Mode */
    switch(DFE_DATA_OUTPUT_MODE_HCC)
    {
    case 1U:
        printf("Data Output Mode: Frame\n");
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode                    = MMWave_DFEDataOutputMode_FRAME;

        /* Populate the frame config */
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpStartIdx            = FRAME_HCC_CHIRP_START_INDEX;
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.chirpEndIdx              = FRAME_HCC_CHIRP_END_INDEX;
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numLoops                 = FRAME_HCC_NUM_LOOPS;
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.numFrames                = FRAME_HCC_NUM_FRAMES;
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.framePeriodicity         = (uint32_t)((float)FRAME_HCC_FRAME_PERIODICITY * 1000000 / 5);
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.triggerSelect            = FRAME_HCC_TRIGGER_SELECT;
        gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.frameCfg.frameTriggerDelay        = (uint32_t)((float)FRAME_HCC_FRAME_TRIG_DELAY * 1000000 / 5);

        memcpy((void *)&gMmwMssMCB.cfg.ctrlCfg.u.frameCfg.profileHandle[0], (void *)&profileHandle[0], sizeof(profileHandle[0]));
        break;

    case 2U:
        printf("Data Output Mode: Continuous\n");
        printf("Continuous Mode is not supported with OOB demos. Please choose either Frame Mode or Advanced Frame Mode.\n");
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_CONTINUOUS;

        break;

    case 3U:
        printf("Data Output Mode: Advanced Frame\n");
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode = MMWave_DFEDataOutputMode_ADVANCED_FRAME;

#ifdef PROFILE_ADVANCED_SUBFRAME

        /* Populate the advanced subframe configs */
        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames                              = numAdvSubframes;
        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.numSubFrames                               = numAdvSubframes;

        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.forceProfile                                = ADVFRAME_HCC_FORCE_PROFILE;

        if (numAdvSubframes >= 1){
            memcpy((void *)&gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[0], (void *)&profileHandle[0], sizeof(profileHandle[0]));

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].forceProfileIdx              = SUBFRAME_HCC_0_FORCE_PROFILE_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].chirpStartIdx                = SUBFRAME_HCC_0_CHIRP_START_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].numOfChirps                  = SUBFRAME_HCC_0_NUM_CHIRPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].numLoops                     = SUBFRAME_HCC_0_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].burstPeriodicity             = (uint32_t)((float)SUBFRAME_HCC_0_BURST_PERIODICITY * 1000000 / 5);
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].chirpStartIdxOffset          = SUBFRAME_HCC_0_CHIRP_START_OFFSET;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].numOfBurst                   = SUBFRAME_HCC_0_NUM_BURST;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].numOfBurstLoops              = SUBFRAME_HCC_0_NUM_BURST_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[0].subFramePeriodicity          = (uint32_t)((float)SUBFRAME_HCC_0_SUBFRAME_PERIODICITY * 1000000 / 5);

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[0].numAdcSamples           = PROFILE_HCC_0_ADC_SAMPLE_VAL * 2;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[0].totalChirps             = SUBFRAME_HCC_0_NUM_CHIRPS * SUBFRAME_HCC_0_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[0].numChirpsInDataPacket   = SUBFRAME_HCC_0_NUM_CHIRPS;
        }
        if (numAdvSubframes >= 2){
            memcpy((void *)&gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[1], (void *)&profileHandle[1], sizeof(profileHandle[1]));

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].forceProfileIdx              = SUBFRAME_HCC_1_FORCE_PROFILE_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].chirpStartIdx                = SUBFRAME_HCC_1_CHIRP_START_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].numOfChirps                  = SUBFRAME_HCC_1_NUM_CHIRPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].numLoops                     = SUBFRAME_HCC_1_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].burstPeriodicity             = (uint32_t)((float)SUBFRAME_HCC_1_BURST_PERIODICITY * 1000000 / 5);
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].chirpStartIdxOffset          = SUBFRAME_HCC_1_CHIRP_START_OFFSET;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].numOfBurst                   = SUBFRAME_HCC_1_NUM_BURST;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].numOfBurstLoops              = SUBFRAME_HCC_1_NUM_BURST_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[1].subFramePeriodicity          = (uint32_t)((float)SUBFRAME_HCC_1_SUBFRAME_PERIODICITY * 1000000 / 5);

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[1].numAdcSamples           = PROFILE_HCC_1_ADC_SAMPLE_VAL * 2;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[1].totalChirps             = SUBFRAME_HCC_1_NUM_CHIRPS * SUBFRAME_HCC_1_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[1].numChirpsInDataPacket   = SUBFRAME_HCC_1_NUM_CHIRPS;
        }
        if (numAdvSubframes >= 3){
            memcpy((void *)&gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[2], (void *)&profileHandle[2], sizeof(profileHandle[2]));

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].forceProfileIdx              = SUBFRAME_HCC_2_FORCE_PROFILE_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].chirpStartIdx                = SUBFRAME_HCC_2_CHIRP_START_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].numOfChirps                  = SUBFRAME_HCC_2_NUM_CHIRPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].numLoops                     = SUBFRAME_HCC_2_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].burstPeriodicity             = (uint32_t)((float)SUBFRAME_HCC_2_BURST_PERIODICITY * 1000000 / 5);
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].chirpStartIdxOffset          = SUBFRAME_HCC_2_CHIRP_START_OFFSET;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].numOfBurst                   = SUBFRAME_HCC_2_NUM_BURST;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].numOfBurstLoops              = SUBFRAME_HCC_2_NUM_BURST_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[2].subFramePeriodicity          = (uint32_t)((float)SUBFRAME_HCC_2_SUBFRAME_PERIODICITY * 1000000 / 5);

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[2].numAdcSamples           = PROFILE_HCC_2_ADC_SAMPLE_VAL * 2;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[2].totalChirps             = SUBFRAME_HCC_2_NUM_CHIRPS * SUBFRAME_HCC_2_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[2].numChirpsInDataPacket   = SUBFRAME_HCC_2_NUM_CHIRPS;
        }
        if (numAdvSubframes == 4){
            memcpy((void *)&gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.profileHandle[3], (void *)&profileHandle[3], sizeof(profileHandle[3]));

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].forceProfileIdx              = SUBFRAME_HCC_3_FORCE_PROFILE_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].chirpStartIdx                = SUBFRAME_HCC_3_CHIRP_START_INDEX;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].numOfChirps                  = SUBFRAME_HCC_3_NUM_CHIRPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].numLoops                     = SUBFRAME_HCC_3_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].burstPeriodicity             = (uint32_t)((float)SUBFRAME_HCC_3_BURST_PERIODICITY * 1000000 / 5);
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].chirpStartIdxOffset          = SUBFRAME_HCC_3_CHIRP_START_OFFSET;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].numOfBurst                   = SUBFRAME_HCC_3_NUM_BURST;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].numOfBurstLoops              = SUBFRAME_HCC_3_NUM_BURST_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[3].subFramePeriodicity          = (uint32_t)((float)SUBFRAME_HCC_3_SUBFRAME_PERIODICITY * 1000000 / 5);

            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[3].numAdcSamples           = PROFILE_HCC_3_ADC_SAMPLE_VAL * 2;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[3].totalChirps             = SUBFRAME_HCC_3_NUM_CHIRPS * SUBFRAME_HCC_3_NUM_LOOPS;
            gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameData.subframeDataCfg[3].numChirpsInDataPacket   = SUBFRAME_HCC_3_NUM_CHIRPS;
        }

        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.numFrames                                   = ADVFRAME_HCC_NUM_FRAMES;
        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.triggerSelect                               = ADVFRAME_HCC_TRIGGER_SELECT;
        gMmwMssMCB.cfg.ctrlCfg.u.advancedFrameCfg.frameCfg.frameSeq.frameTrigDelay                              = (uint32_t)((float)ADVFRAME_HCC_FRAME_TRIG_DELAY * 1000000 / 5);
#endif
        break;

    default:
        printf("DFE = Default!\n");
        printf("Error: Invalid Mode.\n");
        break;
    }

    /*****************************************************************************
     * Configuration :: Initializing Data Path Configurations
     *****************************************************************************/

    uint8_t                 numProf;
    numProf = 1;
    #ifdef PROFILE_ADVANCED_SUBFRAME
    numProf = numAdvSubframes;
    #endif

    /* Call HCC helper functions to configure data path */
    MmwDemo_HCCGuiMonSel(numProf);
    MmwDemo_HCCCfarCfg(2*numProf);
    MmwDemo_HCCCfarFovCfg(2*numProf);
    MmwDemo_HCCAoAFovCfg();
    MmwDemo_HCCExtendedMaxVelocity();
    MmwDemo_HCCMultiObjBeamForming(numProf);
    MmwDemo_HCCCalibDcRangeSig (numProf);
    MmwDemo_HCCClutterRemoval();
    MmwDemo_HCCADCBufCfg();
    MmwDemo_HCCCompRangeBiasAndRxChanPhaseCfg();
    MmwDemo_HCCMeasureRangeBiasAndRxChanPhaseCfg();
    MmwDemo_HCCBpmCfg();
    MmwDemo_HCCChirpQualityRxSatMonCfg(numProf);
    MmwDemo_HCCChirpQualitySigImgMonCfg(numProf);
    MmwDemo_HCCAnalogMonitorCfg();
    MmwDemo_HCCLvdsStreamCfg();

    /* Configure Pre-Start Common Config */
    gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames = MmwDemo_RFParser_getNumSubFrames(&gMmwMssMCB.cfg.ctrlCfg);

    /*****************************************************************************
     * Deployment :: Configuring and Starting Sensor
     *****************************************************************************/

    /* Call MMW Demo helper function to configure sensor */
    printf("Sending configSensor command to EVM.\n");
    if (MmwDemo_configSensor() < 0)
        printf("MmwDemo_configSensor command returned an error.\n");
    else
        printf("Sensor was successfully configured.\n");

    /* Call MMW Demo helper function to start sensor */
    if (MmwDemo_startSensor() < 0)
        printf("MmDemo_startSensor command returned an error.\n");
    else
        printf("Hard Coded Configuration successfully sent to EVM.\n");

    return;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (uint8_t taskPriority)
{
    Task_Params         taskParams;

    /* Initialize the Task Parameters */
    Task_Params_init(&taskParams);
    taskParams.priority  = taskPriority;
    Task_create(mmwDemo_sensorConfig_task, &taskParams, NULL);
}

