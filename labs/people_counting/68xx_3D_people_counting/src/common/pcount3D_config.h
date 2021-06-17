/**
 *   @file  pcount3D_config.h
 *
 *   @brief
 *      This is the header file that describes configurations for the 3D people counting Demo..
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
#ifndef PCOUNT3DDEMO_CONFIG_H
#define PCOUNT3DDEMO_CONFIG_H

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#include <ti/common/sys_common.h>

/* Data path Include Files */
#include <common/src/dpc/objdetrangehwa/objdetrangehwa.h>
#include <common/src/dpc/capon3d/objectdetection.h>

#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief
 *  Millimeter Wave Demo Platform Configuration.
 *
 * @details
 *  The structure is used to hold all the relevant configuration for
 *  the Platform.
 */
typedef struct Pcount3DDemo_platformCfg_t
{
    /*! @brief   GPIO index for sensor status */
    uint32_t            SensorStatusGPIO;
    
    /*! @brief   CPU Clock Frequency. */
    uint32_t            sysClockFrequency;

    /*! @brief   UART Logging Baud Rate. */
    uint32_t            loggingBaudRate;

    /*! @brief   UART Command Baud Rate. */
    uint32_t            commandBaudRate;
} Pcount3DDemo_platformCfg;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
typedef struct Pcount3DDemo_Cfg_t
{
    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Platform specific configuration. */
    Pcount3DDemo_platformCfg platformCfg;
} Pcount3DDemo_Cfg;

/**
 * @brief
 *  Data path DPC common configuraiton.
 *
 * @details
 *  The structure is used to hold all the relevant configuration for
 *  DPC common configuration.
 */
typedef struct Pcount3DDemo_DPC_ObjDet_CommonCfg_t
{
    /*! @brief   Number of sub-frames */
    uint8_t numSubFrames;

} Pcount3DDemo_DPC_ObjDet_CommonCfg;

/**
 * @brief
 *  Data path DPC dynamic configuraiton.
 *
 * @details
 *  The structure is used to hold all the relevant configuration for
 *  DPC dynamic configuration.
 */
typedef struct Pcount3DDemo_DPC_ObjDet_DynCfg_t
{
    /*! @brief dynamic config */
    DPC_ObjectDetectionRangeHWA_DynCfg   r4fDynCfg;

    /*! @brief dynamic config */
    DPC_ObjectDetection_DynCfg      dspDynCfg;
} Pcount3DDemo_DPC_ObjDet_DynCfg;

#ifdef __cplusplus
}
#endif

#endif /* PCOUNT3DDEMO_CONFIG_H */
