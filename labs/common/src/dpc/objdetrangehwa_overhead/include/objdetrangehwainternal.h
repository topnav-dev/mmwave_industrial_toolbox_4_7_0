/**
 *   @file  objdetrangehwa.h
 *
 *   @brief
 *      Object Detection rnage hwa only DPC Header File
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

/** @defgroup DPC_OBJDETRANGEHWA_INTERNAL       Object Detection Range HWA only DPC (Data Path Chain) Internal
 */
/**
@defgroup DPC_OBJDETRANGEHWA_IOCTL__INTERNAL_DATA_STRUCTURES      Object Detection DPC Internal Data Structures
@ingroup DPC_OBJDET_INTERNAL
@brief
*   This section has a list of all the internal data structures which are a part of the DPC.
*/
/**
@defgroup DPC_OBJDETRANGEHWA_IOCTL__INTERNAL_DEFINITIONS      Object Detection DPC Internal Definitions
@ingroup DPC_OBJDETRANGEHWA_INTERNAL
@brief
*   This section has a list of all the internal defines which are a part of the DPC.
*/
/**
@defgroup DPC_OBJDETRANGEHWA__INTERNAL_FUNCTION             Object Detection DPC Internal Functions
@ingroup DPC_OBJDETRANGEHWA_INTERNAL
@brief
*   This section has a list of all the internal function which are a part of the DPC.
*   These are not exposed to the application.
*/

#ifndef DPC_OBJDETRANGEHWA_INTERNAL_H
#define DPC_OBJDETRANGEHWA_INTERNAL_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/dpm/dpm.h>

#include <ti/control/mmwavelink/mmwavelink.h>
#include <common/src/dpc/objdetrangehwa_overhead/objdetrangehwa.h>

#ifdef __cplusplus
extern "C" {
#endif

 /** @addtogroup DPC_OBJDETRANGEHWA_IOCTL__INTERNAL_DATA_STRUCTURES
  @{ */

/**
 * @brief  The structure is used to hold all the relevant information for
 *         each of the sub-frames for the object detection DPC.
 */
typedef struct SubFrameObj_t
{
    /*! brief   Pointer to hold Range Proc DPU handle */
    DPU_RangeProcHWA_Handle             dpuRangeObj;

    /*! brief   Dynamic configuration */
    DPC_ObjectDetectionRangeHWA_DynCfg  dynCfg;

    /*! brief   Range DPU configuration storage */
    DPU_RangeProcHWA_Config             rangeCfg;
} SubFrameObj;

/*
 * @brief Memory pool object to manage memory based on @ref DPC_ObjectDetection_MemCfg_t.
 */
typedef struct MemPoolObj_t
{
    /*! @brief      Memory configuration */
    DPC_ObjectDetectionRangeHWA_MemCfg cfg;

    /*! @brief   Pool running adress.*/
    uintptr_t    currAddr;

    /*! @brief   Pool max address. This pool allows setting address to desired
     *           (e.g for rewinding purposes), so having a running maximum
     *           helps in finding max pool usage
     */
    uintptr_t   maxCurrAddr;
} MemPoolObj;

/**
 * @brief
 *  Millimeter Object Detection DPC object/instance
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  object detection DPC.
 */
typedef struct ObjDetObj_t
{
    /*! @brief DPM Initialization Configuration */
    DPM_InitCfg   dpmInitCfg;

    /*! @brief Handle to the DPM Module */
    DPM_Handle    dpmHandle;

    /*! @brief Handle to the SOC Module: */
    SOC_Handle    socHandle;

    /*! @brief   Per sub-frame object */
    SubFrameObj   subFrameObj[RL_MAX_SUBFRAMES];

    /*! @brief Sub-frame index */
    uint8_t       subFrameIndx;
#ifdef TRACKERPROC_EN
    /*! @brief Tracker DPU Object: This is not sub-frame based */
    DPU_TrackerProc_Handle  dpuTrackerObj;
#endif
    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle   edmaHandle;

    /*! @brief   Handle of the EDMA driver. */
    HWA_Handle    hwaHandle;

    /*! @brief  Used for checking that inter-frame (inter-sub-frame) processing
     *          finished on time */
    int32_t       interSubFrameProcToken;

    /*! @brief L3 ram memory pool object */
    MemPoolObj    L3RamObj;

    /*! @brief Core Local ram memory pool object */
    MemPoolObj    CoreLocalRamObj;

    /*! @brief  HWA memory bank addresses */
    uint32_t      hwaMemBankAddr[4];

    /*! @brief  HWA memory single bank size in bytes */
    uint16_t      hwaMemBankSize;

    /*! @brief  Common configuration storage */
    DPC_ObjectDetectionRangeHWA_PreStartCommonCfg commonCfg;

    /*! @brief Flag to make sure pre start common config is received by the DPC
     *          before any pre-start (sub-frame specific) configs are received. */
    bool     isCommonCfgReceived;

   /*! @brief   Process call back function configuration */
    DPC_ObjectDetectionRangeHWA_ProcessCallBackCfg processCallBackFxn;

    /*! @brief   Stats structure to convey to Application timing and related information. */
    DPC_ObjectDetectionRangeHWA_Stats stats;
} ObjDetObj;

/**
@}
*/
extern void _DPC_Objdet_Assert(DPM_Handle handle, int32_t expression, const char *file, int32_t line);
#define DPC_Objdet_Assert(handle, expression) {                \
              _DPC_Objdet_Assert(handle, expression,           \
                                 __FILE__, __LINE__);          \
               DebugP_assert(expression);                      \
                                      }

#ifdef __cplusplus
}
#endif

#endif /* DPC_OBJDETRANGEHWA_INTERNAL_H */

