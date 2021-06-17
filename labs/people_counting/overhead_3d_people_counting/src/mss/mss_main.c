/**
 *   @file  mss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
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

/** @mainpage 3D People Counting Demo for XWR68XX
 * [TOC]
 *  @section intro_sec Introduction
 *
 *  The 3D People Counting Demo shows the indoor people counting capabilities using XWR68xx SoC.
 *  The demo uses adapted range HWA DPU and DPC offerred by the mmWave SDK (Software Development Kit) 
 *  for range processing on R4F and HWA, and uses advanced Capon beamforming on DSP to estimate 
 *  range, Doppler, azimuth angle and elevation angle of indoor targets. 
 *
 *  Please refer to SDK documentation for range HWA DPU and DPC details. The additional adaptions are 
 *  the following:
 *  - In rangeHWA DPU, modified range FFT scaling settings for HWA to maximize the SNR and dynamic 
 *    range for indoor environment and the particular FFT size we use.
 *  - In rangeHWA DPC, added support for radar cube format 2. 
 *  - Modified mmwdemo_rfparser.c to output chirp interval and frame period. Also fixed the error 
 *    in Doppler step calculation.
 *
 *  For Capon beamforming technique on DSP, please refer to document included in the packages for details.
 *
 *  Please refer to the users guide for UART output format and chirp configuration details.
 *
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


/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>
#include <ti/sysbios/utils/Load.h>
#include <ti/drivers/osal/MemoryP.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <people_counting/overhead_3d_people_counting/src/common/mmwdemo_rfparser.h>
#include <people_counting/overhead_3d_people_counting/src/common/mmwdemo_adcconfig.h>

#include <people_counting/overhead_3d_people_counting/src/common/pcount3D_config.h>
#include <people_counting/overhead_3d_people_counting/src/common/pcount3D_output.h>
#include <people_counting/overhead_3d_people_counting/src/common/pcount3D_hwres.h>
#include <people_counting/overhead_3d_people_counting/src/mss/pcount3D_mss.h>


/* Profiler Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <people_counting/overhead_3d_people_counting/src/mss/tracker_output.h>
#include <common/src/dpc/objdetrangehwa_overhead/include/objdetrangehwainternal.h>
#include <common/src/dpc/objdetrangehwa_overhead/objdetrangehwa.h>
#include <people_counting/overhead_3d_people_counting/src/common/swpform.h>
/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from BSS
 * that need quick action in real-time.
 *
 * CLI task must be at a lower priority than object detection
 * dpm task priority because the dynamic CLI command handling in the objection detection
 * dpm task assumes CLI task is held back during this processing. The alternative
 * is to use a semaphore between the two tasks.
 */
#define MMWDEMO_CLI_TASK_PRIORITY                 2
#define MMWDEMO_TRACKERDPU_TASK_PRIORITY          3
#define MMWDEMO_UARTTX_TASK_PRIORITY              4
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      5
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY         6

#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY)
#error CLI task priority must be < Object Detection DPM task priority
#endif

#define DPC_OBJDET_R4F_INSTANCEID       (0xFEEDFEED)
#define DEBUG(_x) _x

#define UARTOVERRATE (1)

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
Pcount3DDemo_MSS_MCB    gMmwMssMCB;

extern ObjDetObj     *gObjDetObj;

 /*! TCMB RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_LOCALRAM_SIZE (8U * 1024U)
uint8_t gDPCTCM[MMWDEMO_OBJDET_LOCALRAM_SIZE];
#pragma DATA_SECTION(gDPCTCM, ".dpcLocalRam");
#pragma DATA_ALIGN(gDPCTCM, 4);

/*! L3 RAM buffer for object detection DPC */
uint8_t gMmwL3[0xA8000];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit(uint8_t taskPriority);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions prototype *************
 **************************************************************************/

/* MMW demo functions for datapath operation */
static void Pcount3DDemo_dataPathOpen(void);
static int32_t Pcount3DDemo_dataPathConfig (void);
static void Pcount3DDemo_dataPathStart (void);
static void Pcount3DDemo_dataPathStop (void);
static void Pcount3DDemo_handleObjectDetResult(DPM_Buffer  *ptrResult);
static void Pcount3DDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
#ifdef TRACKERPROC_EN
static void MmwDemo_trackerDPUTask(UArg arg0, UArg arg1);
#endif
static void MmwDemo_uartTxTask(UArg arg0, UArg arg1);
/*Pcount3DDemo_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    Pcount3DDemo_output_message_UARTpointCloud   *result,
    uint32_t        frameIdx,
    uint8_t         subFrameIdx,
    Pcount3DDemo_output_message_stats        *timingInfo
);*/
static int32_t Pcount3DDemo_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
);
static int32_t Pcount3DDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx);

static void Pcount3DDemo_initTask(UArg arg0, UArg arg1);
static void Pcount3DDemo_platformInit(Pcount3DDemo_platformCfg *config);

/* Mmwave control functions */
static void Pcount3DDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
static int32_t Pcount3DDemo_mmWaveCtrlStop (void);
static int32_t Pcount3DDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void Pcount3DDemo_sleep(void);

/* Edma related functions */
static void Pcount3DDemo_edmaInit(Pcount3DDemo_DataPathObj *obj, uint8_t instance);
static void Pcount3DDemo_edmaOpen(Pcount3DDemo_DataPathObj *obj, uint8_t instance);
static void Pcount3DDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo);
static void Pcount3DDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo);


/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_edmaInit(Pcount3DDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t errorCode;

    errorCode = EDMA_init(instance);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void Pcount3DDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gMmwMssMCB.dataPathObj.EDMA_errorInfo = *errorInfo;
    Pcount3DDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void Pcount3DDemo_EDMA_transferControllerErrorCallbackFxn
(
    EDMA_Handle handle,
    EDMA_transferControllerErrorInfo_t *errorInfo
)
{
    gMmwMssMCB.dataPathObj.EDMA_transferControllerErrorInfo = *errorInfo;
    Pcount3DDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj           Pointer to data path object
 *  @param[in] instance      EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_edmaOpen(Pcount3DDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t             errCode;
    EDMA_instanceInfo_t edmaInstanceInfo;
    EDMA_errorConfig_t  errorConfig;

    obj->edmaHandle = EDMA_open(instance, &errCode, &edmaInstanceInfo);

    if (obj->edmaHandle == NULL)
    {
        Pcount3DDemo_debugAssert (0);
        return;
    }

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = Pcount3DDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = Pcount3DDemo_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close EDMA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void Pcount3DDemo_edmaClose(Pcount3DDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      HWA driver init
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_hwaInit(Pcount3DDemo_DataPathObj *obj)
{
    /* Initialize the HWA */
    HWA_init();
}

/**
 *  @b Description
 *  @n
 *      Open HWA driver instance
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] socHandle    SOC driver handle
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_hwaOpen(Pcount3DDemo_DataPathObj *obj, SOC_Handle socHandle)
{
    int32_t             errCode;

    /* Open the HWA Instance */
    obj->hwaHandle = HWA_open(0, socHandle, &errCode);
    if (obj->hwaHandle == NULL)
    {
        //System_printf("Error: Unable to open the HWA Instance err:%d\n",errCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close HWA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void Pcount3DDemo_hwaClose(Pcount3DDemo_DataPathObj *obj)
{
    int32_t             errCode;

    /* Close the HWA Instance */
    errCode = HWA_close(obj->hwaHandle);
    if (errCode != 0)
    {
        Pcount3DDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 *
 *  @param[in] expression           Expression for evaluation
 *  @param[in] file                 C file that caused assertion
 *  @param[in] line                 Line number in C fine that caused assertion
 *
 */
void _Pcount3DDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to set the pending state of configuration.
 *
 *  @param[in] subFrameCfg Pointer to Sub-frame specific configuration
 *  @param[in] offset       Configuration structure offset that uniquely identifies the
 *                          configuration to set to the pending state.
 *
 *  @retval None
 */
static void Pcount3DDemo_setSubFramePendingState(Pcount3DDemo_SubFrameCfg *subFrameCfg, uint32_t offset)
{
    switch (offset)
    {
        case PCOUNT3DDEMO_ADCBUFCFG_OFFSET:
            subFrameCfg->isAdcBufCfgPending = 1;
        break;
        default:
            Pcount3DDemo_debugAssert(0);
        break;
    }
}


/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending static (non-dynamic) configuration
 *
 */
void Pcount3DDemo_resetStaticCfgPendingState(void)
{
    uint8_t indx;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.numSubFrames; indx++)
    {
        gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending = 0;
    }

}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in pending state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in pending state, else return 0
 */
uint8_t Pcount3DDemo_isAllCfgInPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.numSubFrames; indx++)
    {
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 1);
    }

    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in non-pending (cleared) state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in non-pending state, else return 0
 */
uint8_t Pcount3DDemo_isAllCfgInNonPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.numSubFrames; indx++)
    {
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 0);
    }

    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to apply configuration to specified sub-frame
 *
 *  @param[in] srcPtr Pointer to configuration
 *  @param[in] offset Offset of configuration within the parent structure
 *  @param[in] size   Size of configuration
 *  @param[in] subFrameNum Sub-frame Number (0 based) to apply to, broadcast to
 *                         all sub-frames if special code MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void Pcount3DDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == PCOUNT3DDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[indx] + offset), srcPtr, size);
            Pcount3DDemo_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[indx], offset);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[subFrameNum] + offset), srcPtr, size);
        Pcount3DDemo_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[subFrameNum], offset);
    }
}

/** @brief Transmits detection data over UART  
*
*   @param[in] uartHandle   UART driver handle
*   @param[in] result       Pointer to result from object detection DPC processing
*   @param[in] timingInfo   Pointer to timing information provided from core that runs data path
*/
void MmwDemo_uartTxTask(UArg arg0, UArg arg1)
/*static void Pcount3DDemo_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    Pcount3DDemo_output_message_UARTpointCloud   *result,
    uint32_t        frameIdx,
    uint8_t         subFrameIdx,
    Pcount3DDemo_output_message_stats        *timingInfo
)*/
{
    UART_Handle     uartHandle;
    Pcount3DDemo_output_message_header header;
    uint32_t        tlvIdx = 0;
    uint32_t        packetLen, subFrameIdx, frameIdx;
    uint16_t        *headerPtr;
    Pcount3DDemo_output_message_stats        *timingInfo;

    Pcount3DDemo_output_message_UARTpointCloud *objOut;
    uint32_t        sum, n, targetListLength=0, targetIndexLength=0, presenceIndLength=0;
    volatile uint32_t                        startTime;
    Pcount3DDemo_output_message_tl tl;
    
    /* Clear message header */
    memset((void *)&header, 0, sizeof(Pcount3DDemo_output_message_header));


    /* Header: */
    header.platform =  0xA6843;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.version =    MMWAVE_SDK_VERSION_BUILD |
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);


    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
		uint32_t numTargets, numIndices;
		uint8_t 	*tList;
		uint8_t 	*tIndex;
		
		
        Semaphore_pend(gMmwMssMCB.uartTxSemHandle, BIOS_WAIT_FOREVER);
		startTime 		=	Cycleprofiler_getTimeStamp();

        tlvIdx          =   0;
        uartHandle      =   gMmwMssMCB.loggingUartHandle;
        objOut          =   &(gMmwMssMCB.pointCloudToUart);
        subFrameIdx     =   gMmwMssMCB.currSubFrameIdx;
        timingInfo      =   &gMmwMssMCB.subFrameStats[subFrameIdx].outputStats;
        frameIdx        =   gMmwMssMCB.frameStatsFromDSP->frameStartIntCounter;

        packetLen = sizeof(Pcount3DDemo_output_message_header);
        header.chirpProcessingMargin        =   timingInfo->interChirpProcessingMargin;
        header.frameProcessingTimeInUsec    =   timingInfo->frameProcessingTimeInUsec;
        header.trackingProcessingTimeInUsec =   gMmwMssMCB.trackerProcessingTimeInUsec;
        header.uartSendingTimeInUsec        =   gMmwMssMCB.uartProcessingTimeInUsec; 
		numTargets 							=	gMmwMssMCB.numTargets;
		numIndices 							=	gMmwMssMCB.numIndices;
		tList 								=	(uint8_t*)gMmwMssMCB.trackerOutput.tList[gMmwMssMCB.trackerOutput.currentDescr];
		tIndex 								=	(uint8_t*)gMmwMssMCB.trackerOutput.tIndex[gMmwMssMCB.trackerOutput.currentDescr];
		
        if (objOut->header.length > 0)
        {
            packetLen += objOut->header.length;
            tlvIdx++;
        }
        if (numTargets > 0) 
        {
            targetListLength = sizeof(Pcount3DDemo_output_message_tl) + numTargets*sizeof(trackerProc_Target);
            packetLen += targetListLength;
            tlvIdx++;
        }
        if ((numIndices > 0) && (numTargets > 0))
        {
            targetIndexLength = sizeof(Pcount3DDemo_output_message_tl) + numIndices*sizeof(trackerProc_TargetIndex);
            packetLen += targetIndexLength;
            tlvIdx++;
        }
        if(gMmwMssMCB.presenceDetEnabled)
        {
            presenceIndLength = sizeof(Pcount3DDemo_output_message_tl) + sizeof(uint32_t);;
            packetLen += presenceIndLength;
            tlvIdx++;
        }

        header.numTLVs = tlvIdx;
        header.totalPacketLen   =   packetLen;
        header.frameNumber      =   frameIdx;
        header.subFrameNumber   =   subFrameIdx;
        header.checkSum         =   0;


        headerPtr               =   (uint16_t *)&header;
        for(n=0, sum = 0; n < sizeof(Pcount3DDemo_output_message_header)/sizeof(uint16_t); n++)
                                sum += *headerPtr++;
        header.checkSum         =   ~((sum >> 16) + (sum & 0xFFFF));

        /* Send packet header */
        UART_write (uartHandle,
                           (uint8_t*)&header,
                           sizeof(Pcount3DDemo_output_message_header));

        /* Send detected Objects */
        if (objOut->header.length > 0)
        {
            UART_write (uartHandle,
                               (uint8_t*)objOut,
                               objOut->header.length);
        }
        Task_sleep(1);
        /*Send Tracker information*/
        if (numTargets > 0) 
        {
            tl.type = MMWDEMO_OUTPUT_MSG_TARGET_LIST;
            tl.length = targetListLength;
            UART_write(uartHandle, (uint8_t*)&tl, sizeof(Pcount3DDemo_output_message_tl));
            UART_write(uartHandle, tList, targetListLength-sizeof(Pcount3DDemo_output_message_tl));
            //GPIO_toggle(gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO);
        }
        /*Send Tracker Index Information*/
        if ((numIndices > 0) && (numTargets > 0))
        {
            tl.type = MMWDEMO_OUTPUT_MSG_TARGET_INDEX;
            tl.length = targetIndexLength;
            UART_write(uartHandle, (uint8_t*)&tl, sizeof(Pcount3DDemo_output_message_tl));
            UART_write(uartHandle, tIndex, targetIndexLength-sizeof(Pcount3DDemo_output_message_tl));
        }
    
        /* Send Presence TLV if presence detect is enabled */
        if(gMmwMssMCB.presenceDetEnabled)
        {
            tl.type = MMWDEMO_OUTPUT_PRESENCE_IND;
            tl.length =  presenceIndLength;
 
            UART_write(uartHandle, (uint8_t*)&tl, sizeof(Pcount3DDemo_output_message_tl));
            UART_write(uartHandle, (uint8_t*)&(gMmwMssMCB.presenceInd), sizeof(uint32_t));
        }
	
        gMmwMssMCB.uartProcessingTimeInUsec	=	(Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
    }
}



/**************************************************************************
 ******************** Millimeter Wave Demo control path Functions *****************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to stop generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t Pcount3DDemo_mmWaveCtrlStop (void)
{
    int32_t                 errCode = 0;

    DebugP_log0("App: Issuing MMWave_stop\n");

    /* Stop the mmWave module: */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            Pcount3DDemo_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }
    }

    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            Pcount3DDemo_debugAssert (0);
        }
    }
}

/**************************************************************************
 ******************** Millimeter Wave Demo data path Functions *******************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
static int32_t Pcount3DDemo_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
)
{
    int32_t retVal = 0;

    DebugP_log3("Pcount3DDemo_DPM_ioctl_blocking: cmd = %d, *arg = %d, len = %d\n", cmd, *(uint8_t *)arg, argLen);
    retVal = DPM_ioctl(handle,
                     cmd,
                     arg,
                     argLen);

    if(retVal == 0)
    {
        /* Wait until ioctl completed */
        Semaphore_pend(gMmwMssMCB.DPMioctlSemHandle, BIOS_WAIT_FOREVER);
    }

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Perform Data path driver open 
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_dataPathOpen(void)
{
#ifdef PLATFORMES2
    gMmwMssMCB.adcBufHandle = MmwDemo_ADCBufOpen(gMmwMssMCB.socHandle);
#else
    gMmwMssMCB.adcBufHandle = MmwDemo_ADCBufOpen();
#endif
    if(gMmwMssMCB.adcBufHandle == NULL)
    {
        Pcount3DDemo_debugAssert(0);
    }
}



/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t Pcount3DDemo_dataPathConfig (void)
{
    int32_t                         errCode;
    MMWave_CtrlCfg                  *ptrCtrlCfg;
    Pcount3DDemo_DPC_ObjDet_CommonCfg *objDetCommonCfgDSP;
    Pcount3DDemo_SubFrameCfg             *subFrameCfg;
    int8_t                          subFrameIndx;
    MmwDemo_RFParserOutParams       RFparserOutParams;
    DPC_ObjectDetectionRangeHWA_PreStartCfg objDetPreStartR4fCfg;
    DPC_ObjectDetectionRangeHWA_StaticCfg *staticCfgR4F;
    DPC_ObjectDetection_PreStartCfg objDetPreStartDspCfg;
    DPC_ObjectDetectionRangeHWA_PreStartCommonCfg preStartCommonCfgHWA;
    uint8_t                        radarCubeFormat = DPIF_RADARCUBE_FORMAT_2;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMssMCB.cfg.ctrlCfg;

    objDetCommonCfgDSP = &gMmwMssMCB.objDetCommonCfg;
    staticCfgR4F = &objDetPreStartR4fCfg.staticCfg;

    /* Get RF frequency scale factor */
    gMmwMssMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMssMCB.socHandle, &errCode);
    if (errCode < 0)
    {
        System_printf ("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        Pcount3DDemo_debugAssert(0);
    }

    objDetCommonCfgDSP->numSubFrames =  MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);

    DEBUG(System_printf("App: Issuing Pre-start Common Config IOCTL to R4F\n");)

    /* DPC pre-start common config */

    DEBUG(System_printf("App: Issuing Pre-start Common Config IOCTL to DSP\n");)
    errCode = Pcount3DDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         objDetCommonCfgDSP,
                         sizeof (Pcount3DDemo_DPC_ObjDet_CommonCfg));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    preStartCommonCfgHWA.numSubFrames = objDetCommonCfgDSP->numSubFrames;
    errCode = Pcount3DDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &preStartCommonCfgHWA,
                         sizeof (preStartCommonCfgHWA));
    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the Pcount3DDemo_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for(subFrameIndx = objDetCommonCfgDSP->numSubFrames -1; subFrameIndx >= 0;
        subFrameIndx--)
    {
        subFrameCfg  = &gMmwMssMCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        DEBUG(System_printf("App: Calling MmwDemo_RFParser_parseConfig\n");)
        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                         &gMmwMssMCB.cfg.openCfg, ptrCtrlCfg,
                                         &subFrameCfg->adcBufCfg,
                                         gMmwMssMCB.rfFreqScaleFactor,
                                         0); 

        if (errCode != 0)
        {
            System_printf ("Error: MmwDemo_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }

        /* The following code is to enable processing for number of doppler chirps that are
         * less than 16 (the minimal numDopplerBins supported in doppler DPU DSP).
         * In this case, interpolate to detect better with CFAR tuning. E.g. a 2 -pt FFT will
         * be problematic in terms of distinguishing direction of motion */
        if (RFparserOutParams.numDopplerChirps <= 8)
        {
            RFparserOutParams.dopplerStep = RFparserOutParams.dopplerStep / (16 / RFparserOutParams.numDopplerBins);
            RFparserOutParams.numDopplerBins = 16;
        }

        /* Workaround for range DPU limitation for FFT size 1024 and 12 virtual antennas case*/
        if ((RFparserOutParams.numVirtualAntennas == 12) && (RFparserOutParams.numRangeBins == 1024))
        {
            RFparserOutParams.numRangeBins = 1022;
        }

        subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;

        DEBUG(System_printf("App: Calling MmwDemo_ADCBufConfig\n");)
        errCode = MmwDemo_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 subFrameCfg->numChirpsPerChirpEvent,
                                 subFrameCfg->adcBufChanDataSize,
                                 &subFrameCfg->adcBufCfg,
                                 &staticCfgR4F->ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", errCode);
            Pcount3DDemo_debugAssert (0);
        }

        if (errCode < 0)
        {
            goto exit;
        }

        /* DPC pre-start config R4F HWA*/
        {

            /***********************************************************************
              Pre-start preparation for objdetR4FHWA
             ***********************************************************************/
            objDetPreStartR4fCfg.subFrameNum = subFrameIndx;

            /* Fill static configuration */
            staticCfgR4F->ADCBufData.data = (void *)SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS;
            staticCfgR4F->ADCBufData.dataProperty.adcBits = 2; /* 16-bit */

            /* only complex format supported */
            Pcount3DDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0);

            if (subFrameCfg->adcBufCfg.iqSwapSel == 1)
            {
                staticCfgR4F->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_IMRE;
            }
            else
            {
                staticCfgR4F->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_REIM;
            }
            if (subFrameCfg->adcBufCfg.chInterleave == 0)
            {
                staticCfgR4F->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_INTERLEAVE_MODE;
            }
            else
            {
                staticCfgR4F->ADCBufData.dataProperty.interleave    =   DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            }
            staticCfgR4F->radarCubeFormat                           =   radarCubeFormat;

            staticCfgR4F->ADCBufData.dataProperty.numAdcSamples     =   RFparserOutParams.numAdcSamples;
            staticCfgR4F->ADCBufData.dataProperty.numChirpsPerChirpEvent    =   RFparserOutParams.numChirpsPerChirpEvent;
            staticCfgR4F->ADCBufData.dataProperty.numRxAntennas     =   RFparserOutParams.numRxAntennas;
            staticCfgR4F->ADCBufData.dataSize                       =   RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
            staticCfgR4F->numChirpsPerFrame                         =   RFparserOutParams.numChirpsPerFrame;
            staticCfgR4F->numDopplerChirps                          =   RFparserOutParams.numDopplerChirps;
            staticCfgR4F->numRangeBins                              =   RFparserOutParams.numRangeBins;
            staticCfgR4F->numTxAntennas                             =   RFparserOutParams.numTxAntennas;
            staticCfgR4F->numVirtualAntennas                        =   RFparserOutParams.numVirtualAntennas;

            /* Fill dynamic configuration for the sub-frame */
            objDetPreStartR4fCfg.dynCfg = subFrameCfg->objDetDynCfg.r4fDynCfg;

            DebugP_log1("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\n", subFrameIndx);

            /* send pre-start config to R4F chain, using blocking call here */
            errCode = Pcount3DDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartR4fCfg,
                                 sizeof (DPC_ObjectDetectionRangeHWA_PreStartCfg));
            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
            DebugP_log0("App: DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG is processed \n");
        }

        /* DPC pre-start config DSP*/
        DEBUG(System_printf("App: pre-start config for DSP \n");)
        {
            DPU_radarProcessConfig_t *pParam_s;

            /***********************************************************************
              Pre-start preparation for objdetdsp
             ***********************************************************************/
            /* Reset preStart config */
            memset((void *)&objDetPreStartDspCfg, 0, sizeof(DPC_ObjectDetection_PreStartCfg));

            /* DPC configuration */
            objDetPreStartDspCfg.subFrameNum    =   subFrameIndx;
            objDetPreStartDspCfg.dynCfg         =   subFrameCfg->objDetDynCfg.dspDynCfg;
            pParam_s                            =   &objDetPreStartDspCfg.dynCfg.caponChainCfg;
            memcpy((void *)pParam_s, &(gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg), sizeof(DPU_radarProcessConfig_t));

            pParam_s->numRangeBins              =   RFparserOutParams.numRangeBins;
            pParam_s->numTxAntenna              =   RFparserOutParams.numTxAntennas;
            pParam_s->numPhyRxAntenna           =   RFparserOutParams.numRxAntennas;
            pParam_s->numAntenna                =   pParam_s->numTxAntenna * pParam_s->numPhyRxAntenna;
            if (pParam_s->numTxAntenna  >   1)
                pParam_s->mimoModeFlag =    1;
            else
                pParam_s->mimoModeFlag  =   0;
            pParam_s->numAdcSamplePerChirp          =   RFparserOutParams.numAdcSamples;
            pParam_s->dynamicCfarConfig.rangeRes    =   RFparserOutParams.rangeStep;
            pParam_s->staticCfarConfig.rangeRes     =   pParam_s->dynamicCfarConfig.rangeRes;
            pParam_s->numChirpPerFrame              =   RFparserOutParams.numDopplerChirps ;
            gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg.numChirpPerFrame = RFparserOutParams.numDopplerChirps ;
            pParam_s->framePeriod                   =   RFparserOutParams.framePeriod;
            pParam_s->chirpInterval                 =   RFparserOutParams.chirpInterval;
            pParam_s->bandwidth                     =   RFparserOutParams.bandwidth;
            pParam_s->centerFreq                    =   RFparserOutParams.centerFreq;
            gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg.chirpInterval = RFparserOutParams.chirpInterval;
            gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg.framePeriod   = RFparserOutParams.framePeriod;
            gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg.bandwidth     = RFparserOutParams.bandwidth;
            gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg.dspDynCfg.caponChainCfg.centerFreq    = RFparserOutParams.centerFreq;

            pParam_s->dynamicCfarConfig.dopplerRes  =   RFparserOutParams.dopplerStep;
            pParam_s->dynamicCfarConfig.cfarType    =   RADARDEMO_DETECTIONCFAR_RA_CASOCFAR;  //hardcoded, only method can be used in this chain
            pParam_s->dynamicCfarConfig.inputType   =   RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP;  //hardcoded, only method can be used in this chain
            pParam_s->staticCfarConfig.cfarType     =   RADARDEMO_DETECTIONCFAR_RA_CASOCFARV2;  //hardcoded, only method can be used in this chain
            pParam_s->staticCfarConfig.inputType    =   RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP;  //hardcoded, only method can be used in this chain
            pParam_s->maxNumDetObj                  =   (uint16_t)MAX_RESOLVED_OBJECTS_PER_FRAME;

            /* The L3 memory and radarCube memory usage are reported and saved in @ref Pcount3DDemo_DPC_ObjectDetection_reportFxn.
               The memory information is configured here and passed to objdetdsp chain.
             */
            objDetPreStartDspCfg.dynCfg.radarCubeFormat = radarCubeFormat;
            if(gMmwMssMCB.dataPathObj.radarCubeMem.addr != 0)
            {
                /* Update DPC radar cube configuration */
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr = gMmwMssMCB.dataPathObj.radarCubeMem.addr;
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.size = gMmwMssMCB.dataPathObj.radarCubeMem.size;

                /* Update DPC L3 RAM configuration */
                objDetPreStartDspCfg.shareMemCfg.L3Ram.addr = (void *)((uint32_t)(gMmwMssMCB.dataPathObj.radarCubeMem.addr) +
                                                                   gMmwMssMCB.dataPathObj.memUsage.L3RamUsage);
                objDetPreStartDspCfg.shareMemCfg.L3Ram.size =gMmwMssMCB.dataPathObj.memUsage.L3RamTotal - gMmwMssMCB.dataPathObj.memUsage.L3RamUsage;

                /* Convert address for DSP core */
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
                DebugP_assert ((uint32_t)objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr != SOC_TRANSLATEADDR_INVALID);

                objDetPreStartDspCfg.shareMemCfg.L3Ram.addr = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.shareMemCfg.L3Ram.addr,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
                DebugP_assert ((uint32_t)objDetPreStartDspCfg.shareMemCfg.L3Ram.addr != SOC_TRANSLATEADDR_INVALID);

                /* Enable shared memory configuration */
                objDetPreStartDspCfg.shareMemCfg.shareMemEnable = true;
            }

            /* send pre-start config */
            DEBUG(System_printf("App: pre-start config ready to be set to DSP \n");)
            errCode = Pcount3DDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartDspCfg,
                                 sizeof (DPC_ObjectDetection_PreStartCfg));
            DebugP_log0("App: DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG is processed \n");

            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
        }


    }
exit:
    return errCode;
}
#ifdef TRACKERPROC_EN
/**
 *  @b Description
 *  @n
 *      The function is used to configure the tracker DPU.
 *
 *  @retval
 *      0 if no error, error code otherwise.
 */
int32_t MmwDemo_trackerConfig (void)
{
    int32_t    errCode;

    DebugP_log0("App: Issuing Tracker Static Config IOCTL\n");

    /* DPC pre-start common config */
    //errCode =  DPM_ioctl (dataPathObj->objDetDpmHandle,
    errCode =  DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG,
                         &(gMmwMssMCB.trackerCfg),
                         sizeof (DPC_ObjectDetection_TrackerConfig));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__TRACKER_STATIC_CFG [Error:%d]\n", errCode);
        goto exit;
    }

exit:
    return errCode;
}
#endif

/**
 *  @b Description
 *  @n
 *      This function is used to start data path to handle chirps from front end.
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_dataPathStart (void)
{
    int32_t retVal;
    int32_t pointCloudSize;

    DebugP_log0("App: Issuing DPM_start\n");

    pointCloudSize                          =   MAX_RESOLVED_OBJECTS_PER_FRAME * sizeof(DPIF_PointCloudSpherical);
    gMmwMssMCB.pointCloudFromDSP            =   (DPIF_PointCloudSpherical *)MemoryP_ctrlAlloc(pointCloudSize, sizeof(float));
    gMmwMssMCB.pointCloudSideInfoFromDSP    =   (DPIF_PointCloudSideInfo *)MemoryP_ctrlAlloc(MAX_RESOLVED_OBJECTS_PER_FRAME * sizeof(DPIF_PointCloudSideInfo), sizeof(int16_t));

    /* Start the DPM Profile: */
    if ((retVal = DPM_start(gMmwMssMCB.objDetDpmHandle)) < 0)
    {
        /* Error: Unable to start the profile */
        System_printf("Error: Unable to start the DPM [Error: %d]\n", retVal);
        Pcount3DDemo_debugAssert(0);
    }

    /* Wait until start completed */
    Semaphore_pend(gMmwMssMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);

    DebugP_log0("App: DPM_start Done (post Semaphore_pend on reportFxn reporting start)\n");
}

/**
 *  @b Description
 *  @n
 *      This function is used to stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_dataPathStop (void)
{
    int32_t retVal;
    int32_t pointCloudSize;

    DebugP_log0("App: Issuing DPM_stop\n");

    pointCloudSize                      =   MAX_RESOLVED_OBJECTS_PER_FRAME * sizeof(Pcount3DDemo_output_message_point);
    MemoryP_ctrlFree(gMmwMssMCB.pointCloudFromDSP, pointCloudSize);

    retVal = DPM_stop (gMmwMssMCB.objDetDpmHandle);
    if (retVal < 0)
    {
        System_printf ("DPM_stop failed[Error code %d]\n", retVal);
        Pcount3DDemo_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always return 0
 */
static int32_t Pcount3DDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    Pcount3DDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    Pcount3DDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    Pcount3DDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }
                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    gMmwMssMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    gMmwMssMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gMmwMssMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMssMCB.stats.sensorStopped++;
                    DebugP_log0("App: BSS stop (frame end) received\n");

                    Pcount3DDemo_dataPathStop();
                    break;
                }
                default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    /* link reports protocol error in the async report from BSS */
                    Pcount3DDemo_debugAssert(0);
                    break;
                }            
                case RL_MMWL_AE_INTERNALERR_REPORT:
                {
                    /* link reports internal error during BSS communication */
                    Pcount3DDemo_debugAssert(0);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{
    /* Only errors are logged on the console: */
    if ((errCode != 0) )
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("App: DPM Report IOCTL, command = %d\n", arg0);

            if (arg0 == DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG)
            {
                DPC_ObjectDetectionRangeHWA_PreStartCfg *cfg;
                DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage *memUsage;

                cfg = (DPC_ObjectDetectionRangeHWA_PreStartCfg*)arg1;

                /* Save radar cube memory information, it will be passed to objdetdsp chain for inter-frame processing */
                memcpy((void *)&gMmwMssMCB.dataPathObj.radarCubeMem, (void *)&cfg->radarCubeMem,
                    sizeof(DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem));

                /* Get memory usage and print the usage */
                memUsage = &cfg->memUsage;

                /* Save memory usage, it will be passed to objdetdsp chain for inter-frame processing */
                memcpy((void *)&gMmwMssMCB.dataPathObj.memUsage, (void *)memUsage,
                    sizeof(DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage));

                System_printf("============ Heap Memory Stats ============\n");
                System_printf("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n", "System Heap(TCMB)",
                              memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                              memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                              memUsage->SystemHeapDPCUsed);

                System_printf("%20s %12d %12d %12d\n", "L3",
                              memUsage->L3RamTotal,
                              memUsage->L3RamUsage,
                              memUsage->L3RamTotal - memUsage->L3RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(TCMB)",
                              memUsage->CoreLocalRamTotal,
                              memUsage->CoreLocalRamUsage,
                              memUsage->CoreLocalRamTotal - memUsage->CoreLocalRamUsage);
            }

            switch(arg0)
            {
                /* The following ioctls take longer time to finish. It causes DPM to queue IOCTL requests on DSS before
                 * they are handled. However DPM has limited pipe queues, hence adding sync points in demo to avoid 
                 * sending too many such ioctls to DSS at a time.
                 * The semaphore blocks CLI task to wait for the response from DSS before sending the next ioctl.
                 */
                case DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG:
                case DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG:
                case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
#ifdef TRACKERPROC_EN
                case DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG:
#endif
                /* The following ioctl returns memory information needs to be used in configuration follows the ioctl.
                 * The semaphore blocks CLI task to wiat for the response from DPC before further execution.
                 */
                case DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG:
                    System_printf("DPM IOCTL report msg = %d\n", arg0 );
                    Semaphore_post(gMmwMssMCB.DPMioctlSemHandle);
                    break;
                default:
                    break;
            }
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Started\n");
            Semaphore_post(gMmwMssMCB.DPMstartSemHandle);
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * datapath has finished frame processing, results are reported
             *****************************************************************/
            DPM_Buffer*     ptrResult;

            /* Get the result: */
            ptrResult = (DPM_Buffer*)arg0;

            Pcount3DDemo_handleObjectDetResult(ptrResult);
            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            CLI_write("Obj Det DPC Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Stopped\n");
            Semaphore_post(gMmwMssMCB.DPMstopSemHandle);
            break;
        }
        case DPM_Report_DPC_INFO:
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /* Currently objDetDsp does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t Pcount3DDemo_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return(nextIndx);
}

/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
/*
static uint8_t Pcount3DDemo_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return(prevIndx);
}
*/
/**
 *  @b Description
 *  @n
 *      Processes any pending dynamic configuration commands for the specified
 *      sub-frame by fanning out to the respective DPUs using IOCTL interface, and
 *      resets (clears) the pending state after processing.
 *
 *  @param[in] subFrameIndx Sub-frame index of desired sub-frame to process
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t Pcount3DDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx)
{
    int32_t retVal =0;

    return(retVal);
}


/**
 *  @b Description
 *  @n
 *      Function to handle frame processing results from DPC
 *
 *  @param[in] ptrResult      Pointer to DPC result
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_handleObjectDetResult
(
    DPM_Buffer  *ptrResult
)
{
    int32_t     retVal;

    int32_t     pntIdx;
    DPC_ObjectDetection_ExecuteResultExportedInfo exportInfo;
    DPC_ObjectDetection_ExecuteResult        *dpcResults;
    volatile uint32_t                        startTime;
    uint8_t                                  nextSubFrameIdx;
    uint8_t                                  numSubFrames;
    uint8_t                                  currSubFrameIdx;
    Pcount3DDemo_SubFrameStats               *currSubFrameStats;
    float *                                  heatmapBuff;
    radarProcessOutput                       * outputFromDSP;

    /*****************************************************************
     * datapath has finished frame processing, results are reported
     *****************************************************************/

    /* Validate DPC results buffer */
    DebugP_assert (ptrResult->size[0] == sizeof(DPC_ObjectDetection_ExecuteResult));

    /* Translate the address: */
    dpcResults = (DPC_ObjectDetection_ExecuteResult *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[0],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)dpcResults != SOC_TRANSLATEADDR_INVALID);

    /* Translate the address: */
    gMmwMssMCB.frameStatsFromDSP = (DPC_ObjectDetection_Stats *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[1],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)(gMmwMssMCB.frameStatsFromDSP) != SOC_TRANSLATEADDR_INVALID);

    outputFromDSP       =   &(dpcResults->objOut);

    heatmapBuff         =   outputFromDSP->heatMapOut.data;
    heatmapBuff         =   (float *)SOC_translateAddress((uint32_t)heatmapBuff,
                                                                                      SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                                                      &retVal);
    //DebugP_log2("Pcount3DDemo_handleObjectDetResult: heatmap = (float *)0x%x, size = %d \n", (uint32_t)heatmapBuff, outputFromDSP->heatMapOut.dataSize );

    gMmwMssMCB.heatMapOutFromDSP.dataSize   =    outputFromDSP->heatMapOut.dataSize;
    gMmwMssMCB.heatMapOutFromDSP.data       =    outputFromDSP->heatMapOut.data;

    //copy to the format for output, and to future tracker
    gMmwMssMCB.pointCloudToUart.header.length               =   sizeof(Pcount3DDemo_output_message_tl) + sizeof(Pcount3DDemo_output_message_point_unit) + sizeof(Pcount3DDemo_output_message_UARTpoint) * outputFromDSP->pointCloudOut.object_count;
    if ( outputFromDSP->pointCloudOut.object_count == 0)
        gMmwMssMCB.pointCloudToUart.header.length           =   0;
    gMmwMssMCB.pointCloudToUart.header.type                 =   MMWDEMO_OUTPUT_MSG_POINT_CLOUD;
    gMmwMssMCB.pointCloudToUart.pointUint.azimuthUnit       =   0.01f;
    gMmwMssMCB.pointCloudToUart.pointUint.elevationUnit     =   0.01f;
    gMmwMssMCB.pointCloudToUart.pointUint.rangeUnit         =   0.00025f;
    gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit       =   0.00028f;
    gMmwMssMCB.pointCloudToUart.pointUint.snrUint           =   0.04f;
    gMmwMssMCB.numDetectedPoints                            =   outputFromDSP->pointCloudOut.object_count;
    for (pntIdx = 0; pntIdx < (int32_t)outputFromDSP->pointCloudOut.object_count; pntIdx++ )
    {
        //output to host
        gMmwMssMCB.pointCloudToUart.point[pntIdx].azimuth   =   (int8_t)round((outputFromDSP->pointCloudOut.pointCloud[pntIdx].azimuthAngle - gMmwMssMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorAzimuthTilt ) / gMmwMssMCB.pointCloudToUart.pointUint.azimuthUnit);
        //gMmwMssMCB.pointCloudToUart.point[pntIdx].elevation =   (int8_t)round((outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle - gMmwMssMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorElevationTilt)/ gMmwMssMCB.pointCloudToUart.pointUint.elevationUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].elevation =   (int8_t)round((outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle)/ gMmwMssMCB.pointCloudToUart.pointUint.elevationUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].range     =   (uint16_t)round(outputFromDSP->pointCloudOut.pointCloud[pntIdx].range / gMmwMssMCB.pointCloudToUart.pointUint.rangeUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].doppler   =   (int16_t)round(outputFromDSP->pointCloudOut.pointCloud[pntIdx].velocity / gMmwMssMCB.pointCloudToUart.pointUint.dopplerUnit);
        gMmwMssMCB.pointCloudToUart.point[pntIdx].snr       =   (uint16_t)round((float)outputFromDSP->pointCloudOut.snr[pntIdx].snr * 0.125f / gMmwMssMCB.pointCloudToUart.pointUint.snrUint);

        //future tracker input
        gMmwMssMCB.pointCloudFromDSP[pntIdx].azimuthAngle        =   outputFromDSP->pointCloudOut.pointCloud[pntIdx].azimuthAngle - gMmwMssMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorAzimuthTilt;
        //gMmwMssMCB.pointCloudFromDSP[pntIdx].elevAngle      =   outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle - gMmwMssMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorElevationTilt;
        gMmwMssMCB.pointCloudFromDSP[pntIdx].elevAngle      =   outputFromDSP->pointCloudOut.pointCloud[pntIdx].elevAngle;
        gMmwMssMCB.pointCloudFromDSP[pntIdx].range          =   outputFromDSP->pointCloudOut.pointCloud[pntIdx].range;
        gMmwMssMCB.pointCloudFromDSP[pntIdx].velocity        =   outputFromDSP->pointCloudOut.pointCloud[pntIdx].velocity;
        gMmwMssMCB.pointCloudSideInfoFromDSP[pntIdx].snr            =   (float)outputFromDSP->pointCloudOut.snr[pntIdx].snr * 0.125f;
    }

    numSubFrames                =   gMmwMssMCB.objDetCommonCfg.numSubFrames;
    currSubFrameIdx             =   dpcResults->subFrameIdx;
    currSubFrameStats           =   &gMmwMssMCB.subFrameStats[currSubFrameIdx];
    gMmwMssMCB.currSubFrameIdx  =   currSubFrameIdx;

    /*****************************************************************
     * Transmit results
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();

    currSubFrameStats->outputStats.interChirpProcessingMargin   =   gMmwMssMCB.subFrameStats[currSubFrameIdx].outputStats.interChirpProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingTime     =   gMmwMssMCB.frameStatsFromDSP->interFrameExecTimeInUsec;
    currSubFrameStats->outputStats.interFrameCPULoad            =   (uint32_t) (100.f * (float) gMmwMssMCB.frameStatsFromDSP->interFrameExecTimeInUsec / gMmwMssMCB.subFrameCfg[currSubFrameIdx].objDetDynCfg.dspDynCfg.caponChainCfg.framePeriod);
    currSubFrameStats->outputStats.frameProcessingTimeInUsec    =   gMmwMssMCB.frameStatsFromDSP->activeFrameProcTimeInUsec;
    currSubFrameStats->outputStats.activeFrameCPULoad           =   (uint32_t) (100.f * (float) gMmwMssMCB.frameStatsFromDSP->activeFrameProcTimeInUsec /
                                                                    (gMmwMssMCB.subFrameCfg[currSubFrameIdx].objDetDynCfg.dspDynCfg.caponChainCfg.framePeriod - gMmwMssMCB.subFrameCfg[currSubFrameIdx].objDetDynCfg.dspDynCfg.caponChainCfg.chirpInterval
                                                                     * (float)gMmwMssMCB.subFrameCfg[currSubFrameIdx].objDetDynCfg.dspDynCfg.caponChainCfg.numChirpPerFrame) ) ;

    /*****************************************************************
     * Send notification to data path after results are handled --
     * all data are local now, release the memory to DSP
     *****************************************************************/
    /* Indicate result consumed and end of frame/sub-frame processing */
    exportInfo.subFrameIdx = currSubFrameIdx;
    retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                         &exportInfo,
                         sizeof (DPC_ObjectDetection_ExecuteResultExportedInfo));
    if (retVal < 0) {
        System_printf ("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                       retVal);
        Pcount3DDemo_debugAssert (0);
    }
    
    /* Run the tracker DPU */
 #ifdef TRACKERPROC_EN
    /* Run the tracker DPU*/

    Semaphore_post(gMmwMssMCB.trackerDPUSemHandle);
 #endif

    /* Transmit processing results for the frame */
    Semaphore_post(gMmwMssMCB.uartTxSemHandle);
    
    /*
    Pcount3DDemo_transmitProcessedOutput(gMmwMssMCB.loggingUartHandle,
                                         &(gMmwMssMCB.pointCloudToUart),
                                         frameStatsDSS->frameStartIntCounter,
                                         dpcResults->subFrameIdx,
                                         &currSubFrameStats->outputStats);*/

    /* Update current frame transmit time */
    currSubFrameStats->outputStats.transmitOutputTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ; /* In micro seconds */


    /*****************************************************************
     * Handle dynamic pending configuration
     * For non-advanced frame case:
     *   process all pending dynamic config commands.
     * For advanced-frame case:
     *  Process next sub-frame related pending dynamic config commands.
     *  If the next sub-frame was the first sub-frame of the frame,
     *  then process common (sub-frame independent) pending dynamic config
     *  commands.
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();

    nextSubFrameIdx = Pcount3DDemo_getNextSubFrameIndx(currSubFrameIdx,   numSubFrames);
    retVal = Pcount3DDemo_processPendingDynamicCfgCommands(nextSubFrameIdx);
    if (retVal != 0)
    {
        System_printf ("Error: Executing Pending Dynamic Configuration Commands [Error code %d]\n",
                       retVal);
        Pcount3DDemo_debugAssert (0);
    }
    currSubFrameStats->pendingConfigProcTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;

    /*****************************************************************
     * Prepare for subFrame switch
     *****************************************************************/
    if(numSubFrames > 1)
    {
        Pcount3DDemo_SubFrameCfg  *nextSubFrameCfg;
        uint16_t dummyRxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

        startTime = Cycleprofiler_getTimeStamp();

        nextSubFrameCfg = &gMmwMssMCB.subFrameCfg[nextSubFrameIdx];

        /* Configure ADC for next sub-frame */
        retVal = MmwDemo_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 nextSubFrameCfg->numChirpsPerChirpEvent,
                                 nextSubFrameCfg->adcBufChanDataSize,
                                 &nextSubFrameCfg->adcBufCfg,
                                 &dummyRxChanOffset[0]);
        if(retVal < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", retVal);
            Pcount3DDemo_debugAssert (0);
        }

        currSubFrameStats->subFramePreparationTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
    }
    else
    {
        currSubFrameStats->subFramePreparationTime = 0;
    }


}


#ifdef TRACKERPROC_EN
void MmwDemo_trackerDPUTask(UArg arg0, UArg arg1)
{
    volatile uint32_t           startTime;
    int32_t                     retVal;
    DPU_TrackerProc_OutParams   outTrackerProc;
    //DPC_TrackerResults        trackerResults;
    //int32_t errCode;

    while(1)
    {
        Semaphore_pend(gMmwMssMCB.trackerDPUSemHandle, BIOS_WAIT_FOREVER);

        startTime = Cycleprofiler_getTimeStamp();
        retVal = DPU_TrackerProc_process(gObjDetObj->dpuTrackerObj, gMmwMssMCB.numDetectedPoints,
                    gMmwMssMCB.pointCloudFromDSP, gMmwMssMCB.pointCloudSideInfoFromDSP, &outTrackerProc);
        DebugP_assert(retVal == 0);

        gMmwMssMCB.trackerOutput.currentDescr                                           = outTrackerProc.currentTargetDesc;
        gMmwMssMCB.numTargets                                                           = outTrackerProc.numTargets[outTrackerProc.currentTargetDesc];
        gMmwMssMCB.trackerOutput.tList[gMmwMssMCB.trackerOutput.currentDescr]           = ((trackerProcObjType*)gObjDetObj->dpuTrackerObj)->targetDescrHandle->tList[outTrackerProc.currentTargetDesc];
        gMmwMssMCB.numIndices                                                           = outTrackerProc.numIndices[outTrackerProc.currentTargetDesc];
        gMmwMssMCB.trackerOutput.tIndex[gMmwMssMCB.trackerOutput.currentDescr]          = ((trackerProcObjType*)gObjDetObj->dpuTrackerObj)->targetDescrHandle->tIndex[outTrackerProc.currentTargetDesc];
        gMmwMssMCB.presenceInd                                                          = outTrackerProc.presenceInd[outTrackerProc.currentTargetDesc];   
        gMmwMssMCB.trackerProcessingTimeInUsec                                          = (float)(Cycleprofiler_getTimeStamp() - startTime)/(float)R4F_CLOCK_MHZ;
    }
}
#endif

/**
 *  @b Description
 *  @n
 *      DPM Execution Task which executes the DPM Instance which manages the
 *      HL Profiles executing on the MSS.
 *
 *  @retval
 *      Not Applicable.
 */
static void mmwDemo_mssDPMTask(UArg arg0, UArg arg1)
{
    int32_t     errCode;
    DPM_Buffer  result;

    while (1)
    {
        /* Execute the DPM module: */
        errCode = DPM_execute (gMmwMssMCB.objDetDpmHandle, &result);
        if (errCode < 0)
        {
            System_printf ("Error: DPM execution failed [Error code %d]\n", errCode);
        }
    }
}


/**************************************************************************
 ******************** Millimeter Wave Demo sensor management Functions **********
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization. 
 *      User need to fill gMmwMssMCB.cfg.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen     If true then issues MMwave_open
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Pcount3DDemo_openSensor(bool isFirstTimeOpen)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {

        /*  Open mmWave module, this is only done once */
        /* Setup the calibration frequency*/
        gMmwMssMCB.cfg.openCfg.freqLimitLow = 600U;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = 640U;

        /* start/stop async events */
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent  = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;

#ifdef PLATFORMES2
        /* calibration monitoring base time unit
         * setting it to one frame duration as the demo doesnt support any
         * monitoring related functionality
         */
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit            = 1;
#endif


        /* Open the mmWave module: */
        if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            System_printf ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
            return -1;
        }
#if 0 // seems to be LVDS related only
        /*Set up HSI clock*/
        if(MmwDemo_mssSetHsiClk() < 0)
        {
            System_printf ("Error: MmwDemo_mssSetHsiClk failed.\n");
            return -1;
        }
#endif
        /* Open the datapath modules that runs on MSS */
        Pcount3DDemo_dataPathOpen();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      MMW demo helper Function to configure sensor. User need to fill gMmwMssMCB.cfg.ctrlCfg and
 *      add profiles/chirp to mmWave before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Pcount3DDemo_configSensor(void)
{
    int32_t     errCode = 0;

    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                        mmWaveErrorCode, subsysErrorCode);
    }
    else
    {
        errCode = Pcount3DDemo_dataPathConfig();
        if (errCode < 0)
            goto exit;
#ifdef TRACKERPROC_EN
        errCode = MmwDemo_trackerConfig();
        if (errCode < 0)
            goto exit;
#endif
    }

exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t Pcount3DDemo_startSensor(void)
{
    int32_t     errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    /*****************************************************************************
     * Data path :: start data path first - this will pend for DPC to ack
     *****************************************************************************/
    Pcount3DDemo_dataPathStart();

    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = false;//true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = false;//true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    DebugP_log0("App: MMWave_start Issued\n");

    System_printf("Starting Sensor (issuing MMWave_start)\n");

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
        return -1;
    }

    /*****************************************************************************
     * The sensor has been started successfully. Switch on the LED 
     *****************************************************************************/
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 1U);

    gMmwMssMCB.sensorStartCount++;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void Pcount3DDemo_sensorStopEpilog(void)
{
    Task_Stat stat;
    Hwi_StackInfo stackInfo;
    Bool stackOverflow;

    /* Print task statistics, note data path has completely stopped due to
     * end of frame, so we can do non-real time processing like prints on
     * console */
    System_printf("Data Path Stopped (last frame processing done)\n");

    System_printf("============================================\n");
    System_printf("MSS Task Stack Usage (Note: Task Stack Usage) ==========\n");
    System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");


    Task_stat(gMmwMssMCB.taskHandles.initTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "Init",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.mmwaveCtrl, &stat);
    System_printf("%20s %12d %12d %12d\n", "Mmwave Control",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.objDetDpmTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    stackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (stackOverflow == TRUE)
    {
        System_printf("HWI Stack overflowed\n");
        Pcount3DDemo_debugAssert(0);
    }
    else
    {
        System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize, stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
}


/**
 *  @b Description
 *  @n
 *      Stops the RF and datapath for the sensor. Blocks until both operation are completed.
 *      Prints epilog at the end.
 *
 *  @retval  None
 */
void Pcount3DDemo_stopSensor(void)
{
    /* Stop sensor RF , data path will be stopped after RF stop is completed */
    Pcount3DDemo_mmWaveCtrlStop();

    /* Wait until DPM_stop is completed */
    Semaphore_pend(gMmwMssMCB.DPMstopSemHandle, BIOS_WAIT_FOREVER);



    /* Print epilog */
    Pcount3DDemo_sensorStopEpilog();

    /* The sensor has been stopped successfully. Switch off the LED */
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

    gMmwMssMCB.sensorStopCount++;

    /* print for user */
    System_printf("Sensor has been stopped: startCount: %d stopCount %d\n",
                  gMmwMssMCB.sensorStartCount,gMmwMssMCB.sensorStopCount);
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during inter-frame
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void Pcount3DDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwMssMCB.subFrameStats[subFrameIndx].outputStats.interFrameCPULoad = Load_getCPULoad();
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of inter-frame/inter-sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during active frame (chirping)
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void Pcount3DDemo_DPC_ObjectDetection_processInterFrameCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwMssMCB.subFrameStats[subFrameIndx].outputStats.activeFrameCPULoad = Load_getCPULoad();
}

/**************************************************************************
 ******************** Millimeter Wave Demo init Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Platform specific hardware initialization.
 *
 *  @param[in]  config     Platform initialization configuraiton
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_platformInit(Pcount3DDemo_platformCfg *config)
{

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN5_PADBE, SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);    
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINN4_PADBD, SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINF14_PADAJ, SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX);

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWR68XX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR68XX_PINK13_PADAZ, SOC_XWR68XX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the GPIO:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    config->SensorStatusGPIO    = SOC_XWR68XX_GPIO_2;

    /* Initialize the DEMO configuration: */
    config->sysClockFrequency   = MSS_SYS_VCLK;
    config->loggingBaudRate     = 921600 * UARTOVERRATE;
    config->commandBaudRate     = 115200;

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (config->SensorStatusGPIO, GPIO_CFG_OUTPUT);
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    DPM_InitCfg         dpmInitCfg;
    DMA_Params          dmaParams;
    DMA_Handle          dmaHandle;

    DPC_ObjectDetectionRangeHWA_InitParams objDetInitParams;
//    int32_t             i;

    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Initialize the UART */
    UART_init();

    /* Initialize the DMA for UART */
    DMA_init ();

    /* Open the DMA Instance */
    DMA_Params_init(&dmaParams);
    dmaHandle = DMA_open(0, &dmaParams, &errCode);
    if (dmaHandle == NULL)
    {
        printf ("Error: Unable to open the DMA Instance [Error code %d]\n", errCode);
        return;
    }

    /* Initialize the GPIO */
    GPIO_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Platform specific configuration */
    Pcount3DDemo_platformInit(&gMmwMssMCB.cfg.platformCfg);

    /*****************************************************************************
     * Open the mmWave SDK components:
     *****************************************************************************/
    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        Pcount3DDemo_debugAssert (0);
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    uartParams.dmaHandle      = dmaHandle;
    uartParams.txDMAChannel   = UART_DMA_TX_CHANNEL;  
    uartParams.rxDMAChannel   = UART_DMA_RX_CHANNEL;  

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Logging UART Instance\n");
        Pcount3DDemo_debugAssert (0);
        return;
    }

    /* Create binary semaphores which is used to signal DPM_start/DPM_stop/DPM_ioctl is done
     * to the sensor management task. The signalling (Semaphore_post) will be done
     * from DPM registered report function (which will execute in the DPM execute task context). */
    Semaphore_Params_init(&semParams);
    semParams.mode              = Semaphore_Mode_BINARY;
    gMmwMssMCB.DPMstartSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMstopSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMioctlSemHandle   = Semaphore_create(0, &semParams, NULL);

    /* Open EDMA driver */
    Pcount3DDemo_edmaInit(&gMmwMssMCB.dataPathObj, DPC_OBJDET_R4F_EDMA_INSTANCE);

    /* Use EDMA instance 0 on MSS */
    Pcount3DDemo_edmaOpen(&gMmwMssMCB.dataPathObj, DPC_OBJDET_R4F_EDMA_INSTANCE);

    Pcount3DDemo_hwaInit(&gMmwMssMCB.dataPathObj);
    Pcount3DDemo_hwaOpen(&gMmwMssMCB.dataPathObj, gMmwMssMCB.socHandle);

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMssMCB.socHandle;
    initCfg.eventFxn                = Pcount3DDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode           = MMWave_ExecutionMode_ISOLATION;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    if (MMWave_sync (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Synchronization was successful\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 2800;
    gMmwMssMCB.taskHandles.mmwaveCtrl = Task_create(Pcount3DDemo_mmWaveCtrlTask, &taskParams, NULL);


    /*****************************************************************************
         * Create a task to do DMA based UART data transfer
    *****************************************************************************/
    /* Create a binary semaphore for application task to pend */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMssMCB.uartTxSemHandle = Semaphore_create(0, &semParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_UARTTX_TASK_PRIORITY;
    taskParams.stackSize = 800;
    Task_create(MmwDemo_uartTxTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset ((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetectionRangeHWA_InitParams));

    /* Note this must be after Pcount3DDemo_dataPathOpen() above which opens the hwa */
    objDetInitParams.L3ramCfg.addr = (void *)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreLocalRamCfg.addr = &gDPCTCM[0];
    objDetInitParams.CoreLocalRamCfg.size = sizeof(gDPCTCM);
    objDetInitParams.edmaHandle = gMmwMssMCB.dataPathObj.edmaHandle;
    objDetInitParams.hwaHandle = gMmwMssMCB.dataPathObj.hwaHandle;

    /* DPC Call-back config */
    objDetInitParams.processCallBackFxn.processInterFrameBeginCallBackFxn =
        Pcount3DDemo_DPC_ObjectDetection_processInterFrameCallBackFxn;

    objDetInitParams.processCallBackFxn.processFrameBeginCallBackFxn =
        Pcount3DDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn;

    /* Setup the configuration: */
    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));
    dpmInitCfg.socHandle        = gMmwMssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = &gDPC_ObjDetRangeHWACfg;;
    dpmInitCfg.instanceId       = DPC_OBJDET_R4F_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_DISTRIBUTED;
    dpmInitCfg.reportFxn        = Pcount3DDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_ObjectDetectionRangeHWA_InitParams);

    /* Initialize the DPM Module: */
    gMmwMssMCB.objDetDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gMmwMssMCB.objDetDpmHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        Pcount3DDemo_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the datapath module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gMmwMssMCB.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            Pcount3DDemo_debugAssert (0);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }

#ifdef TRACKERPROC_EN
    /*****************************************************************************
         * Create a task to run tracker DPU at lower priority than HWA DPC
    *****************************************************************************/
    /* Create a binary semaphore for application task to pend */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    gMmwMssMCB.trackerDPUSemHandle = Semaphore_create(0, &semParams, NULL);

    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_TRACKERDPU_TASK_PRIORITY;
    taskParams.stackSize = 7*1024;
    Task_create(MmwDemo_trackerDPUTask, &taskParams, NULL);
#endif

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 7*1024;
    gMmwMssMCB.taskHandles.objDetDpmTask = Task_create(mmwDemo_mssDPMTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialize the Profiler
     *****************************************************************************/
    Cycleprofiler_init();

    /*****************************************************************************
     * Initialize the CLI Module:
     *****************************************************************************/
    Pcount3DDemo_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void Pcount3DDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;
    socCfg.mpuCfg = SOC_MPUCfg_CONFIG;
    socCfg.dssCfg = SOC_DSSCfg_UNHALT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        Pcount3DDemo_debugAssert (0);
        return -1;
    }

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(Pcount3DDemo_MSS_MCB));

    gMmwMssMCB.socHandle = socHandle;

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the MMW Demo on MSS\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    gMmwMssMCB.taskHandles.initTask = Task_create(Pcount3DDemo_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

