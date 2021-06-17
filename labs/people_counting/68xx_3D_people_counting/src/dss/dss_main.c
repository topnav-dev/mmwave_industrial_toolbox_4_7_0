/**
 *   @file  dss_main.c
 *
 *   @brief
 *      This is the main file which implements the 3D people counting Demo on DSS.
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
#include <ti/sysbios/utils/Load.h>
#include <ti/sysbios/family/c64p/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/osal/DebugP.h>

/* Data path Include Files */
#include <common/src/dpc/capon3d/objectdetection.h>

/* Demo Include Files */
#include <people_counting/68xx_3D_people_counting/src/common/pcount3D_output.h>
#include "pcount3D_dss.h"
#include <people_counting/68xx_3D_people_counting/src/common/pcount3D_hwres.h>

/* Demo Profiling Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/**
 * @brief Task Priority settings:
 */
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      5

#define DEBUG(_x) //_x
//#define BUILDFORMETHOD2


#ifdef BUILDFORMETHOD2
#define L2HEAPSIZE (0x1B000)  // for method 2 and full FoV support of ODS
#define L3HEAPSIZE (0x2D000)  // for method 2 and full FoV support of ODS
#else
#define L2HEAPSIZE (0x19600)    // for method 1 ISK and +/- 20 degree elev FOV of ODS
#define L3HEAPSIZE (0x21000)    // for method 1 ISK and +/- 20 degree elev FOV of ODS
#endif

#define L2SCRATCHSIZE (0x900)
#define L1SCRATCHSIZE (0x1200)
#define L1HEAPSIZE (0x2E00)
#pragma DATA_SECTION(memHeapL2, ".dpc_l2Heap")
uint8_t memHeapL2[L2HEAPSIZE];
#pragma DATA_SECTION(memHeapL3, ".l3data")
uint8_t memHeapL3[L3HEAPSIZE];
#pragma DATA_SECTION(l2ScratchMem, ".dpc_l2Heap")
uint8_t l2ScratchMem[L2SCRATCHSIZE];
#pragma DATA_SECTION(l1ScratchMem, ".dpc_l1Heap")
uint8_t l1ScratchMem[L1SCRATCHSIZE];
#pragma DATA_SECTION(l1HeapMem, ".dpc_l1Heap")
uint8_t l1HeapMem[L1HEAPSIZE];

#define DPC_OBJDET_DSP_INSTANCEID       (0xDEEDDEED)

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
Pcount3DDemo_DSS_MCB    gPcount3DDssMCB;

/**
 * @brief
 *  Global Variable for DPM result buffer
 */
DPM_Buffer  resultBuffer;


/**************************************************************************
 ******************* Millimeter Wave Demo Functions Prototype *******************
 **************************************************************************/
static void Pcount3DDemo_dssInitTask(UArg arg0, UArg arg1);
static void Pcount3DDemo_DPC_RadarProc_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
static void Pcount3DDemo_DPC_processFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void Pcount3DDemo_DPC_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx);
static void Pcount3DDemo_DPC_RadarProc_dpmTask(UArg arg0, UArg arg1);
static void Pcount3DDemo_sensorStopEpilog(void);


/* copy table related */
extern far COPY_TABLE _pcount3DDemo_fastCode_L1PSRAM_copy_table;
extern far COPY_TABLE _pcount3DDemo_configCode_HSRAM_copy_table;

static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp);
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size);


/**
 *  @b Description
 *  @n
 *      Performs linker generated copy table copy using EDMA. Currently this is
 *      used to page in fast code from L3 to L1PSRAM.
 *  @param[in]  handle EDMA handle
 *  @param[in]  tp Pointer to copy table
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_copyTable(EDMA_Handle handle, COPY_TABLE *tp)
{
    uint16_t i;
    COPY_RECORD crp;
    uint32_t loadAddr;
    uint32_t runAddr;

    for (i = 0; i < tp->num_recs; i++)
    {
        crp = tp->recs[i];
        loadAddr = (uint32_t)crp.load_addr;
        runAddr = (uint32_t)crp.run_addr;

        /* currently we use only one count of EDMA which is 16-bit so we cannot
           handle tables bigger than 64 KB */
        DebugP_assert(crp.size <= 65536U);

        if (crp.size)
        {
            MmwDemo_edmaBlockCopy(handle, loadAddr, runAddr, crp.size);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Performs simple block copy using EDMA. Used for the purpose of copying
 *      linker table for L3 to L1PSRAM copy. memcpy cannot be used because there is
 *      no data bus access to L1PSRAM.
 *
 *  @param[in]  handle EDMA handle
 *  @param[in]  loadAddr load address
 *  @param[in]  runAddr run address
 *  @param[in]  size size in bytes
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaBlockCopy(EDMA_Handle handle, uint32_t loadAddr, uint32_t runAddr, uint16_t size)
{
    EDMA_channelConfig_t config;
    volatile bool isTransferDone;

    config.channelId = EDMA_TPCC0_REQ_FREE_0;
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.paramId = (uint16_t)EDMA_TPCC0_REQ_FREE_0;
    config.eventQueueId = 0;

    config.paramSetConfig.sourceAddress = (uint32_t) SOC_translateAddress((uint32_t)loadAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);
    config.paramSetConfig.destinationAddress = (uint32_t) SOC_translateAddress((uint32_t)runAddr,
        SOC_TranslateAddr_Dir_TO_EDMA, NULL);

    config.paramSetConfig.aCount = size;
    config.paramSetConfig.bCount = 1U;
    config.paramSetConfig.cCount = 1U;
    config.paramSetConfig.bCountReload = 0U;

    config.paramSetConfig.sourceBindex = 0U;
    config.paramSetConfig.destinationBindex = 0U;

    config.paramSetConfig.sourceCindex = 0U;
    config.paramSetConfig.destinationCindex = 0U;

    config.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    config.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;
    config.paramSetConfig.transferCompletionCode = (uint8_t) EDMA_TPCC0_REQ_FREE_0;
    config.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    config.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;

    /* don't care because of linear addressing modes above */
    config.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;

    config.paramSetConfig.isStaticSet = false;
    config.paramSetConfig.isEarlyCompletion = false;
    config.paramSetConfig.isFinalTransferInterruptEnabled = true;
    config.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
    config.paramSetConfig.isFinalChainingEnabled = false;
    config.paramSetConfig.isIntermediateChainingEnabled = false;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = NULL;

    if (EDMA_configChannel(handle, &config, false) != EDMA_NO_ERROR)
    {
        DebugP_assert(0);
    }

    if (EDMA_startDmaTransfer(handle, config.channelId) != EDMA_NO_ERROR)
    {
        DebugP_assert(0);
    }

    /* wait until transfer done */
    do {
        if (EDMA_isTransferComplete(handle,
                config.paramSetConfig.transferCompletionCode,
                (bool *)&isTransferDone) != EDMA_NO_ERROR)
        {
            DebugP_assert(0);
        }
    } while (isTransferDone == false);

    /* make sure to disable channel so it is usable later */
    EDMA_disableChannel(handle, config.channelId, config.channelType);
}


/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_edmaInit(Pcount3DDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t errorCode;

    errorCode = EDMA_init(instance);
    if (errorCode != EDMA_NO_ERROR)
    {
        //System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
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
void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gPcount3DDssMCB.dataPathObj.EDMA_errorInfo = *errorInfo;
    Pcount3DDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    gPcount3DDssMCB.dataPathObj.EDMA_transferControllerErrorInfo = *errorInfo;
    Pcount3DDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaOpen(Pcount3DDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t              errCode;
    EDMA_instanceInfo_t  edmaInstanceInfo;
    EDMA_errorConfig_t   errorConfig;

    obj->edmaHandle = EDMA_open(
        instance,
        &errCode,
        &edmaInstanceInfo);
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
    errorConfig.callbackFxn = MmwDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MmwDemo_EDMA_transferControllerErrorCallbackFxn;
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
void MmwDemo_edmaClose(Pcount3DDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
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
    Hwi_StackInfo   stackInfo;
    Task_Stat       stat;
    bool            hwiStackOverflow;

    DEBUG(System_printf("Data Path Stopped (last frame processing done)\n");)

    /* Print DSS task statistics */
    DEBUG(System_printf("DSS Task Stack Usage (Note: Task Stack Usage) ==========\n");)

    Task_stat(gPcount3DDssMCB.initTaskHandle, &stat);
    DEBUG(System_printf("%20s %12d %12d %12d\n", "initTask",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);)

    Task_stat(gPcount3DDssMCB.radarProcDpmTaskHandle, &stat);
    DEBUG(System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");)
    DEBUG(System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize,
                  stat.used,
                  stat.stackSize - stat.used);)

    DEBUG(System_printf("HWI Stack (same as System Stack) Usage ============\n");)
    hwiStackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (hwiStackOverflow == TRUE)
    {
        DEBUG(System_printf("DSS HWI Stack overflowed\n");)
        Pcount3DDemo_debugAssert(0);
    }
    else
    {
        DEBUG(System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");)
        DEBUG(System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize,
                      stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);)
    }
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
static void Pcount3DDemo_DPC_RadarProc_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{

    /* Only errors are logged on the console: */
    if (errCode != 0)
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        DEBUG(System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);)
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
            DebugP_log1("DSSApp: DPM Report IOCTL, command = %d\n", arg0);
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("DSSApp: DPM Report start\n");
            gPcount3DDssMCB.dpmStartEvents++;
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * DPC Results have been passed:
             * - This implies that we have valid profile results which have
             *   been received from the profile.
             *****************************************************************/

            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /*****************************************************************
             * DPC Results have been acked:
             * - This implies that MSS received the results.
             *****************************************************************/

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
            System_printf ("DSS Exception: %s, line %d.\n", ptrAssert->fileName,
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
            DebugP_log0("DSSApp: DPM Report stop\n");
            gPcount3DDssMCB.dpmStopEvents++;
            if (gPcount3DDssMCB.dpmStopEvents % 2 == 1) {
                Pcount3DDemo_sensorStopEpilog();
            }
            break;
        }
        case DPM_Report_DPC_INFO:
        {
            /* Currently chain does not use this feature. */
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
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing.
 *      Note: In this demo objdetdsp DPC only have inter-frame processing, hence this 
 *      callback function won't be called.
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void Pcount3DDemo_DPC_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    /* Empty function */
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
static void Pcount3DDemo_DPC_processInterFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gPcount3DDssMCB.dataPathObj.subFrameStats[subFrameIndx].interFrameCPULoad = Load_getCPULoad();
}


/**
 *  @b Description
 *  @n
 *      DPM Execution Task. DPM execute results are processed here:
 *      a) Update states based on timestamp from DPC.
 *      b) Copy results to shared memory to be shared with MSS.
 *      c) Send Results to MSS by calling DPM_sendResult()
 *
 *  @retval
 *      Not Applicable.
 */
static void Pcount3DDemo_DPC_RadarProc_dpmTask(UArg arg0, UArg arg1)
{
    int32_t     retVal;
    //DPC_ObjectDetection_ExecuteResult *result;
    volatile uint32_t              startTime;

    while (1)
    {
        /* Execute the DPM module: */
        //DebugP_log0("DSS main: Pcount3DDemo_DPC_RadarProc_dpmTask\n");
        retVal = DPM_execute (gPcount3DDssMCB.dataPathObj.radarProcDpmHandle, &resultBuffer);
        if (retVal < 0) {
            DEBUG(System_printf ("Error: DPM execution failed [Error code %d]\n", retVal);)
            Pcount3DDemo_debugAssert (0);
        }
        else
        {
            if ((resultBuffer.size[0] == sizeof(DPC_ObjectDetection_ExecuteResult)))
            {

                retVal = DPM_sendResult (gPcount3DDssMCB.dataPathObj.radarProcDpmHandle, true, &resultBuffer);
                if (retVal < 0)
                {
                    DEBUG(System_printf ("Error: Failed to send results [Error: %d] to remote\n", retVal);)
                }

            }
        }
        //writeback all the data shared with R4 in L3, and prepare cache for next frames radar cube from R4.
        cache_wbInvAllL2Wait();
    }
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
static void Pcount3DDemo_dssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    Task_Params         taskParams;
    DPM_InitCfg         dpmInitCfg;
    DPC_ObjectDetection_InitParams      objDetInitParams;

    /*****************************************************************************
     * Driver Init:
     *****************************************************************************/

    /*****************************************************************************
     * Driver Open/Configuraiton:
     *****************************************************************************/
    /* Initialize EDMA */
    MmwDemo_edmaInit(&gPcount3DDssMCB.dataPathObj, DPC_OBJDET_DSP_EDMA_INSTANCE);

    /* Use instance 1 on DSS */
    MmwDemo_edmaOpen(&gPcount3DDssMCB.dataPathObj, DPC_OBJDET_DSP_EDMA_INSTANCE);

    /* Copy code from L3 to L1PSRAM, this code related to data path processing */
    MmwDemo_copyTable(gPcount3DDssMCB.dataPathObj.edmaHandle, &_pcount3DDemo_fastCode_L1PSRAM_copy_table);
    MmwDemo_copyTable(gPcount3DDssMCB.dataPathObj.edmaHandle, &_pcount3DDemo_configCode_HSRAM_copy_table);

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_DSS);


    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    DebugP_log0("DSS main: Pcount3DDemo_dssInitTask\n");

    memset ((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetection_InitParams));
	/*Set up init params for memory osal*/
	objDetInitParams.L3HeapCfg.addr 		=	(void *) &memHeapL3[0];
	objDetInitParams.L3HeapCfg.size 		=	L3HEAPSIZE;
	objDetInitParams.L3ScratchCfg.addr 		=	(void *) NULL;
	objDetInitParams.L3ScratchCfg.size 		=	0;
	
	objDetInitParams.CoreL2HeapCfg.addr 	=	(void *) &memHeapL2[0];
	objDetInitParams.CoreL2HeapCfg.size 	=	L2HEAPSIZE;
	objDetInitParams.CoreL2ScratchCfg.addr 	=	(void *) &l2ScratchMem[0];
	objDetInitParams.CoreL2ScratchCfg.size 	=	L2SCRATCHSIZE;
	
	objDetInitParams.CoreL1HeapCfg.addr 	=	(void *) &l1HeapMem[0];
	objDetInitParams.CoreL1HeapCfg.size 	=	L1HEAPSIZE;
	objDetInitParams.CoreL1ScratchCfg.addr 	=	(void *) &l1ScratchMem[0];
	objDetInitParams.CoreL1ScratchCfg.size 	=	L1SCRATCHSIZE;

    /* DPC Call-back config */
    objDetInitParams.processCallBackCfg.processFrameBeginCallBackFxn =
        Pcount3DDemo_DPC_processFrameBeginCallBackFxn;
    objDetInitParams.processCallBackCfg.processInterFrameBeginCallBackFxn =
        Pcount3DDemo_DPC_processInterFrameBeginCallBackFxn;

    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));

    /* Setup the configuration: */
    dpmInitCfg.socHandle        = gPcount3DDssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = &gDPC_ObjectDetectionCfg;
    dpmInitCfg.instanceId       = DPC_OBJDET_DSP_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_DISTRIBUTED;
    dpmInitCfg.reportFxn        = Pcount3DDemo_DPC_RadarProc_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_ObjectDetection_InitParams);

    /* Initialize the DPM Module: */
    gPcount3DDssMCB.dataPathObj.radarProcDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gPcount3DDssMCB.dataPathObj.radarProcDpmHandle == NULL)
    {
        DEBUG(System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);)
        Pcount3DDemo_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gPcount3DDssMCB.dataPathObj.radarProcDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            DEBUG(System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);)
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
    System_printf ("Debug: DPM Module Sync is done\n");

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 3*1024;
    gPcount3DDssMCB.radarProcDpmTaskHandle = Task_create(Pcount3DDemo_DPC_RadarProc_dpmTask, &taskParams, NULL);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the DSP using IDLE instruction.
 *     When DSP has no work left to do,
 *     the BIOS will be in Idle thread and will call this function.
 *
 *  @retval
 *      Not Applicable.
 */
void Pcount3DDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" IDLE ");
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

    // set cache state for l3 and hsram
    cache_setMar((unsigned int *)0x20000000, 0xC0000, Cache_PC | Cache_PFX);
    //cache_setMar((unsigned int *)0x21080000, 0x8000, Cache_PC | Cache_PFX);


    /* Initialize and populate the demo MCB */
    memset ((void*)&gPcount3DDssMCB, 0, sizeof(Pcount3DDemo_DSS_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_BYPASS_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        DEBUG(System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);)
        Pcount3DDemo_debugAssert (0);
        return -1;
    }

    gPcount3DDssMCB.socHandle = socHandle;

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 1536;
    gPcount3DDssMCB.initTaskHandle = Task_create(Pcount3DDemo_dssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
