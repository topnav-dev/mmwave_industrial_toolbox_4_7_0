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
#include <ti/utils/mathutils/mathutils.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>
#include <xdc/runtime/System.h>

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
//#pragma diag_push
//#pragma diag_suppress 48
//#include <ti/mathlib/mathlib.h>
//#pragma diag_pop

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this object detection DPC are prefixed by DPC_OBJDET_ */
#include APP_RESOURCE_FILE

#include <ti/control/mmwavelink/mmwavelink.h>

/* Obj Det instance etc */
#include <common/src/dpc/capon3d/include/objectdetectioninternal.h>
#include <common/src/dpc/capon3d/objectdetection.h>


//#define DBG_DPC_OBJDET

#ifdef DBG_DPC_OBJDET
ObjDetObj     *gObjDetObj;
#endif

/**************************************************************************
 ************************** Local Definitions **********************************
 **************************************************************************/

/**
@}
*/
/*! Maximum Number of objects that can be detected in a frame */
#define DPC_OBJDET_MAX_NUM_OBJECTS                       DOA_OUTPUT_MAXPOINTS

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
int32_t DPC_ObjectDetection_dataInjection(DPM_DPCHandle handle, DPM_Buffer* ptrBuffer);

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/

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

    //DebugP_log1("ObjDet DPC: DPC_ObjectDetection_dataInjection, handle = 0x%x\n", (uint32_t)handle);
    DebugP_assert (DPM_notifyExecute (objDetObj->dpmHandle, handle, true) == 0);

    return 0;
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
    //SubFrameObj *subFrmObj;

    //subFrmObj = &objDetObj->subFrameObj[subFrameIndx];

    //retVal = DPU_CFARCAProcDSP_config(subFrmObj->dpuCFARCAObj, &subFrmObj->dpuCfg.cfarCfg);
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
    int32_t         retVal = 0;

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
    int32_t         retVal = 0;

    radarOsal_memDeInit();

    return(retVal);
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
    DPC_ObjectDetection_PreStartCfg  *preStartCfg
)
{
    int32_t retVal = 0;
    DPC_ObjectDetection_DynCfg     *dynCfg;
    DPIF_RadarCube radarCube;
	DPU_ProcessErrorCodes procErrorCode;

    dynCfg 					= 	&preStartCfg->dynCfg;

    /* Save configs to object. We need to pass this stored config (instead of
       the input arguments to this function which will be in stack) to
       the DPU config functions inside of this function because the DPUs
       have pointers to dynamic configurations which are later going to be
       reused during re-configuration (intra sub-frame or inter sub-frame)
     */
    subFrameObj->dynCfg 	= *	dynCfg;

    /* L3 allocations */
    /* L3 - radar cube */
    radarCube.dataSize 		= 	dynCfg->caponChainCfg.numRangeBins * dynCfg->caponChainCfg.numChirpPerFrame *
            dynCfg->caponChainCfg.numAntenna * sizeof(cplx16_t);
    DebugP_log1("ObjDet DPC: DPC_ObjDetDSP_preStartConfig, radarCubeFormat = %d\n", dynCfg->radarCubeFormat);
    if(preStartCfg->shareMemCfg.shareMemEnable == true)
    {
        if((preStartCfg->shareMemCfg.radarCubeMem.addr != NULL) &&
          (preStartCfg->shareMemCfg.radarCubeMem.size == radarCube.dataSize))
        {
            /* Use assigned radar cube address */
            radarCube.data 	= 	preStartCfg->shareMemCfg.radarCubeMem.addr;
        }
        else
        {
            retVal 			= 	DPC_OBJECTDETECTION_EINVAL__COMMAND;
            goto exit;
        }
#ifdef      RADARDEMO_AOARADARCUDE_RNGCHIRPANT
        if (subFrameObj->dynCfg.radarCubeFormat != DPIF_RADARCUBE_FORMAT_2)
        {
            retVal = DPC_OBJECTDETECTION_EINVAL_CUBE;
            goto exit;
        }
#endif
    }
    else
    {
		retVal = DPC_OBJECTDETECTION_EINVAL_CUBE;
        goto exit;
    }

    /* Only supported radar Cube format in this DPC */
    radarCube.datafmt 		= 	DPIF_RADARCUBE_FORMAT_3;
	subFrameObj->dataIn		=	radarCube.data;
	
    subFrameObj->dpuCaponObj 	= 	DPU_radarProcess_init(&subFrameObj->dynCfg.caponChainCfg, &procErrorCode);
    if (procErrorCode > PROCESS_OK)
    {
		retVal 				= 	DPC_OBJECTDETECTION_EINTERNAL;
	    DebugP_log1("DPC config error %d\n", procErrorCode);
        goto exit;
    }

    //printf("DPC configuration done!\n");
    DebugP_log0("DPC config done\n");
    /* Report RAM usage */
    radarOsal_printHeapStats();

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

    objDetObj->stats->frameStartTimeStamp = Cycleprofiler_getTimeStamp();

    //DebugP_log2("ObjDet DPC: Frame Start, frameIndx = %d, subFrameIndx = %d\n",
    //            objDetObj->stats.frameStartIntCounter, objDetObj->subFrameIndx);

    /* Check if previous frame (sub-frame) processing has completed */
    DPC_Objdet_Assert(objDetObj->dpmHandle, (objDetObj->interSubFrameProcToken == 0));
    objDetObj->interSubFrameProcToken++;

    /* Increment interrupt counter for debugging purpose */
    if (objDetObj->subFrameIndx == 0)
    {
        objDetObj->stats->frameStartIntCounter++;
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

    objDetObj->stats->frameStartIntCounter = 0;

    /* Start marks consumption of all pre-start configs, reset the flag to check
     * if pre-starts were issued only after common config was issued for the next
     * time full configuration happens between stop and start */
    objDetObj->isCommonCfgReceived = false;

    /* App must issue export of last frame after stop which will switch to sub-frame 0,
     * so start should always see sub-frame indx of 0, check */
    DebugP_assert(objDetObj->subFrameIndx == 0);

    if(objDetObj->numSubframes > 1U)
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
    DPC_ObjectDetection_ProcessCallBackCfg *processCallBack;
	int32_t procErrorCode;
	

	radarProcessOutput *result;
    int32_t retVal = 0;
    volatile uint32_t                        startTime;
    int32_t i;

    objDetObj = (ObjDetObj *) handle;
    DebugP_assert (objDetObj != NULL);
    DebugP_assert (ptrResult != NULL);

    DebugP_log1("ObjDet DPC: Processing sub-frame %d\n", objDetObj->subFrameIndx);

    processCallBack                         =   &objDetObj->processCallBackCfg;

    objDetObj->executeResult->subFrameIdx   =   objDetObj->subFrameIndx;
    result                                  =   &objDetObj->executeResult->objOut;

    subFrmObj                               =   &objDetObj->subFrameObj[objDetObj->subFrameIndx];

	if (processCallBack->processInterFrameBeginCallBackFxn != NULL)
	{
		(*processCallBack->processInterFrameBeginCallBackFxn)(objDetObj->subFrameIndx);
	}

	//DebugP_log0("ObjDet DPC: Range Proc Output Ready\n");

	startTime                               =   Cycleprofiler_getTimeStamp();
	DPU_radarProcess_process(subFrmObj->dpuCaponObj, subFrmObj->dataIn, result, &procErrorCode);
	if (procErrorCode > PROCESS_OK)
	{
		retVal 	=	-1;
		goto exit;
	}

    DebugP_log0("ObjDet DPC: Frame Proc Done\n");

    objDetObj->stats->interFrameEndTimeStamp            =   Cycleprofiler_getTimeStamp();
    memcpy(&(objDetObj->stats->subFrbenchmarkDetails), result->benchmarkOut, sizeof(radarProcessBenchmarkElem));
    objDetObj->stats->interFrameExecTimeInUsec          =   (uint32_t)((float)(objDetObj->stats->interFrameEndTimeStamp - objDetObj->stats->frameStartTimeStamp) * _rcpsp((float)DSP_CLOCK_MHZ));
    objDetObj->stats->activeFrameProcTimeInUsec         =   (uint32_t)((float)(objDetObj->stats->interFrameEndTimeStamp - startTime) * _rcpsp((float)DSP_CLOCK_MHZ));


    /* populate DPM_resultBuf - first pointer and size are for results of the processing */
	ptrResult->ptrBuffer[0] = (uint8_t *)objDetObj->executeResult;
	ptrResult->size[0] = sizeof(DPC_ObjectDetection_ExecuteResult);

    ptrResult->ptrBuffer[1] = (uint8_t *)objDetObj->stats;
    ptrResult->size[1] = sizeof(DPC_ObjectDetection_Stats);



	/* clear rest of the result */
	for (i = 2; i < DPM_MAX_BUFFER; i++)
	{
		ptrResult->ptrBuffer[i] = NULL;
		ptrResult->size[i] = 0;
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
    else if (cmd == DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
        objDetObj->numSubframes =   *(uint8_t *)arg;
        objDetObj->isCommonCfgReceived = true;
        //DebugP_log1("ObjDet DPC: Pre-start Config IOCTL processed common config (numSubframes = %d)\n", objDetObj->numSubframes);
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED)
    {
        DPC_ObjectDetection_ExecuteResultExportedInfo *inp;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetection_ExecuteResultExportedInfo));

        inp = (DPC_ObjectDetection_ExecuteResultExportedInfo *)arg;

        /* input sub-frame index must match current sub-frame index */
        DebugP_assert(inp->subFrameIdx == objDetObj->subFrameIndx);

        /* Reconfigure all DPUs resources for next sub-frame as EDMA and scrach buffer
         * resources overlap across sub-frames */
        if (objDetObj->numSubframes > 1)
        {
            /* Next sub-frame */
            objDetObj->subFrameIndx++;
            if (objDetObj->subFrameIndx == objDetObj->numSubframes)
            {
                objDetObj->subFrameIndx = 0;
            }

            DPC_ObjDetDSP_reconfigSubFrame(objDetObj, objDetObj->subFrameIndx);
        }
        DebugP_log0("ObjDet DPC: received ack from MSS for output data\n");

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
            /* Related to pre-start configuration */
            case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            {
                DPC_ObjectDetection_PreStartCfg *cfg;

                /* Pre-start common config must be received before pre-start configs
                 * are received. */
                if (objDetObj->isCommonCfgReceived == false)
                {
                    //DebugP_log0("ObjDet DPC IOCTL: false isCommonCfgReceived\n");
                    retVal = DPC_OBJECTDETECTION_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;
                    goto exit;
                }

                DebugP_assert(argLen == sizeof(DPC_ObjectDetection_PreStartCfg));

                cfg = (DPC_ObjectDetection_PreStartCfg*)arg;


                //DebugP_log4("ObjDet DPC IOCTL: function called with cfg = 0x%x, subFrmObj = 0x%x, cmd = %d, subFrameNum = %d\n", (uint32_t )arg, (uint32_t )subFrmObj, cmd, *(uint8_t *)arg);
                retVal = DPC_ObjDetDSP_preStartConfig(subFrmObj,
                             cfg);
                if (retVal != 0)
                {
                    goto exit;
                }


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
	radarOsal_heapConfig heapconfig[3];

    *errCode = 0;

    //DebugP_log0("DPC: DPC_ObjectDetection_init\n");
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
	
	/*Set up heap and mem osal*/
	{
		memset(heapconfig, 0, sizeof(heapconfig));
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType 	= 	RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr   = 	(int8_t *) dpcInitParams->L3HeapCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize   = 	dpcInitParams->L3HeapCfg.size;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr= 	(int8_t *) dpcInitParams->L3ScratchCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize= 	dpcInitParams->L3ScratchCfg.size;

		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL2;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr   		= 	(int8_t *) dpcInitParams->CoreL2HeapCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize   		= 	dpcInitParams->CoreL2HeapCfg.size;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr   	= 	(int8_t *) dpcInitParams->CoreL2ScratchCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize   	= 	dpcInitParams->CoreL2ScratchCfg.size;

		heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL1;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr   		= 	(int8_t *) dpcInitParams->CoreL1HeapCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize   		= 	dpcInitParams->CoreL1HeapCfg.size;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr   	= 	(int8_t *) dpcInitParams->CoreL1ScratchCfg.addr;
		heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize   	= 	dpcInitParams->CoreL1ScratchCfg.size;
		if(radarOsal_memInit(&heapconfig[0], 3) == RADARMEMOSAL_FAIL)
		{
			*errCode = DPC_OBJECTDETECTION_MEMINITERR;
			goto exit;
		}
	}
	
	
	objDetObj = MemoryP_ctrlAlloc(sizeof(ObjDetObj), 0);

#ifdef DBG_DPC_OBJDET
    gObjDetObj = objDetObj;
#endif

    System_printf("ObjDet DPC: objDetObj address = (ObjDetObj     *) 0x%x\n", (uint32_t) objDetObj);

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

    objDetObj->processCallBackCfg = dpcInitParams->processCallBackCfg;

    objDetObj->executeResult    =   (DPC_ObjectDetection_ExecuteResult *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(DPC_ObjectDetection_ExecuteResult), 1);
    objDetObj->stats            =   (DPC_ObjectDetection_Stats *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(DPC_ObjectDetection_Stats), 1);

	*errCode = DPC_ObjDetDSP_initDPU(objDetObj, RL_MAX_SUBFRAMES);
    //printf ("DPC init done!\n");

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
    DPC_ObjectDetection_dataInjection,   /* Inject Data Function:            */
    NULL,                                /* Chirp Available Function:        */
    DPC_ObjectDetection_frameStart       /* Frame Start Function:            */
};

/* @} */

