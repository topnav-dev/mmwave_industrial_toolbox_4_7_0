/*
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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
/**
 *   @file  trackerproc.c
 *
 *   @brief
 *      Implements DPU wrapper for the gtrack lib.
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

/* mmWave SDK drivers/common Include Files */
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Data Path Include files */
#include "../trackerproc.h"
#include <ti/alg/gtrack/gtrack.h>

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/* Scenery parameters includes up to two boundary boxes and up to two static boxes */
/* Each box is in format {x1,x2, y1,y2, z1,z2}. In 2D cases, the z parameters are ignored */
GTRACK_sceneryParams appSceneryParamTable[4] = {
    /* TM: 1 boundary box: {-1,12, 15,75, 0,0}, and 1 static box {0,14, 19,50, 0,0} */
    1,{{-1.f,12.f, 15.f,75.f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}},1,{{0.f,11.f, 19.f,50.f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f}},
    /* PEOPLE COUNTING: 1 boundary box: {-4,4, 0.5,7.5, 0,0}, and 1 static box {-3,3, 2,6, 0,0} */
    1,{{-4.f,4.f, 0.5f,7.5f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-3.f,3.f,2.f,6.f,0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}},
    /* OUTDOOR: 1 boundary box: {-39,19, 2,50, 0,0}, and 1 static box {-30,16, 4,44, 0,0} */
    1,{{-29.f,39.f, 2.f,59.f, -1.f,3.f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-20.f,20.f, 12.f,40.f, 0.f, 2.f},{0.f,0.f,0.f,0.f,0.f,0.f}},
    /* CEILING MOUNT: 1 boundary box: {-4,4, 0.5,7.5, -1,3}, and 1 static box {-3,3, 2,6, -0.5,2.5} */
    1,{{-4.f,4.f, 0.5f,7.5f, -1.f,3.0f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-3.f,3.f,2.f,6.f,-0.5,2.5f},{0.f,0.f,0.f,0.f,0.f,0.f}}
};

/* Gating Volume 2 "liters", Limits are set to 2m in depth and width, no limit in height and doppler */
GTRACK_gatingParams appGatingParamTable[4] = {
    /* TM: Gating volume = 16, Limits are set to 12m in depth, 8m in width, ignore the height, no limit in doppler */
    {4.f, {12.f, 6.f, 4.f, 12.f}},
    /* PEOPLE COUNTING: Gating gain = 3, Limits are 2m in depth, 2m in width, ignore the height, 12m/s in doppler */
    {3.f,  {2.f,  2.f, 2.f, 12.f}},
    /* OUTDOOR: Gating gain = 4, Limits are set to 6m in depth, 6m in width, ignore the height, 10m/s limit in doppler */
    {4.f, {6.f,  6.f, 4.f, 10.f}},
    /* CEILING MOUNT: Gating volume = 2, Limits are 2m in depth, 2m in width, 2m the height, no limit in doppler */
    {2.f,  {2.f,  2.f, 2.f, 10.f}}
};
GTRACK_stateParams appStateParamTable[4] = {
    {3U,	3U,  	5U,		100U, 	5U}, 	/* TM: det2act, det2free, act2free, stat2free, exit2free */
    {10U,	5U, 	50U,	100U, 	5U}, 	/* PC: det2act, det2free, act2free, stat2free, exit2free */
    {4U, 	10U, 	60U, 	600U,	20U},	/* OUTDOOR: det2act, det2free, act2free, stat2free, exit2free */
    {10U,	5U, 	10U, 	100U,	5U} 	/* CEILING MOUNT: det2act, det2free, act2free, stat2free, exit2free */
};
GTRACK_allocationParams appAllocationParamTable[4] = {
	{100.f, 100.f, 1.f,   3U, 4.f,   2.f},	/* TM: 100 (100) SNRs, 1m/s minimal velocity, 3 points with 4m in distance, 2m/c in velocity  separation */
	{60.f, 	200.f, 0.1f,  5U, 1.5f,  2.f},	/* PC: 150 (250 obscured), 0.1 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
	{40.f,  200.f, 0.5f,  3U, 2.f,   2.f},  /* OUTDOOR: 50 (200 obscured), 0.5 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
	{60.f,  200.f, 0.1f,  5U, 1.5f,  2.f}  	/* CEILING MOUNT: 150 (200 obscured), 0.5 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
};
/* This parameter is ignored in 2D/3D tracker versions */
GTRACK_varParams appVariationParamTable[4] = {
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f},
    {0.f, 0.f, 0.f}
};

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/
typedef enum {
    TRACKING_DEFAULT_PARAM_SET = 0,
    TRACKING_TRAFFIC_MONITORING_PARAM_SET,
    TRACKING_PEOPLE_COUNTING_PARAM_SET,
    TRACKING_OUTDOOR_PARAM_SET,	
    TRACKING_CEILING_MOUNT_PARAM_SET	
} TRACKING_ADVANCED_PARAM_SET;

typedef enum {
    TRACKING_PARAM_SET_TM = 0,
    TRACKING_PARAM_SET_PC,
    TRACKING_PARAM_SET_OUTDOOR,
    TRACKING_PARAM_SET_CEILING_MOUNT
} TRACKING_ADVANCED_PARAM_SET_TABLE;

/**************************************************************************
 ************************ Internal Functions Prototyp**********************
 **************************************************************************/
void DPU_trackerProc_updateParamTables
(
    DPU_TrackerProc_StaticConfig    *pStaticCfg
);

/**************************************************************************
 ************************TrackerProc Internal Functions **********************
 **************************************************************************/
void DPU_trackerProc_updateParamTables
(
    DPU_TrackerProc_StaticConfig    *pStaticCfg
)
{
    /* Update scenery params: */
    memcpy(&appSceneryParamTable[TRACKING_PARAM_SET_PC],
            &pStaticCfg->sceneryParams,
            sizeof(GTRACK_sceneryParams));

    /* Update gating params: */
    memcpy(&appGatingParamTable[TRACKING_PARAM_SET_PC],
            &pStaticCfg->gatingParams,
            sizeof(GTRACK_gatingParams));

    /* Update allocation params: */
    memcpy(&appAllocationParamTable[TRACKING_PARAM_SET_PC],
            &pStaticCfg->allocationParams,
            sizeof(GTRACK_allocationParams));

    /* Update state params: */
    memcpy(&appStateParamTable[TRACKING_PARAM_SET_PC],
            &pStaticCfg->stateParams,
            sizeof(GTRACK_stateParams));

    /* Update variation params: */
    memcpy(&appVariationParamTable[TRACKING_PARAM_SET_PC],
            &pStaticCfg->variationParams,
            sizeof(GTRACK_varParams));

}
    
/**************************************************************************
 ************************TrackerProc External APIs **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is TrackerProc DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid TrackerProc handle
 *  @retval
 *      Error       - NULL
 */
DPU_TrackerProc_Handle DPU_TrackerProc_init
(
    int32_t*    errCode
)
{

    trackerProcObjType     *trackerProcObj = NULL;

    /* Allocate Memory for trackerProc */
    trackerProcObj = MemoryP_ctrlAlloc(sizeof(trackerProcObjType), 0);
    if(trackerProcObj == NULL)
    {
        *errCode = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)trackerProcObj, 0, sizeof(trackerProcObjType));

exit:
    return ((DPU_TrackerProc_Handle)trackerProcObj);

}

/**
 *  @b Description
 *  @n
 *      The function is trackerProc DPU config function.
 *  
 *  @pre    DPU_TrackerProc_init() has been called
 *
 *  @param[in]  handle                  trackerProc DPU handle
 *  @param[in]  pConfigIn               Pointer to trackerProc configuration data structure
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_TrackerProc_config
(
    DPU_TrackerProc_Handle  handle,
    DPU_TrackerProc_Config  *pConfigIn
)
{
    trackerProcObjType             *trackerProcObj;
    DPU_TrackerProc_StaticConfig   *pStaticCfg;
    int32_t                         retVal = 0;

    int32_t                     errCode;    
    GTRACK_advancedParameters   advParams;

    uint32_t                pointCloudSize;
    uint32_t                targetListSize;
    uint32_t                targetIndexSize;

    trackerProcObj = (trackerProcObjType *)handle;
    if(trackerProcObj == NULL)
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }

#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!pConfigIn || !(pConfigIn->staticCfg))
      
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }
#endif
    /* Convinience pointer to config params */
    pStaticCfg = &pConfigIn->staticCfg;

    DPU_trackerProc_updateParamTables(pStaticCfg);

    /* Use Param set table according to use case */
    switch(pStaticCfg->trackingParamSet)
    {
        case TRACKING_DEFAULT_PARAM_SET:
            // Do not configure advanced parameters, use library default parameters
            pStaticCfg->gtrackModuleConfig.advParams = 0;
            break;

        case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_TM];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_TM];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_TM];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_TM];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_TM];
            pStaticCfg->gtrackModuleConfig.advParams = &advParams;
            
            break;

        case TRACKING_PEOPLE_COUNTING_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_PC];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_PC];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_PC];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_PC];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_PC];
            pStaticCfg->gtrackModuleConfig.advParams = &advParams;

            break;

        case TRACKING_OUTDOOR_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_OUTDOOR];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_OUTDOOR];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_OUTDOOR];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_OUTDOOR];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_OUTDOOR];
            pStaticCfg->gtrackModuleConfig.advParams = &advParams;

            break;

        case TRACKING_CEILING_MOUNT_PARAM_SET:
            /* Initialize CLI configuration: */
            memset ((void *)&advParams, 0, sizeof(GTRACK_advancedParameters));
            advParams.sceneryParams = &appSceneryParamTable[TRACKING_PARAM_SET_CEILING_MOUNT];
            advParams.allocationParams = &appAllocationParamTable[TRACKING_PARAM_SET_CEILING_MOUNT];
            advParams.gatingParams = &appGatingParamTable[TRACKING_PARAM_SET_CEILING_MOUNT];
            advParams.stateParams = &appStateParamTable[TRACKING_PARAM_SET_CEILING_MOUNT];
            advParams.variationParams = &appVariationParamTable[TRACKING_PARAM_SET_CEILING_MOUNT];
            pStaticCfg->gtrackModuleConfig.advParams = &advParams;

            break;

    }

    /* Save config for later use */
    memcpy((void *)&trackerProcObj->pDpuCfg, (void *)pConfigIn, sizeof(DPU_TrackerProc_Config));

    
    pointCloudSize = (pStaticCfg->gtrackModuleConfig.maxNumPoints)*sizeof(GTRACK_measurementPoint);
    trackerProcObj->pointCloud = (trackerProc_Point *)MemoryP_ctrlAlloc(pointCloudSize, sizeof(GTRACK_measurementPoint));

    if(trackerProcObj->pointCloud == NULL) {
        //System_printf("Error: Unable to allocate %d bytes for pointCloudIn\n", pointCloudSize);
        retVal = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }
    
    trackerProcObj->targetDescrHandle = (trackerProc_TargetDescrHandle *)MemoryP_ctrlAlloc(sizeof(trackerProc_TargetDescrHandle), sizeof(float));
    if(trackerProcObj->targetDescrHandle == NULL) {
        //System_printf("Error: Unable to allocate %d bytes for targetDescr handle\n", sizeof(trackerProc_TargetDescrHandle));
        retVal = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }
    memset ((void *)trackerProcObj->targetDescrHandle, 0, sizeof(trackerProc_TargetDescrHandle));
    trackerProcObj->targetDescrHandle->currentDescr = 0; /*For now, use only the ping side */

    /* Allocate memory for ping/pong target lists */
    targetListSize = (pStaticCfg->gtrackModuleConfig.maxNumTracks)*sizeof(GTRACK_targetDesc);
    trackerProcObj->targetDescrHandle->tList[0] = (trackerProc_Target *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));
    trackerProcObj->targetDescrHandle->tList[1] = (trackerProc_Target *)MemoryP_ctrlAlloc(targetListSize, sizeof(float));

    if((trackerProcObj->targetDescrHandle->tList[0] == NULL) || (trackerProcObj->targetDescrHandle->tList[1] == NULL)){
        //System_printf("Error: Unable to allocate %d bytes for targetLists\n", targetListSize*2);
        retVal = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }

    /* Allocate memory for ping/pong target indices */
    targetIndexSize = (pStaticCfg->gtrackModuleConfig.maxNumPoints)*sizeof(trackerProc_TargetIndex);
    trackerProcObj->targetDescrHandle->tIndex[0] = (trackerProc_TargetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));
    trackerProcObj->targetDescrHandle->tIndex[1] = (trackerProc_TargetIndex *)MemoryP_ctrlAlloc(targetIndexSize, sizeof(float));

    if((trackerProcObj->targetDescrHandle->tIndex[0] == NULL) || (trackerProcObj->targetDescrHandle->tIndex[1] == NULL)){
        //System_printf("Error: Unable to allocate %d bytes for targetIndices\n", targetIndexSize*2);
        retVal = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }


    if(trackerProcObj->gtrackHandle != NULL)
        gtrack_delete(trackerProcObj->gtrackHandle);

    trackerProcObj->gtrackHandle = gtrack_create(&(pStaticCfg->gtrackModuleConfig), &errCode);
    if(trackerProcObj->gtrackHandle == NULL) {
        //System_printf("Error: Unable to allocate memory for Tracker\n");
        retVal = DPU_TRACKERPROC_ENOMEM;
        goto exit;
    }

    
exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is trackerProc DPU process function.
 *
 *  @pre    DPU_trackerProc_init() has been called
 *
 *  @param[in]  handle                  trackerProc DPU handle
 *  @param[in]  numObjsIn               number of input points
 *  @param[in]  detObjIn                input point cloud in Spherical format
 *  @param[in]  detObjInSideInfo        point cloud side info
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_TrackerProc_process
(
    DPU_TrackerProc_Handle      handle,
    uint32_t                    numObjsIn,
    DPIF_PointCloudSpherical    *detObjIn,
    DPIF_PointCloudSideInfo     *detObjInSideInfo,
    DPU_TrackerProc_OutParams   *outParams
)
{


    trackerProc_Target      *targetList;
    trackerProc_TargetIndex *targetIndex;
    GTRACK_targetDesc targetDescr[20];

    trackerProcObjType      *trackerProcObj;
    DPU_TrackerProc_Config  *pDpuCfg;
    
    GTRACK_measurementPoint *points;

    uint16_t                mNum;
    uint16_t                tNum;
    uint16_t                n;
    int32_t                 retVal = 0;

    _Bool currentDescr;

    trackerProcObj = (trackerProcObjType *)handle;
    if(trackerProcObj == NULL)
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }

    if(trackerProcObj->inProgress == true)
    {
        retVal = DPU_TRACKERPROC_EINPROGRESS;
        goto exit;
    }
    else
    {
        trackerProcObj->inProgress = true;
    }

    /* Convinience pointer to config params */
    pDpuCfg = &trackerProcObj->pDpuCfg;

    pDpuCfg->res.detObjIn = detObjIn;
    pDpuCfg->res.numDetObjIn = numObjsIn;
    pDpuCfg->res.detObjInSideInfo = detObjInSideInfo;
        
    if((pDpuCfg->res.numDetObjIn) > (pDpuCfg->staticCfg.gtrackModuleConfig.maxNumPoints))
        mNum = pDpuCfg->staticCfg.gtrackModuleConfig.maxNumPoints;
    else
        mNum = pDpuCfg->res.numDetObjIn;

    /* Copy input points to tracker object local point cloud structure */
    for (n=0; n< mNum; n++)
    {
        trackerProcObj->pointCloud[n].range = pDpuCfg->res.detObjIn[n].range;     
        trackerProcObj->pointCloud[n].azimuth =
            pDpuCfg->res.detObjIn[n].azimuthAngle + pDpuCfg->staticCfg.sensorAzimuthTilt;     
        trackerProcObj->pointCloud[n].doppler = pDpuCfg->res.detObjIn[n].velocity;     
        trackerProcObj->pointCloud[n].snr = pDpuCfg->res.detObjInSideInfo[n].snr;     
    }

    points = (GTRACK_measurementPoint *)trackerProcObj->pointCloud;

    currentDescr = trackerProcObj->targetDescrHandle->currentDescr;
    targetList = trackerProcObj->targetDescrHandle->tList[currentDescr];
    targetIndex = trackerProcObj->targetDescrHandle->tIndex[currentDescr];

    // Execute tracking
    gtrack_step(trackerProcObj->gtrackHandle, points, NULL, mNum, targetDescr, &tNum, targetIndex, NULL);

    for(n=0; n<tNum; n++) {
        targetList[n].tid  = (uint32_t)targetDescr[n].uid;
        targetList[n].posX = targetDescr[n].S[0];
        targetList[n].posY = targetDescr[n].S[1];
        targetList[n].velX = targetDescr[n].S[2];
        targetList[n].velY = targetDescr[n].S[3];
        targetList[n].accX = targetDescr[n].S[4];
        targetList[n].accY = targetDescr[n].S[5];

        memcpy(targetList[n].ec, targetDescr[n].EC, sizeof(targetDescr[n].EC));

        targetList[n].g = targetDescr[n].G;
    }
    /* Fill output parameters */
    if(tNum > 0)
        outParams->numTargets = tNum;
    else
        outParams->numTargets = 0;
    
    /* Target Indices exist only when we have both points AND targets */
    if ((mNum > 0) && (tNum > 0))
        outParams->numIndices = mNum;
    else
        outParams->numIndices = 0;

    outParams->currentTargetDesc = currentDescr;

    /* Clear inProgress state */
    trackerProcObj->inProgress = false;

exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is the TrackerProc DPU control function. 
 *
 *  @pre    DPU_TrackerProc_init() has been called
 *
 *  @param[in]  handle           TrackerProc DPU handle
 *  @param[in]  cmd              TrackerProc DPU control command
 *  @param[in]  arg              TrackerProc DPU control argument pointer
 *  @param[in]  argSize          TrackerProc DPU control argument size
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_TrackerProc_control
(
    DPU_TrackerProc_Handle     handle,
    DPU_TrackerProc_Cmd        cmd,
    void*                      arg,
    uint32_t                   argSize
)
{
    int32_t             retVal = 0;
    trackerProcObjType  *trackerProcObj;
    
    /* Get trackerProc data object */
    trackerProcObj = (trackerProcObjType *)handle;
    
    /* Sanity check */
    if(trackerProcObj == NULL)
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }

    /* Check if control() is called during processing time */
    if(trackerProcObj->inProgress == true)
    {
        retVal = DPU_TRACKERPROC_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        case DPU_TrackerProc_Cmd_sensorTiltCfg:
        {

            retVal = 0;//DPU_TRACKERPROC_EINVAL;
        }
        break;

        default:
            retVal = DPU_TRACKERPROC_ECMD;
            break;
    }
exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is the TrackerProc DPU deinit function. It frees up the 
 *   resources allocated during init.
 *
 *  @pre    DPU_TrackerProc_init() has been called
 *
 *  @param[in]  handle           TrackerProc DPU handle
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_TrackerProc_deinit
(
    DPU_TrackerProc_Handle handle
)
{
    trackerProcObjType  *trackerProcObj;
    int32_t             retVal = 0;

    /* Get trackerProc data object */
    trackerProcObj = (trackerProcObjType *)handle;

    /* Sanity Check */
    if(trackerProcObj == NULL)
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }
    else
    {
        /* Free memory */
        MemoryP_ctrlFree(handle, sizeof(trackerProcObj));
    }
exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      Utility function to convert Cartesian point cloud to Spherical.
 *
 *  @pre    None
 *
 *  @param[in]  pointCloudCartesianIn   Input point cloud in cartesian format 
 *  @param[out] pointCloudCartesianIn   Output point cloud in spherical format  
 *  @param[in]  numPoints               Number of input points
 *
 *  \ingroup    DPU_TRACKERPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_TrackerProc_CartesianToSpherical
(
    DPIF_PointCloudCartesian    *pointCloudCartesianIn,
    DPIF_PointCloudSpherical    *pointCloudSphericalOut,
    uint16_t                    numPoints
)
{
    int32_t retVal = 0;
    float   x, y, z;
    float   phi, theta;
    float   temp, range;
    int16_t count;

    /* Basic error Checking */
    if((pointCloudCartesianIn == NULL) || (pointCloudSphericalOut == NULL)) 
    {
        retVal = DPU_TRACKERPROC_EINVAL;
        goto exit;
    }

    if(numPoints == 0)
    {
        goto exit;
    }

    for(count = 0; count < numPoints; count++)
    {
        /* Just for better readibility */
        x = pointCloudCartesianIn[count].x;
        y = pointCloudCartesianIn[count].y;
        z = pointCloudCartesianIn[count].z;

        /* Calculate range */
        temp = (y*y) + (x*x) + (z*z);
        
        if(temp > 0)
        {
            range = sqrt(temp);
        }
        else
        {
            goto exit;
        }
        
        /* Calculate elevation angle */
        phi = asin(z/range);

        /* Calculate azimuth angle */
        theta = asin(x/(range * cos(phi)));

        /* Assign the values to output structure */
        pointCloudSphericalOut[count].range = range;
        pointCloudSphericalOut[count].azimuthAngle = theta;
        pointCloudSphericalOut[count].elevAngle = phi;
        pointCloudSphericalOut[count].velocity = pointCloudCartesianIn[count].velocity;
    }


exit:
    return retVal;

}
