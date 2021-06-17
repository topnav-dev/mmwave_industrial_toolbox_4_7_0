/*
 *  
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
 *   @file  trackerproc.h
 *
 *   @brief
 *      Implements Tracker processing functionality.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef TRACKERPROC_H
#define TRACKERPROC_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */

/* mmWave SDK Data Path Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>

/* Industrial tool box */
#include <common/src/alg/gtrack/gtrack.h>

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_TRACKERPROC_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_TRACKERPROC_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT


#ifdef __cplusplus
extern "C" {
#endif

#define DP_ERRNO_TRACKER_PROC_BASE              (MMWAVE_ERRNO_DPU_BASE -900)

/** @addtogroup DPU_TRACKERPROC_ERROR_CODE
 *  Base error code for the trackerProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_TRACKERPROC_EINVAL                  (DP_ERRNO_TRACKER_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_TRACKERPROC_ENOMEM                  (DP_ERRNO_TRACKER_PROC_BASE-2)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_TRACKERPROC_EINTERNAL               (DP_ERRNO_TRACKER_PROC_BASE-3)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_TRACKERPROC_ENOTIMPL                (DP_ERRNO_TRACKER_PROC_BASE-4)

/**
 * @brief   Error Code: In Progress
 */
#define DPU_TRACKERPROC_EINPROGRESS             (DP_ERRNO_TRACKER_PROC_BASE-5)

/**
 * @brief   Error Code: Invalid control command
 */
#define DPU_TRACKERPROC_ECMD                    (DP_ERRNO_TRACKER_PROC_BASE-6)


/* @TODO: Remove this hardcoding and define these correctly in the DPC or elsewhere.
 */

#define TRACKERPROC_OUTPUT_TARGET_LIST 7
#define TRACKERPROC_OUTPUT_TARGET_INDEX 8

/**
 * @brief
 *  TrackerProc static configuration
 *
 * @details
 *  The structure is used to hold the static configuraiton used by trackerProc
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_TrackerProc_StaticConfig_t
{
    /*! @brief      Application level parameters */ 
    uint8_t                     trackerEnabled;         
    /*! @brief      Application level parameters */ 
    float                       sensorAzimuthTilt;
    /*! @brief      Application level parameters */ 
    GTRACK_allocationParams     allocationParams;
    /*! @brief      Application level parameters */ 
    GTRACK_gatingParams         gatingParams;
    /*! @brief      Application level parameters */ 
    GTRACK_stateParams          stateParams;
    /*! @brief      Application level parameters */ 
    GTRACK_varParams            variationParams;
    /*! @brief      Application level parameters */ 
    GTRACK_sceneryParams        sceneryParams;
    /*! @brief      Application level parameters */ 
    float                       accelerationParams[3];
    /*! @brief      Application level parameters */
    uint32_t                    trackingParamSet;                     
    /*! @brief      GTRACK lib Core Parameters */
    GTRACK_moduleConfig         gtrackModuleConfig;
}DPU_TrackerProc_StaticConfig; 


/**
 * @brief
 *  TrackerProc DPU Hardware resources
 *
 * @details
 *  TrackerProc DPU Hardware resources
 *
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_TrackerProc_Resources_t
{

    /*! @brief      Number of AoA DPU detected points*/
    uint32_t                    numDetObjIn;

    /*! @brief      Detected objects input list sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_TRACKERPROC_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudSpherical    *detObjIn;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_TRACKERPROC_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT */
    DPIF_PointCloudSideInfo     *detObjInSideInfo;
} DPU_TrackerProc_HW_Resources;

/**
 * @brief
 *  Tracking configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to configure Tracking module
 */
typedef struct DPU_TrackerProc_Config_t
{
    /*! @brief      trackerProc static configuration */
    DPU_TrackerProc_StaticConfig    staticCfg;

    /*! @brief      Hardware resources */
    DPU_TrackerProc_HW_Resources    res;

    /*! @brief      trackerProc dynamic configuration */
    /*@TODO: If needed*/
}DPU_TrackerProc_Config;

#if 0
/**
 * @brief
 *  TrackerProc DPU init param structure
 *
 * @details
 *  TBD
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_TrackerProc_InitParams_t
{
    /*! @brief   Tracker Handle */
    void        *gtrackHandle;
    
}DPU_TrackerProc_InitParams;
#endif

/**
 * @brief
 *  TrackerProc output parameter structure
 *
 * @details
 *  The structure is used to hold the output parameters for TrackerProc
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_TrackerProc_OutParams_t
{
    /*! @brief      Current target descriptor list flag (ping or pong) */
    bool                    currentTargetDesc;

    /*! @brief     rangeProcHWA stats */
    //DPU_TrackerProc_stats   stats;
    uint32_t                numTargets;
    uint32_t                numIndices;

}DPU_TrackerProc_OutParams;

/**
 * @brief
 *  TrackerProc control command
 *
 * @details
 *  The enum defines the TrackerProc supported run time command
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef enum DPU_TrackerProc_Cmd_e
{
    /*! @brief     Command to update configuration */
    DPU_TrackerProc_Cmd_sensorTiltCfg
}DPU_TrackerProc_Cmd;

/*!
 * @brief
 * Structure holds the target features
 *
 * @details
 * For each detected target, we report position, velocity, and measurement error covariance
 */
typedef struct trackerProc_Target_t
{
    /*! @brief   tracking ID */
    uint32_t    tid;
    /*! @brief   Detected target X coordinate, in m */
    float        posX;
    /*! @brief   Detected target Y coordinate, in m */
    float        posY;
    /*! @brief   Detected target X velocity, in m/s */
    float        velX;
    /*! @brief   Detected target Y velocity, in m/s */
    float        velY;
    /*! @brief   Detected target X acceleration, in m/s2 */
    float        accX;
    /*! @brief   Detected target Y acceleration, in m/s2 */
    float        accY;

#ifdef GTRACK_3D
    /*! @brief   Detected target Z coordinate, in m */
    float        posZ;
    /*! @brief   Detected target Y velocity, in m/s */
    float        velZ;
    /*! @brief   Detected target Z velocity, in m/s */
    float        accZ;
    /*! @brief   Detected target dimensions */
    //float        dim[3];
#endif

#ifdef TRACKER_EC_OUTPUT    
#ifdef GTRACK_3D
    /*! @brief   Target Error covarience matrix, [4x4 float], in row major order, range, azimuth, elev, doppler */
    float       ec[16];
    float       g;
#else
    /*! @brief   Target Error covarience matrix, [3x3 float], in row major order, range, azimuth, doppler */
    float       ec[9];
    float       g;
#endif
#endif
    
} trackerProc_Target;
typedef uint8_t trackerProc_TargetIndex;

typedef struct trackerProc_TargetDescrHandle_t
{
    bool    currentDescr;
    trackerProc_Target  *tList[2];
    trackerProc_TargetIndex *tIndex[2];
    /*! @brief   UART processing time in usec */
    uint32_t     uartProcessingTime;
    /*! @brief   track processing time in usec */
    uint32_t     trackProcessingTime;
} trackerProc_TargetDescrHandle;


/*!
 * @brief
 * Structure holds the message body for the  Point Cloud
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler 
 */
/* Compatible with GTRACK_measurementPoint */
typedef struct trackerProc_Point_t
{
    /*! @brief Detected point range, in m */
    float   range;
    /*! @brief Detected point azimuth, in rad */
    float   azimuth;
    /*! @brief Detected point elevation, in rad */
#ifdef GTRACK_3D
    float   elevation;
#endif
    float   doppler;
    /*! @brief Range detection SNR, linear */
    float   snr;

} trackerProc_Point;
#if 0
typedef struct trackerProc_PointCloud_t
{
    trackerProc_Tl        header;
    trackerProc_Point     point[];
} trackerProc_PointCloud;
#endif

/**
 * @brief
 *  TrackerProc DPU Handle
 *
 *  \ingroup DPU_TRACKERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef void* DPU_TrackerProc_Handle;

/**
 * @brief
 *  TrackerProc DPU Object
 *
 * @details
 *  The structure is used to hold TrackerProc internal data object
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 */
typedef struct trackerProcObj_t
{
    /*! @brief  Structure holds the message body for the Point Cloud */
    DPU_TrackerProc_Handle          gtrackHandle;
    
    /*! @brief Point Cloud Size */
    uint32_t                        pointCloudSize; 
    
    /*! @brief  Structure holds the Input Point Cloud */
    trackerProc_Point               *pointCloud;
#if 0
    /*! @brief  Structure holds the Input Point Cloud TLV*/
    trackerProc_OutputPointCloud    *pointCloudTlv;
#endif
    /*! @brief   Target Descriptors */
    trackerProc_TargetDescrHandle   *targetDescrHandle;

    /*! @brief   Pointer to DPU config */
    DPU_TrackerProc_Config          pDpuCfg;    

    /*! @brief   trackerProc DPU is in processing state */
    bool                            inProgress;

} trackerProcObjType;

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
);

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
);

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
);

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
);

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
);

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
);

#ifdef __cplusplus
}
#endif

#endif
