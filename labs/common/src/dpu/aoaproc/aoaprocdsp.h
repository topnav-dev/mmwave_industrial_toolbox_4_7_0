/**
 *   @file  aoaprocdsp.h
 *
 *   @brief
 *      AoA DSP header file
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
#ifndef AOAPROC_DSP_H
#define AOAPROC_DSP_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Datapath files */
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpif/dp_error.h>
#include <common/src/dpu/aoastaticproc/aoaproc_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_AOAPROC_ERROR_CODE
 *  Base error code for the aoaProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument 
 */
#define DPU_AOAPROCDSP_EINVAL                  (DP_ERRNO_AOA_PROC_BASE-1)

/**
 * @brief   Error Code: Invalid radar cube format
 */
#define DPU_AOAPROCDSP_EINVAL__RADARCUBE_DATAFORMAT  (DP_ERRNO_AOA_PROC_BASE-2)

/**
 * @brief   Error Code: Out of memory 
 */
#define DPU_AOAPROCDSP_ENOMEM                               (DP_ERRNO_AOA_PROC_BASE-20)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDSP_HW_Resources::cfarRngDopSnrList
 */
#define DPU_AOAPROCDSP_ENOMEMALIGN_CFAR_DET_LIST            (DP_ERRNO_AOA_PROC_BASE-21)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDSP_HW_Resources::detObjOut
 */
#define DPU_AOAPROCDSP_ENOMEMALIGN_POINT_CLOUD_CARTESIAN    (DP_ERRNO_AOA_PROC_BASE-22)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDSP_HW_Resources::detObjOutSideInfo
 */
#define DPU_AOAPROCDSP_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO    (DP_ERRNO_AOA_PROC_BASE-23)

/**
 * @brief   Error Code: Memory not aligned for one of the buffers required by DPU
 */
#define DPU_AOAPROCDSP_ENOMEMALIGN_BUFF  (DP_ERRNO_AOA_PROC_BASE-24)

/**
 * @brief   Error Code: Insufficient memory allocated for azimuth static heat map
 */
#define DPU_AOAPROCDSP_ENOMEM__AZIMUTH_STATIC_HEAT_MAP      (DP_ERRNO_AOA_PROC_BASE-25)


/**
 * @brief   Error Code: azimuth heat-map flag enabled and single virtual antenna not valid combination
 */
#define DPU_AOAPROCDSP_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP (DP_ERRNO_AOA_PROC_BASE-27)

/**
 * @brief   Error Code: Number of Doppler chirps is not a multiple of 4
 */
#define DPU_AOAPROCDSP_ENUMDOPCHIRPS           (DP_ERRNO_AOA_PROC_BASE-28)

/**
 * @brief   Error Code: Number of Doppler bins is less than 16
 */
#define DPU_AOAPROCDSP_ENUMDOPBINS             (DP_ERRNO_AOA_PROC_BASE-29)

/**
 * @brief   Error Code: One of the provided scratch buffers has insufficient size
 */
#define DPU_AOAPROCDSP_ESCRATCHSIZE            (DP_ERRNO_AOA_PROC_BASE-30)

/**
 * @brief   Error Code: DPU configuration is invalid. It exceeds the maximum EDMA jump size of (32K - 1)
 */
#define DPU_AOAPROCDSP_EEXCEEDMAXEDMA           (DP_ERRNO_AOA_PROC_BASE-31)

/**
@}
*/

/*! @brief Alignment for memory allocation purpose.
 */
#define DPU_AOAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT          DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT (sizeof(int16_t))

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT (sizeof(float))

/*! @brief Alignment for memory allocation purpose of all remaining buffers required by the DPU. 
 */
#define DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT 8U

/**
 * @brief   Number of angle bins for the Azimuth/Elevation FFT
 */
#define DPU_AOAPROCDSP_NUM_ANGLE_BINS          (64U)


/**
* @brief
*  DPU control command
*
* @details
*  The enum defines the DPU supported run time command
*
*  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
*/
typedef enum DPU_AoAProcDSP_Cmd_e
{
 /*! @brief     Command to update field of view configuration, azimuth and elevation selected range*/
 DPU_AoAProcDSP_Cmd_FovAoACfg,
 /*! @brief     Command to update multi-object beam forming configuration */
 DPU_AoAProcDSP_Cmd_MultiObjBeamFormingCfg,
 /*! @brief     Command to update rx channel phase compensation */
 DPU_AoAProcDSP_Cmd_CompRxChannelBiasCfg,
 /*! @brief     Command to update Azimuth heat-map configuration */
 DPU_AoAProcDSP_Cmd_PrepareRangeAzimuthHeatMap,
 /*! @brief     Command to update static clutter removal configuration.*/
DPU_AoAProcDSP_Cmd_staticClutterCfg,
 /*! @brief     Command to update field of extended maximum velocity */
 DPU_AoAProcDSP_Cmd_ExtMaxVelocityCfg
} DPU_AoAProcDSP_Cmd;


/**
 * @brief
 *  AoAProcDSP DPU Hardware resources
 *
 * @details
 *  AoAProcDSP DPU Hardware resources
 *
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDSP_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     EDMA configuration for AOA data In (Ping)*/
    DPEDMA_ChanCfg       edmaPing;

    /*! @brief     EDMA configuration for AOA data In (Pong)*/
    DPEDMA_ChanCfg       edmaPong;
    
    /*! @brief     Radar Cube structure */
    DPIF_RadarCube      radarCube;

    /*! @brief      List of CFAR detected objects of @ref cfarRngDopSnrListSize elements,
     *              must be aligned to @ref DPU_AOAPROCDSP_CFAR_DET_LIST_BYTE_ALIGNMENT */
    DPIF_CFARDetList *cfarRngDopSnrList;

    /*! @brief      CFAR detected objects list size */
    uint16_t        cfarRngDopSnrListSize;

    /*! @brief      Detected objects output list sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudCartesian *detObjOut;

    /*! @brief      Detected objects output list in Spherical format, sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudSpherical *detObjOutSph;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT */
    DPIF_PointCloudSideInfo *detObjOutSideInfo;

    /*! @brief      This field dimensions several other fields in this structure as
     *              referred in their descriptions. It is determined by the dpc/application based
     *              on balancing between maximum number of objects expected to be
     *              detected in the scene (this can depend on configuration like cfar thresholds,
     *              static clutter removal etc) and memory and MIPS limitations. */
    uint32_t        detObjOutMaxSize;
    
    /*! @brief      Detected objects azimuth index for debugging,
     *              sized to @ref detObjOutMaxSize elements */
    uint8_t         *detObj2dAzimIdx;

    /*! @brief      Detected object elevation angle for debugging,
     *              sized to @ref detObjOutMaxSize elements, must be aligned to
     *              @ref DPU_AOAPROCDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT.\n 
     *              This buffer does not need to be allocated if elevation antenna is not used or not supported.*/
    float           *detObjElevationAngle;

    /*! @brief      Pointer to range-azimuth static heat map, this is a 2D FFT
     *              array in range direction (x[numRangeBins][numVirtualAntAzim]),
     *              at doppler index 0, sized to @ref azimuthStaticHeatMapSize elements of
     *              type cplx16ImRe_t.
     *              Alignment should be @ref DPU_AOAPROCDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT */
    cmplx16ImRe_t   *azimuthStaticHeatMap;

    /*! @brief      Number of elements of azimuthStaticHeatMap, this should be
     *              numVirtualAntAzim * numRangeBins */
    uint32_t        azimuthStaticHeatMapSize;

    /*! @brief      Pointer for buffer with window coefficients. Data in this buffer needs to be preserved
                    (or recomputed) between frames. Window must be symmetric, therefore only numDopplerChirps / 2
                    coefficients are needed to determine the window. Window coefficients must be provided by the application.\n
                    Size: sizeof(int32_t) * numDopplerChirps / 2\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
    */
    int32_t         *windowCoeff;

    /*!  @brief     2D FFT window size in bytes.*/
    uint32_t        windowSize;
    
    /*! @brief      Scratch buffer pointer for ping pong input from radar cube. \n
                    Size: 2 * sizeof(cmplx16ImRe_t) * numDopplerChirps \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx16ImRe_t   *pingPongBuf; 
    
    /*! @brief      Size of the Ping pong buffer */
    uint32_t        pingPongSize;

    /*! @brief      Pointer for twiddle table for Angle FFT. Data in this buffer needs to be preserved
                    (or recomputed) between frames. This array is populated by the DPU during config time.   \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *angleTwiddle32x32;

    /*! @brief      Size of the buffer for twiddle table */
    uint32_t        angleTwiddleSize;

    /*! @brief      Pointer for twiddle table for 2D FFT. Data in this buffer needs to be preserved
                    (or recomputed) between frames. This array is populated by the DPU during config time.   \n
                    Size: sizeof(cmplx32ReIm_t) * numDopplerBins\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *twiddle32x32;

    /*! @brief      Size of the buffer for twiddle table */
    uint32_t        twiddleSize;
     
    /*! @brief      Pointer for angle (azimuth and elevation) FFT input buffer.    \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *angleFftIn;

    /*! @brief      Size of the buffer for angle FFT input */
    uint32_t        angleFftInSize;

    /*! @brief      Pointer for elevation FFT output buffer.    \n
                    It's also used in extended max velocity feature for temporary buffer of Azimuth FFT output of hypothesis. \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT \n
                    This buffer does not need to be allocated if elevation antenna is not used or not supported,
                    and also extended max velocity feature is disabled.
     */
    cmplx32ReIm_t   *elevationFftOut;

    /*! @brief      Size of the buffer for elevation FFT output */
    uint32_t        elevationFftOutSize;

    /*! @brief      This scratch buffer is used for the following 2 operations:\n
                    1) 2D Windowing output buffer\n
                    2) Azimuth FFT output buffer\n 
                    Size: Max[(sizeof(cmplx32ReIm_t) * numDopplerBins) , (sizeof(cmplx32ReIm_t) * DPU_AOAPROCDSP_NUM_ANGLE_BINS)] \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    void            *scratch1Buff;

    /*! @brief      Size of the scratch1 buffer */
    uint32_t        scratch1Size;

    /*! @brief      This scratch buffer is used for the following 2 operations:\n
                    1) 2D FFT output buffer\n
                    2) Azimuth magnitude squared buffer\n 
                    Size: Max[(sizeof(cmplx32ReIm_t) * numDopplerBins) , (sizeof(float) * DPU_AOAPROCDSP_NUM_ANGLE_BINS)] \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDSP_BUFFERS_BYTE_ALIGNMENT
     */
    void            *scratch2Buff;

    /*! @brief      Size of the scratch2 buffer */
    uint32_t        scratch2Size;

#ifdef AOA_TYPE_2D	
	 /*! @brief      This scratch buffer is used for the AOA2D storage
	 
	 */
    void            *scratch3Buff;

    /*! @brief      Size of the scratch3 buffer */
    uint32_t        scratch3Size;	
	
#endif	
} DPU_AoAProcDSP_HW_Resources;

/**
 * @brief
 *  AoAProcDSP DPU static configuration parameters
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDSP_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;

    /*! @brief  Number of virtual azimuth antennas */
    uint8_t     numVirtualAntAzim;

    /*! @brief  Number of virtual elevation antennas */
    uint8_t     numVirtualAntElev;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief Number of chirps for Doppler computation purposes.
               For example, in TDM/BPM-MIMO scheme, this is the physical chirps
               in a frame per transmit antenna
               i.e numDopplerChirps = numChirpsPerFrame / numTxAntennas.\n
               Must be multiple of 4. */
    uint16_t    numDopplerChirps;

    /*! @brief  Number of doppler bins. Must be at least 16. Must be power of 2. */
    uint16_t    numDopplerBins;

    /*! @brief  Range conversion factor for range FFT index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;
    
    /*! @brief Flag that indicates if BPM is enabled. 
    BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;

} DPU_AoAProcDSP_StaticConfig;

/**
 * @brief
 *  AoAProcDSP DPU configuration
 *
 * @details
 *  The structure is used to hold the AoAProcDSP configuration
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDSP_Config_t
{
    /*! @brief     Data path common parameters */
    DPU_AoAProcDSP_StaticConfig   staticCfg;

    /*! @brief     Hardware resources */
    DPU_AoAProcDSP_HW_Resources res;

    /*! @brief     Dynamic configuration */
    DPU_AoAProc_DynamicConfig dynCfg;

}DPU_AoAProcDSP_Config;

/**
 * @brief
 *  Output parameters populated during Processing time
 *
 * @details
 *  The structure is used to hold the output parameters
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDSP_OutParams_t
{

    /*! @brief     AoAProcDSP stats */
    DPU_AoAProc_Stats       stats;

    /*! @brief      Number of AoA DPU detected points*/
    uint32_t numAoADetectedPoints;

}DPU_AoAProcDSP_OutParams;

/**
 * @brief
 *  AoAProcDSP DPU Handle
 *
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef void* DPU_AoAProcDSP_Handle ;

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDSP DPU initialization function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[out]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid AoAProcDSP handle
 *  @retval
 *      Error       - NULL
 */
DPU_AoAProcDSP_Handle DPU_AoAProcDSP_init
(
    int32_t*            errCode
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDSP DPU configuration function. It saves buffer pointer and configurations
 *  including system resources and configures EDMA for runtime.
 *
 *  @pre    DPU_AoAProcDSP_init() has been called
 *
 *  @param[in]  handle                 AoAProcDSP DPU handle
 *  @param[in]  aoaDspCfg              Pointer to AoAProcDSP configuration data structure
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDSP_config
(
    DPU_AoAProcDSP_Handle    handle,
    DPU_AoAProcDSP_Config    *aoaDspCfg
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDSP DPU process function.
 *
 *  @pre    DPU_AoAProcDSP_init(), DPU_AoAProcDSP_config() have been called
 *
 *  @param[in]  handle                  AoAProcDSP DPU handle
 *
 *  @param[in]  numObjsIn               Number of detected objects by CFAR DPU
 *
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success = 0
 *  @retval
 *      Error  != 0
 */
uint32_t DPU_AoAProcDSP_process
(
    DPU_AoAProcDSP_Handle    handle,
    uint32_t                 numObjsIn,
    DPU_AoAProcDSP_OutParams *outParams
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDSP DPU control function.
 *
 *  @pre     DPU_AoAProcDSP_init() has been called
 *
 *  @param[in]  handle           AoAProcDSP DPU handle
 *  @param[in]  cmd              AoAProcDSP DPU control command
 *  @param[in]  arg              AoAProcDSP DPU control argument pointer
 *  @param[in]  argSize          AoAProcDSP DPU control argument size
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDSP_control
(
   DPU_AoAProcDSP_Handle handle,
   DPU_AoAProcDSP_Cmd cmd,
   void *arg,
   uint32_t argSize
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDSP DPU deinitialization function. It frees up the
 *   resources allocated during initialization.
 *
 *  @pre    DPU_AoAProcDSP_init() has been called
 *
 *  @param[in]  handle           AoAProcDSP DPU handle
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDSP_deinit
(
    DPU_AoAProcDSP_Handle handle
);

#ifdef __cplusplus
}
#endif

#endif
