/**
 *   @file  staticdetprocdsp.h
 *
 *   @brief
 *      Implements Data path for static detection functionality.
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
#ifndef STATICDETPROC_DSP_H
#define STATICDETPROC_DSP_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */

/* DPIF Components Include Files */
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>

/* mmWave SDK Data Path Include Files */
#include <ti/datapath/dpif/dp_error.h>
#include <common/src/dpu/staticdetproc/staticdetproccommon.h>
#include <common/src/dpu/aoastaticproc/aoaproc_common.h>



#ifdef __cplusplus
extern "C" {
#endif

#define DPU_STATICDETPROC_PIOVER180 3.1415926/180
#define DPU_STATICDETPROC_PI 3.141926
#define STATICDET_RECORDING_NUMFRAMETOAVG 16
#define STATICDET_RECORDING_NUMFRAMETOAVGINV 0.0625 // 1/STATICDET_RECORDING_NUMFRAMETOAVG
#define STATICDET_MAX_NUM_ANT 12

#ifdef XWR68XX_ISK_ANTENNA_PATTERN
#define STATICDET_NUM_ADOA_ANT   8 // has to be 4 or 8
#define STATICDET_NUM_EDOA_ANT   4 // has to be 4 or 8 
#define STATICDET_NUM_ADOA_SET   1
#define STATICDET_NUM_EDOA_SET   4
#define STATICDET_HEATMAPGEN_NOTCH_ENABLE 0
#define STATICDET_NUM_EDOA_ANT_TOT STATICDET_NUM_EDOA_ANT * (STATICDET_NUM_EDOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_NUM_ADOA_ANT_TOT STATICDET_NUM_ADOA_ANT * (STATICDET_NUM_ADOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_MAX_NUM_ANT_FOR_HEATMAP 8
#endif

#ifdef XWR68XX_ODS_ANTENNA_PATTERN
#define STATICDET_NUM_ADOA_ANT   4 // has to be 4 or 8
#define STATICDET_NUM_EDOA_ANT   4 // has to be 4 or 8
#define STATICDET_NUM_ADOA_SET   1
#define STATICDET_NUM_EDOA_SET   2
#define STATICDET_HEATMAPGEN_NOTCH_ENABLE 1
#define STATICDET_NUM_EDOA_ANT_TOT STATICDET_NUM_EDOA_ANT * (STATICDET_NUM_EDOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_NUM_ADOA_ANT_TOT STATICDET_NUM_ADOA_ANT * (STATICDET_NUM_ADOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_MAX_NUM_ANT_FOR_HEATMAP 4

#endif

#ifdef XWR68XX_AOP_ANTENNA_PATTERN
#define STATICDET_NUM_ADOA_ANT   4 // has to be 4 or 8
#define STATICDET_NUM_EDOA_ANT   4 // has to be 4 or 8
#define STATICDET_NUM_ADOA_SET   1
#define STATICDET_NUM_EDOA_SET   2
#define STATICDET_HEATMAPGEN_NOTCH_ENABLE 1
#define STATICDET_NUM_EDOA_ANT_TOT STATICDET_NUM_EDOA_ANT * (STATICDET_NUM_EDOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_NUM_ADOA_ANT_TOT STATICDET_NUM_ADOA_ANT * (STATICDET_NUM_ADOA_SET <<  STATICDET_HEATMAPGEN_NOTCH_ENABLE)
#define STATICDET_MAX_NUM_ANT_FOR_HEATMAP 4

#endif

/**************************************************************************
 * Base Error Code for the mmWave data path DPUs
 **************************************************************************/
#define DP_ERRNO_STATICDET_PROC_BASE               (MMWAVE_ERRNO_DPU_BASE -700)


/** @addtogroup DPU_STATICDETPROC_ERROR_CODE
 *  Base error code for the staticDetProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_STATICDETPROCDSP_EINVAL                  (DP_ERRNO_STATICDET_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_STATICDETPROCDSP_ENOMEM                  (DP_ERRNO_STATICDET_PROC_BASE-2)

/**
 * @brief   Error Code: DPU is in progress
 */
#define DPU_STATICDETPROCDSP_EINPROGRESS             (DP_ERRNO_STATICDET_PROC_BASE-3)

/**
 * @brief   Error Code: Semaphore creation failed
 */
#define DPU_STATICDETPROCDSP_ESEMA                   (DP_ERRNO_STATICDET_PROC_BASE-4)

/**
 * @brief   Error Code: Bad semaphore status 
 */
#define DPU_STATICDETPROCDSP_ESEMASTATUS             (DP_ERRNO_STATICDET_PROC_BASE-5)

/**
 * @brief   Error Code: Unsupported radar cube format 
 */
#define DPU_STATICDETPROCDSP_ECUBEFORMAT             (DP_ERRNO_STATICDET_PROC_BASE-6)

/**
 * @brief   Error Code: Unsupported detection matrix format 
 */
#define DPU_STATICDETPROCDSP_EDETMFORMAT             (DP_ERRNO_STATICDET_PROC_BASE-7)

/**
 * @brief   Error Code: Insufficient detection matrix size
 */
#define DPU_STATICDETPROCDSP_EDETMSIZE               (DP_ERRNO_STATICDET_PROC_BASE-8)

/**
 * @brief   Error Code: Wrong window size
 */
#define DPU_STATICDETPROCDSP_EWINDSIZE               (DP_ERRNO_STATICDET_PROC_BASE-9)

/**
 * @brief   Error Code: Bad alignment for input buffer 
 */
#define DPU_STATICDETPROCDSP_EBUFALIGNMENT           (DP_ERRNO_STATICDET_PROC_BASE-10)

/**
 * @brief   Error Code: Number of Doppler chirps is not a multiple of 4
 */
#define DPU_STATICDETPROCDSP_ENUMDOPCHIRPS           (DP_ERRNO_STATICDET_PROC_BASE-11)

/**
 * @brief   Error Code: Number of Doppler bins is less than 16
 */
#define DPU_STATICDETPROCDSP_ENUMDOPBINS             (DP_ERRNO_STATICDET_PROC_BASE-12)

/**
 * @brief   Error Code: Invalid control command
 */
#define DPU_STATICDETPROCDSP_ECMD                    (DP_ERRNO_STATICDET_PROC_BASE-13)

/**
 * @brief   Error Code: One of the provided scratch buffers has insufficient size
 */
#define DPU_STATICDETPROCDSP_ESCRATCHSIZE            (DP_ERRNO_STATICDET_PROC_BASE-14)

/**
 * @brief   Error Code: DPU configuration is invalid. It exceeds the maximum EDMA jump size of (32K - 1)
 */
#define DPU_STATICDETPROCDSP_EEXCEEDMAXEDMA          (DP_ERRNO_STATICDET_PROC_BASE-15)

/**
 * @brief   Error Code: Bad BPM configuration
 */
#define DPU_STATICDETPROCDSP_EBPMCFG                 (DP_ERRNO_STATICDET_PROC_BASE-16)

/**
@}
*/

/*! Alignment for all buffers required by DPU */
#define DPU_STATICDETPROCDSP_BUFFER_BYTE_ALIGNMENT 8U
#define DPU_STATICDETPROCDSP_HEATMAP_BYTE_ALIGNMENT 8U
#define DPU_STATICDETPROCDSP_STEERINGVEC_BYTE_ALIGNMENT 8U

/*! @brief  Complex data type*/
typedef struct cplxfReIm_t_
{
    float real; /*!< @brief real part */
    float imag; /*!< @brief imaginary part */
} cplxfReIm_t;

/*! @brief  Complex data type */
typedef struct cplxfImRe_t_
{
    float imag; /*!< @brief imaginary part */
    float real; /*!< @brief real part */
} cplxfImRe_t;

/*!
 *  @brief   Handle for StaticDet Processing DPU.
 */
typedef void*  DPU_StaticDetProcDSP_Handle;

/**
 * @brief
 *  staticDetProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_EdmaCfg_t
{
    /*! @brief  EDMA driver handle. */
    EDMA_Handle edmaHandle;
    
    /*! @brief  EDMA configuration for Input data (Radar cube -> internal memory). */
    DPU_StaticDetProc_Edma edmaIn;
    
    /*! @brief  EDMA configuration for Output data (internal memory -> detection matrix). */
    DPEDMA_ChanCfg  edmaOut;
    
}DPU_StaticDetProcDSP_EdmaCfg;


/**
 * @brief
 *  Field of view - Static DET Configuration
 *
 * @details
 *  The structure contains the heatmap generation related stattic det configuration used in data path
 *
 *  \ingroup    DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_StaticDetProc_heatmapGenCfg_t
{
	/*! @brief    recording mode: recording mode and non-recording mode (detection mode) */
	uint32_t     recordingMode; 
	
	/*! @@brief  phase rotation for RX beamforming*/
	float        phaseRotDeg;
	
    /*! @brief    minimum range bin exported to Host*/
    uint32_t     minRangeBin;

    /*! @brief    max range bin exported to Host*/
    uint32_t     maxRangeBin;

    /*! @brief    minimum angle (in degrees) exported to Host*/
    float        maxAngleDeg;

    /*! @brief    angle step (in degrees) exported to Host*/
    float        angleStepDeg;

    /*! @brief    range bin used for thermal noise calculation */
    uint32_t     rangeBinForNoiseLevelCalc;
	

} DPU_StaticDetProc_heatmapGenCfg;


/**
 * @brief
 *  Field of view - Static DET Detection Configuration
 *
 * @details
 *  The structure contains the detection related stattic det configuration used in data path
 *
 *  \ingroup    DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_StaticDetProc_peakDetectionCfg_t
{
	
	/*! @brief   theshold used to indicate it is a local peak*/
    uint32_t     numAngleBinToSum;

    /*! @brief   minimum azimuth angle (in degrees) acceptable to the detection*/
    float        minAziAngleDeg;

    /*! @brief   maximum azimuth angle (in degrees)  acceptable to the detection*/
    float        maxAziAngleDeg;

    /*! @brief   minimum elevation angle (in degrees)  acceptable to the detection*/
    float        minEleAngleDeg;

    /*! @brief   maximum elevation angle (in degrees)  acceptable to the detection*/
    float        maxEleAngleDeg;

	/*! @brief   theshold used to indicate it is a local peak*/
    float        localPeakTH;

	/*! @brief   theshold used to indicate it is an big enough change*/
    float        heatmapDiffTH;

	/*! @brief   theshold used to indicate it is a significant adds-on*/
    float        significantTH;
	
	/*! @brief   Max allowed Difference between maxEInd and maxEInd_delta */
	uint32_t     eAngleBinDiffTH;

	/*! @brief   tilting angle for the sensor antenna */
	float        tiltAngleDeg; 
	
	/*! @brief   threshold used to indicate the heatmap is significant than the thermal noise level*/
	float        heatmapDiffToNoiseTH;

} DPU_StaticDetProc_peakDetectionCfg;

/**
 * @brief
 *  Doppler DPU HW configuration parameters
 *
 * @details
 *  The structure is used to hold the  HW configuration parameters
 *  for the Doppler DPU
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_HW_Resources_t
{
    /*! @brief  EDMA configuration */
    DPU_StaticDetProcDSP_EdmaCfg edmaCfg;
   
    /*! @brief  Radar Cube */
    DPIF_RadarCube radarCube;
               
    /*! @brief      Scratch buffer pointer for ping pong input from radar cube. \n
                    Size: 2 * sizeof(cmplx16ImRe_t) * numDopplerChirps \n
                    Byte alignment Requirement = @ref DPU_STATICDETPROCDSP_BUFFER_BYTE_ALIGNMENT
     */
    cmplx16ImRe_t   *pingPongBuf;

    /*! @brief      Size of the Ping pong buffer */
    uint32_t        pingPongSize;	
    
    /*! @brief      Range Azimuth heatmap     */
    float           *rangeAzimuthHeatmap;
    float           *rangeAzimuthHeatmap_record;
    float           *rangeElevationHeatmap;
    float           *rangeElevationHeatmap_record;
	uint32_t        heatmapSize; 
	float           *heatmapLocal; 
	float           *heatmapAvg;
	float           *heatmapDiff; 

	/*! @brief      saved steering vector for spectrum calculation */
    cplxfImRe_t     *steeringVec;
	uint32_t        steeringVecSize;
	

    /*! @brief      Scratch buffer pointer for angle BF output. \n
                    Size: \n
                         numDopplerBins * sizeof(cmplx32ReIm_t), if BPM is not enabled\n
                         numDopplerBins * sizeof(cmplx32ReIm_t) * numTxAntennas, if BPM is enabled\n
                    Byte alignment Requirement = @ref DPU_STATICDETPROCDSP_BUFFER_BYTE_ALIGNMENT
     */
    cplxfImRe_t     *zeroDopplerOut;
    cplxfImRe_t     *zeroDopplerSum;
    cplxfImRe_t     *highDopplerOut;

	
       
    /*! @brief      Detected objects output list sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudCartesian *detObjOut;

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
}DPU_StaticDetProcDSP_HW_Resources;

/**
 * @brief
 *  Doppler DPU static configuration parameters
 *
 * @details
 *  The structure is used to hold the static configuration parameters
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;
    
    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
    uint8_t     nAntForHeatmap;

	uint16_t     numRangeBins;
    /*! @brief  Number of Doppler chirps. Must be a multiple of 4. */
    uint16_t    numDopplerChirps;
    

	
	// needed for RX beamforming to reduce the FOV, useful for ODS or AOP antenna.
	bool antNotchEnable;
	cplxfImRe_t antNotchPhaseRot;

	bool        recordingMode;
	uint32_t    recordingFrameCnt; 
	bool        recordingHeatmapReady;
	    
    /*! @brief  Number of range bins */
    uint32_t    minRangeIndex;
	uint32_t    maxRangeIndex; 
	uint32_t    numRangeBinsForDetection;
	
	uint32_t    maxAAngleIndex;
	uint32_t    minAAngleIndex;

	uint32_t    maxEAngleIndex;
	uint32_t    minEAngleIndex;

	float       localPeakTH; 
	float       heatmapDiffTH; 
    float       significantTH;	
	uint32_t    eAngleBinDiffTH;	
	uint32_t    numAngleBinToSum;
	float       rangeStep; 
	float       tiltAngleDeg; 
	float       noiseLevel_record;
        
    /*! @brief Flag that indicates if BPM is enabled. 
        BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;
    
	float     estAngleRange;
	float     angleStepDeg;
	
    /*! @brief   threshold used to indicate the heatmap is significant than the thermal noise level*/
	float        heatmapDiffToNoiseTH;

    /*! @brief    range bin used for thermal noise calculation */
    uint32_t     rangeBinForNoiseLevelCalc;
	
	

}DPU_StaticDetProcDSP_StaticConfig;

/**
 * @brief
 *  staticDetProc dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration for the DPU
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_DynamicConfig_t
{   
    /*! @brief     Pointer to Rx channel compensation configuration */
    DPU_AoAProc_compRxChannelBiasCfg compRxChanCfg;

}DPU_StaticDetProcDSP_DynamicConfig;

/**
 * @brief
 *  staticDetProc DPU configuration parameters
 *
 * @details
 *  The structure is used to hold the configuration parameters
 *  for the Doppler Processing removal DPU
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_Config_t
{
    /*! @brief HW resources. */
    DPU_StaticDetProcDSP_HW_Resources  hwRes;
    
    /*! @brief Static configuration. */
    DPU_StaticDetProcDSP_StaticConfig  staticCfg;

    /*! @brief Dynamic configuration */
    DPU_StaticDetProcDSP_DynamicConfig dynCfg;    
}DPU_StaticDetProcDSP_Config;

/**
 * @brief
 *  DPU processing output parameters
 *
 * @details
 *  The structure is used to hold the output parameters DPU processing
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProcDSP_OutParams_t
{
	/*! @brief number of detected points for static chain */
	uint32_t  numDetectedPoints; 

    /*! @brief DPU statistics */
    DPU_StaticDetProc_Stats  stats;
}DPU_StaticDetProcDSP_OutParams;

/**
 * @brief
 *  Doppler DPU control commands
 *
 * @details
 *  The enum defines the Doppler DPU supported run time command
 *
 *  \ingroup DPU_STATICDETPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef enum DPU_StaticDetProcDSP_Cmd_e
{
 /*! @brief     Command to update field of view configuration, azimuth and elevation selected range*/
 DPU_StaticDetProcDSP_Cmd_FovStaticDetCfg,
 /*! @brief     Command to update rx channel phase compensation */
 DPU_StaticDetProcDSP_Cmd_CompRxChannelBiasCfg,
 /*! @brief     Command to update Azimuth heat-map configuration */
 DPU_StaticDetProcDSP_Cmd_PrepareRangeAzimuthHeatMap,
 /*! @brief     Command to update range angle heat-map recording */
 DPU_StaticDetProcDSP_Cmd_RangeAngleHeatMapCfg,
}DPU_StaticDetProcDSP_Cmd;

DPU_StaticDetProcDSP_Handle DPU_StaticDetProcDSP_init(int32_t* errCode);
void DPU_StaticDetProcDSP_steeringVecGen(DPU_StaticDetProcDSP_Config   *cfg);
int32_t DPU_StaticDetProcDSP_process(DPU_StaticDetProcDSP_Handle handle, DPU_StaticDetProcDSP_OutParams *outParams);
int32_t DPU_StaticDetProcDSP_deinit(DPU_StaticDetProcDSP_Handle handle);
int32_t DPU_StaticDetProcDSP_config(DPU_StaticDetProcDSP_Handle handle, DPU_StaticDetProcDSP_Config *cfg);
int32_t DPU_StaticDetProcDSP_control(DPU_StaticDetProcDSP_Handle handle, DPU_StaticDetProcDSP_Cmd cmd,
                                   void* arg, uint32_t argSize);


#ifdef __cplusplus
}
#endif

#endif
