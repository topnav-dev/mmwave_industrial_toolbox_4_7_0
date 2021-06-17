/*!
 *  \file   radarProcess.h
 *
 *  \brief   Header file for 2D-Capon-based radar signal processing chain 
 *           from range FFT output till point cloud generated for tracker.
 *
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
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
 *
*/
#ifndef _RADARPROCESS_H
#define _RADARPROCESS_H

#include <common/src/dpu/capon3d_overhead/modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>
#include <common/src/dpu/capon3d_overhead/modules/DoA/CaponBF2D/api/RADARDEMO_aoaEst2DCaponBF.h>

#if defined (_TMS320C6X) || defined (LITTLE_ENDIAN_HOST)
#include <common/src/dpu/capon3d_overhead/modules/utilities/radar_commonMath.h>
#endif

#include <common/src/dpu/capon3d_overhead/include/swpform.h>

#if defined(_WIN32) || defined(CCS)
#include <stdio.h>
#endif



/**
 * @brief
 *  radarProcess DPU Handle
 *
 */
typedef void* DPU_radarProcess_Handle; 


#define  NUM_RADAR_TXANT        3    // 2 transmitting antennas
#define  NUM_RADAR_RXANT        4    // 4 receiving antennas

#define  DOA_OUTPUT_MAXPOINTS   (MAX_DYNAMIC_CFAR_PNTS * 4 + MAX_STATIC_CFAR_PNTS)
#define  MAX_RESOLVED_OBJECTS_PER_FRAME   DOA_OUTPUT_MAXPOINTS  //zz: src is defined in odmInterface.h
#define  MAX_DYNAMIC_CFAR_PNTS       150
#define  MAX_STATIC_CFAR_PNTS        150
#define  MAX_NUM_RANGE_BINS (256)

#if defined (SUBSYS_MSS) || defined (SUBSYS_DSS)
#include <ti/datapath/dpif/dpif_pointcloud.h>
#include <ti/datapath/dpif/dpif_detmatrix.h>


#else
/**
 * @brief
 *  Point cloud definition in spherical coordinate system
 */
typedef struct DPIF_PointCloudSpherical_t
{
    /*! @brief     Range in meters */
    float  range;

    /*! @brief     Azimuth angle in degrees in the range [-90,90],
     *             where positive angle represents the right hand side as viewed
     *             from the sensor towards the scene and negative angle
     *             represents left hand side */
    float  azimuthAngle;

    /*! @brief     Elevation angle in degrees in the range [-90,90], 
                   where positive angle represents above the sensor and negative
     *             below the sensor */
    float  elevAngle;

    /*! @brief  Doppler velocity estimate in m/s. Positive velocity means target
     *          is moving away from the sensor and negative velocity means target
     *          is moving towards the sensor. */
    float    velocity;
}DPIF_PointCloudSpherical;

/**
 * @brief
 *  Point cloud side information such as SNR and noise level
 *
 * @details
 *  The structure describes the field for a point cloud in XYZ format
 */
typedef struct DPIF_PointCloudSideInfo_t
{
    /*! @brief  snr - CFAR cell to side noise ratio in dB expressed in Q3 format  */
    int16_t  snr;

    /*! @brief  y - CFAR noise level of the side of the detected cell in dB expressed in 0.1 steps of dB */
    int16_t  noise;
}DPIF_PointCloudSideInfo;
/**
 * @brief
 *  Detection matrix buffer interface
 *
 * @details
 *  The structure defines the detection matrix buffer interface, including data property, 
 * size and pointer
 */
typedef struct DPIF_DetMatrix_t
{
    /*! @brief  Detection Matrix data Format @ref DPIF_DETMATRIX_FORMAT */
    uint32_t                datafmt;

    /*! @brief  Detection Matrix buffer size in bytes */
    uint32_t                dataSize;

    /*! @brief  Detection Matrix data pointer
                User could remap this to specific typedef using 
                information in @ref DPIF_DETMATRIX_FORMAT */
    void                    *data;
} DPIF_DetMatrix;
#endif

// Output point cloud structure to Gtracker
typedef struct _radarProcessOutputToTracker_
{
    int32_t object_count;                         //number of objects (points)
    DPIF_PointCloudSpherical          pointCloud[DOA_OUTPUT_MAXPOINTS];
    DPIF_PointCloudSideInfo           snr[DOA_OUTPUT_MAXPOINTS];
} radarProcessOutputToTracker;

typedef struct _radarProcessBenchmarkElem_
{
	uint32_t dynNumDetPnts;
	uint32_t dynHeatmpGenCycles;
	uint32_t dynCfarDetectionCycles;
	uint32_t dynAngleDopEstCycles;
	uint32_t staticNumDetPnts;
	uint32_t staticHeatmpGenCycles;
	uint32_t staticCfarDetectionCycles;
	uint32_t staticAngleEstCycles;
} radarProcessBenchmarkElem;

typedef struct _radarProcessBenchmarkObj_
{
	uint32_t bufferLen;
	uint32_t bufferIdx;
	radarProcessBenchmarkElem *buffer;
#ifdef CAPON2DMODULEDEBUG
	RADARDEMO_aoaEst2DCaponBF_moduleCycles    * aoaCyclesLog;
#endif
} radarProcessBenchmarkObj;

//user input configuration parameters
typedef struct _DPU_radarModuleConfig_
{
	//rangeFFT/Doppler parameters
	float	framePeriod;					/**< Frame period in msec. */
	uint16_t	numAdcSamplePerChirp;		/**< number of adc samples per chirp. */
	uint16_t	numAdcBitsPerSample;		/**< number of adc bits per sample. */
	uint16_t	numChirpPerFrame;			/**< number of chirps per frame. */
	uint16_t	numTxAntenna;				/**< number of antennas. */
	uint16_t	numAntenna;					/**< number of virtual antennas. */
	uint16_t	numPhyRxAntenna;			/**< number of physical RX antennas. */
	uint16_t    mimoModeFlag;				/**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	uint32_t    numTotalChirpProfile;		/**<number of chirp profiles*/
	uint32_t    numUniqueChirpProfile;		/**<number of unique chirp profiles*/
	float       chirpInterval;
    float       bandwidth;
    float       centerFreq;

	//detection parameters
	RADARDEMO_detectionCFAR_config dynamicCfarConfig;	/**< CFAR configuration for dynamic scene. */
	RADARDEMO_detectionCFAR_config staticCfarConfig;	/**< CFAR configuration for static scene. */
	RADARDEMO_aoaEst2DCaponBF_config doaConfig;			/**< 2D Capon DOA configuration. */

	float		dynamicSideLobeThr;			/**< CFAR sidelobe threshold for dynamic scene. */
	float		staticSideLobeThr;			/**< CFAR sidelobe threshold for static scene. */

	uint16_t	maxNumDetObj;				/**< max number of detected points. */
	uint8_t		dopplerOversampleFactor;	/**< doppler oversampling factor -- currently not in use. */

	uint16_t	numRangeBins;				/**< number of range bins, output from the init function -- in case to be used in framework. */
	uint32_t    heatMapMemSize;				/**< heatmap size, output from the init function -- in case to be used in framework. */
	float       *heatMapMem;				/**< heatmap pointer, output from the init function -- in case to be used in framework. */
	radarProcessBenchmarkObj * benchmarkPtr;	/**< pointer to benchmark structure, output from the init function -- in case to be used in framework. */
}DPU_radarProcessConfig_t;


// Output of the radar process
typedef struct _radarProcessOutput_
{
    radarProcessOutputToTracker		pointCloudOut;                         
    radarProcessBenchmarkElem       *benchmarkOut;
    DPIF_DetMatrix					heatMapOut;
} radarProcessOutput;

typedef enum
{
	PROCESS_OK = 0,
	PROCESS_ERROR_INIT_MEMALLOC_FAILED,
	PROCESS_ERROR_RANGEPROC_INIT_FAILED,
	PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_CFARPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_CFARPROC_INIT_FAILED,
	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_CFARPROC_NDET_EXCEEDLIMIT,
	PROCESS_ERROR_DOAPROC_NONSUPPORTEDMETHOD,
	PROCESS_ERROR_DOAPROC_INIT_FAILED,
	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED,
	PROCESS_ERROR_NOT_SUPPORTED
} DPU_ProcessErrorCodes;


/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU init function. It allocates memory to store
 *		its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              radarProcess initialization parameters.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *
 *  @retval
 *      Success     - valid radarProcess handle
 *  @retval
 *      Error       - NULL
 */
DPU_radarProcess_Handle DPU_radarProcess_init
(
    DPU_radarProcessConfig_t	* initParams,
    DPU_ProcessErrorCodes			* errCode
);

/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU config function. Currently not used.
 *
 *  @param[in]  handle                  radarProcess handle.
 *  @param[in]  sampleProcCfg           radarProcess configurations.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_SAMPLEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */
 
int32_t DPU_radarProcess_config
(
	DPU_radarProcess_Handle     hndle,
    DPU_radarProcessConfig_t	* initParams,
    DPU_ProcessErrorCodes		* errCode
);


/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU process function. It runs from the data path 
 *  processing chain, with configuration and buffers assiged at config time. 
 *
 *  @param[in]   handle                  radarProcess handle.
 *  @param[in]   pDataIn                 Input buffer for the processing -- range FFT output in radar cube
 *  @param[out]  pDataOut                3D point cloud data
 *  @param[out]  errCode                 Pointer to errCode generates from the API
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */

int32_t DPU_radarProcess_process (void *handle, cplx16_t * pDataIn, void * pDataOut, int32_t *errCode);

/**
 *  @brief      Perform Doppler processing. Processing is done per all antennas per range bin.
 *
 *  @param[in]  handle               Pointer to instance handler
 *  @param[in]  frameNum             Frame number
 *  @param[in]  rangeIdx             Input range index.
 *  @param[in]  pDataIn              Pointer to the input data (1D FFT output) for all antennas.
 *  @param[out] pDataOut             Pointer to the output integrated signal.
 *
 *  @remarks
 */

/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU deinit function. It release resources used for the DPU.
 *
 *  @param[in]  handle                  radarProcess handle.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */
int32_t DPU_radarProcess_deinit
(
    DPU_radarProcess_Handle		handle,
    int32_t						*errCode
);

#endif  // _PROCESS_H

/* Nothing past this point */
