/*!
 *  \file   RADARDEMO_aoaEst2DCaponBF.h
 *
 *  \brief   Header file for RADARDEMO_aoaEst2DCaponBF module
 *
 * Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef RADARDEMO_AOACAPONESTBF_H
#define RADARDEMO_AOACAPONESTBF_H

//! \brief include file <swpform.h>
//!
#include <people_counting/68xx_3D_people_counting/src/common/swpform.h>
//! \brief include file <radarOsal_malloc.h>
//!
#include <common/src/dpu/capon3d/modules/utilities/radarOsal_malloc.h>

// Input has to be one of the following type
#define RADARDEMO_AOARADARCUDE_RNGCHIRPANT
//#define RADARDEMO_AOARADARCUDE_RNGANTCHIRP

#define MAX_VIRTUAL_RXANT	(12)
#define MAX_DOPCFAR_DET		(10)
#define MAX_RANGEBINS	    (128)
#define MAX_NPNTS			(600)

#ifdef _TMS320C6X
//#define CAPON2DMODULEDEBUG
#endif

//!  \brief   Error code for BF AoA estimation module.
//!
typedef enum
{
    RADARDEMO_AOACAPONBF_NO_ERROR = 0,                   /**< no error */
    RADARDEMO_AOACAPONBF_ESTMETHOD_NOTSUPPORTED,         /**< number of antennas not supported */
    RADARDEMO_AOACAPONBF_NUMANT_NOTSUPPORTED,            /**< number of antennas not supported */
    RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_HANDLE,           /**< RADARDEMO_aoAEstBF_create failed to allocate handle */
    RADARDEMO_AOACAPONBF_FAIL_ALLOCATE_LOCALINSTMEM,	 /**< RADARDEMO_aoAEstBF_create failed to allocate memory for buffers in local instance  */
    RADARDEMO_AOACAPONBF_INOUTPTR_NOTCORRECT			 /**< input and/or output buffer for RADARDEMO_aoAEstBF_run are either NULL, or not aligned properly  */
} RADARDEMO_aoaEst2DCaponBF_errorCode;


//!  \brief   Configuration substructure RADARDEMO_aoaEst2D_rangeAngleCfg for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2D_rangeAngleCfg_
{
	float       searchStep; /**angle search resolution */
	float		mvdr_alpha; /**diagonol loading weight.*/
	uint8_t		detectionMethod;  /**< detection method,
									0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
									1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
									2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation. */
	uint8_t		dopplerEstMethod;  /**< Doppler estimation method, 0-single peak search, 1-CFAR.*/
} RADARDEMO_aoaEst2D_rangeAngleCfg;


//!  \brief   Configuration substructure RADARDEMO_aoaEst2D_2DAngleCfg for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2D_2DAngleCfg_
{
	float       elevSearchStep; /**eleveation search resolution */
	float		mvdr_alpha; /**diagonol loading weight.*/
	uint8_t		maxNpeak2Search;  /**< Max number of peak to search, max at 6. */
	uint8_t		peakExpSamples;  /**< neighbor ppoint to do peak expansion on each side.*/
	uint8_t     elevOnly;        /**< elevation estimation only */
	float       sideLobThr; /**Sidelobe threshold */
	float       peakExpRelThr; /**peak expansion relative threshold -- only include neighbors with power higher than  peakExpRelThr * peakPower*/
	float       peakExpSNRThr; /**peak expansion SNR threshold -- only expand peak with SNR higher than this threshold */
} RADARDEMO_aoaEst2D_2DAngleCfg;

//!  \brief   Configuration substructure RADARDEMO_aoaEst2D_2DZoomInCfg for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2D_2DZoomInCfg_
{
	uint8_t     zoominFactor;		/**< Zoom in factor */
	uint8_t		zoominNn8bors;		/**< number of neighbors to zoom in on each side.*/
	uint8_t		peakExpSamples;		/**< neighbor ppoint to do peak expansion on each side.*/
	float       peakExpRelThr;		/**peak expansion relative threshold -- only include neighbors with power higher than  peakExpRelThr * peakPower*/
	float       peakExpSNRThr;		/**peak expansion SNR threshold -- only expand peak with SNR higher than this threshold */
	uint8_t     localMaxCheckFlag;	/**Loca max check flag: 0 - no check; 1 - elevation domain only; 2 - both elevation and azimuth */
} RADARDEMO_aoaEst2D_2DZoomInCfg;

//!  \brief   Configuration substructure RADARDEMO_aoaEst2D_staticCfg for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2D_staticCfg_
{
	uint8_t     staticProcEnabled;			/**< Enable static scene processing if set to 1 */
	uint8_t     staticAzimStepDeciFactor;	/**< static azimuth search step decimation factor, over the azimuth search steps in RADARDEMO_aoaEst2D_rangeAngleCfg*/
	uint8_t     staticElevStepDeciFactor;	/**< static elevation search step decimation factor, over the elevation search steps in RADARDEMO_aoaEst2D_2DAngleCfg*/
} RADARDEMO_aoaEst2D_staticCfg;

//!  \brief   Configuration substructure RADARDEMO_aoaEst2D_staticCfg for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2D_dopCfarCfg_
{
	uint16_t	cfarDiscardLeft;  /**< Number of left cells discarded.*/
	uint16_t	cfarDiscardRight;  /**< Number of left cells discarded.*/
	uint16_t	refWinSize; /**< reference window size in each side in two directions for clutter variance estimation. */
	uint16_t	guardWinSize; /**< guard window size in each side in two directions. */
	float		thre; /**< threshold used for compare. */
} RADARDEMO_aoaEst2D_dopCfarCfg;


//!  \brief   Structure element of the list of descriptors for RADARDEMO_aoaEst2DCaponBF configuration.
//!
typedef struct _RADARDEMO_aoaEst2DCaponBF_config_
{
    RADARDEMO_aoaEst2D_rangeAngleCfg		rangeAngleCfg;						/**< range angle heatmap generation config.*/
	union {
		RADARDEMO_aoaEst2D_2DAngleCfg		azimElevAngleEstCfg;				/**< azimuth elevation angle estimation config.*/
		RADARDEMO_aoaEst2D_2DZoomInCfg		azimElevZoominCfg;					/**< azimuth elevation angle zoom in config.*/
	} angle2DEst;
	RADARDEMO_aoaEst2D_staticCfg			staticEstCfg;						/**< static scene angle estimation config.*/
	RADARDEMO_aoaEst2D_dopCfarCfg			dopCfarCfg;							/**< configurations, if CFAR is selected to use for Doppler estimation.*/
	int64_t                                 dummyfiller1;
	cplxf_t									phaseCompVect[MAX_VIRTUAL_RXANT];   /**< antenna partern configuration: phase compensation vector -- board dependant.*/
    int8_t                                  m_ind[MAX_VIRTUAL_RXANT];           /**< antenna partern configuration 1 .*/
    int8_t                                  n_ind[MAX_VIRTUAL_RXANT];           /**< antenna partern configuration 2.*/
    int8_t                                  phaseRot[MAX_VIRTUAL_RXANT];        /**< antenna partern configuration: phase rotation.*/
	float                                   fovCfg[2];                          /**< antenna partern configuration: field of view for azimuth fovCfg[0], and elevation fovCfg[1].*/
	int16_t									numInputRangeBins;					/**< Number of input range bins.*/
	int16_t									numInputChirps;						/**< Number of input chirps.*/
	int16_t									dopperFFTSize;						/**< Doppler FFT size.*/
	int8_t									nRxAnt;								/**< Number of virtual rx antennas.*/
	int16_t									numRAangleBin;						/**< Number of input chirps.*/
	int16_t									numAzimBins;						/**< Number of input chirps.*/
	int16_t									numElevBins;						/**< Number of input chirps.*/
} RADARDEMO_aoaEst2DCaponBF_config;


/**
 *  \struct   _RADARDEMO_aoaEst2DCaponBF_input_
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoaEst2DCaponBF module input.
 *
 *
 */

typedef struct _RADARDEMO_aoaEst2DCaponBF_input_
{
	uint8_t		  processingStepSelector;   /**< Flag to select which processing to be done, if set to 0, construct rangeAzimuthHeatMap; if set to 1, estimate azimuth-elevation angle and doppler for detected points. */
    cplx16_t      * inputRangeProcOutSamples;        /**< input samples after range processing for rangeBin rangeIndx, array in format of (nVirtRxAnt * nChirps). nVirtRxAnt = nTxAnt * nPhyRxAnt. It will be 
													      modified (clutter removed) if clutterRemovalFlag = 1. */
	uint16_t      rangeIndx;	            /**< Index to the current range bin to be processed (out of CFAR). */
    uint16_t      angleIndx;				 /**< Index to the angle bin for the detection (out of CFAR) */
    float         noise;					/**< Noise estimation for the detection (out of CFAR) */
	uint8_t		  fallBackToConvBFFlag;     /**< Flag to indicate falling back to conventional BF using covariance matrix if set to 1. */
	uint8_t       clutterRemovalFlag;       /**< flag to indicate clutter removal needed. */
    uint16_t      nChirps;                  /**< number of chirps to be used for covariance matrix estimation.*/
	uint8_t		  lastRB2Process;           /**< Flag to to indicate the last range bin to process. */
} RADARDEMO_aoaEst2DCaponBF_input;

/**
 *  \struct   _RADARDEMO_aoaEst2DCaponBF_moduleCycles_
 *
 *  \brief   Structure element of the cycle information if debug is enabled.
 *
 *
 */

typedef struct _RADARDEMO_aoaEst2DCaponBF_moduleCycles_
{
    int32_t       raDetCnt;							/**< range-angle detection counter. */
    int32_t       uniqueRngCnt;						/**< unique range counter for range-angle detection. */
    int32_t       dopDetCnt;						/**< Doppler detection counter. */
    int32_t       crCycles[MAX_RANGEBINS];			/**< cycles for clutter removal, array, one value per range bin. */
    int32_t       RArnCycles[MAX_RANGEBINS];		/**< cycles for range-angle Rn construction and inverse, array, one value per range bin. */
    int32_t       RAHeatmapCycles[MAX_RANGEBINS];	/**< cycles for range-angle heatmap gen, array, one value per range bin. */
    int32_t       AErnCycles[MAX_RANGEBINS];		/**< cycles for azimuth-elevation Rn construction and inverse, array, one value per unique range bin. */
    int32_t       AEEstCycles[MAX_NPNTS];	                /**< cycles for azimuth-elevation estimation, array, one value per detected (range-angle) point. */
    int32_t       dopEstCycles[MAX_NPNTS];	            /**< cycles for Doppler estimation, array, one value per detected (range-azim-elev) point. */

} RADARDEMO_aoaEst2DCaponBF_moduleCycles;

/**
 *  \struct   _RADARDEMO_aoaEst2DCaponBF_output_
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_aoaEst2DCaponBF module output.
 *
 *
 */

typedef struct _RADARDEMO_aoaEst2DCaponBF_output_
{
    float		* rangeAzimuthHeatMap;      /**< output range azimuth heatmap, array in format of (numInputRangeBins * nAzimuthBins).*/
    uint16_t	* dopplerIdx;               /**< Estimated Doppler index.*/
	uint8_t		numDopplerIdx;				/**< number of output doppler index.*/
	uint8_t		numAngleEst;				/**< number of output doppler index.*/
    float		* azimEst;                  /**< azimuth angle estimation.*/
    float		* elevEst;                  /**< elevation angle estimation.*/
    float		* peakPow;                  /**< peak power estimation.*/
    float		* dopplerDetSNR;			/**< output doppler detection SNR, if NULL, then same as range-azimuth detection SNR, otherwise, contains doppler CFAR detection linear SNR.*/
	cplxf_t		* static_information;		/**< Zero doppler samples for the range bins, for all the antennas, arranged in ant x rangeBin format.*/
    cplxf_t     * bwFilter;					/**< the beamweight filter for Doppler estimation, only used as output for processingStepSelector = 0 or 1. */
	cplxf_t		*invRnMatrices;				/**< Pointer to vovariance matrices memory, in the order number of range bins, and upper triangle of nRxAnt x nRxAnt Hermitian matrix, only used as input 
											     for processingStepSelector = 0. */
	float       * malValPerRngBin;			/**< output peak value in angle domain, per range bin.*/
#ifdef CAPON2DMODULEDEBUG
	RADARDEMO_aoaEst2DCaponBF_moduleCycles    *cyclesLog;
#endif
} RADARDEMO_aoaEst2DCaponBF_output;

 
/*!
 *   \fn     RADARDEMO_aoaEstimationBF_create
 *
 *   \brief   Create and initialize RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    moduleConfig
 *
 *   \param[out]    errorCode
 *
 *   \ret     void pointer to the module handle
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void * RADARDEMO_aoaEst2DCaponBF_create(
                            IN  RADARDEMO_aoaEst2DCaponBF_config * moduleConfig,
                            OUT RADARDEMO_aoaEst2DCaponBF_errorCode * errorCode);

/*!
 *   \fn     RADARDEMO_aoaEstimationBF_delete
 *
 *   \brief   Delete RADARDEMO_aoaEstimationBF module.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void RADARDEMO_aoaEst2DCaponBF_delete(
                            IN  void * handle);


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_run
 *
 *   \brief   Estimate the angle of arrival of each detected object using BF.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \param[in]    input
 *               Input antenna signal and noise power for the detected object.
 *
 *   \param[out]    estOutput
 *               Pointer to the estimation output.
 *   \ret  error code
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern RADARDEMO_aoaEst2DCaponBF_errorCode RADARDEMO_aoaEst2DCaponBF_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoaEst2DCaponBF_input * input,
                            OUT RADARDEMO_aoaEst2DCaponBF_output   * estOutput);

/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_static_run
 *
 *   \brief   Estimate the angle of arrival for static scene.
 *
 *   \param[in]    handle
 *               Module handle.
 *
 *   \param[in]    input
 *               Input antenna signal and noise power for the detected object.
 *
 *   \param[out]    estOutput
 *               Pointer to the estimation output.
 *   \ret  error code
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern RADARDEMO_aoaEst2DCaponBF_errorCode    RADARDEMO_aoaEst2DCaponBF_static_run(
                            IN  void * handle,
                            IN  RADARDEMO_aoaEst2DCaponBF_input * input,
                            OUT RADARDEMO_aoaEst2DCaponBF_output   * estOutput);

#endif //RADARDEMO_AOACAPONESTBF_H

