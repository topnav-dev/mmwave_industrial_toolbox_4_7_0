/*!
 *  \file   RADARDEMO_aoaEst2DCaponBF_priv.h
 *
 *  \brief   Header file for RADARDEMO_aoaEst2DCaponBF_priv.c
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef RADARDEMO_AOAEST2DCAPONBF_PRIV_H
#define RADARDEMO_AOAEST2DCAPONBF_PRIV_H

//! \brief include file <swpform.h>
//!
#include <people_counting/overhead_3d_people_counting/src/common/swpform.h>
#include <math.h>
#include <common/src/dpu/capon3d_overhead/modules/DoA/CaponBF2D/api/RADARDEMO_aoaEst2DCaponBF.h>
#include <common/src/dpu/capon3d_overhead/modules/postProcessing/matrixFunc/api/MATRIX_cholesky.h>
#include <common/src/dpu/capon3d_overhead/modules/utilities/radar_commonMath.h>

#ifndef _TMS320C6600
#include <common/src/dpu/capon3d_overhead/modules/utilities/radar_c674x.h>
#endif

#ifdef _WIN32
extern void DSPF_sp_fftSPxSP(int N, float *ptr_x, float *ptr_w, float *ptr_y, unsigned char *brev, int n_min, int offset, int n_max);
#else
#ifndef _TMS320C6600
#include <ti/dsplib/src/DSPF_sp_fftSPxSP/DSPF_sp_fftSPxSP.h>
#else
#include <ti/dsplib/src/DSPF_sp_fftSPxSP/c66/DSPF_sp_fftSPxSP.h>
#endif
#endif
#include <common/src/dpu/capon3d_overhead/modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>

#define RADARDEMO_AOAESTBF_PIOVER180 (3.141592653589793/180.0)      //!< define the pi/180
#define RADARDEMO_AOAESTBF_PI        (3.141592653589793f)           //!< define pi

//!  \brief   Subtask handle definition for 2D capon beamforming: range-angle heatmap generation.
//!
typedef struct _RADARDEMO_aoaEst2DRAHeatMap_handle_
{
    uint8_t      nRxAnt;                   /**< number of receive antennas used for range-angle heatmap estimation.*/
    uint8_t       *virtAntInd2Proc;         /**< The virtual antenna indices to process for range-angle heatmap gen.*/
    uint16_t      numInputRangeBins;        /**< number of input range bins to be processed.*/
    uint16_t      numChirps;				/**< number of chirps per frame.*/
    uint32_t      * scratchPad;             //!< Pointer to the scratch memory used in this function, must of size:
											//!<8-antenna:max{2*DopplerFFTsize*8bytes, 360*4bytes}
											//!<4-antenna:max{2*DopplerFFTsize*8bytes, 82*4bytes}
    float         gamma;                    //!< Diagnol loading scaling factor
    cplxf_t     * steeringVecAzim;          //!< steering vector in azimuth domain.
    cplxf_t     * steeringVecElev;          //!< steering vector in elevation domain.
    cplxf_t     * steeringVec;              //!< steering vector in azimuth-elevation domain.
    float         muStep;                   /**< Step value of mu --  elevation est step.*/
    float         muInit;                   /**< Initial value of mu -- elevation est starting value*/
    float         nuStep;                   /**< Step value of nu -- azimuth est step.*/
    float         nuInit;                   /**< Initial value of nu -- azimuth est starting value*/
    uint16_t      azimSearchLen;            //!< Azimuth search length
    uint16_t      elevSearchLen;            //!< Elevation search length
	uint8_t		  azimOnly;					/**< range-azimuth estimation only */
} RADARDEMO_aoaEst2D_RAHeatMap_handle;

//!  \brief   Subtask handle definition for 2D capon beamforming: azimuth-elevation heatmap generation, and detection.
//!
typedef struct _RADARDEMO_aoaEst2D_aeEst_handle_
{
    uint8_t       zoomInFlag;               /**< Flag to indication zoom in, or regular azimuth-elevation heatmap gen.*/
    uint8_t      nRxAnt;                   /**< number of receive antennas used for 2D azimuth-elevation estimation.*/
    uint16_t      numInputRangeBins;        /**< number of input range bins to be processed.*/
    uint16_t      numChirps;				/**< number of chirps per frame.*/
    float         muStep;					/**< Step value of mu --  elevation est step.*/
    float         muInit;					/**< Initial value of mu -- elevation est starting value*/
    float         nuStep;					/**< Step value of nu -- azimuth est step.*/
    float         nuInit;					/**< Initial value of nu -- azimuth est starting value*/
    uint8_t       *virtAntInd2Proc;         /**< The virtual antenna indices to process for range-angle heatmap gen.*/
    uint16_t      azimSearchLen;			//!< Azimuth search length
    uint16_t      elevSearchLen;			//!< Elevation search length
    cplxf_t       * steeringVec;			//!< steering vector: (for static processing, only set to address other than NULL if dynamic processing using 3D and steering vec can be reused)
    cplxf_t       * steeringVecAzimInit;    //!< steering vector: azimuth initial values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
    cplxf_t       * steeringVecAzimStep;    //!< steering vector: azimuth step values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
    cplxf_t       * steeringVecElevInit;    //!< steering vector: elevation initial values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
    cplxf_t       * steeringVecElevStep;    //!< steering vector: elevation step values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
    uint32_t      scratchPadSize;           //!< Size of the scratch memory used in this function.
    uint32_t      * scratchPad;             //!< Pointer to the scratch memory used in this function, must of size of scratchPadSize, must be aligned to 8-byte boundary.
    float         gamma;                    //!< Diagnol loading scaling factor
	uint8_t		  maxNpeak2Search;			/**< Max number of peak to search, max at 6. */
	uint8_t		  peakExpSamples;			/**< neighbor ppoint to do peak expansion on each side.*/
	uint8_t		  elevOnly;					/**< elevation estimation only */
	uint32_t	  * procRngBinMask;			/**< Processed range bin mask, to indicate which rangen bin's Rn inv has been calculated to avoid repeating calculations. */
	float         sideLobThr;				/**Sidelobe threshold */
	float         peakExpRelThr;			/**peak expansion relative threshold -- only include neighbors with power higher than  peakExpRelThr * peakPower*/
	float         peakExpSNRThr;			/**peak expansion SNR threshold -- only expand peak with SNR higher than this threshold */
	uint8_t       zoominFactor;				/**< Zoom in factor */
	uint8_t		  zoominNn8bors;			/**< number of neighbors to zoom in on each side.*/
	uint8_t       localMaxCheckFlag;		/**Local max check flag: 0 - no check; 1 - elevation domain only; 2 - both elevation and azimuth */
} RADARDEMO_aoaEst2D_aeEst_handle;

////!  \brief   Subtask handle definition for 2D capon beamforming: azimuth-elevation heatmap generation, and detection.
////!
//typedef struct _RADARDEMO_aoaEst2DStaticEst_handle_
//{
//    uint32_t      nRxAnt;                   /**< number of receive antennas, only support 4 and 8 antennas now!!!.*/
//    uint32_t      numInputRangeBins;        /**< number of input range bins to be processed.*/
//    uint32_t      numChirps;				/**< number of chirps per frame.*/
//    uint32_t      azimSearchLen;			//!< Azimuth search length
//    uint32_t      elevSearchLen;			//!< Elevation search length
//    float         muStep;					/**< Step value of mu --  elevation est step.*/
//    float         muInit;					/**< Initial value of mu -- elevation est starting value*/
//    float         nuStep;					/**< Step value of nu -- azimuth est step.*/
//    float         nuInit;					/**< Initial value of nu -- azimuth est starting value*/
//    cplxf_t       * steeringVecAzimInit;    //!< steering vector: azimuth initial values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
//    cplxf_t       * steeringVecAzimStep;    //!< steering vector: azimuth step values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
//    cplxf_t       * steeringVecElevInit;    //!< steering vector: elevation initial values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
//    cplxf_t       * steeringVecElevStep;    //!< steering vector: elevation step values for all virtual antennas (from ant 0 to ant 11 if 12 virtual antennas)
//    uint32_t      * scratchPad;             //!< Pointer to the scratch memory used in this function, must of size:
//											//!<8-antenna:max{2*DopplerFFTsize*8bytes, 360*4bytes}
//} RADARDEMO_aoaEst2D_StaticEst_handle;


//!  \brief   Handle definition for 2D capon beamforming.
//!
typedef struct _RADARDEMO_aoaEst2DCaponBF_handle_
{
	RADARDEMO_aoaEst2D_RAHeatMap_handle   * raHeatMap_handle;
	RADARDEMO_aoaEst2D_aeEst_handle       * aeEstimation_handle;
	uint8_t		detectionMethod;  /**< detection method,
									0: range-azimuth detection, plus 2D capon angle heatmap, and estimation (azimuth, elevation) with peak expansion
									1: range-azimuth detection, plus 2D capon angle heatmap, and estimation elevation only, with peak expansion
									2: range-azimuth-elevation detection, plus zoom-in for finer angle estimation. */
	uint8_t     staticProcEnabled;				/**< Enable static scene processing if set to 1 */
	uint8_t     staticAzimStepDeciFactor;		/**< static azimuth search step decimation factor */
	uint8_t     staticElevStepDeciFactor;		/**< static elevation search step decimation factor */
    uint16_t    staticElevSearchLen;			//!< static Azimuth search length
    uint16_t    staticAzimSearchLen;			//!< static Azimuth search length
    uint8_t		nRxAnt;                 /**< number of receive antennas used for 2D capon beamforming.*/
    uint16_t    numChirps;				/**< number of chirps per frame.*/

	uint16_t    dopplerFFTSize;         /**< Size of Doppler FFT.*/
	float		*dopTwiddle;			/**< Doppler FFT twiddle factor.*/
    uint32_t    * scratchPad;           //!< Pointer to the scratch memory used in this function, must of size:
										//!<8-antenna:max{2*DopplerFFTsize*8bytes, 360*4bytes}
										//!<4-antenna:max{2*DopplerFFTsize*8bytes, 82*4bytes}
    uint8_t     useCFAR4DopDet;         /**< Flag, if set to 1, indicating using CFAR-CASO for Doppler detection. Otherwise, use simple peak search.*/
	void		* dopCFARHandle;		/**< CFAR handle Doppler detection. Otherwise, if useCFAR4DopDet is set to 1.*/
	cplx16_t	* tempInputWOstatic;	/**< temporary buffer in scratch memory to hold the clutter removed signal per range bin.*/	
	RADARDEMO_detectionCFAR_input		* dopCFARInput;
	RADARDEMO_detectionCFAR_output		* dopCFARout; /**< Pointer Doppler CFAR output, if useCFAR4DopDet is set to 1, otherwise, NULL.*/
} RADARDEMO_aoaEst2DCaponBF_handle;


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_clutterRemoval
 *
 *   \brief   Per range bin, removal static clutter from the input signal.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed. 
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    outputAntSamples
 *               output samples from after clutter removal for the current (one) range bin.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    static_information
 *               Zero Doppler antenna samples for the range bin.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void		RADARDEMO_aoaEst2DCaponBF_clutterRemoval(
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN cplx16_t * inputAntSamples,
				OUT cplx16_t * outputAntSamples,
				OUT cplxf_t  * static_information);

/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_covInv
 *
 *   \brief   Per range bin, estimate the covariance matrices from input 1D FFT results, and calculate the inverse of these matrices.
 *
 *   \param[in]    invFlag
 *               Flag to indicate matrix inversion will be performed. 
 *               If set to 1, output invRnMatrices will contain inversion of covariance matrices.
 *               If set to 0, output invRnMatrices will contain covariance matrices without inversion.
 *
 *   \param[in]    gamma
 *               Scaling factor for diagnal loading.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of input chirps
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size TBD!!!
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    virtAntInd2Proc
 *               Input array that defines the antennas need to be processed, contains indices out of the full virtual antenna array indices.
 *
 *   \param[in]    inputAntSamples
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    invRnMatrices
 *               Output inverse of covariance matrices for the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void		RADARDEMO_aoaEst2DCaponBF_covInv(
				IN uint8_t clutterRmFlag,
				IN float gamma,
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN int32_t * scratch,
				IN uint8_t * virtAntInd2Proc,
				IN cplx16_t * inputAntSamples,
				OUT cplxf_t  * invRnMatrices
				);


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_raHeatmap
 *
 *   \brief   Use Capon beamforming to generate range angle heatmap per range bin.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    steerVecAnts
 *               number of antenna for steering vectors, it is the same as total number of virtual antennas in the system.
 *
 *   \param[in]    numAzimuthBins
 *               number of Azimuth bins
 *
  *   \param[in]    numElevationBins
 *               number of elevation bins
 *
*   \param[in]    steeringVecAzim
 *              steering vector for azimuth domain beamforming.
 *
*   \param[in]    steeringVecElev
 *              steering vector for elevtion domain beamforming.
 *
*   \param[in]    ant2Proc
 *              Antennas to process in the azimuth domain.
 *
 *   \param[in]    scratch
 *               scratch memory, must be of size of nRxAnt * nRxAnt * 2 * 4 bytes.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    maxValPerRngBin
 *               Output peak value in angle domain, per range bin.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    rangeAzimuthHeatMap
 *               Output range azimuth heatmap per range bin length of number of angle bins
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void RADARDEMO_aoaEst2DCaponBF_raHeatmap(
				IN uint8_t bfFlag,
				IN int32_t  nRxAnt,
				IN int32_t  steerVecAnts,
				IN int32_t  numAzimuthBins,
				IN int32_t  numElevationBins,
				IN cplxf_t * RESTRICT steeringVecAzim,
				IN cplxf_t * RESTRICT steeringVecElev,
				IN uint8_t * RESTRICT ant2Proc,
				IN int32_t * scratch,
				IN cplxf_t * RESTRICT invRnMatrices,
				IN float   * RESTRICT maxValPerRngBin,
				OUT float  * RESTRICT rangeAzimuthHeatMap);


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly
 *
 *   \brief   Use Capon beamforming to estimation elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input azimuth index for a detected point. 
 *
 *   \param[in]    aeEstimation_handle
 *               Input azimuth elevation estimation handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    elevEst
 *               Output elevation estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \param[out]    azimElevHeatMap
 *               Output azim-elev heatMap estimations.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern int32_t RADARDEMO_aoaEst2DCaponBF_aeEstElevOnly(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * aeEstimation_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter, 
				OUT float    * RESTRICT azimElevHeatMap
			);

/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim
 *
 *   \brief   Use Capon beamforming to estimation azimuth and elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input azimuth index for a detected point. 
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    elevEst
 *               Output elevation estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \param[out]    azimElevHeatMap
 *               Output azim-elev heatMap estimations.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern int32_t RADARDEMO_aoaEst2DCaponBF_aeEstElevAzim(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter, 
				OUT float    * RESTRICT azimElevHeatMap
			);


/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_aeEstZoomin
 *
 *   \brief   Use Capon beamforming to estimation azimuth and elevation angle in 2D domain.
 *
 *   \param[in]    bfFlag
 *               Flag to indicate which covariance matrix based beamforming will be performed. 
 *               If set to 1, Capon BF.
 *               If set to 0, conventional BF.
 *
 *   \param[in]    azimuthIdx
 *               Input coarse estimation of azimuth index for a detected point. 
 *
 *   \param[in]    elevationIdx
 *               Input coarse estimation of elevation index for a detected point. 
 *
 *   \param[in]    noise
 *               Input noise estimation for a detected point.  
 *
 *   \param[in]    aeEstimation_handle
 *               Input azimuth elevation estimation handle. 
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[in]    invRnMatrices
 *               Output inverse of covariance matrices or the current range bin, in order of upper triangle of nRxAnt x nRxAnt Hermitian matrix.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    azimEst
 *               Output azimuth estimations, length of numAngleOut are valid.
 *
 *   \param[out]    peakPow
 *               Output peak power estimations, length of numAngleOut are valid.
 *
 *   \param[out]    beamFilter
 *               Output beamFilter estimations, in complex float, length of numAngleOut * aeEstimation_handle->nRxAnt are valid.
 *
 *   \ret       numAngleOut: Number of estimated angle pairs.
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern int32_t RADARDEMO_aoaEst2DCaponBF_aeEstZoomin(
				IN uint8_t bfFlag,
				IN uint16_t azimuthIdx,
				IN uint16_t elevationIdx,
				IN float noise,
				IN RADARDEMO_aoaEst2D_aeEst_handle   * aeEstimation_handle,
				IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
				IN cplxf_t * RESTRICT invRnMatrices,
				OUT float  * RESTRICT azimEst, 
				OUT float  * RESTRICT elevEst, 
				OUT float  * RESTRICT peakPow, 
				OUT cplxf_t  * RESTRICT beamFilter
			);

/*!
 *   \fn     RADARDEMO_aoaEst2DCaponBF_dopperEstInput
 *
 *   \brief   Calculate the beam forming output for doppler estimation, for the current range bin and angle bins.
 *
 *   \param[in]    nRxAnt
 *               number of antenna
 *
 *   \param[in]    nChirps
 *               number of chirps
 *
 *   \param[in]    inputAntSamples
 *              Input 1D FFT results for the current range bin.
 *              Must be aligned to 8-byte boundary.
 *
 *   \param[in]    steeringVec
 *              steering vector for beamforming for the current azimuth bin.
 *
 *   \param[in]    bweights
 *               Beam filter coefficients for the detection.
 *
 *   \param[out]    bfOutput
 *               Beamforming output for the current range bin and azimuth bin.
 *               Must be in the order of real0, imag0, real1, imag1... as required by DSP LIB single precision floating-point FFT.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */
extern void		RADARDEMO_aoaEst2DCaponBF_dopperEstInput(
				IN int32_t nRxAnt,
				IN int32_t nChirps,
				IN cplx16_t * inputAntSamples,
				IN cplxf_t * bweights,
				OUT float * RESTRICT bfOutput
			);


/*!
 *   \fn     RADARDEMO_aoaEstimationBFSinglePeak_static
 *
 *   \brief   Use Bartlett beamforming to generate range azimuth heatmap per range bin for static scene.
 *
 *   \param[in]    sigIn
 *               input samples from radar cube (1D FFT output) for the current (one) range bin to be processed
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[in]    capon_handle
 *               Input capon beamforming handle. 
 *
 *   \param[out]    heatmap
 *               Output azimuth-elevation heatmap per range bin, length of number of angle bins, arranged in [elev][azim] format.
 *               Must be aligned to 8-byte boundary.
 *
 *   \param[out]    peakVal
 *               Output peak value in angle domain, per range bin.
 *               Must be aligned to 8-byte boundary.
 *
 *   \ret       none
 *
 *   \pre       none
 *
 *   \post      none
 *
 *
 */

extern void  RADARDEMO_aoaEstimationBFSinglePeak_static(
                            IN  cplxf_t * sigIn,
							IN RADARDEMO_aoaEst2DCaponBF_handle   * capon_handle,
                            OUT  float   * heatmap,
                            OUT float   * peakVal
							);

extern void tw_gen_float (float *w, int n);

#endif //RADARDEMO_AOAEST2DCAPONBF_PRIV_H
