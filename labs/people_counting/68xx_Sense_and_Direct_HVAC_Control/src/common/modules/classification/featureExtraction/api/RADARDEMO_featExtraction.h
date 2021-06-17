/*! 
 *  \file   RADARDEMO_featExtraction.h
 *
 *  \brief   Header file for RADARDEMO_featExtraction module
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
/** Detail description of feature extraction module
 *
 *  The module provide a tool to extract a defined set of features from radar detected point cloud
 *  from low level signal processing chain (range, doppler, angle -- currently azimuth only, and 
 *  SNR estimation), as well as information from the tracking modules (position, velocity, acceleration, 
 *  tracker variance matrix and gating function etc). It assumes tracker always runs after the detection, 
 *  and detected points are localized to different tracks representing different objects within the scene.
 *
 *  The supported source for feature extraction:
 *  - Point cloud information
 *  - Track information
 *
 * For features from point cloud, only the following categories are supported
 *  - Range: from input (x, y) estimation 
 *  - Doppler: from input doppler estimation
 *
 * For each point cloud categories (range or doppler), all following 8 types of features will be extracted:
 *  - Center of mass (CoM) types, 2: mean and standard deviation (std). For each frame, and for each track/object, 
 *    CoM calculation is done using range/doppler estimation of point cloud associated with the track, and using 
 *    linear SNR estimation as the weight. Then mean or std is calculated from a configurable number of frames, defined in
 *    \ref RADARDEMO_featExtract_config.
 *  - envelope types, 5 total: bandwidth(BW), offest and torso bandwidth(torsoBW), as defined by Kim and Ling's paper 
 *    "Human Activity Classification Based on Micro-Doppler Signatures Using a Support Vector Machine"
 *    For each frame, and for each track/objec, an upper envelope and lower envelope of the signal is calculated as the max of the 
 *    signal and min of the signal. Then the BW, offest and torsoBW calculated from a a configurable number of frames as following:
 *       BW      = max(upperEnv) - min(lowerBW)
 *       offest  = mean(upperEnv) - mean(lowerBW)
 *       torsoBW = min(upperEnv) - max(lowerBW)
 *    As well as 
 *       peak to average ratio of upper envelope
 *       peak to average ratio of lower envelope
 *  - Histogram of the signal dispersion, hist(signal - centerOfMass). This is done per frame per object, and accumulated along frames. 
 *
 * For features from tracker, only the following categories are supported
 *  - Position (Pos) 
 *  - Velocity (Vel)
 *  - Acceleration (Accel)
 *  - EC matrix
 *  - G factor
 *
 * For each tracker categories (pos etc), all following 4 types statistics will be extracted:
 *  - Minimum, maximum, mean, std: For each frame, and for each track/object, the corresponding track information will be stored.
 *    Then min/max/mean/std can be calculated from a configurable number of frames.
 *
 * In the implementation, this module is configured, and executed per track/object. 
 *
 * The feature category is described in \ref RADARDEMO_featExtract_featDescr, must have one descriptor per feature category.
 * This feature descriptor, along with other parameters such as blkLen etc are part of \ref RADARDEMO_featExtract_config. 
 *
 * Input structure contains per track point cloud and tracker information, \ref RADARDEMO_featExtract_input.
 *
 * For each feature category described in RADARDEMO_featExtract_featDescr in the configuration, the module will extract all the features/statistics
 * associated to the feature category, and stores in the output structure.
 *
 * Output structure contains output features, they are arrange in the same order as defined feature decriptors in the configuration, 
 * and stored in the buffer that can be indexed by the feature selection Index defined in \ref RADARDEMO_featExtract_pntCloudFeatsIdx 
 * and \ref RADARDEMO_featExtract_featStatIdx. User can select subset of the features for classification.
 * 
 * There are a set of utility functions provided in RADARDEMO_featExtrUtil_priv.c function for feature extractions, including 
 * calculation of center of mass, min, max, mean, std and histogram of dispersion. If features provided in this RADARDEMO_featExtraction 
 * does not meet the user's need, the user can leverage these utility functions to calculate their own feature set. 
 */


#ifndef RADARDEMO_FEATEXTRACTION_H
#define RADARDEMO_FEATEXTRACTION_H

#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>


/**
 * \enum RADARDEMO_featExtract_errorCode
 *  \brief   enum for feature extraction error code.
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_NO_ERROR = 0,						/**< No error */
	RADARDEMO_FEATEXTRACT_FEATSOURCE_NOTSUPPORTED,			/**< Invalid source of feature: currently only support point cloud and tracker output, see 			
															     \ref RADARDEMO_featExtract_featSource for details*/
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEATCAT_NOTVALID,			/**< Invalid point cloud feature category, currently only support uD and uR, 
																 see \ref RADARDEMO_featExtract_pntCloudFeatCat for details. */ 
	RADARDEMO_FEATEXTRACT_TRKFEATCAT_NOTVALID,				/**< Invalid track feature category, see \ref RADARDEMO_featExtract_trackFeatCat for details. */
	RADARDEMO_FEATEXTRACT_FAIL_ALLOCATE_HANDLE,				/**< RADARDEMO_featExtract_create failed to allocate handle */ 
	RADARDEMO_FEATEXTRACT_FAIL_ALLOCATE_LOCALINSTMEM,		/**< RADARDEMO_featExtract_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_FEATEXTRACT_INOUTPTR_NOTCORRECT				/**< input and/or output buffer for RADARDEMO_featExtract_run are either NULL, or not aligned properly  */
} RADARDEMO_featExtract_errorCode;


/**
 *  \enum RADARDEMO_featExtract_featSource;
 *
 *  \brief   enum for feature extraction source.
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD = 0,			/**< feature source from point cloud*/
	RADARDEMO_FEATEXTRACT_FEATSRC_GTRACK,				/**< feature source from group tracker*/
	RADARDEMO_FEATEXTRACT_FEATSRC_NOT_SUPPORTED
} RADARDEMO_featExtract_featSource;

/**
 *  \enum RADARDEMO_featExtract_pntCloudFeatCat;
 *
 *  \brief   enum for point cloud feature category.
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER = 0,			/**< point cloud doppler feature*/
	RADARDEMO_FEATEXTRACT_PNTCLOUD_RANGE,				/**< point cloud range feature*/
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEATCAT_NOTSUPPORTED
} RADARDEMO_featExtract_pntCloudFeatCat;

/**
 *  \enum RADARDEMO_featExtract_pntCloudFeatsIdx;
 *
 *  \brief   enum for Index to select features that can be extracted from point cloud .
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_COMMEAN_IDX = 0,		/**< Index for point cloud feature mean of the center of mass. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_COMSTD_IDX,		    /**< Index for point cloud feature std of the center of mass. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_BW_IDX,				/**< Index for point cloud feature bandwidth. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_OFFSET_IDX,			/**< Index for point cloud feature offset. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_TORSOBW_IDX,			/**< Index for point cloud feature torso bandwidth. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_UPENVPAR_IDX,		/**< Index for point cloud feature Uppler envelope PAR. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_LWENVPAR_IDX,		/**< Index for point cloud feature Lower envelope PAR. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_HISTO_IDX,			/**< Index for point cloud feature disperssion histogram. */
	RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_MAX
} RADARDEMO_featExtract_pntCloudFeatsIdx;

/**
 *  \enum RADARDEMO_featExtract_trackFeatCat;
 *
 *  \brief   enum for tracker feature category.
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_TRACK_POS = 0,			/**< position feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_VEL,				/**< velocity feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_ACCEL,				/**< acceleration feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC0,				/**< EC0 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC1,				/**< EC1 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC2,				/**< EC2 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC3,				/**< EC3 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC4,				/**< EC4 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC5,				/**< EC5 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC6,				/**< EC6 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC7,				/**< EC7 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_EC8,				/**< EC8 feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACK_G,					/**< G feature from the tracker. */
	RADARDEMO_FEATEXTRACT_TRACKCAT_NOTSUPPORTED
} RADARDEMO_featExtract_trackFeatCat;


/**
 *  \enum RADARDEMO_featExtract_featStatIdx;
 *
 *  \brief   enum for Indexs to select feature statistics types.
 */

typedef enum
{
	RADARDEMO_FEATEXTRACT_STATS_MIN_IDX = 0,			/**<Index  for minimum */
	RADARDEMO_FEATEXTRACT_STATS_MAX_IDX,				/**<Index  for maximum */
	RADARDEMO_FEATEXTRACT_STATS_MEAN_IDX,				/**<Index  for mean */
	RADARDEMO_FEATEXTRACT_STATS_STD_IDX,				/**<Index  for standard deviation */
	RADARDEMO_FEATEXTRACT_FEATSTATTYPE_MAX
} RADARDEMO_featExtract_featStatIdx;


/**
 *  \struct   _RADARDEMO_featExtract_pntcloudFeat_
 *
 *  \brief   Structure for point cloud feature descriptions.
 */

typedef struct _RADARDEMO_featExtract_pntcloudFeat_
{
	RADARDEMO_featExtract_pntCloudFeatCat pntCloudFeatCat;		/**< point cloud feature category*/
	uint16_t          					  numHistBins;          /**< number of histogram bins. */ 
	float          						  histBinRes;          	/**< histogram bin resolution. */ 
}RADARDEMO_featExtract_pntcloudFeat;

/**
 *  \struct   _RADARDEMO_featExtract_trackFeat_
 *
 *  \brief   Structure for point cloud feature descriptions.
 */

typedef struct _RADARDEMO_featExtract_trackFeat_
{
	RADARDEMO_featExtract_trackFeatCat trackFeatCat;		/**< tracker feature category*/ 
}RADARDEMO_featExtract_trackFeat;


/**
 *  \struct   _RADARDEMO_featExtract_featDescr_
 *
 *  \brief   Structure for feature descriptor to RADARDEMO_featExtract module.
 */

typedef struct _RADARDEMO_featExtract_featDescr_
{
    RADARDEMO_featExtract_featSource		featSrc;				/**< Feature source*/ 
	union{
		RADARDEMO_featExtract_pntcloudFeat	    pntCloudFeatDescr;	/**< point cloud feature descriptions*/ 
		RADARDEMO_featExtract_trackFeat			trackFeatDescr;		/**< tracker feature descriptions*/ 
	} feats; 														/**< Feature descriptions*/ 
} RADARDEMO_featExtract_featDescr;


/**
 *  \struct   _RADARDEMO_featExtract_pntCloud_input_
 *
 *  \brief   Structure for point cloud information to RADARDEMO_featExtract module.
 *
 */

typedef struct _RADARDEMO_featExtract_pntCloud_input_
{
    float		*x;							/**< Pointer to x coordinate of detected points in m.*/ 
    float		*y;							/**< Pointer to y coordinate of detected points in m.*/ 
    float		*doppler;					/**< Pointer to doppler of detected points in m/s.*/ 
    float       *power;						/**< Pointer to power of detected points inlinear scale.*/ 
} RADARDEMO_featExtract_pntCloud_input;

/**
 *  \struct   _RADARDEMO_featExtract_track_input_
 *
 *  \brief   Structure for track information to RADARDEMO_featExtract module.
 *
 */

typedef struct _RADARDEMO_featExtract_track_input_
{
    float		posX;						/**< X coordinate of detected target in m.*/ 
    float		posY;						/**< Y coordinate of detected target in m.*/ 
    float		velX;						/**< X velocity of detected target in m/s.*/ 
    float		velY;						/**< Y velocity of detected target in m/s.*/ 
    float       accX;						/**< X acceleration of detected target in m/s2.*/ 
    float       accY;						/**< Y acceleration of detected target in m/s2.*/ 
    float		ec[9];						/**< Target Error covarience matrix, [3x3 float], in row major order, range, azimuth, doppler.*/ 
    float       g;							/**< Target gain factor.*/ 
} RADARDEMO_featExtract_track_input;

/**
 *  \struct   _RADARDEMO_featExtract_input_
 *
 *  \brief   Structure for input to RADARDEMO_featExtract module, currently only contains the point cloud and track information per track.
 *
 */

typedef struct _RADARDEMO_featExtract_input_
{
	uint8_t     forceUpdateFlag; 						/**< The feature extraction will only update the output every slideWinLen frames, 
															 except forceUpdateFlag is set to 1. User needs to study the performance of feature extract on 
															 different period than every slideWinLen frame. */
	uint32_t    numPntPerTrack;						    /**< Number of detected points associated with this track.*/ 
	RADARDEMO_featExtract_pntCloud_input pntcloudInput; /**< Input point cloud information */ 
	RADARDEMO_featExtract_track_input 	 trackInput; 	/**< Input track information */ 
} RADARDEMO_featExtract_input;

/**
 *  \struct   _RADARDEMO_featExtract_config_
 *
 *  \brief   Structure element of the list of descriptors for RADARDEMO_featExtract configuration.
 *
 */

typedef struct _RADARDEMO_featExtract_config_
{
	uint8_t      numFeatCat;                       /**< Number categories of feature to extract */ 
	RADARDEMO_featExtract_featDescr *featDescr; /**< Features descriptors, size of numFeats */ 
	uint32_t     blockLen;  					/**< Feature extraction block length, in number of frames*/
	uint32_t     slideWinLen;  					/**< Feature extraction sliding window length, in number of frames*/
	uint8_t      minNPtrPerTrack;  	       		/**< Minimum number of points in the track to extract feature. If below, skip the frame for feature extraction.*/
} RADARDEMO_featExtract_config;


/**
 *  \struct   _RADARDEMO_featExtract_pcloudfeatOut_
 *
 *  \brief   Structure for pointcloud feature output.
 *
 */
typedef struct _RADARDEMO_featExtract_pcloudfeatOut_
{
	float        featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_MAX]; /**< Order is defined in RADARDEMO_featExtract_pntCloudFeatsIdx*/
	uint32_t     *histogram;  					/**< Feature histogram buffer. Contains the histogram of the dispersion of the point cloud Doppler or range hist(input - center of mass).
													 User must set this pointer to NULL if not used. User must initialize the buffer to 0 when allocating this buffer, or reset it as needed. */
} RADARDEMO_featExtract_pcloudfeatOut;


/**
 *  \struct   _RADARDEMO_featExtract_featOut_
 *
 *  \brief   Structure for feature output.
 *
 */

typedef union _RADARDEMO_featExtract_featOut_
{
	RADARDEMO_featExtract_pcloudfeatOut        pcloudFeatOut;       /**< point cloud output feature values.*/ 
	float        trkFeatOut[RADARDEMO_FEATEXTRACT_FEATSTATTYPE_MAX]; /**< Order is defined in RADARDEMO_featExtract_featStatIdx*/
} RADARDEMO_featExtract_featOut;



/**
 *  \struct   _RADARDEMO_featExtract_output_
 *
 *  \brief   Structure for output of RADARDEMO_featExtract module.
 *
 */

typedef struct _RADARDEMO_featExtract_output_
{
	uint8_t      validFlag;                      /**< Flag to indicate valid output, based on the periodicity defined by slideWinLen */ 
	uint8_t      numFeatCats;                    /**< Number of categories of features to extract */ 
	RADARDEMO_featExtract_featOut *featOut; 	/**< List of features outputs, size of numFeats, in the same order of featDescr in RADARDEMO_featExtract_config. */ 
} RADARDEMO_featExtract_output;


/*! 
   \fn     RADARDEMO_featExtract_create
 
   \brief   Create and initialize RADARDEMO_featExtract module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_featExtract module.

   \param[out]    errorCode
               Output error code.

			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	* RADARDEMO_featExtract_create(
                            IN  RADARDEMO_featExtract_config * moduleConfig, 
							OUT RADARDEMO_featExtract_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_featExtract_delete
 
   \brief   Delete RADARDEMO_featExtract module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_featExtract_delete(
                            IN  void * handle);


/*! 
   \fn     RADARDEMO_featExtract_reset
 
   \brief   Reset RADARDEMO_featExtract module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	RADARDEMO_featExtract_reset(
                            IN  void * handle);

/*! 
   \fn     RADARDEMO_featExtract_run
 
   \brief   Feature extraction module.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    featExtrInput
               Pointer to feature extraction input structure. See RADARDEMO_featExtract_input definition for details.
 
   \param[out]    featExtrOutput
               Pointer to feature extraction output structure. See RADARDEMO_featExtract_featOut definition for details. 
 
   \ret error code

   \pre       none
 
   \post      none
  
 
 */
RADARDEMO_featExtract_errorCode	RADARDEMO_featExtract_run(
                            IN  void * handle,
							IN RADARDEMO_featExtract_input * featExtrInput,
							OUT RADARDEMO_featExtract_output  * featExtrOutput); 
#endif //RADARDEMO_FEATEXTRACTION_H

