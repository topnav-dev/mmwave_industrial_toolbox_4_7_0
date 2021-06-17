/**
 *   @file  classifierkNN_process.h
 *
 *   @brief
 *      Process function for kNN classifier.
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/ 
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

/** Detail description of kNN classifier process module
 *
 *  The module is the encapsulation of feature extraction and classification using kNN.
 *
 *  It calls a utility function to parse the input data, whose format follows the current TI people
 *  counting demo output format. The output is a list of active target IDs and their tags.
 *
 *  There are 2 utility functions which do target mangement pre-processing
 *  (Util_ClassifierkNN_preProc) and post-processing (Util_ClassifierkNN_preProc). Details are in the
 *  correspoding header files.
 *
*/

#ifndef CLASSIFIERKNN_PROCESS_H
#define CLASSIFIERKNN_PROCESS_H
#include <swpform.h>
#include <modules/utilities/radarOsal_malloc.h>

#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierkNN.h>
#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierSVM.h>
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>
#include <modules/classification/classificationUtilities/src/RADARDEMO_distanceUtil_priv.h>
//#include "Util_ClassifierkNN_inputParser.h"
//#include "Util_ClassifierkNN_targetManager.h"

// API structure definitions, copied from People counting demo
#define MAX_DETPNTS_PER_FRAME (800)

/**
 * \enum classifierkNN_process_errorCode
 *  \brief   enum for classifier kNN process error code.
 */

typedef enum
{
	CLASSIFIERKNN_PROCESS_NO_ERROR = 0,						/**< No error */
	CLASSIFIERKNN_PROCESS_CONFIG_ERR,						/**< Error in configuration. */ 
	CLASSIFIERKNN_PROCESS_FEATEXTR_INIT_ERR,				/**< Error initializing feature extraction handles. */ 
	CLASSIFIERKNN_PROCESS_CLASSIFIERKNN_INIT_ERR,				/**< Error initializing classifier kNN handles. */ 
	CLASSIFIERKNN_PROCESS_CLASSIFIERSVM_INIT_ERR,			/**< Error initializing classifier SVM handles. */ 
	CLASSIFIERKNN_PROCESS_FAIL_ALLOCATE_HANDLE,				/**< classifierkNN_process_create failed to allocate handle */ 
	CLASSIFIERKNN_PROCESS_FAIL_ALLOCATE_LOCALINSTMEM,		/**< classifierkNN_process_create failed to allocate memory for buffers in local instance  */ 
	CLASSIFIERKNN_PROCESS_INOUTPTR_NOTCORRECT,				/**< input and/or output buffer for classifierkNN_process_run are either NULL, or not aligned properly  */
	CLASSIFIERKNN_PROCESS_FEATEXTR_EXEC_ERR,				/**< Error executing feature extraction module. */ 
	CLASSIFIERKNN_PROCESS_CLASSIFIERKNN_EXEC_ERR				/**< Error executing classifier kNN module. */ 
} classifierkNN_process_errorCode;


/**
 * \enum classifierkNN_process_classifierTypes
 *  \brief   enum for classifier types supported.
 */

typedef enum
{
	CLASSIFIERKNN_PROCESS_TYPE_KMEANS = 0,				/**< kmeans */
	CLASSIFIERKNN_PROCESS_TYPE_KNN,						/**< kNN. */ 
	CLASSIFIERKNN_PROCESS_TYPE_SVMLIN,					/**< SVM-linear kernel. */ 
	CLASSIFIERKNN_PROCESS_TYPE_SVMGAUS,					/**< SVM-gaussian kernel. */ 
	CLASSIFIERKNN_PROCESS_TYPE_SVMPOLY,					/**< SVM-polynomial kernel. */ 
	CLASSIFIERKNN_PROCESS_TYPE_NOTSUPPORTED				/**< non-supported type. */ 
} classifierkNN_process_classifierTypes;


/**
 *  \struct   _classifierkNN_process_config_
 *
 *  \brief   Structure element of the list of descriptors for classifierkNN_process configuration.
 *
 */

typedef struct _classifierkNN_process_config_
{
	uint8_t    classifierSelector;				/**< Flag to select training/tag set to use, 
												if set to 0, use kmeans training/tag set, 
												if set to 1, use kNN traing/tag set,
												if set to 2, use SVM-liear, 
												if set to 3, use SVM-gaussian, 
												if set to 4, use SVM-polynomial.*/ 
	uint16_t   cbLen;                               /**<code book length*/
	uint32_t     maxNumTracks;  					/**< max number of tracks*/
	uint8_t     k;  								/**< k value for kNN*/
	uint8_t      minNpntsPObj;  	       		/**< Minimum number of points in the track to extract feature. If below, skip the frame for feature extraction.*/
	uint32_t     blockLen;  					/**< Feature extraction block length, in number of frames*/
	uint32_t     slideWinLen;  					/**< Feature extraction sliding window length, in number of frames*/
	uint8_t      outBufLen;  	       			/**< Output buffer length.*/
	float      gamma;  	       				/**< Smoothing factor for normal distanced targets.*/
	float      gamma1;  	       				/**< Smoothing factor for close distance targets.*/
	float      	 maxDisPosThr;  	       		/**< max disposition to declear a non-moving-clutter, currently not used*/
	float      neighborDistSqrThr;  	       		/**< Threshold for close neighboring targets, in distance squared, to use gamma1 as smoothing factor.*/
	float      histRangeBinSize;  	       		/**< Range histogram bin size, if range histogram is required.*/
	uint16_t      histNumRangeBin;  	       		/**< Number of range histogram bins, if range histogram is required.*/
	float      histDopplerBinSize;  	       	/**< Doppler histogram bin size, if Doppler histogram is required.*/
	uint16_t      histNumDopplerBin;  	       		/**< Number of Doppler histogram bins, if Doppler histogram is required.*/
} classifierkNN_process_config;




/**
 *  \enum classifierkNN_process_tags;
 *
 *  \brief   enum for kNN output tags.
 */

typedef enum
{
	CLASSIFIERKNN_PROCESS_TAGS_CLUTTER = -1,			/**< Output tag for moving clutter*/
	CLASSIFIERKNN_PROCESS_TAGS_HUAMAN = 1,			/**< Output tag for human*/
	CLASSIFIERKNN_PROCESS_TAGS_NOTYETCLASSIFIED = 0xFF			/**< Output tag for not yet classified*/
} classifierkNN_process_tags;

/**
 *  \enum classifierkNN_process_input;
 *
 *  \brief   enum for kNN input.
 */


typedef struct _classifierkNN_process_input_
{
    void 		*pointCloudTLV;				/**< Point to the input point cloud TLV for the current frame.*/ 
    void		*targetListTLV;				/**< Pointer to the input target list TLV for the current frame.*/ 
    void		*targetIndexTLV;			/**< Pointer to the input target index TLV for the current frame.*/ 
} classifierkNN_process_input;


/**
 *  \struct classifierkNN_process_stats;
 *
 *  \brief   struct for kNN process output.
 */


typedef struct _classifierkNN_process_stats_
{
    uint8_t 	statsBufLen;				/**< buffer len, i.e., total number of frames to store stats.*/
    uint8_t		statsBufCnt;				/**< buffer counters.*/
    uint8_t     *numActiveTgts;				/**< number of active targets per frame, length of statsBufLen.*/
    uint8_t     *numTrks;					/**< number of tracks per frame, length of statsBufLen.*/
    uint16_t    *numDetPnt;					/**< number of detected points per frame, length of statsBufLen.*/
    uint32_t    *preProcCycles; 			/**< Cycle cost for preproc, length of statsBufLen.*/
    uint32_t    *inputParseCycles; 			/**< Cycle cost for input parser, length of statsBufLen.*/
    uint32_t    *processingCycles; 			/**< Cycle cost for feature extraction and classification, length of statsBufLen.*/
    uint32_t    *postProcCycles; 			/**< Cycle cost for post processing, length of statsBufLen.*/
} classifierkNN_process_stats;


/**
 *  \struct classifierkNN_process_output;
 *
 *  \brief   struct for kNN process output.
 */


typedef struct _classifierkNN_process_output_
{
    uint8_t 	numActiveTrks;				/**< number of active tracks in this frame.*/
    uint8_t		*activeTrkIDs;				/**< active track IDs, only the first numActiveTrks are valid.*/
    classifierkNN_process_tags		*activeTrkTags;			/**< Tags for the active tracks, only the first numActiveTrks are valid.*/
    classifierkNN_process_stats   *stats;  /**< stats for the process.*/
} classifierkNN_process_output;

/** 
 *  \struct   _targetInstance_
 *
 *  \brief   Structure to store target information.
 *
 *
 */

typedef struct _targetInstance_
{
	uint8_t		activeTargetFlag;			 /**< Flag to indicate an active target */ 
	uint32_t    activeFrCnt; 				 /**< Counter for number of frames that this target has been active. */ 	
	float       intialPos[2];				 /**< target initial x/y position. */
	float 		currPos[2];					 /**< target current x/y position. */
	float       maxDisPosSqr; 				 /**< Maximum square disposition of the target. */
    float       *outputFeats;                /**< output features for this target, length of: number of features from the codebook. */
	classifierkNN_process_tags *classifierOutput;				/**< classifier output buffer, store total outBufLen of history. */
	uint8_t     classifierOutCnt;            /**< Counter for classifier output buffer*/
	float 		classifierDecisionCur;       /**< Current smoothed classifier output using smooting factors. */
	//float 		* neighborDistSqr; 			 /**< Distance to the neighbor targets, length of maxNumTracks, only the ones between active targets are populated. */
	float       minNeighborDist; 			 /**< minimum distance to other active targets. */
} targetInstance;


/** 
 *  \struct   _classifierkNN_process_handle_
 *
 *  \brief   Structure definition of handle for kNN classifier process.
 *
 *
 */

typedef struct _classifierkNN_process_handle_
{
	uint8_t    classifierSelector;				/**< Flag to select training/tag set to use, 
												if set to 0, use kmeans training/tag set, 
												if set to 1, use kNN traing/tag set,
												if set to 2, use SVM-liear, 
												if set to 3, use SVM-gaussian, 
												if set to 4, use SVM-polynomial.*/ 
	float      * trainingFeatSet;                       /**< pointer to the training set */ 
	int8_t     *trainingFeatSetTags; 					/**< pointer to the training set TAGS*/ 
	uint8_t    numTAGS;									/**< Number of unique tags in the training set.*/
	int8_t    *tagVal;									/**< List of unique tag values, length of numTAGS.*/
	uint8_t      numOutFeats;  	       					/**< Number of output features, matching the number of features in the codebook.*/
	uint8_t      *outkNNFeaturesCatIdx;  	       		/**< Indices buffer to the output feature categeory featExtrOutput, matching outkNNFeaturesCatIdx from the codebook.*/
	uint8_t      *outkNNFeaturesIdx;  	       		/**< Indices buffer to the output features featExtrOutput->pcloudFeatOut or featExtrOutput->trkFeatOut, matching outkNNFeaturesIdx from the codebook.*/
	uint8_t     kNNUniqueFeatCat;					/**< Number of unique categories in the codebook.*/
	uint16_t      *outkNNFeatureSrcCat; 				/**< List of feature src (upper 8-bit) and categories (lower 8-bit)for each output feature.*/
	uint32_t     maxNumTracks;  					/**< max number of tracks*/
	uint32_t     k;  								/**< k value for kNN*/
	uint8_t      minNpntsPObj;  	       		/**< Minimum number of points in the track to extract feature. If below, skip the frame for feature extraction.*/
	uint32_t     blockLen;  					/**< Feature extraction block length, in number of frames*/
	uint32_t     slideWinLen;  					/**< Feature extraction sliding window length, in number of frames*/
	uint8_t      outBufLen;  	       			/**< Output buffer length.*/
	float      gamma;  	       				/**< Smoothing factor for normal distanced targets.*/
	float      gamma1;  	       				/**< Smoothing factor for close distance targets.*/
	float      	 maxDisPosThr;  	       		/**< max disposition to declear a non-moving-clutter, currently not used*/
	float      neighborDistSqrThr;  	       		/**< Threshold for close neighboring targets, in distance squared, to use gamma1 as smoothing factor.*/
	float      histRangeBinSize;  	       		/**< Range histogram bin size, if range histogram is required.*/
	uint16_t      histNumRangeBin;  	       		/**< Number of range histogram bins, if range histogram is required.*/
	float      histDopplerBinSize;  	       	/**< Doppler histogram bin size, if Doppler histogram is required.*/
	uint16_t      histNumDopplerBin;  	       		/**< Number of Doppler histogram bins, if Doppler histogram is required.*/
	
	targetInstance *targetInfoList;              /**< target information list, per target, length of maxNumTracks.*/

	uint8_t rangeFeatCatReq;				/*Flag, if set to 1, indicating range feature category is required in feature extraction. */
	uint8_t dopplerFeatCatReq;				/*Flag, if set to 1, indicating doppler feature category is required in feature extraction. */
	uint8_t trackerFeatCatReq;				/*Flag, if set to 1, indicating any tracker feature category is required in feature extraction. */
	float *inputPointCloudX; /**<local buffer to hold x pos info per frame */
	float *inputPointCloudY; /**<local buffer to hold y pos info per frame */
	float *inputPointCloudDoppler; /**<local buffer to hold Doppler info per frame */
	float *inputPointCloudPower; /**<local buffer to hold Azimuth info per frame */
	
	void ** featExtrHandles; 					 /**< feature extraction handle list, per target, length of maxNumTracks.*/
	void ** classifierHandles;				 /**< classifier handle list, per target, length of maxNumTracks.*/

	RADARDEMO_featExtract_input *featExtrInput;   /**< input to feature extraction module, array of max number tracks */
	RADARDEMO_featExtract_output *featExtrOutput;   /**< Output from feature extraction module, arrary of max number of tracks */
	RADARDEMO_classifierkNN_input *classifierkNNInput; /**< input to classifier kNN module */
	int8_t classifierkNNOutput;					   /**< Output from classifier kNN module */
} classifierkNN_process_handle;



/*! 
   \fn     classifierkNN_process_create
 
   \brief   Create and initialize classifierkNN_process module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for classifierkNN_process module.

   \param[out]    errorCode
               Output error code.

			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	* classifierkNN_process_create(
                            IN  classifierkNN_process_config * moduleConfig, 
							OUT classifierkNN_process_errorCode * errorCode);

/*! 
   \fn     classifierkNN_process_delete
 
   \brief   Delete classifierkNN_process module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

extern void	classifierkNN_process_delete(
                            IN  void * handle);


/*! 
   \fn     classifierkNN_process_run
 
   \brief   Feature extraction module.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    kNNprocessInput
               Pointer to kNN processing input structure. See classifierkNN_process_input definition for details.
 
   \param[out]    kNNprocessOutput
               Pointer to kNN processing  output structure. See classifierkNN_process_output definition for details. 
 
   \ret error code

   \pre       none
 
   \post      none
  
 
 */
extern classifierkNN_process_errorCode	classifierkNN_process_run(
                            IN  void * handle,
							IN classifierkNN_process_input * kNNprocessInput,
							OUT classifierkNN_process_output  * kNNprocessOutput); 

 

#endif //CLASSIFIERKNN_PROCESS_H
