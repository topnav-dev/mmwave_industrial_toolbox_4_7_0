/*! 
 *  \file   classifierkNN_process.c
 *
 *  \brief   Process function for kNN classifier.
 *
 * Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/ 
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
#include "classifierkNN_process.h"
#include "Util_ClassifierkNN_inputParser.h"
#include "Util_ClassifierkNN_targetManager.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif

#ifdef CCS
#include <modules/utilities/cycle_measure.h>
#else
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#endif


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
							OUT classifierkNN_process_errorCode * errorCode)
							
{
	int32_t		i, j;
	classifierkNN_process_handle * handle;
	
	*errorCode	=	CLASSIFIERKNN_PROCESS_NO_ERROR;

	if (moduleConfig->classifierSelector >= CLASSIFIERKNN_PROCESS_TYPE_NOTSUPPORTED)
	{
		*errorCode	=	CLASSIFIERKNN_PROCESS_CONFIG_ERR;
		return (NULL);
	}

	handle						=	(classifierkNN_process_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(classifierkNN_process_handle), 8);
	if (handle == NULL)
	{
		*errorCode	=	CLASSIFIERKNN_PROCESS_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	memset(handle, 0, sizeof(classifierkNN_process_handle));
	
	if (moduleConfig->classifierSelector == CLASSIFIERKNN_PROCESS_TYPE_KMEANS)
	{
		handle->trainingFeatSet		=	kmeansFeatSet;
		handle->trainingFeatSetTags	=	kmeansFeatSetTags;
		moduleConfig->cbLen			=	sizeof(kmeansFeatSetTags);
		moduleConfig->k				=	1;
	}
	else if (moduleConfig->classifierSelector == CLASSIFIERKNN_PROCESS_TYPE_KNN)
	{
		handle->trainingFeatSet		=	kNNtrainingFeatSet;
		handle->trainingFeatSetTags	=	kNNtrainingFeatSetTags;
	}
	handle->maxNumTracks		=	moduleConfig->maxNumTracks;
	handle->k					=	moduleConfig->k;
	handle->minNpntsPObj		=	moduleConfig->minNpntsPObj;
	handle->blockLen			=	moduleConfig->blockLen;
	handle->slideWinLen			=	moduleConfig->slideWinLen;
	handle->outBufLen			=	moduleConfig->outBufLen;
	handle->gamma				=	moduleConfig->gamma;
	handle->gamma1				=	moduleConfig->gamma1;
	handle->maxDisPosThr		=	moduleConfig->maxDisPosThr;
	handle->neighborDistSqrThr		=	moduleConfig->neighborDistSqrThr;
	handle->histRangeBinSize	=	moduleConfig->histRangeBinSize;
	handle->histNumRangeBin		=	moduleConfig->histNumRangeBin;
	handle->histDopplerBinSize	=	moduleConfig->histDopplerBinSize;
	handle->histNumDopplerBin	=	moduleConfig->histNumDopplerBin;
	handle->numOutFeats			=	numOutFeatsInCB;  
	handle->outkNNFeaturesCatIdx		=	outFeaturesCatIdx;  
	handle->outkNNFeaturesIdx			=	outFeaturesIdx;  
	handle->numTAGS		=	numTagsInCB;		
	handle->tagVal		=	&cbTagVals[0];		
	
	handle->outkNNFeatureSrcCat		=	(uint16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->numOutFeats * sizeof(uint16_t), 1);
	for (j = 0; j < (int32_t)handle->numOutFeats; j++)
	{
		handle->outkNNFeatureSrcCat[j] =	(featuresCat[outFeaturesCatIdx[j]].featSrc << 8) || (featuresCat[outFeaturesCatIdx[j]].feats.pntCloudFeatDescr.pntCloudFeatCat);
	}
	handle->kNNUniqueFeatCat	=	uniqueFeatCat;

	for (j = 0; j < uniqueFeatCat; j++ )
	{
		if (featuresCat[j].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
		{
			if(featuresCat[j].feats.pntCloudFeatDescr.pntCloudFeatCat == RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
			{
				handle->dopplerFeatCatReq = 1;
			}
			else
			{
				handle->rangeFeatCatReq = 1;
			}
		}
		else
		{
			handle->trackerFeatCatReq = 1;
		}
	}

	if (handle->dopplerFeatCatReq == 1)
	{
		handle->inputPointCloudDoppler	=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, MAX_DETPNTS_PER_FRAME * sizeof(float), 8);
	}
	if (handle->rangeFeatCatReq == 1)
	{
		handle->inputPointCloudX		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, MAX_DETPNTS_PER_FRAME * sizeof(float), 8);
		handle->inputPointCloudY		=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, MAX_DETPNTS_PER_FRAME * sizeof(float), 8);
	}
	handle->inputPointCloudPower	=	(float *)radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, MAX_DETPNTS_PER_FRAME * sizeof(float), 8);

	//construct feature descriptor from codebook description
	
	handle->targetInfoList		=	(targetInstance *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(targetInstance), 8);
	handle->featExtrHandles		=	(void **) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(void *), 8);
	handle->classifierSelector  =   moduleConfig->classifierSelector;
	handle->classifierHandles		=	(void **) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(void *), 8);
	for ( i = 0; i < (int32_t)handle->maxNumTracks; i++ )
	{
		memset(&handle->targetInfoList[i], 0, sizeof(targetInstance));
		handle->targetInfoList[i].outputFeats 	=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, numOutFeatsInCB * sizeof(float), 8);
		//handle->targetInfoList[i].neighborDistSqr 	=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(float), 8);
		handle->targetInfoList[i].classifierOutput 	=	(classifierkNN_process_tags *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->outBufLen * sizeof(classifierkNN_process_tags), 8);
		memset(handle->targetInfoList[i].outputFeats, 0, numOutFeatsInCB * sizeof(float));
		//memset(handle->targetInfoList[i].neighborDistSqr, 0, handle->maxNumTracks * sizeof(float));
		memset(handle->targetInfoList[i].classifierOutput, 0xff, handle->outBufLen * sizeof(classifierkNN_process_tags));
		handle->targetInfoList[i].classifierDecisionCur = 255.f;

		handle->targetInfoList[i].minNeighborDist = 1e10;
		//initialize all handle->maxNumTracks feature extraction handles
		{
			RADARDEMO_featExtract_config moduleCfg;
			RADARDEMO_featExtract_errorCode featExtrErrCode = RADARDEMO_FEATEXTRACT_NO_ERROR;
			memset(&moduleCfg, 0, sizeof(RADARDEMO_featExtract_config));
			
			moduleCfg.numFeatCat =  (int32_t)uniqueFeatCat;
			moduleCfg.featDescr = &featuresCat[0];			
			memcpy(moduleCfg.featDescr, &featuresCat[0], moduleCfg.numFeatCat *sizeof(RADARDEMO_featExtract_featDescr));
			for (j = 0; j < moduleCfg.numFeatCat; j++ )
			{
				if (moduleCfg.featDescr[j].featSrc	==	RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
				{
					if( moduleCfg.featDescr[j].feats.pntCloudFeatDescr.pntCloudFeatCat == RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
					{
						moduleCfg.featDescr[j].feats.pntCloudFeatDescr.numHistBins = handle->histNumDopplerBin;
						moduleCfg.featDescr[j].feats.pntCloudFeatDescr.histBinRes = handle->histDopplerBinSize;
					}
					else
					{
						moduleCfg.featDescr[j].feats.pntCloudFeatDescr.numHistBins = handle->histNumRangeBin;
						moduleCfg.featDescr[j].feats.pntCloudFeatDescr.histBinRes = handle->histRangeBinSize;
					}
				}
			}
			moduleCfg.blockLen		=	handle->blockLen;
			moduleCfg.slideWinLen	=	handle->slideWinLen;
			moduleCfg.minNPtrPerTrack		=	handle->minNpntsPObj;

			handle->featExtrHandles[i] = RADARDEMO_featExtract_create(&moduleCfg, &featExtrErrCode);
			if (featExtrErrCode > RADARDEMO_FEATEXTRACT_NO_ERROR)
			{
				*errorCode = CLASSIFIERKNN_PROCESS_FEATEXTR_INIT_ERR;
				return(NULL);
			}
		}
		
		//initialize all handle->maxNumTracks classifierkNN handles
		if(handle->classifierSelector <= CLASSIFIERKNN_PROCESS_TYPE_KNN)
		{
			RADARDEMO_classifierkNN_config moduleCfg;
			RADARDEMO_classifierkNN_errorCode classifierKNNErrCode = RADARDEMO_CLASSIFIERKNN_NO_ERROR;
			memset(&moduleCfg, 0, sizeof(RADARDEMO_classifierkNN_config));
			
			moduleCfg.k =  handle->k;
			moduleCfg.codebookDescr.cbLen		=	moduleConfig->cbLen;		
			moduleCfg.codebookDescr.numFeats	=	numOutFeatsInCB;		
			moduleCfg.codebookDescr.cbPtr		=	handle->trainingFeatSet;		
			moduleCfg.codebookDescr.tags		=	handle->trainingFeatSetTags;		
			moduleCfg.codebookDescr.numTAGS		=	numTagsInCB;		
			moduleCfg.codebookDescr.tagVal		=	cbTagVals;		

			handle->classifierHandles[i] = RADARDEMO_classifierkNN_create(
                            &moduleCfg, 
							&classifierKNNErrCode);

			if (classifierKNNErrCode > RADARDEMO_CLASSIFIERKNN_NO_ERROR)
			{
				*errorCode = CLASSIFIERKNN_PROCESS_CLASSIFIERKNN_INIT_ERR;
				return(NULL);
			}
		}
		else
		{
			RADARDEMO_classifierSVM_config moduleCfg;
			RADARDEMO_classifierSVM_errorCode classifierSVMErrCode = RADARDEMO_CLASSIFIERSVM_NO_ERROR;
			memset(&moduleCfg, 0, sizeof(RADARDEMO_classifierSVM_config));
			
			if (handle->classifierSelector == CLASSIFIERKNN_PROCESS_TYPE_SVMLIN)
				moduleCfg.kernelType =  RADARDEMO_CLASSIFIERSVM_LINEAR;
			else if (handle->classifierSelector == CLASSIFIERKNN_PROCESS_TYPE_SVMGAUS)
				moduleCfg.kernelType =  RADARDEMO_CLASSIFIERSVM_GAUSSIAN;
			else if (handle->classifierSelector == CLASSIFIERKNN_PROCESS_TYPE_SVMPOLY)
				moduleCfg.kernelType =  RADARDEMO_CLASSIFIERSVM_POLYNOMIAL;
			else
				moduleCfg.kernelType =  RADARDEMO_CLASSIFIERSVM_KERNAL_NOTSUPPORTED;

			handle->classifierHandles[i] = RADARDEMO_classifierSVM_create(
                            &moduleCfg, 
							&classifierSVMErrCode);

			if (classifierSVMErrCode > RADARDEMO_CLASSIFIERKNN_NO_ERROR)
			{
				*errorCode = CLASSIFIERKNN_PROCESS_CLASSIFIERSVM_INIT_ERR;
				return(NULL);
			}
		}
	}

	//input and output structure initialization
	handle->featExtrInput	=	(RADARDEMO_featExtract_input *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(RADARDEMO_featExtract_input), 8);
	handle->featExtrOutput	=	(RADARDEMO_featExtract_output *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->maxNumTracks * sizeof(RADARDEMO_featExtract_output), 8);
	for ( i = 0; i < (int32_t)handle->maxNumTracks; i++ )
	{
		handle->featExtrOutput[i].numFeatCats = uniqueFeatCat;
		handle->featExtrOutput[i].featOut	=	(RADARDEMO_featExtract_featOut *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, uniqueFeatCat * sizeof(RADARDEMO_featExtract_featOut), 8);
		for (j = 0; j < uniqueFeatCat; j++ )
		{
			if (featuresCat[j].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
			{
				if( featuresCat[j].feats.pntCloudFeatDescr.pntCloudFeatCat == RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
				{
					handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram = (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->histNumDopplerBin * sizeof(uint32_t), 8);
					memset(handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, 0, handle->histNumDopplerBin * sizeof(uint32_t));
				}
				else
				{
					handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram = (uint32_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->histNumRangeBin * sizeof(uint32_t), 8);
					memset(handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, 0, handle->histNumRangeBin * sizeof(uint32_t));
				}
			}
		}
	}

	handle->classifierkNNInput	=	(RADARDEMO_classifierkNN_input *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_classifierkNN_input), 8);
	handle->classifierkNNInput->numFeats	=	numOutFeatsInCB;
	handle->classifierkNNInput->featVal		=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, numOutFeatsInCB * sizeof(float), 8);

	return((void *)handle);
}


/*! 
   \fn     classifierkNN_process_delete
 
   \brief   Delete classifierkNN_process module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	classifierkNN_process_delete(
                            IN  void * handle)
{
	uint32_t i, j;
	classifierkNN_process_handle * inst;
	
	inst 			=	(classifierkNN_process_handle *)handle;

	radarOsal_memFree(inst->classifierkNNInput->featVal, numOutFeatsInCB * sizeof(float));
	radarOsal_memFree(inst->classifierkNNInput, sizeof(RADARDEMO_classifierkNN_input));
	radarOsal_memFree(inst->outkNNFeatureSrcCat, inst->numOutFeats * sizeof(uint16_t));

	for (i = 0; i < inst->maxNumTracks; i++)
	{
		for (j = 0; j < inst->kNNUniqueFeatCat; j++ )
		{
			if (featuresCat[j].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
			{
				if( featuresCat[j].feats.pntCloudFeatDescr.pntCloudFeatCat == RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
				{
					radarOsal_memFree(inst->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, inst->histNumDopplerBin * sizeof(uint32_t));
				}
				else
				{
					radarOsal_memFree(inst->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, inst->histNumRangeBin * sizeof(uint32_t));
				}
			}
		}
		radarOsal_memFree(inst->featExtrOutput[i].featOut, uniqueFeatCat * sizeof(RADARDEMO_featExtract_featOut));
	}
	radarOsal_memFree(inst->featExtrOutput, inst->maxNumTracks * sizeof(RADARDEMO_featExtract_output *));
	radarOsal_memFree(inst->featExtrInput, sizeof(RADARDEMO_featExtract_input));


	for ( i = 0; i < inst->maxNumTracks; i++ )
	{
		radarOsal_memFree(inst->targetInfoList[i].outputFeats, numOutFeatsInCB * sizeof(float));
		//radarOsal_memFree(inst->targetInfoList[i].neighborDistSqr, inst->maxNumTracks * sizeof(float));
		radarOsal_memFree(inst->targetInfoList[i].classifierOutput, inst->outBufLen * sizeof(classifierkNN_process_tags));

		RADARDEMO_featExtract_delete(inst->featExtrHandles[i]);
		if (inst->classifierSelector <= CLASSIFIERKNN_PROCESS_TYPE_KNN)
			RADARDEMO_classifierkNN_delete(inst->classifierHandles[i]);
		else
			RADARDEMO_classifierSVM_delete(inst->classifierHandles[i]);
	}

	radarOsal_memFree(handle, sizeof(inst));
}

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
classifierkNN_process_errorCode	classifierkNN_process_run(
                            IN  void * handle,
							IN classifierkNN_process_input * kNNprocessInput,
							OUT classifierkNN_process_output  * kNNprocessOutput)

{
	classifierkNN_process_errorCode errorCode = CLASSIFIERKNN_PROCESS_NO_ERROR;
	classifierkNN_process_handle	*classifierkNNInst;
	RADARDEMO_featExtract_errorCode FEerrorCode = RADARDEMO_FEATEXTRACT_NO_ERROR;
	int32_t i, j;
	uint32_t t1, t2;

	classifierkNNInst		=	(classifierkNN_process_handle *) handle;
	
	/* check error conditions */
	if ((kNNprocessInput == NULL) || ( kNNprocessInput->pointCloudTLV == NULL) 
		|| ( kNNprocessInput->targetIndexTLV == NULL) || ( kNNprocessInput->targetListTLV == NULL) )
	{
		errorCode		=		CLASSIFIERKNN_PROCESS_INOUTPTR_NOTCORRECT;
		return (errorCode);
	}
	kNNprocessOutput->numActiveTrks = 0;

#ifdef CCS
	t1 = ranClock();
#else
	t1 = Cycleprofiler_getTimeStamp();
#endif
	// parse input data
	Util_ClassifierkNN_inputParser(
			kNNprocessInput->pointCloudTLV,
			kNNprocessInput->targetListTLV,
			kNNprocessInput->targetIndexTLV,
			MAX_DETPNTS_PER_FRAME,
			classifierkNNInst->maxNumTracks,
			((float)(classifierkNNInst->histNumRangeBin >> 1) * classifierkNNInst->histRangeBinSize + 1.f),
			((float)(classifierkNNInst->histNumDopplerBin >> 1) * classifierkNNInst->histDopplerBinSize + 1.f),
			classifierkNNInst->inputPointCloudX,
			classifierkNNInst->inputPointCloudY,
			classifierkNNInst->inputPointCloudDoppler,
			classifierkNNInst->inputPointCloudPower,
			classifierkNNInst->featExtrInput);
#ifdef CCS
	t2 = ranClock();
#else
	t2 = Cycleprofiler_getTimeStamp();
#endif
	kNNprocessOutput->stats->inputParseCycles[kNNprocessOutput->stats->statsBufCnt] = t2 - t1;
	t1		=	t2;

	// Classifier preprocessing
	Util_ClassifierkNN_preProc(
			kNNprocessInput->targetListTLV,
			classifierkNNInst);
#ifdef CCS
	t2 = ranClock();
#else
	t2 = Cycleprofiler_getTimeStamp();
#endif
	kNNprocessOutput->stats->preProcCycles[kNNprocessOutput->stats->statsBufCnt] = t2 - t1;
	t1		=	t2;

	for (i = 0; i < (int32_t)classifierkNNInst->maxNumTracks; i++ )
	{
		if (classifierkNNInst->targetInfoList[i].activeTargetFlag)
		{
			FEerrorCode = RADARDEMO_featExtract_run(
                            classifierkNNInst->featExtrHandles[i],
							&classifierkNNInst->featExtrInput[i],
							&classifierkNNInst->featExtrOutput[i]);

			if (FEerrorCode > RADARDEMO_FEATEXTRACT_NO_ERROR)
			{
				errorCode		=		CLASSIFIERKNN_PROCESS_FEATEXTR_EXEC_ERR;
			}

			if (classifierkNNInst->featExtrOutput[i].validFlag)
			{
				// init classification input
				for (j = 0; j < classifierkNNInst->classifierkNNInput->numFeats; j++)
				{
					if ((classifierkNNInst->outkNNFeatureSrcCat[j] >> 8) == (uint8_t)RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
					{
						classifierkNNInst->classifierkNNInput->featVal[j]	=	
							classifierkNNInst->featExtrOutput[i].featOut[classifierkNNInst->outkNNFeaturesCatIdx[j]].pcloudFeatOut.featValue[classifierkNNInst->outkNNFeaturesIdx[j]];
					}
					else
					{
						classifierkNNInst->classifierkNNInput->featVal[j]	=	
							classifierkNNInst->featExtrOutput[i].featOut[classifierkNNInst->outkNNFeaturesCatIdx[j]].trkFeatOut[classifierkNNInst->outkNNFeaturesIdx[j]];
					}

				}
				if (classifierkNNInst->classifierSelector <= CLASSIFIERKNN_PROCESS_TYPE_KNN)
				{
					RADARDEMO_classifierkNN_errorCode KNNerrorCode = RADARDEMO_CLASSIFIERKNN_NO_ERROR;
					KNNerrorCode = RADARDEMO_classifierkNN_run(
								classifierkNNInst->classifierHandles[i],
								classifierkNNInst->classifierkNNInput,
								&classifierkNNInst->classifierkNNOutput);
					if (KNNerrorCode > RADARDEMO_CLASSIFIERKNN_NO_ERROR)
					{
						errorCode		=		CLASSIFIERKNN_PROCESS_CLASSIFIERKNN_EXEC_ERR;
					}
				}
				else
				{
					RADARDEMO_classifierSVM_errorCode SVMerrorCode = RADARDEMO_CLASSIFIERSVM_NO_ERROR;
					SVMerrorCode = RADARDEMO_classifierSVM_run(
								classifierkNNInst->classifierHandles[i],
								classifierkNNInst->classifierkNNInput,
								&classifierkNNInst->classifierkNNOutput);
					if (SVMerrorCode > RADARDEMO_CLASSIFIERSVM_NO_ERROR)
					{
						errorCode		=		CLASSIFIERKNN_PROCESS_CLASSIFIERKNN_EXEC_ERR;
					}
				}

				//update target database
				classifierkNNInst->targetInfoList[i].activeFrCnt++;
				classifierkNNInst->targetInfoList[i].classifierOutput[classifierkNNInst->targetInfoList[i].classifierOutCnt] = (classifierkNN_process_tags)classifierkNNInst->classifierkNNOutput;
				for (j = 0; j < classifierkNNInst->classifierkNNInput->numFeats; j++)
				{
					classifierkNNInst->targetInfoList[i].outputFeats[j]		=	classifierkNNInst->classifierkNNInput->featVal[j];
				}
				classifierkNNInst->targetInfoList[i].classifierOutCnt++;
				if (classifierkNNInst->targetInfoList[i].classifierOutCnt >= classifierkNNInst->outBufLen)
					classifierkNNInst->targetInfoList[i].classifierOutCnt = 0;
			}
		}
	}
#ifdef CCS
	t2 = ranClock();
#else
	t2 = Cycleprofiler_getTimeStamp();
#endif
	kNNprocessOutput->stats->processingCycles[kNNprocessOutput->stats->statsBufCnt] = t2 - t1;
	t1		=	t2;
	
	Util_ClassifierkNN_postProc(
			classifierkNNInst,
			kNNprocessOutput);
#ifdef CCS
	t2 = ranClock();
#else
	t2 = Cycleprofiler_getTimeStamp();
#endif
	kNNprocessOutput->stats->postProcCycles[kNNprocessOutput->stats->statsBufCnt] = t2 - t1;
	t1		=	t2;

	kNNprocessOutput->stats->numActiveTgts[kNNprocessOutput->stats->statsBufCnt] = kNNprocessOutput->numActiveTrks;
	kNNprocessOutput->stats->statsBufCnt++;
	if (kNNprocessOutput->stats->statsBufCnt >= kNNprocessOutput->stats->statsBufLen)
		kNNprocessOutput->stats->statsBufCnt = 0;

	return (errorCode);
}

