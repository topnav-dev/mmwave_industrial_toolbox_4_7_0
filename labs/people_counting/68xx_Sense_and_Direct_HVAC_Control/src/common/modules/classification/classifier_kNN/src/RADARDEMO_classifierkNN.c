/*! 
 *  \file   RADARDEMO_classifierkNN.c
 *
 *  \brief   Classifier -- kNN module. 
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

#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierkNN.h>
#include <modules/classification/classifier_kNN/src/RADARDEMO_classifierkNN_priv.h>
#include <modules/classification/classificationUtilities/src/RADARDEMO_distanceUtil_priv.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*! 
   \fn     RADARDEMO_classifierkNN_create
 
   \brief   Create and initialize RADARDEMO_classifierkNN module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_classifierkNN module.

   \param[out]    errorCode
               Output error code.

			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_classifierkNN_create(
                            IN  RADARDEMO_classifierkNN_config * moduleConfig, 
							OUT RADARDEMO_classifierkNN_errorCode * errorCode)
							
{

	RADARDEMO_classifierkNN_handle * handle;
	
	*errorCode	=	RADARDEMO_CLASSIFIERKNN_NO_ERROR;

	if ( ( moduleConfig->codebookDescr.cbPtr == NULL ) 
	    || ( moduleConfig->codebookDescr.tags == NULL ) || ( moduleConfig->codebookDescr.tagVal == NULL ))
	{
		*errorCode	=	RADARDEMO_CLASSIFIERKNN_INVALID_CFG;
		return (NULL);
	}
	
	if (( moduleConfig->k < 1 ) || ( moduleConfig->k > moduleConfig->codebookDescr.cbLen))
	{
		*errorCode	=	RADARDEMO_CLASSIFIERKNN_INVALID_CFG;
		return (NULL);
	}
	
		
	handle						=	(RADARDEMO_classifierkNN_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_classifierkNN_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_CLASSIFIERKNN_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	memset(handle, 0, sizeof(RADARDEMO_classifierkNN_handle));
	
	handle->numFeats			=	moduleConfig->codebookDescr.numFeats;
	handle->cbLen				=	moduleConfig->codebookDescr.cbLen;
	handle->cbPtr				=	moduleConfig->codebookDescr.cbPtr;
	handle->tags				=	moduleConfig->codebookDescr.tags;
	handle->k					=	moduleConfig->k;
	handle->numTAGS				=	moduleConfig->codebookDescr.numTAGS;
	handle->tagVal				=	moduleConfig->codebookDescr.tagVal;
	handle->distLocBuf			=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 1, handle->cbLen * sizeof(float), 8);
	handle->indicesLocalBuf		=	(uint16_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->k * sizeof(uint16_t), 1);
	handle->votes				=	(uint8_t *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->numTAGS * sizeof(uint8_t), 1);
	
	return((void *)handle);
}


/*! 
   \fn     RADARDEMO_classifierkNN_delete
 
   \brief   Delete RADARDEMO_classifierkNN module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_classifierkNN_delete(
                            IN  void * handle)
{
	RADARDEMO_classifierkNN_handle * inst;
	
	inst 			=	(RADARDEMO_classifierkNN_handle *)handle;
	
	radarOsal_memFree(inst->distLocBuf, inst->cbLen * sizeof(float));
	radarOsal_memFree(inst->indicesLocalBuf, inst->k * sizeof(uint16_t));
	radarOsal_memFree(inst->votes, inst->k * sizeof(uint8_t));
	radarOsal_memFree(handle, sizeof(inst));
}


/*! 
   \fn     RADARDEMO_classifierkNN_run
 
   \brief   Feature extraction module.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    classifierInput
               Pointer to feature extraction input structure. See RADARDEMO_classifierkNN_input definition for details.
 
   \param[out]    classifierOut
               Output tag of the classification output, must be a member of tagVal in the \ref RADARDEMO_classifierkNN_cbDescr in \ref RADARDEMO_classifierkNN_config.
 
   \ret error code

   \pre       none
 
   \post      none
 
 */

RADARDEMO_classifierkNN_errorCode	RADARDEMO_classifierkNN_run(
                            IN  void * handle,
							IN RADARDEMO_classifierkNN_input * classifierInput,
							OUT int8_t  		*classifierOut)

{
	RADARDEMO_classifierkNN_errorCode errorCode = RADARDEMO_CLASSIFIERKNN_NO_ERROR;
	RADARDEMO_classifierkNN_handle	*classifierkNNInst;
	//float 	minDist;
	int32_t i;
	int8_t 	output;

	classifierkNNInst		=	(RADARDEMO_classifierkNN_handle *) handle;
	
	/* check error conditions */
	if (classifierInput->featVal == NULL) 
	{
		errorCode		=		RADARDEMO_CLASSIFIERKNN_INOUTPTR_NOTCORRECT;
		return (errorCode);
	}

	for (i = 0; i < classifierkNNInst->numTAGS; i++)
	{
		classifierkNNInst->votes[i] = 0;
	}
	
	//minDist 	=	RADARDEMO_distanceUtilSqrEuc(
	RADARDEMO_distanceUtilSqrEuc(
                            classifierkNNInst->cbLen,
							classifierkNNInst->numFeats,
                            classifierInput->featVal,
                            classifierkNNInst->cbPtr,
							&classifierkNNInst->indicesLocalBuf[0], 
							classifierkNNInst->distLocBuf);
	
	output 		=	classifierkNNInst->tags[classifierkNNInst->indicesLocalBuf[0]];
	if (classifierkNNInst->k == 1)
	{   //k = 1, output tag is the tag of the CW with minimum distance
		*classifierOut 	=	output;
	}
	else
	{ // k>1, need k minimum distances and there CW tags, output tags is decided on the majority vote.
		int32_t j, maxVote, maxVoteIdx;
		
		for (j = 0; j < classifierkNNInst->numTAGS; j++)
		{
			if ( output == classifierkNNInst->tagVal[j] )
				classifierkNNInst->votes[j]++;
		}
		for (i = 1; i < (int32_t) classifierkNNInst->k; i++ )
		{
			classifierkNNInst->distLocBuf[classifierkNNInst->indicesLocalBuf[i-1]] = 1e10;
//			minDist =  RADARDEMO_distanceFindMin(
			RADARDEMO_distanceFindMin(
                            classifierkNNInst->cbLen,
                            classifierkNNInst->distLocBuf,
							&classifierkNNInst->indicesLocalBuf[i]);
			output 		=	classifierkNNInst->tags[classifierkNNInst->indicesLocalBuf[i]];
							
			for (j = 0; j < classifierkNNInst->numTAGS; j++)
			{
				if ( output == classifierkNNInst->tagVal[j] )
					classifierkNNInst->votes[j]++;
			}
		}
		maxVote 	=	0;
		maxVoteIdx  = 	-1;
		for (j = 0; j < classifierkNNInst->numTAGS; j++)
		{
			if ( classifierkNNInst->votes[j] >  maxVote)
			{
				maxVote 	=	classifierkNNInst->votes[j];
				maxVoteIdx	=	j;
			}
		}
		*classifierOut 		=	classifierkNNInst->tagVal[maxVoteIdx];
	}
	
	return (errorCode);
}

