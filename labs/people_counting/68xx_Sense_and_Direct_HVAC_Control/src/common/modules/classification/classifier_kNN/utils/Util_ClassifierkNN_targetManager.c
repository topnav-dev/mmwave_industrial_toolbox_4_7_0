/**
 *   @file  Util_ClassifierkNN_targetManager.c
 *
 *   @brief
 *      Utility function for target mangement, including pre/post classification processing.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#include "Util_ClassifierkNN_targetManager.h"


//! \copydoc Util_ClassifierkNN_preProc 
void Util_ClassifierkNN_preProc(
			IN void * targetListTLV,
			OUT classifierkNN_process_handle * handle
		)
{
	MmwDemo_output_message_targetList * targetListInput = (MmwDemo_output_message_targetList * )targetListTLV;
	MmwDemo_output_message_target * tlBody;
	int32_t i, j, numTarget, tind;
	uint32_t tl_len;
	uint8_t  trackExist;

	tlBody 		=	(MmwDemo_output_message_target *) targetListInput->target;
	tl_len	 	=	targetListInput->header.length;
	numTarget   = 	tl_len/sizeof(MmwDemo_output_message_target);

	for (i = 0; i < (int32_t)handle->maxNumTracks; i++)
	{
		trackExist	=	0;
		for (j = 0; j < numTarget; j++ )
		{
			if (tlBody[j].tid == i)
			{
				trackExist	=	1;
				tind		=	j;
			}
		}
		if ((handle->targetInfoList[i].activeTargetFlag == 1)  && (trackExist == 0))
		{
			//reset target when there is discoutinuity of tracks
			handle->targetInfoList[i].activeTargetFlag	=	0;
			handle->targetInfoList[i].activeFrCnt		=	0;
			memset(handle->targetInfoList[i].intialPos, 0, sizeof(handle->targetInfoList[i].intialPos));
			memset(handle->targetInfoList[i].currPos, 0, sizeof(handle->targetInfoList[i].currPos));
			handle->targetInfoList[i].maxDisPosSqr			=	0.f;
			memset(handle->targetInfoList[i].outputFeats, 0, handle->numOutFeats*sizeof(float));
			memset(handle->targetInfoList[i].classifierOutput, 0xff, handle->outBufLen*sizeof(classifierkNN_process_tags));
			handle->targetInfoList[i].classifierOutCnt	=	0;
			handle->targetInfoList[i].classifierDecisionCur	=	255.f;
			handle->targetInfoList[i].minNeighborDist = 1e10;

			/*memset(handle->targetInfoList[i].neighborDistSqr, 0, handle->maxNumTracks * sizeof(float));

			for (j = 0; j < (int32_t)handle->maxNumTracks; j++)
			{
				handle->targetInfoList[j].neighborDistSqr[i]	=	0.f;
			}*/
			
			RADARDEMO_featExtract_reset(handle->featExtrHandles[i]);
			for (j = 0; j < handle->featExtrOutput[i].numFeatCats; j++ )
			{
				if (handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram != NULL)
				{
					if (featuresCat[j].feats.pntCloudFeatDescr.pntCloudFeatCat == RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
						memset(handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, 0, handle->histNumDopplerBin*sizeof(uint32_t));
					else
						memset(handle->featExtrOutput[i].featOut[j].pcloudFeatOut.histogram, 0, handle->histNumRangeBin*sizeof(uint32_t));
				}
			}
		}
		if ((handle->targetInfoList[i].activeTargetFlag == 0)  && (trackExist == 1))
		{
			handle->targetInfoList[i].activeTargetFlag	=	1;
			handle->targetInfoList[i].intialPos[0]		=	tlBody[tind].posX;
			handle->targetInfoList[i].intialPos[1]		=	tlBody[tind].posY;
		}
		if ((handle->targetInfoList[i].activeTargetFlag == 1)  && (trackExist == 1))
		{
			handle->targetInfoList[i].currPos[0]		=	tlBody[tind].posX;
			handle->targetInfoList[i].currPos[1]		=	tlBody[tind].posY;
		}
	}
}


//! \copydoc Util_ClassifierkNN_postProc 
void Util_ClassifierkNN_postProc(
			INOUT classifierkNN_process_handle * handle,
			OUT classifierkNN_process_output *output
		)
{
	int32_t i, j, tempIdx, totalActiveTargets;
	float x, y, ftemp, dist, smoothingFactor;

	//update target distances and max disposition
	for (i = 0; i < (int32_t)handle->maxNumTracks; i++)
	{
		handle->targetInfoList[i].minNeighborDist = 1e10;
	}
	for (i = 0; i < (int32_t)handle->maxNumTracks; i++)
	{
		if (handle->targetInfoList[i].activeTargetFlag == 1)
		{
			//max disposition
			x	=	handle->targetInfoList[i].currPos[0] - handle->targetInfoList[i].intialPos[0];
			y	=	handle->targetInfoList[i].currPos[1] - handle->targetInfoList[i].intialPos[1];
			ftemp	=	x * x + y * y;
			dist	=	ftemp;
			if (dist > handle->targetInfoList[i].maxDisPosSqr)
				handle->targetInfoList[i].maxDisPosSqr		=	dist;

			//distances of active targets
			for (j = i+1; j < (int32_t)handle->maxNumTracks; j++)
			{
				if (handle->targetInfoList[j].activeTargetFlag == 1)
				{
					x	=	handle->targetInfoList[i].currPos[0] - handle->targetInfoList[j].currPos[0];
					y	=	handle->targetInfoList[i].currPos[1] - handle->targetInfoList[j].currPos[1];
					dist	=	x * x + y * y;
					if (dist < handle->targetInfoList[i].minNeighborDist)
						handle->targetInfoList[i].minNeighborDist = dist;
					if (dist < handle->targetInfoList[j].minNeighborDist)
						handle->targetInfoList[j].minNeighborDist = dist;
					//handle->targetInfoList[i].neighborDistSqr[j]	=	dist;
					//handle->targetInfoList[j].neighborDistSqr[i]	=	dist;
				}
			}
		}
	}

	//Classification results smoothing
	for (i = 0; i < (int32_t)handle->maxNumTracks; i++)
	{
		if ((handle->targetInfoList[i].activeTargetFlag == 1) && (handle->featExtrOutput[i].validFlag))
		{
			smoothingFactor		=	handle->gamma;
			if (handle->targetInfoList[i].minNeighborDist < handle->neighborDistSqrThr)
				smoothingFactor		=	handle->gamma1;
			/*for (j = 0; j < (int32_t)handle->maxNumTracks; j++)
			{
				if ((handle->targetInfoList[i].neighborDistSqr[j] > 0.f) && (handle->targetInfoList[i].neighborDistSqr[j] < handle->neighborDistSqrThr))
					smoothingFactor		=	handle->gamma1;
			}*/

			tempIdx		=	handle->targetInfoList[i].classifierOutCnt - 1;
			if (tempIdx < 0)
				tempIdx =	handle->outBufLen - 1;

			ftemp		=	handle->targetInfoList[i].classifierDecisionCur;
			if (ftemp > 254.f)
				ftemp = 0.f;
			handle->targetInfoList[i].classifierDecisionCur		=	ftemp * smoothingFactor + (1 - smoothingFactor) * (float)handle->targetInfoList[i].classifierOutput[tempIdx];

			//printf("trkID=%d, preRes=%1.3f, curRes=%1.3f\n", i, ftemp, handle->targetInfoList[i].classifierDecisionCur);
		}
	}

	//update output
	totalActiveTargets = 0;
	for (i = 0; i < (int32_t)handle->maxNumTracks; i++)
	{
		if (handle->targetInfoList[i].activeTargetFlag == 1)
		{
			output->activeTrkIDs[totalActiveTargets]	=	i;
			if (handle->targetInfoList[i].classifierDecisionCur > 254.f)
			{
				output->activeTrkTags[totalActiveTargets] = CLASSIFIERKNN_PROCESS_TAGS_NOTYETCLASSIFIED;
			}
			else
			{
				dist	=	1e10;
				for (j = 0; j < handle->numTAGS; j++ )
				{
#ifdef OPTIMIZED4DSP
					ftemp	=	_fabsf(handle->targetInfoList[i].classifierDecisionCur - (float)handle->tagVal[j]);
#else
					ftemp	=	abs(handle->targetInfoList[i].classifierDecisionCur - (float)handle->tagVal[j]);
#endif
					if (ftemp < dist)
					{
						dist	=	ftemp;
						output->activeTrkTags[totalActiveTargets]	=	(classifierkNN_process_tags)handle->tagVal[j];
					}
				}
			}
			totalActiveTargets++;
		}
	}
	output->numActiveTrks	=	totalActiveTargets;
}
