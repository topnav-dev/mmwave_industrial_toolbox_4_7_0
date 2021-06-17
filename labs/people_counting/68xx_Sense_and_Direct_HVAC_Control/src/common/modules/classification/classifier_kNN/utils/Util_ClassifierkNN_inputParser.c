/**
 *   @file  Util_ClassifierkNN_inputParser.c
 *
 *   @brief
 *      Utility function for parsing input data for classifier-kNN test.
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

#include "Util_ClassifierkNN_inputParser.h"

//! \copydoc Util_ClassifierkNN_inputParser 
Util_ClassifierkNN_inputParser_ErrorCode Util_ClassifierkNN_inputParser(
			IN void * pointcloudTLV,
			IN void * targetListTLV,
			IN void * targetIndexTLV,
			IN uint32_t maxNumDetPnts,
			IN uint32_t maxNumTrks,
			IN float maxRange,
			IN float maxDoppler,
			IN float *xBuff,
			IN float *yBuff,
			IN float *dopplerBuff,
			IN float *powerBuff,
			OUT RADARDEMO_featExtract_input * parsedData
		)
{
	int32_t i, j, k, buffIdx, errorFlag;
	MmwDemo_output_message_pointCloud * pointcloudInput = (MmwDemo_output_message_pointCloud *)pointcloudTLV;
	MmwDemo_output_message_targetList * targetListInput = (MmwDemo_output_message_targetList *)targetListTLV;
	MmwDemo_output_message_targetIndex * targetIndexInput = (MmwDemo_output_message_targetIndex *) targetIndexTLV;
	MmwDemo_output_message_tl *pcHeader;
	MmwDemo_output_message_tl *tlHeader;
	MmwDemo_output_message_tl *tiHeader;
	MmwDemo_output_message_point * pcBody;
	MmwDemo_output_message_target * tlBody;
	uint8_t * tiBody;
	uint32_t pc_len;
	uint32_t tl_len;
	uint32_t ti_len;
	int32_t numTarget;
	int32_t numDetPnt;
	Util_ClassifierkNN_inputParser_ErrorCode errorCode = UTIL_CLASSIFIERKNN_INPUTPARSER_NOERROR;
	uint32_t tid;
	float range;
	float angle;
	int32_t tempLen;
#ifdef OPTIMIZED4DSP
	float fsin;
#endif

	pcHeader 	=	&pointcloudInput->header;
	tlHeader 	=	&targetListInput->header;
	tiHeader 	=	&targetIndexInput->header;
	
	if (   (pcHeader->type != (uint32_t) MMWDEMO_OUTPUT_MSG_POINT_CLOUD)
		|| (tlHeader->type != (uint32_t) MMWDEMO_OUTPUT_MSG_TARGET_LIST)
		|| (tiHeader->type != (uint32_t) MMWDEMO_OUTPUT_MSG_TARGET_INDEX))
	{
		errorCode = UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_TYPES;
	}
	
	pcBody 		=	(MmwDemo_output_message_point *) pointcloudInput->point;
	tlBody 		=	(MmwDemo_output_message_target *) targetListInput->target;
	tiBody 		=	(uint8_t *) targetIndexInput->index;
	
	if (pcHeader->length == 0)
		pc_len	=	0;
	else
		pc_len	 	=	pcHeader->length - sizeof(MmwDemo_output_message_tl);

	if (tlHeader->length == 0)
		tl_len		=	0;
	else
		tl_len	 	=	tlHeader->length - sizeof(MmwDemo_output_message_tl);

	if (tiHeader->length == 0)
		ti_len		=	0;
	else
		ti_len	 	=	tiHeader->length - sizeof(MmwDemo_output_message_tl);
	
#ifdef OPTIMIZED4DSP
	numDetPnt   = 	(int32_t)((float)pc_len * _rcpsp((float)sizeof(MmwDemo_output_message_point)) + 0.5f);
#else
	numDetPnt   = 	pc_len/sizeof(MmwDemo_output_message_point);
#endif
	if (numDetPnt * sizeof(MmwDemo_output_message_point) != pc_len )
	{
		errorCode = UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_PC_LEN;
		return (errorCode);
	}
	
#ifdef OPTIMIZED4DSP
	numTarget   = 	(int32_t)((float)tl_len * _rcpsp((float)sizeof(MmwDemo_output_message_target)) + 0.5f);
#else
	numTarget   = 	tl_len/sizeof(MmwDemo_output_message_target);
#endif
	if (numTarget * sizeof(MmwDemo_output_message_target) != tl_len )
	{
		errorCode = UTIL_CLASSIFIERKNN_INPUTPARSER_WRONG_TL_LEN;
		return (errorCode);
	}
	
	//parsing data based on target list
	buffIdx = 0;
	for (i = 0; i < numTarget; i++ )
	{
		// parse tracks target list info first
		tid		=	tlBody[i].tid;
		if ( tid > maxNumTrks )
			continue;
		parsedData[tid].trackInput.posX = 	tlBody[i].posX;
		parsedData[tid].trackInput.posY = 	tlBody[i].posY;
		parsedData[tid].trackInput.velX = 	tlBody[i].velX;
		parsedData[tid].trackInput.velY = 	tlBody[i].velY;
		parsedData[tid].trackInput.accX = 	tlBody[i].accX;
		parsedData[tid].trackInput.accY = 	tlBody[i].accY;
		for ( j = 0; j < 9; j ++ )
			parsedData[tid].trackInput.ec[j] = 	tlBody[i].ec[j];
		parsedData[tid].trackInput.g = 	tlBody[i].g;
		
		//parse point cloud data
		k = 0;
		if (xBuff != NULL)
		{
			parsedData[tid].pntcloudInput.x 	= 	&xBuff[buffIdx];
			parsedData[tid].pntcloudInput.y 	= 	&yBuff[buffIdx];
		}
		if (dopplerBuff != NULL)
			parsedData[tid].pntcloudInput.doppler 	= 	&dopplerBuff[buffIdx];
		parsedData[tid].pntcloudInput.power 	= 	&powerBuff[buffIdx];

		tempLen = ti_len;
		if (numDetPnt < tempLen)
			tempLen = numDetPnt;

		errorFlag	=	0;
		for ( j = 0; j < (int32_t)tempLen; j ++ )
		{
			if (( (uint8_t)tiBody[j] == (uint8_t)tid ) && (k < (int32_t)maxNumDetPnts))
			{
				range 								=	pcBody[j].range;
				angle 								=	pcBody[j].azimuth;
	#ifdef OPTIMIZED4DSP
				if (_fabsf(range) > maxRange)
					errorFlag						=	1;
				if (_fabsf(angle) > MAX_AZIMUTH)
					errorFlag						=	1;
	#else
				if (abs(range) > maxRange)
					errorFlag						=	1;
				if (abs(angle) > MAX_AZIMUTH)
					errorFlag						=	1;
	#endif
				if (xBuff != NULL)
				{
	#ifdef OPTIMIZED4DSP
					fsin								=	sinsp_i(angle);
					xBuff[buffIdx]						=	range * fsin;
					fsin								=	1 - fsin * fsin;

					yBuff[buffIdx]						=	range * fsin * _rsqrsp(fsin);
	#else
					xBuff[buffIdx]						=	range * (float)sin(angle);
					yBuff[buffIdx]						=	range * (float)cos(angle);
	#endif
				}
				if (dopplerBuff != NULL)
				{
					dopplerBuff[buffIdx] 				=	pcBody[j].doppler;
					if (abs(pcBody[j].doppler) > maxDoppler)
						errorFlag						=	1;
				}
				powerBuff[buffIdx] 					=	pcBody[j].snr;
				if (abs(pcBody[j].snr) > MAX_POWER)
					errorFlag						=	1;
				if (errorFlag	== 1)
					break;
				buffIdx++;
				k++;
			}
		}
		if (errorFlag == 0)
			parsedData[tid].numPntPerTrack 	= 	k;
		else
			parsedData[tid].numPntPerTrack 	= 	0;
		//printf("trkID = %d, numPnt = %d\n", tid, parsedData[tid].numPntPerTrack);

	}
	return (errorCode);
		
}
