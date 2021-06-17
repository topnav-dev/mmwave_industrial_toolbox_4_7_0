/*! 
 *  \file   RADARDEMO_featExtract.c
 *
 *  \brief   Feature extraction module. 
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

#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>
#include <modules/classification/classificationUtilities/src/RADARDEMO_featExtrUtil_priv.h>
#include "RADARDEMO_featExtract_priv.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


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

void	* RADARDEMO_featExtract_create(
                            IN  RADARDEMO_featExtract_config * moduleConfig, 
							OUT RADARDEMO_featExtract_errorCode * errorCode)
							
{
	int32_t		i;
	RADARDEMO_featExtract_handle * handle;
	
	*errorCode	=	RADARDEMO_FEATEXTRACT_NO_ERROR;

	for ( i = 0; i < moduleConfig->numFeatCat; i++ )
	{
		if ( moduleConfig->featDescr[i].featSrc >= RADARDEMO_FEATEXTRACT_FEATSRC_NOT_SUPPORTED )
			*errorCode	=	RADARDEMO_FEATEXTRACT_FEATSOURCE_NOTSUPPORTED;

		if (  (moduleConfig->featDescr[i].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD) 
		   && (moduleConfig->featDescr[i].feats.pntCloudFeatDescr.pntCloudFeatCat >= RADARDEMO_FEATEXTRACT_PNTCLOUDFEATCAT_NOTSUPPORTED ))
			*errorCode	=	RADARDEMO_FEATEXTRACT_PNTCLOUDFEATCAT_NOTVALID;

			
		if (  (moduleConfig->featDescr[i].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_GTRACK) 
		   && (moduleConfig->featDescr[i].feats.trackFeatDescr.trackFeatCat >= RADARDEMO_FEATEXTRACT_TRACKCAT_NOTSUPPORTED ))
			*errorCode	=	RADARDEMO_FEATEXTRACT_TRKFEATCAT_NOTVALID;

	}

	if (*errorCode > RADARDEMO_FEATEXTRACT_NO_ERROR)
		return (NULL);
		
	handle						=	(RADARDEMO_featExtract_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_featExtract_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_FEATEXTRACT_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	memset(handle, 0, sizeof(RADARDEMO_featExtract_handle));
	handle->numFeatCat			=	moduleConfig->numFeatCat;
	handle->featDescr			=	(RADARDEMO_featExtract_featDescr *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, 
		                               handle->numFeatCat * sizeof(RADARDEMO_featExtract_featDescr), 1);
	memcpy(&handle->featDescr[0], &moduleConfig->featDescr[0], handle->numFeatCat * sizeof(RADARDEMO_featExtract_featDescr));
	handle->blockLen			=	moduleConfig->blockLen;
	handle->slideWinLen			=	moduleConfig->slideWinLen;
	handle->minNPtrPerTrack		=	moduleConfig->minNPtrPerTrack;
	handle->blockCnt			=	0;

	handle->featBuffPerFrame	=	(float **) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->numFeatCat * sizeof(float *), 1);
	for (i = 0; i < handle->numFeatCat; i++ )
	{
		handle->featBuffPerFrame[i]	=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->blockLen * sizeof(float), 1);
	}
	
	handle->upperEnv			=	(float **) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->numFeatCat * sizeof(float *), 1);
	handle->lowerEnv			=	(float **) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->numFeatCat * sizeof(float *), 1);
	for (i = 0; i < handle->numFeatCat; i++ )
	{
		if (handle->featDescr[i].featSrc 	==	RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
		{
			handle->upperEnv[i]		=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->blockLen * sizeof(float), 1);
			handle->lowerEnv[i]		=	(float *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, handle->blockLen * sizeof(float), 1);
					
		}
	}
	
	return((void *)handle);
}


/*! 
   \fn     RADARDEMO_featExtract_delete
 
   \brief   Delete RADARDEMO_featExtract module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_featExtract_delete(
                            IN  void * handle)
{
	int32_t i;
	RADARDEMO_featExtract_handle *featExtractInst;
	
	featExtractInst	=	(RADARDEMO_featExtract_handle *) handle;

	if (featExtractInst->upperEnv !=	NULL )
	{
		for (i = 0; i < featExtractInst->numFeatCat; i++ )
		{
			if (featExtractInst->upperEnv[i] != NULL)
				radarOsal_memFree(featExtractInst->upperEnv[i], featExtractInst->blockLen * sizeof(float));
			if (featExtractInst->lowerEnv[i] != NULL)
				radarOsal_memFree(featExtractInst->lowerEnv[i], featExtractInst->blockLen * sizeof(float));

		}
		radarOsal_memFree(featExtractInst->upperEnv, featExtractInst->numFeatCat * sizeof(float *));
		radarOsal_memFree(featExtractInst->lowerEnv, featExtractInst->numFeatCat * sizeof(float *));
	}
	
	for (i = 0; i < featExtractInst->numFeatCat; i++ )
	{
		radarOsal_memFree(featExtractInst->featBuffPerFrame[i], featExtractInst->blockLen * sizeof(float));
	}
	radarOsal_memFree(featExtractInst->featBuffPerFrame, featExtractInst->numFeatCat * sizeof(float *));
	radarOsal_memFree(featExtractInst->featDescr, featExtractInst->numFeatCat * sizeof(RADARDEMO_featExtract_featDescr));
	radarOsal_memFree(handle, sizeof(featExtractInst));
	
}


/*! 
   \fn     RADARDEMO_featExtract_reset
 
   \brief   Reset RADARDEMO_featExtract module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_featExtract_reset(
                            IN  void * handle)
{
	int32_t i;
	RADARDEMO_featExtract_handle *featExtractInst;
	
	featExtractInst	=	(RADARDEMO_featExtract_handle *) handle;

	if (featExtractInst->upperEnv !=	NULL )
	{
		for (i = 0; i < featExtractInst->numFeatCat; i++ )
		{
			if (featExtractInst->upperEnv[i] != NULL)
				memset(featExtractInst->upperEnv[i], 0, featExtractInst->blockLen * sizeof(float));
			if (featExtractInst->lowerEnv[i] != NULL)
				memset(featExtractInst->lowerEnv[i], 0, featExtractInst->blockLen * sizeof(float));

		}
	}
	
	for (i = 0; i < featExtractInst->numFeatCat; i++ )
	{
		memset(featExtractInst->featBuffPerFrame[i], 0, featExtractInst->blockLen * sizeof(float));
	}
	featExtractInst->startFeatExtrFlag	=	0;
	featExtractInst->blockCnt			=	0;
}


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
							OUT RADARDEMO_featExtract_output  * featExtrOutput)

{
	RADARDEMO_featExtract_errorCode errorCode = RADARDEMO_FEATEXTRACT_NO_ERROR;
	RADARDEMO_featExtract_handle	*featExtractInst;
	float min, max, mean, std;
	int32_t  featIdx;

	featExtractInst		=	(RADARDEMO_featExtract_handle *) handle;
	
	featExtrOutput->validFlag = 0;
	/* check error conditions */
	if ((featExtrInput->pntcloudInput.power == NULL) || (featExtrOutput->featOut == NULL) )
	{
		errorCode		=		RADARDEMO_FEATEXTRACT_INOUTPTR_NOTCORRECT;
	}

	if ( featExtrInput->numPntPerTrack < featExtractInst->minNPtrPerTrack )
	{

		return (errorCode);
	}
		
	/* update local feature buffers */
	for (featIdx = 0; featIdx < featExtractInst->numFeatCat; featIdx++ )
	{
		if (featExtractInst->featDescr[featIdx].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
		{
			/*  point cloud feature extractions */
			float	* weights;
			RADARDEMO_featExtract_pntcloudFeat  featsDesc = featExtractInst->featDescr[featIdx].feats.pntCloudFeatDescr;
			
			weights 		=		featExtrInput->pntcloudInput.power;
			
			if (featsDesc.pntCloudFeatCat ==	RADARDEMO_FEATEXTRACT_PNTCLOUD_DOPPLER)
			{   /* doppler related features */
				float  	* doppler;
				float estCoM;

				doppler		=		featExtrInput->pntcloudInput.doppler;
				estCoM = RADARDEMO_featExtrUtilCoMEst(
                            featExtrInput->numPntPerTrack,
                            doppler,
                            weights);
				
				/* doppler center of mass type of faetures -- COMMEAN, COMSTD */
				featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = estCoM;
				/* doppler envolope type of features -- BW, OFFSET, TORSEBW, HISTO*/
				RADARDEMO_featExtrUtilStatisticsEst(
                            featExtrInput->numPntPerTrack,
                            doppler,
                            featsDesc.histBinRes,
                            featsDesc.numHistBins,
                            estCoM,
                            &min,
                            &max,
                            &mean,
                            &std,
                            featExtrOutput->featOut[featIdx].pcloudFeatOut.histogram
							);
				featExtractInst->upperEnv[featIdx][featExtractInst->blockCnt] = max - estCoM;
				featExtractInst->lowerEnv[featIdx][featExtractInst->blockCnt] = min - estCoM;
			}
			if (featsDesc.pntCloudFeatCat ==	RADARDEMO_FEATEXTRACT_PNTCLOUD_RANGE)
			{ /* range related features */
				float  	* x, * y;
				float 	estCoMX, estCoMY;
				float ftemp;

				x		=		featExtrInput->pntcloudInput.x;
				estCoMX = RADARDEMO_featExtrUtilCoMEst(
                            featExtrInput->numPntPerTrack,
                            x,
                            weights);
				y		=		featExtrInput->pntcloudInput.y;
				estCoMY = RADARDEMO_featExtrUtilCoMEst(
                            featExtrInput->numPntPerTrack,
                            y,
                            weights);
				
				/* range center of mass type of faetures -- COMMEAN, COMSTD */
				ftemp		=	estCoMX * estCoMX + estCoMY * estCoMY;
#ifdef OPTIMIZED4DSP
				featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = ftemp * _rsqrsp(ftemp);
#else
				featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = sqrt(ftemp);
#endif
				/* range envolope type of features -- BW, OFFSET, TORSEBW, HISTO*/
				RADARDEMO_featExtrUtil2DStatisticsEst(
							1, 
                            featExtrInput->numPntPerTrack,
                            x,
                            y,
                            featsDesc.histBinRes,
                            featsDesc.numHistBins,
                            estCoMX,
                            estCoMY,
                            &min,
                            &max,
                            &mean,
                            &std,
                            featExtrOutput->featOut[featIdx].pcloudFeatOut.histogram
							);
				featExtractInst->upperEnv[featIdx][featExtractInst->blockCnt] = max;
				featExtractInst->lowerEnv[featIdx][featExtractInst->blockCnt] = min;
			}
		}
		else if (featExtractInst->featDescr[featIdx].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_GTRACK)
		{
			float 	x, y, ftemp;
			/*  track feature extractions */
			RADARDEMO_featExtract_trackFeat  featsDesc = featExtractInst->featDescr[featIdx].feats.trackFeatDescr;
			
			switch (featsDesc.trackFeatCat)
			{
				case RADARDEMO_FEATEXTRACT_TRACK_POS:
					x		=	featExtrInput->trackInput.posX;
					y		=	featExtrInput->trackInput.posY;
					ftemp	=	x * x + y * y;
					#ifdef OPTIMIZED4DSP
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = ftemp * _rsqrsp(ftemp);
					#else
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = sqrt(ftemp);
					#endif
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_VEL:
					x		=	featExtrInput->trackInput.velX;
					y		=	featExtrInput->trackInput.velY;
					ftemp	=	x * x + y * y;
					#ifdef OPTIMIZED4DSP
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = ftemp * _rsqrsp(ftemp);
					#else
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = sqrt(ftemp);
					#endif
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_ACCEL:
					x		=	featExtrInput->trackInput.accX;
					y		=	featExtrInput->trackInput.accY;
					ftemp	=	x * x + y * y;
					#ifdef OPTIMIZED4DSP
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = ftemp * _rsqrsp(ftemp);
					#else
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = sqrt(ftemp);
					#endif
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC0:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[0];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC1:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[1];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC2:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[2];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC3:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[3];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC4:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[4];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC5:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[5];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC6:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[6];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC7:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[7];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_EC8:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.ec[8];
					break;
					
				case RADARDEMO_FEATEXTRACT_TRACK_G:
					featExtractInst->featBuffPerFrame[featIdx][featExtractInst->blockCnt] = featExtrInput->trackInput.g;
					break;
					
				default:
					errorCode		=		RADARDEMO_FEATEXTRACT_TRKFEATCAT_NOTVALID;
					return (errorCode);
			}	
		}
		else
		{
			/*  unsupported source */
			errorCode		=		RADARDEMO_FEATEXTRACT_FEATSOURCE_NOTSUPPORTED;
			return (errorCode);
		}
	}
	featExtractInst->blockCnt	+= 	1;
	if ( featExtractInst->blockCnt >= featExtractInst->blockLen )
	{
		featExtractInst->blockCnt 			=	0;
		featExtractInst->startFeatExtrFlag	=	1;
	}
	//printf("blkcnt = %d\n", featExtractInst->blockCnt);

	/* Calculate features if reach the sliding window length*/
	if ( (((featExtractInst->blockCnt)%(featExtractInst->slideWinLen) == 0 ) && (featExtractInst->startFeatExtrFlag == 1))|| (featExtrInput->forceUpdateFlag == 1 ))
	{
		featExtrOutput->validFlag = 1;
		for (featIdx = 0; featIdx < featExtractInst->numFeatCat; featIdx++ )
		{
			if (featExtractInst->featDescr[featIdx].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_PNTCLOUD)
			{
				/*  point cloud feature extractions */
				//RADARDEMO_featExtract_pntcloudFeat  featsDesc = featExtractInst->featDescr[featIdx].feats.pntCloudFeatDescr;
				float minu, maxu, meanu, stdu;
				float minl, maxl, meanl, stdl;

				RADARDEMO_featExtrUtilStatisticsEst(
								featExtractInst->blockLen,
								featExtractInst->featBuffPerFrame[featIdx],
								0.f,
								0,
								0.f,
								&min,
								&max,
								&mean,
								&std,
								NULL
								);
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_COMMEAN_IDX] = mean;
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_COMSTD_IDX] = std;
				RADARDEMO_featExtrUtilStatisticsEst(
								featExtractInst->blockLen,
								featExtractInst->upperEnv[featIdx],
								0.f,
								0,
								0.f,
								&minu,
								&maxu,
								&meanu,
								&stdu,
								NULL
								);
				RADARDEMO_featExtrUtilStatisticsEst(
								featExtractInst->blockLen,
								featExtractInst->lowerEnv[featIdx],
								0.f,
								0,
								0.f,
								&minl,
								&maxl,
								&meanl,
								&stdl,
								NULL
								);
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_BW_IDX] = maxu - minl;
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_OFFSET_IDX] = meanu - meanl;
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_TORSOBW_IDX] = minu - maxl;
#ifdef OPTIMIZED4DSP
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_UPENVPAR_IDX] = maxu * _rcpsp(meanu);
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_LWENVPAR_IDX] = minl * _rcpsp(meanl);
#else
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_UPENVPAR_IDX] = maxu/meanu;
				featExtrOutput->featOut[featIdx].pcloudFeatOut.featValue[RADARDEMO_FEATEXTRACT_PNTCLOUDFEAT_LWENVPAR_IDX] = minl/meanl;
#endif
			}
			else if (featExtractInst->featDescr[featIdx].featSrc == RADARDEMO_FEATEXTRACT_FEATSRC_GTRACK)
			{
				/*  track feature extractions */
				//RADARDEMO_featExtract_trackFeat  featsDesc = featExtractInst->featDescr[featIdx].feats.trackFeatDescr;
				RADARDEMO_featExtrUtilStatisticsEst(
							featExtractInst->blockLen,
							featExtractInst->featBuffPerFrame[featIdx],
							0.f,
							0,
							0.f,
							&min,
							&max,
							&mean,
							&std,
							NULL
							);
				featExtrOutput->featOut[featIdx].trkFeatOut[RADARDEMO_FEATEXTRACT_STATS_MIN_IDX] = min;
				featExtrOutput->featOut[featIdx].trkFeatOut[RADARDEMO_FEATEXTRACT_STATS_MAX_IDX] = max;
				featExtrOutput->featOut[featIdx].trkFeatOut[RADARDEMO_FEATEXTRACT_STATS_MEAN_IDX] = mean;
				featExtrOutput->featOut[featIdx].trkFeatOut[RADARDEMO_FEATEXTRACT_STATS_STD_IDX] = std;
			}
		}
	}
	
	
	return (errorCode);
}

