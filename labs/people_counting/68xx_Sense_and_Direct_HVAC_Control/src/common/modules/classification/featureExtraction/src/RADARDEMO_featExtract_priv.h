/*! 
 *  \file   RADARDEMO_RADARDEMO_featExtract_priv.h
 *
 *  \brief   Header file for RADARDEMO_RADARDEMO_featExtract module's internal structures and functions
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

#ifndef RADARDEMO_FEATEXTRACTION_RPIV_H
#define RADARDEMO_FEATEXTRACTION_RPIV_H

#include <swpform.h>
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif

/** 
 *  \struct   _RADARDEMO_featExtract_handle_
 *
 *  \brief   Structure definition of handle for featExtract module.
 *
 *
 */

typedef struct _RADARDEMO_featExtract_handle_
{
	uint8_t      numFeatCat;                       /**< Number of features to extract */ 
	RADARDEMO_featExtract_featDescr *featDescr; /**< Features descriptors, size of numFeats */ 
	uint32_t     blockLen;  					/**< Feature extraction block length, in number of frames*/
	uint32_t     slideWinLen;  					/**< Feature extraction sliding window length, in number of frames*/
	uint8_t      minNPtrPerTrack;  	       		/**< Minimum number of points in the track to extract feature. If below, skip the frame.*/
	uint8_t      startFeatExtrFlag;  	       	/**< Flag, if set to 1, to indicate the locate feature stats buff is filled and ready for feature extractio.*/
	uint32_t     blockCnt;  					/**< Block counter, indicating the location of updated per frame information. */
	float      **featBuffPerFrame; 				/**< numFeats pointers to per frame features buffer of the feature, each length of blockLen.*/ 
	float      **upperEnv; 						/**< numFeats pointers to upper envelope buffer of the feature, only valid for point cloud features: BW, OFFSET, and TORSOBW.*/ 
	float      **lowerEnv; 						/**< numFeats pointers to lower envelope buffer of the feature, only valid for point cloud features: BW, OFFSET, and TORSOBW.*/ 
} RADARDEMO_featExtract_handle;

#endif //RADARDEMO_FEATEXTRACTION_RPIV_H

