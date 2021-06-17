/*! 
 *  \file   RADARDEMO_RADARDEMO_classifierkNN_priv.h
 *
 *  \brief   Header file for RADARDEMO_RADARDEMO_classifierkNN module's internal structures and functions
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

#ifndef RADARDEMO_CLASSIFIERKNN_RPIV_H
#define RADARDEMO_CLASSIFIERKNN_RPIV_H

#include <swpform.h>
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif

/** 
 *  \struct   _RADARDEMO_classifierkNN_handle_
 *
 *  \brief   Structure definition of handle for classifierkNN module.
 *
 *
 */

typedef struct _RADARDEMO_classifierkNN_handle_
{
	uint8_t      numFeats;                       	/**< Number of features in the codebook, copy from codebook descriptor from the input config. */ 
	uint8_t     k;                                  /**< k value for kNN, meaning number of nearest neighbor for the final decision*/
	uint16_t     cbLen;                       		/**< Length of the codebook, copy from codebook descriptor from the input config. */ 
	float        *cbPtr;                            /**< pointer to the codebook itself, must be organized in the order of cb[0]feat_0, cb[0]feat_1,...  
														 cb[1]feat_0, cb[1]feat_1, ..., cb[cbLen-1]feat_0, cb[cbLen-1]feat_1, ..., cb[cbLen-1]feat_(numFeats-1)
														 This pointer should the copy of cbPtr from codebook descriptor from the input config.*/
	int8_t       *tags;                            /**< pointer to the tags of each corresponding codeword in the cbPtr buffer.
														 This pointer should the copy of tags from codebook descriptor from the input config.*/
	float        *distLocBuf;                       /**< Local buffer that holds the distance to each codeword, much be of length cbLen. This buffer is allocated to scratch memory.*/
	uint16_t     *indicesLocalBuf;                  /**< Local buffer that holds the indices to the k nearest neighbor in the codebook. */
	uint8_t      *votes;                            /**< Local buffer that holds the votes for each TAG. */
	uint8_t      numTAGS;                       	/**< Number of unique tags, copy from codebook descriptor from the input config. */ 
	int8_t      *tagVal;                       	    /**< All possible unique tag values in the codebook, size of numTags, 
														copy from codebook descriptor from the input config.  */ 
} RADARDEMO_classifierkNN_handle;

#endif //RADARDEMO_CLASSIFIERKNN_RPIV_H

