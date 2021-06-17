/*! 
 *  \file   RADARDEMO_RADARDEMO_classifierSVM_priv.h
 *
 *  \brief   Header file for RADARDEMO_RADARDEMO_classifierSVM module's internal structures and functions
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

#ifndef RADARDEMO_CLASSIFIERSVM_RPIV_H
#define RADARDEMO_CLASSIFIERSVM_RPIV_H

#include <swpform.h>
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>

#ifdef _TMS320C6X
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif
#endif

/** 
 *  \struct   _RADARDEMO_classifierSVM_handle_
 *
 *  \brief   Structure definition of handle for classifierSVM module.
 *
 *
 */

typedef struct _RADARDEMO_classifierSVM_handle_
{
	uint8_t     kernelType;                         /**< kernelType = 0: linear kernel
	                                                     kernelType = 1: Gaussian kernel
														 kernelType = 2: polynomial kernel
														 Currently all the trained kernal information are hardcoded in tables in RADARDEMO_classifierkNNCB.c, rather than taking from configurations.*/
	uint8_t      numFeats;                       	/**< Number of features in the codebook, copy from codebook descriptor from the input config. */ 
	uint16_t     svLen;                       		/**< Length of the support vector, copy from codebook descriptor from the input config. */ 
	float        *svPtr;                            /**< pointer to the support vector, must be organized in the order of sv[0]feat_0, sv[0]feat_1,...  
														 sv[1]feat_0, sv[1]feat_1, ..., sv[cbLen-1]feat_0, sv[cbLen-1]feat_1, ..., sv[cbLen-1]feat_(numFeats-1).
														 Only valid for gaussian and polynomial kernels. svmModel.SupportVectors from MATLAB
														 This pointer should the copy of svPtr from codebook descriptor from the input config.*/
	int8_t       *svTags;                            /**< pointer to the tags of each corresponding support vector in the svPtr buffer.
														 This pointer should the copy of tags from support vector descriptor from the input config.*/
	float        scale;                             /**< input scale, svmModel.KernelParameters.Scale from MATLAB*/
	float        bias;                              /**< vmModel.Bias from MATLAB*/
	float        *beta;                             /**< only valid for linear kernel. svmModel.Beta from MATLAB*/
	uint8_t      order;                              /**< oder of polynomial, only valid for polynomial kernel. svmModel.KernelParameters.Order from MATLAB*/
	float        *alpha;  						     /**< weighting factor for support vector. svmModel.Alpha from MATLAB*/
} RADARDEMO_classifierSVM_handle;

#endif //RADARDEMO_CLASSIFIERKNN_RPIV_H

