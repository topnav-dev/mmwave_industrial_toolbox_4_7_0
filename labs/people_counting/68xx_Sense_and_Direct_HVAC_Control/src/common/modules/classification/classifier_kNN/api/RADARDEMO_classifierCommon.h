/*! 
 *  \file   RADARDEMO_classifierionCommon.h
 *
 *  \brief   Header file for common classifier data structures.
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



#ifndef RADARDEMO_CLASSIFIERCOMMON_H
#define RADARDEMO_CLASSIFIERCOMMON_H

#include <swpform.h>
#include <modules/classification/featureExtraction/api/RADARDEMO_featExtraction.h>

/* The following section defines features extracted for feature-based classifiers. 
   They are pre-initialized in RADARDEMO_classifierkNNCB.c.
 */
extern uint8_t numOutFeatsInCB;
extern uint8_t uniqueFeatCat;
extern RADARDEMO_featExtract_featDescr featuresCat[1];
extern uint8_t outFeaturesCatIdx[3];
extern uint8_t outFeaturesIdx[3];
extern uint8_t numTagsInCB;
extern int8_t cbTagVals[2];
/* The following section defines kNN and kmeans releated codebook parameters. 
   They are pre-initialized in RADARDEMO_classifierkNNCB.c.
 */
extern float kNNtrainingFeatSet[1800];
extern int8_t kNNtrainingFeatSetTags[600];
extern float kmeansFeatSet[768];
extern int8_t kmeansFeatSetTags[256];

/* The following section defines trained SVM model related parameters. The models are trained using MATLAB toolbox functions.
   They are pre-initialized in RADARDEMO_classifierkNNCB.c.
   scale: input scale, svmModel.KernelParameters.Scale from MATLAB
   bias:  svmModel.Bias from MATLAB
   beta:  only valid for linear kernel. svmModel.Beta from MATLAB
   order: oder of polynomial, only valid for polynomial kernel. svmModel.KernelParameters.Order from MATLAB
   sv:    support vector, only valid for gaussian and polynomial kernels. svmModel.SupportVectors from MATLAB
   svTAG: tags of support vector, only valid for gaussian and polynomial kernels, only support 1 and -1 tags.
          svmModel.SupportVectorLabels from MATLAB
   alpha: weighting factor for support vector. svmModel.Alpha from MATLAB
*/
extern float svmLinScale;
extern float svmLinBias;
extern float svmLinBeta[3];
extern float svmGausScale;
extern float svmGausBias;
extern uint16_t svmGausSVLen;
extern float svmGausSV[1542];
extern int8_t svmGausSVTags[514];
extern float svmGausAlpha[514];
extern float svmPolyScale;
extern float svmPolyBias;
extern uint16_t svmPolySVLen;
extern uint8_t svmPolyOrder;
extern float svmPolySV[195];
extern int8_t svmPolySVTags[65];
extern float svmPolyAlpha[65];


/**
 *  \struct   RADARDEMO_classifier_cbDescr
 *
 *  \brief   Structure for codebook descriptor used in codebook or support vector based classifiers.
 *
 */

typedef struct _RADARDEMO_classifier_cbDescr_
{
	uint8_t      numFeats;                       	/**< Number of features in the codebook */ 
	uint16_t     cbLen;                       		/**< Length of the codebook */ 
	float        *cbPtr;                            /**< pointer to the codebook itself, must be organized in the order of cb[0]feat_0, cb[0]feat_1,...  
														 cb[1]feat_0, cb[1]feat_1, ..., cb[cbLen-1]feat_0, cb[cbLen-1]feat_1, ..., cb[cbLen-1]feat_(numFeats-1).*/
	int8_t       *tags;                            /**< pointer to the tags of each corresponding codeword in the cbPtr buffer.*/
	uint8_t      numTAGS;                       	/**< Number of unique tags. */ 
	int8_t      *tagVal;                       	    /**< All possible unique tag values in the codebook, size of numTags */ 
	
} RADARDEMO_classifier_cbDescr;


/**
 *  \struct   RADARDEMO_classifier_input
 *
 *  \brief   Structure for input to codebook or support vector based classifiers.
 *
 */
typedef struct _RADARDEMO_classifier_input_
{
	uint8_t      numFeats;                       	/**< Number of features in the codebook, must be the same as numFeats of the codebook \ref 	
														 RADARDEMO_classifierkNN_cbDescr. */ 
	float        *featVal;                          /**< pointer to the feature values, must be organized in the order of Val_feat_0, Val_feat_1,...  
														 Val_feat_(numFeats-1). Must be the same order as cbPtr of the codebook \ref RADARDEMO_classifierkNN_cbDescr, except the TAG value. */
} RADARDEMO_classifier_input;
#endif //RADARDEMO_CLASSIFIERCOMMON_H

