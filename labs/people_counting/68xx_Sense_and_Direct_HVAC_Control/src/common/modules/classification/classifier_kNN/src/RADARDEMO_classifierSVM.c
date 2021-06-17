/*! 
 *  \file   RADARDEMO_classifierSVM.c
 *
 *  \brief   Classifier -- SVM module. 
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

#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierSVM.h>
#include <modules/classification/classifier_kNN/src/RADARDEMO_classifierSVM_priv.h>
#include <modules/classification/classificationUtilities/src/RADARDEMO_svmKernelUtil_priv.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEBUG(_x) //_x

#ifdef _TMS320C6X
#include "c6x.h"
#endif


/*! 
   \fn     RADARDEMO_classifierSVM_create
 
   \brief   Create and initialize RADARDEMO_classifierSVM module. 
  
   \param[in]    moduleConfig
               Pointer to input configurations structure for RADARDEMO_classifierSVM module.

   \param[out]    errorCode
               Output error code.

			   
   \ret     void pointer to the module handle. Return value of NULL indicates failed module creation.
			   
   \pre       none
 
   \post      none
  
 
 */

void	* RADARDEMO_classifierSVM_create(
                            IN  RADARDEMO_classifierSVM_config * moduleConfig, 
							OUT RADARDEMO_classifierSVM_errorCode * errorCode)
							
{

	RADARDEMO_classifierSVM_handle * handle;
	
	*errorCode	=	RADARDEMO_CLASSIFIERSVM_NO_ERROR;

	if ( moduleConfig->kernelType >= RADARDEMO_CLASSIFIERSVM_KERNAL_NOTSUPPORTED)
	{
		*errorCode	=	RADARDEMO_CLASSIFIERSVM_INVALID_CFG;
		return (NULL);
	}
	
		
	handle						=	(RADARDEMO_classifierSVM_handle *) radarOsal_memAlloc((uint8_t) RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_classifierSVM_handle), 1);
	if (handle == NULL)
	{
		*errorCode	=	RADARDEMO_CLASSIFIERSVM_FAIL_ALLOCATE_HANDLE;
		return (handle);
	}
	
	memset(handle, 0, sizeof(RADARDEMO_classifierSVM_handle));
	
	handle->kernelType			=	moduleConfig->kernelType;
	handle->numFeats			=	numOutFeatsInCB;

	if ( handle->kernelType == RADARDEMO_CLASSIFIERSVM_LINEAR)
	{
		handle->scale 			=	svmLinScale;
		handle->bias 			=	svmLinBias;
		handle->beta 			=	&svmLinBeta[0];
	}
	if ( handle->kernelType == RADARDEMO_CLASSIFIERSVM_GAUSSIAN)
	{
		handle->scale 			=	svmGausScale;
		handle->bias 			=	svmGausBias;
		handle->svLen			=	svmGausSVLen;
		handle->svPtr			=	svmGausSV;
		handle->svTags			=	svmGausSVTags;
		handle->alpha			=	svmGausAlpha;
	}
	if ( handle->kernelType == RADARDEMO_CLASSIFIERSVM_POLYNOMIAL)
	{
		handle->scale 			=	svmPolyScale;
		handle->bias 			=	svmPolyBias;
		handle->svLen			=	svmPolySVLen;
		handle->order			=	svmPolyOrder;
		handle->svPtr			=	svmPolySV;
		handle->svTags			=	svmPolySVTags;
		handle->alpha			=	svmPolyAlpha;
	}
	return((void *)handle);
}

/*! 
   \fn     RADARDEMO_classifierSVM_delete
 
   \brief   Delete RADARDEMO_classifierSVM module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
  
 
 */

void	RADARDEMO_classifierSVM_delete(
                            IN  void * handle)
{
	radarOsal_memFree(handle, sizeof(RADARDEMO_classifierSVM_handle));
}


/*! 
   \fn     RADARDEMO_classifierSVM_run
 
   \brief   Feature extraction module.
  
   \param[in]    handle
               Module handle.
 
   \param[in]    classifierInput
               Pointer to feature extraction input structure. See RADARDEMO_classifierSVM_input definition for details.
 
   \param[out]    classifierOut
               Output tag of the classification output, must be a member of tagVal in the \ref RADARDEMO_classifierSVM_cbDescr in \ref RADARDEMO_classifierSVM_config.
 
   \ret error code

   \pre       none
 
   \post      none
 
 */

RADARDEMO_classifierSVM_errorCode	RADARDEMO_classifierSVM_run(
                            IN  void * handle,
							IN RADARDEMO_classifierSVM_input * classifierInput,
							OUT int8_t  		*classifierOut)

{
	RADARDEMO_classifierSVM_errorCode errorCode = RADARDEMO_CLASSIFIERSVM_NO_ERROR;
	RADARDEMO_classifierSVM_handle	*classifierSVMInst;
	//float 	minDist;
	float score;

	classifierSVMInst		=	(RADARDEMO_classifierSVM_handle *) handle;
	
	/* check error conditions */
	if (classifierInput->featVal == NULL) 
	{
		errorCode		=		RADARDEMO_CLASSIFIERSVM_INOUTPTR_NOTCORRECT;
		return (errorCode);
	}

	if ( classifierSVMInst->kernelType ==  RADARDEMO_CLASSIFIERSVM_LINEAR)
	{
		*classifierOut = RADARDEMO_svmPredictLinearKernal(
							(int8_t)  classifierInput->numFeats,
                            classifierInput->featVal,
                            classifierSVMInst->beta,
							classifierSVMInst->scale,
							classifierSVMInst->bias,
							&score);
	}
	else if ( classifierSVMInst->kernelType ==  RADARDEMO_CLASSIFIERSVM_GAUSSIAN)
	{
		*classifierOut	= RADARDEMO_svmPredictGaussianKernal(
							(int8_t)  classifierInput->numFeats,
                            classifierInput->featVal,
							classifierSVMInst->scale,
							classifierSVMInst->svLen,
							classifierSVMInst->svPtr,
							classifierSVMInst->svTags,
							classifierSVMInst->alpha,
							classifierSVMInst->bias,
							&score);
		
	}
	else if ( classifierSVMInst->kernelType ==  RADARDEMO_CLASSIFIERSVM_POLYNOMIAL)
	{
		*classifierOut	= RADARDEMO_svmPredictPolyKernal(
							(int8_t)  classifierInput->numFeats,
                            classifierInput->featVal,
							classifierSVMInst->scale,
							classifierSVMInst->order,
							classifierSVMInst->svLen,
							classifierSVMInst->svPtr,
							classifierSVMInst->svTags,
							classifierSVMInst->alpha,
							classifierSVMInst->bias,
							&score);
	}
	return (errorCode);
}

