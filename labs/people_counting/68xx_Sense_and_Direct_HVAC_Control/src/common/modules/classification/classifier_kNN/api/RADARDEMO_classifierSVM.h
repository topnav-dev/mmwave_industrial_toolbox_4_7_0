/*! 
 *  \file   RADARDEMO_classifierionSVM.h
 *
 *  \brief   Header file for RADARDEMO_classifierionSVM module
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


#ifndef RADARDEMO_CLASSIFIERSVM_H
#define RADARDEMO_CLASSIFIERSVM_H

#include <swpform.h>
#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierCommon.h>


/**
 * \enum RADARDEMO_classifierSVM_errorCode
 *  \brief   enum for classifier SVM error code.
 */

typedef enum
{
	RADARDEMO_CLASSIFIERSVM_NO_ERROR = 0,					/**< No error */
	RADARDEMO_CLASSIFIERSVM_INVALID_CFG,    				/**< Invalide configurations to RADARDEMO_classifierSVM module */ 
	RADARDEMO_CLASSIFIERSVM_FAIL_ALLOCATE_HANDLE,			/**< RADARDEMO_classifierSVM_create failed to allocate handle */
	RADARDEMO_CLASSIFIERSVM_FAIL_ALLOCATE_LOCALINSTMEM,		/**< RADARDEMO_classifierSVM_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_CLASSIFIERSVM_INOUTPTR_NOTCORRECT				/**< input andor output buffer for RADARDEMO_classifierSVM_run are either NULL, or not aligned properly  */
} RADARDEMO_classifierSVM_errorCode;

/**
 * \enum RADARDEMO_classifierkNN_KernelTypes
 *  \brief   enum for feature extraction error code.
 */

typedef enum
{
	RADARDEMO_CLASSIFIERSVM_LINEAR = 0,					    /**< Linear kernel */
	RADARDEMO_CLASSIFIERSVM_GAUSSIAN,        				/**< Gausiian kernel */ 
	RADARDEMO_CLASSIFIERSVM_POLYNOMIAL,			            /**< Polynomial kernel */
	RADARDEMO_CLASSIFIERSVM_KERNAL_NOTSUPPORTED				/**< not support kernal types */
} RADARDEMO_classifierkNN_KernelTypes;


/**
 *  \struct   RADARDEMO_classifierSVM_config
 *
 *  \brief   Structure for RADARDEMO_classifierSVM configuration.
 *
 */

typedef struct _RADARDEMO_classifierSVM_config_
{
	uint8_t     kernelType;                         /**< kernelType = 0: linear kernel
	                                                     kernelType = 1: Gaussian kernel
														 kernelType = 2: polynomial kernel
														 Currently all the trained kernal information are hardcoded in tables in RADARDEMO_classifierkNNCB.c, rather than taking from configurations.*/
} RADARDEMO_classifierSVM_config;

/**
 *  \struct   RADARDEMO_classifierSVM_input
 *
 *  \brief   Structure for input to RADARDEMO_classifierSVM module.
 *
 */
#define  RADARDEMO_classifierSVM_input RADARDEMO_classifier_input

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

extern void	* RADARDEMO_classifierSVM_create(
                            IN  RADARDEMO_classifierSVM_config * moduleConfig, 
							OUT RADARDEMO_classifierSVM_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_classifierSVM_delete
 
   \brief   Delete RADARDEMO_classifierSVM module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
 
 */

extern void	RADARDEMO_classifierSVM_delete(
                            IN  void * handle);


/*! 
   \fn     RADARDEMO_classifierSVM_run
 
   \brief   Classifier SVM module, called for every target. 
  
   \param[in]    handle
               Module handle.
 
   \param[in]    classifierInput
               Pointer to feature extraction input structure. See RADARDEMO_classifierSVM_input definition for details.
 
   \param[out]    classifierOut
               Output tag of the classification output, must be a member of tagVal in the \ref RADARDEMO_classifierSVM_cbDescr in \ref RADARDEMO_classifierkNN_config.
 
   \ret error code

   \pre       none
 
   \post      none
 
 */
RADARDEMO_classifierSVM_errorCode	RADARDEMO_classifierSVM_run(
                            IN  void * handle,
							IN RADARDEMO_classifierSVM_input * classifierInput,
							OUT int8_t  		*classifierOut); 
#endif //RADARDEMO_CLASSIFIERSVM_H

