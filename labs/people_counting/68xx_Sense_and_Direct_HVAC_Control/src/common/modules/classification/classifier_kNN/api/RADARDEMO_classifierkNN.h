/*! 
 *  \file   RADARDEMO_classifierkNNion.h
 *
 *  \brief   Header file for RADARDEMO_classifierkNNion module
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



#ifndef RADARDEMO_CLASSIFIERKNN_H
#define RADARDEMO_CLASSIFIERKNN_H

#include <swpform.h>
#include <modules/classification/classifier_kNN/api/RADARDEMO_classifierCommon.h>


/**
 * \enum RADARDEMO_classifierkNN_errorCode
 *  \brief   enum for feature extraction error code.
 */

typedef enum
{
	RADARDEMO_CLASSIFIERKNN_NO_ERROR = 0,					/**< No error */
	RADARDEMO_CLASSIFIERKNN_INVALID_CFG,    				/**< Invalide configurations to RADARDEMO_classifierkNN module */
	RADARDEMO_CLASSIFIERKNN_FAIL_ALLOCATE_HANDLE,			/**< RADARDEMO_classifierkNN_create failed to allocate handle */ 
	RADARDEMO_CLASSIFIERKNN_FAIL_ALLOCATE_LOCALINSTMEM,		/**< RADARDEMO_classifierkNN_create failed to allocate memory for buffers in local instance  */ 
	RADARDEMO_CLASSIFIERKNN_INOUTPTR_NOTCORRECT				/**< input andor output buffer for RADARDEMO_classifierkNN_run are either NULL, or not aligned properly  */
} RADARDEMO_classifierkNN_errorCode;

/**
 *  \struct   RADARDEMO_classifierkNN_cbDescr
 *
 *  \brief   Structure for codebook descriptor used in RADARDEMO_classifierkNN module.
 *
 */

#define  RADARDEMO_classifierkNN_cbDescr RADARDEMO_classifier_cbDescr

/**
 *  \struct   RADARDEMO_classifierkNN_config
 *
 *  \brief   Structure for RADARDEMO_classifierkNN configuration.
 *
 */

typedef struct _RADARDEMO_classifierkNN_config_
{
	RADARDEMO_classifierkNN_cbDescr codebookDescr;/**< Pointer to the codebook descriptor for RADARDEMO_classifierkNN */ 
	uint8_t     k;                                 /**< k value for kNN, meaning number of nearest neighbor for the final decision*/
} RADARDEMO_classifierkNN_config;

/**
 *  \struct   RADARDEMO_classifierkNN_input
 *
 *  \brief   Structure for input to RADARDEMO_classifierkNN module.
 *
 */
#define  RADARDEMO_classifierkNN_input RADARDEMO_classifier_input

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

extern void	* RADARDEMO_classifierkNN_create(
                            IN  RADARDEMO_classifierkNN_config * moduleConfig, 
							OUT RADARDEMO_classifierkNN_errorCode * errorCode);

/*! 
   \fn     RADARDEMO_classifierkNN_delete
 
   \brief   Delete RADARDEMO_classifierkNN module. 
  
   \param[in]    handle
               Module handle.
			   
   \pre       none
 
   \post      none
 
 */

extern void	RADARDEMO_classifierkNN_delete(
                            IN  void * handle);


/*! 
   \fn     RADARDEMO_classifierkNN_run
 
   \brief   Classifier kNN module, called for every target. 
  
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
							OUT int8_t  		*classifierOut); 
#endif //RADARDEMO_CLASSIFIERKNN_H

