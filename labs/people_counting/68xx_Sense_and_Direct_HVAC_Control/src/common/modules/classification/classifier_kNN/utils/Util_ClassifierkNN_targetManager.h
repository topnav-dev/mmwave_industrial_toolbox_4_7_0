/**
 *   @file  Util_ClassifierkNN_targetManager.h
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

#ifndef UTIL_CLASSIFIERKNN_TARGETMANAGER_H
#define UTIL_CLASSIFIERKNN_TARGETMANAGER_H
#include "swpform.h"
#include <string.h>
#include <math.h>
#ifdef CCS
//#include "testclassifierkNN.h"
#else
#include <common/mmw_output.h>
#endif
#include <modules/classification/classifier_kNN/utils/classifierkNN_process.h>
#include <modules/classification/classifier_kNN/utils/Util_input_message.h>

/*! 
   \fn     Util_ClassifierkNN_preProc
 
   \brief   Preprocessing utility function for target management. Called before feature extraction. 
   	   	    Set the activeTargetFlag for the corresponding target if target ID shows up in the tracker output.
			Resets structures for disqualified targets if target ID does not show up in the tracker output for the active target.
  
   \param[in]    targetListTLV
               Pointer to the input target list TLV.

	\param[out]    handle
               Classifier kNN process module handle.
   \ret  none

   \pre       none
 
   \post      none
 
 */

extern void Util_ClassifierkNN_preProc(
			IN void * targetListTLV,
			OUT classifierkNN_process_handle * handle
		);

/*! 
   \fn     Util_ClassifierkNN_postProc
 
   \brief   Postprocessing utility function for target management. Called after classification module.
			Updates distances between active targets.
			Post filters the classification results.
  
	\param[in, out]    handle
               Classifier kNN process module handle.
  
	\param[out]    output
               Classifier kNN process output.

   \ret  none

   \pre       none
 
   \post      none
 
 */

extern void Util_ClassifierkNN_postProc(
			INOUT classifierkNN_process_handle * handle,
			OUT classifierkNN_process_output *output
		);
#endif //UTIL_CLASSIFIERKNN_TARGETMANAGER_H
