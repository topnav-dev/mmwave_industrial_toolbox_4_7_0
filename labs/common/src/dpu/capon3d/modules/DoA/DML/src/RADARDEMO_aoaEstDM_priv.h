/*! 
 *  \file   RADARDEMO_aoaEstimationDML_priv.h
 *
 *  \brief   Header file for RADARDEMO_aoaEstimationDML_priv.c
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

#ifndef RADARDEMO_AOAESTDML_PRIV_H
#define RADARDEMO_AOAESTDML_PRIV_H

#include <swpform.h>
#ifndef _TMS320C6600
#include <modules/utilities/radar_c674x.h>
#endif


/**< maximum number of searches for the coarse search .*/
#define RADARDEMO_AOAESTDML_MAXNUMFIRSTSTEP (4)
#define RADARDEMO_AOAESTDML_NORMVAETHR (0.02f)

/** 
 *  \struct   _RADARDEMO_aoAEstDML_handle_
 *   {
 *	uint32_t      nRxAnt;  	 
 *	uint32_t      estResolution;  
 *	uint32_t      estRange;
 *	uint32_t      steeringVecSize;  
 *	cplxf_t       *steeringVec;  
 *	uint32_t      *scratchPad;  
 *	uint8_t       firstStageSearchStep;  
 *   }   _RADARDEMO_aoAEstDML_handle_;
 *
 *  \brief   Structure element of the list of descriptors for UL allocations.
 *
 *
 */

typedef struct _RADARDEMO_aoAEstDML_handle_
{
	uint32_t      nRxAnt;  	       			/**< number of receive antennas.*/
	float         estResolution;  	        /**< Estimation resolution in degree.*/
	float         estRange;  	            /**< Range of the estimation, from -estRange to +estRange degrees*/
	uint32_t      steeringVecSize;  	    /**< size of -estRange:estResolution:estRange */
	cplxf_t       * steeringVec;            /**< steering vector for angle: -estRange:estResolution:estRange, for nRxAnt antennas, must be aligned to 8-byte boundary  */
											/**< The steeringVec is arranged in ... degree0ant1 degree0ant2 degree0ant3 degree1ant1 degree1ant2 degree1ant3 ... fashion*/
											/**< Ant0's steeringVec is 1 for all angle possiblities, so we don't save them*/
	uint32_t      * scratchPad;             /**< Pointer to the scratch memory used in this function, must have length sizeof(uint32_t) * (5 * steeringVecSize + 4 + 2* nRxAnt*nRxAnt), must be aligned to 8-byte boundary  */
	uint8_t       firstStageSearchStep;     /**< Search step for the first stage search, if 1, only 1 stage search (multi-stage search is disabled.*/
	float         minimumOutputConf;		/**< Minimum confidence range (in degree) for the output estimation. Estimates with bigger confidence range will not be reported.*/
	float        angleEstBound;             /**< the boundary of the estimates. Angle estimates outside [-angleEstBound angleEstBound] will be filtered out.*/
} RADARDEMO_aoAEstDML_handle;


/*! 
   \fn     RADARDEMO_aoaEstimationDML_4ant
 
   \brief   Estimate the angle of arrival of each detected object using DML for 4 Rx antennas. 
  
   \param[in]    Rn
               input covariance matrix from input antenna samples.
 
   \param[in]    steeringVec
               Pointer to steering vector.
 
   \param[in]    steeringVecSize
               Size of the steering vector.
 
   \param[in]    scratchPad
               Scratch memory.
 
   \param[in]    firstStageSearchStep
               First stage search step size, if 1, no second stage search needed.
 
   \param[out]    normVar
               Output normalized variance of the search metric.
			   
   \param[out]    angleEst
               Output angle estimates.

   \ret        Number of output angle estimates. Retuns 0 if nRxAnt !=4 , 
 
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_aoaEstimationDML_4ant(
							IN  cplxf_t * Rn,
							IN  cplxf_t * steeringVec,
							IN  int32_t steeringVecSize,
							IN  float   * scratchPad,
							IN  uint8_t   firstStageSearchStep,
							OUT float     * normVar,
							OUT int32_t   * angleEst);

/*! 
   \fn     RADARDEMO_aoaEstimationDML_8ant
 
   \brief   Estimate the angle of arrival of each detected object using DML for 8 Rx antennas. 
  
   \param[in]    Rn
               input covariance matrix from input antenna samples.
 
   \param[in]    steeringVec
               Pointer to steering vector.
 
   \param[in]    steeringVecSize
               Size of the steering vector.
 
   \param[in]    scratchPad
               Scratch memory.
 
   \param[in]    firstStageSearchStep
               First stage search step size, if 1, no second stage search needed.
 
   \param[out]    normVar
               Output normalized variance of the search metric.
			   
   \param[out]    angleEst
               Output angle estimates.
			   
   \ret        Number of output angle estimates. Retuns 0 if nRxAnt !=4 , 
 
   \pre       none
 
   \post      none
  
 
 */

extern int32_t	RADARDEMO_aoaEstimationDML_8ant(
							IN  cplxf_t * Rn,
							IN  cplxf_t * steeringVec,
							IN  int32_t steeringVecSize,
							IN  float   * scratchPad,
							IN uint8_t    firstStageSearchStep,
							OUT float     * normVar,
							OUT int32_t   * angleEst);



#endif //RADARDEMO_AOAESTDML_PRIV_H

