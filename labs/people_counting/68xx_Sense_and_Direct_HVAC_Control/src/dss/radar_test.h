/*!
 *  \file   radar_test.h
 *
 *  \brief   Header file for Unit test bench for radar signal processing chain.
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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


#ifndef RADARTEST_H
#define RADARTEST_H

#include <swpform.h>
#include <modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>
#include <modules/dopplerProc/dopplerProc/api/RADARDEMO_dopplerProc.h>
#include <modules/rangeProc/rangeProc/api/RADARDEMO_rangeProc.h>
#include <modules/DoA/DML/api/RADARDEMO_aoaEstDML.h>
#include <modules/DoA/common/api/RADARDEMO_aoaEst_commonDef.h>
#include <modules/utilities/cycle_measure.h>
#include <modules/utilities/radarOsal_malloc.h>


/** 
 *  \struct   _RADARtest_configDesc_
 *   {
 *	uint32_t      ADCnob;
 *	uint32_t      nSamplesValidPerChirp;
 *	uint32_t      nSamplesTotPerChirp;
 *	uint32_t      nChirpsPerFrame;
 *	uint32_t      nRxAnt;
 *	uint32_t      fftSize1D;
 *	uint32_t      fftSize2D;
 *	uint32_t      fftSize3D;
 *	float	      sampRate;
 *	float   	  freqSlope;
 *	float      	  centerLamda;
 *	float         chirpTime;
 *	float         resetTime;
 *	float         antSpacing;
 *	uint32_t      threshWinSize;
 *	uint32_t      threshSkipSize;
 *	float         pfa;
 *  uint8_t       maxNumDetObj;
 *   }   RADARtest_configDesc;
 *
 *  \brief   Structure element of the list of descriptors for UL allocations.
 *
 *
 */

typedef struct _RADARtest_configDesc_
{

	uint32_t      ADCnob;  	                /**< number of bit for ADC.*/
	uint32_t      nSamplesValidPerChirp;  	/**< number of information samples per chirp.*/
	uint32_t      nSamplesTotPerChirp;  	/**< number of total samples per chirp.*/
	uint32_t      nChirpsPerFrame;  		/**< number of chirps per frame.*/
	uint32_t      nRxAnt;  	       			/**< number of receive antennas.*/
	uint32_t      fftSize1D;  	       		/**< 1D FFT size.*/
	uint32_t      fftSize2D;  				/**< 2D FFT size.*/
	uint32_t      fftSize3D;  				/**< 3D FFT size.*/
	float	      sampRate;  				/**< Sampling rate.*/
	float   	  freqSlope;  				/**< Frequency Slope.*/
	float      	  centerLamda;  			/**< Lambda of center frequency.*/
	float         chirpTime;  				/**< Chirp duration in sec.*/
	float         resetTime;  				/**< Rest time between chirps in sec.*/
	float         antSpacing;  				/**< antenna spacing.*/
	uint32_t      threshWinSize;  			/**< Sliding window size.*/
	uint32_t      threshSkipSize;  			/**< Number of skipped samples.*/
	float         pfa;  			        /**< Target false detection ratio.*/
	uint8_t       maxNumDetObj;  			/**< maximun number of detected objects.*/
} RADARtest_configDesc;

#endif //RADARTEST_H

