/*!
 *  \file   radarProcess_internal.h
 *
 *  \brief   Internal header file for 2D-Capon-based radar signal processing chain 
 *           from range FFT output till point cloud generated for tracker.
 *
 *  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef _RADARPROCESS_INTERNAL_H
#define _RADARPROCESS_INTERNAL_H


#include <common/src/dpu/capon3d_overhead/include/swpform.h>
#include <common/src/dpu/capon3d_overhead/radarProcess.h>
#include <common/src/dpu/capon3d_overhead/include/copyTranspose.h>

#if defined(_WIN32) || defined(CCS)
#include <stdio.h>
#endif


/**
 *  \def  _processInstance_
 *
 *  \brief   DPU instance structure definition.
 *
 *  \sa
 */

typedef struct _processInstance_
{
	float 								framePeriod;			/**<Frame period*/
    void  								* dynamicCFARInstance;	/**<dynamic CFAR handle*/
    void  								* staticCFARInstance;	/**<static CFAR handle*/
    void  								* aoaInstance;			/**<2D capon handle*/

	float 								* localHeatmap;			/**<pointer to heatmap memory*/

	float 								** dynamicHeatmapPtr;	/**<2D pointer to heatmap memory for dynamic scene, in [angle][range] format as CFAR input*/
	float 								** staticHeatmapPtr;	/**<2D pointer to heatmap memory for static scene, in [angle][range] format as CFAR input*/
	float								* perRangeBinMax;		/**<per range bin max value from heatmap, for CFAR input*/

	int8_t                              staticProcEnabled;      /**<static processing enabled, if set to 1*/

	RADARDEMO_detectionCFAR_input		* detectionCFARInput;	/**<CFAR input*/
	RADARDEMO_detectionCFAR_output		* detectionCFAROutput;	/**<CFAR output*/

	uint8_t								mimoModeFlag;			/**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	RADARDEMO_aoaEst2DCaponBF_input		* aoaInput;				/**<2D capon input*/
	RADARDEMO_aoaEst2DCaponBF_output	* aoaOutput;			/**<2D capon output*/

	RADARDEMO_detectionCFAR_errorCode	cfarErrorCode;			/**<CFAR error code*/
	RADARDEMO_aoaEst2DCaponBF_errorCode	aoaBFErrorCode;			/**<2D capon error code*/
	float								dynamicSideLobeThr;		/**<dynamic CFAR sidelobe relative threshold*/
	float								staticSideLobeThr;		/**<static CFAR sidelobe relative threshold*/
	
	uint32_t							heatMapMemSize;			/**< heatmap size, output from the init function -- in case to be used in framework. */
	float								* tempHeatMapOut;		/**<heatmap output per range bin, to be transposed and stored to final heatmap buffer*/
	cplxf_t								* static_information;	/**< Zero doppler samples for the range bins, for all the antennas, arranged in ant x rangeBin format.*/	
	int32_t								numRangeBins;			/**<range FFT size*/
	int32_t								DopplerFFTSize;			/**<Doppler FFT size*/
	int32_t								numChirpsPerFrame;		/**<number of chirps per frame*/
	int32_t								numAdcSamplePerChirp;	/**<number of ADC saples per chirps*/
	int32_t								nRxAnt;					/**<number of total virtual RX antennas*/
	int32_t								maxNumDetObj;			/**<max number of detected points per frame*/
	int32_t								numDynAngleBin;			/**<number of angle bins for dynamic range-angle heatmap*/
	int32_t								numStaticAngleBin;		/**<number of angle bins for static range-angle heatmap*/
	int32_t								numAzimuthBin;			/**<number of azimuth bins for 2D capon*/
	int32_t								numElevationBin;		/**<number of elevation bins for 2D capon*/
	float								rangeRes;				/**<range interbin resolution*/
	float								dopplerRes;				/**<Doppler interbin resolution*/
	uint8_t								dopplerOversampleFactor;/**<Doppler FFT oversample factor, currently un-used -- heardcoded to 1*/
	uint8_t								scaleDopCfarOutCFAR;	/**<Doppler output scale to accommondate FFT oversample factor, currently un-used -- heardcoded to 1*/
	uint8_t								cfarRangeSkipLeft;		/**<range domain left side skip samples for CAFR*/
	uint8_t								cfarRangeSkipRight;		/**<range domain right side skip samples for CAFR*/

	radarProcessBenchmarkObj			* benchmarkPtr;
}radarProcessInstance_t;


#endif  // _RADARPROCESS_INTERNAL_H

/* Nothing past this point */
