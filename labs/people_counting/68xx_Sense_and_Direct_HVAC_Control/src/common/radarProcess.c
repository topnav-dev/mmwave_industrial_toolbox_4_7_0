/*!
 *  \file   radarProcess.c
 *
 *  \brief   radar signal processing chain.
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

#include <radarProcess.h>
#include <math.h>
#include <modules/detection/CFAR/api/RADARDEMO_detectionCFAR.h>
#include <modules/DoA/CaponBF/api/RADARDEMO_aoaEstCaponBF.h>
#include <modules/rangeProc/rangeProc/api/RADARDEMO_rangeProc.h>
#include <modules/utilities/cycle_measure.h>
#include <modules/utilities/radarOsal_malloc.h>
#include <modules/utilities/radar_c674x.h>

#if (defined SOC_XWR18XX) || (defined SOC_XWR68XX)
#include <xdc/runtime/System.h>
#endif

#define ONEOVERFACTORIAL3 (1.f/6.f)
#define ONEOVERFACTORIAL5 (1.f/230.f)
#define ONEOVERFACTORIAL7 (1.f/5040.f)
#define MAXANT (8)
#define MAXWIN1DSize (128)
//user input configuration parameters
typedef struct _processInstance_
{
    // frame timing in ms

	float framePeriod;
    void  * rangeFFTInstance;
    void  * detectionInstance;
    void  * DoAInstance;

	RADARDEMO_rangeProc_input *rangeProcInput;
    cplx16_t *pFFT1DBuffer;

	//float *localDopplerInBufPtr;
	float * localPDP;
	float ** localPDPPtr;

	RADARDEMO_detectionCFAR_output * detectionCFAROutput;
	RADARDEMO_detectionCFAR_output * staticCFAROutput;

	uint8_t mimoModeFlag;  /**<Flag for MIMO mode: 0 -- SIMO, 1 -- TDM MIMO, 2 -- FDM or BF*/
	RADARDEMO_aoAEstCaponBF_input *aoaInput;
	float * aoaInputSignal;
	RADARDEMO_aoAEstCaponBF_output *aoaOutput;

	RADARDEMO_rangeProc_errorCode rangeProcErrorCode;
	RADARDEMO_detectionCFAR_errorCode cfarErrorCode;
	RADARDEMO_aoaEstCaponBF_errorCode aoaBFErrorCode;

	RADARDEMO_detectionCFAR_input * detectionCFARInput;

	float sidelobe_dynamic;
	float sidelobe_static;
	
	int32_t fftSize1D;
	int32_t fftSize2D;
	int32_t numChirpsPerFrame;
	int32_t numAdcSamplePerChirp;
	int32_t nRxAnt;
	int32_t maxNumDetObj;
	int32_t numAzimuthBin;
	float   rangeRes;
	float   dopplerRes;
	float   angleRes;
	cplx16_t *antPhaseCompCoeff;
	uint8_t		dopplerOversampleFactor;
	uint8_t		scaleDopCfarOutCFAR;
	float * perRangeBinMax;
	//float    * scratchBuffer;
	radarProcessBenchmarkObj * benchmarkPtr;
}radarProcessInstance_t;

#define RAD_TO_DEG      ( 3.1415926f/180.f)
#define DEG_TO_RAD      ( 180.f/3.1415926f)


void *radarProcessCreate(radarProcessConfig_t  * config, ProcessErrorCodes * procErrorCode)
{
    radarProcessInstance_t *inst;
	int32_t    i;
	int16_t win1D[MAXWIN1DSize];
	int32_t itemp;
	ProcessErrorCodes errorCode = PROCESS_OK;
	
    inst = (radarProcessInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessInstance_t), 8);

	inst->antPhaseCompCoeff		=	(cplx16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, config->numAntenna * sizeof(cplx16_t), 8);

	for (i = 0; i < config->numAntenna; i++)
	{
		inst->antPhaseCompCoeff[i].real = (int32_t)(32767.f * config->antPhaseCompCoeff[2 * i]);
		inst->antPhaseCompCoeff[i].imag = (int32_t)(32767.f * config->antPhaseCompCoeff[2 * i + 1]);
	}
	itemp								=	config->numChirpPerFrame; 
	if ((1 << (30 - _norm(itemp))) == itemp)
		inst->fftSize2D					=	itemp;
	else
		inst->fftSize2D					=	1 << (31 - _norm(itemp));


	config->fftSize2D =  inst->fftSize2D;
	config->dopplerOversampleFactor = config->doaConfig.dopplerOversampleFactor; //hardcoded to 2;
	inst->dopplerOversampleFactor = config->doaConfig.dopplerOversampleFactor;

	for (i = 0; i < (int32_t)config->rangeWinSize; i++ )
	{
//		itemp		=	(int32_t) (32768.f * config->rangeWindow[i] + 0.5f);
//		if (itemp >= 32767) itemp = 32767;
//		win1D[i]	=	itemp;
		win1D[i]	=	config->rangeWindow[i];
	}

	
	inst->numAzimuthBin =   (int32_t) ((2.f * config->doaConfig.estAngleRange) / (config->doaConfig.estAngleResolution) + 0.5f);

	/* range proc */
	{
		RADARDEMO_rangeProc_config rangeProcConfig0;
		RADARDEMO_rangeProc_config *rangeProcConfig = &rangeProcConfig0;

		memset(rangeProcConfig, 0, sizeof(RADARDEMO_rangeProc_config));

		itemp								=	config->numAdcSamplePerChirp; 
		if ((1 << (30 - _norm(itemp))) == itemp)
			inst->fftSize1D					=	itemp;
		else
			inst->fftSize1D					=	1 << (31 - _norm(itemp));
		rangeProcConfig->fft1DSize			=	inst->fftSize1D; 
		config->fftSize1D					=	inst->fftSize1D; 
		rangeProcConfig->nSamplesPerChirp	=	config->numAdcSamplePerChirp;

		rangeProcConfig->nRxAnt				=	(uint8_t)config->numPhyRxAntenna;
		rangeProcConfig->numChirpsPerFrame	=	config->numChirpPerFrame * config->numTxAntenna;
		rangeProcConfig->adcNumOutputBits	=	(uint8_t) config->numAdcBitsPerSample;
		rangeProcConfig->win1D				=	win1D;
		rangeProcConfig->win2D				=	NULL; 
		rangeProcConfig->adjustDCforInput	=	RADARDEMO_RANGEPROC_ADC_NO_ADJUST; 
		rangeProcConfig->win1DLength		=	(int16_t)config->rangeWinSize;

		rangeProcConfig->fft1DType			=	RADARDEMO_RANGEPROC_1DFFT_16x16; 

		rangeProcConfig->include2DwinFlag	=	0;  /* hardcoded, memory limitation from XWR16xx*/

		rangeProcConfig->outputFixedpFlag	=	1;  

		inst->rangeFFTInstance		= (void *) RADARDEMO_rangeProc_create(rangeProcConfig, &inst->rangeProcErrorCode);
		inst->numChirpsPerFrame		=	config->numChirpPerFrame;
		inst->numAdcSamplePerChirp	=	config->numAdcSamplePerChirp;
		inst->nRxAnt				=	config->numAntenna;
		if (inst->rangeProcErrorCode > RADARDEMO_RANGEPROC_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_RANGEPROC_INIT_FAILED;
		}
		inst->rangeProcInput	=	(RADARDEMO_rangeProc_input *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 1*sizeof(RADARDEMO_rangeProc_input), 1);
		if (inst->rangeProcInput == NULL)
		{
			errorCode 	=	PROCESS_ERROR_RANGEPROC_INOUTALLOC_FAILED;
		}
	}
	inst->perRangeBinMax		=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->fftSize1D*sizeof(float), 8);
	/* Detection CFAR */
	{
		RADARDEMO_detectionCFAR_config cfarConfig0;
		RADARDEMO_detectionCFAR_config *cfarConfig = &cfarConfig0;

		cfarConfig->fft1DSize		=	inst->fftSize1D; 
		cfarConfig->fft2DSize		=	inst->numAzimuthBin; 
		if (config->cfarConfig.cfarMethod == 6)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_RA_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 4)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 3)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CASOCFAR;
			cfarConfig->enableSecondPassSearch		=	0;
		}
		else if (config->cfarConfig.cfarMethod == 2)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CAVGCFAR;
			cfarConfig->enableSecondPassSearch		=	1;
		}
		else if (config->cfarConfig.cfarMethod == 1)
		{
			cfarConfig->cfarType		=	RADARDEMO_DETECTIONCFAR_CAVGCFAR;
			cfarConfig->enableSecondPassSearch		=	0;
		}
		else
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_NONSUPPORTEDMETHOD;
		}

		inst->sidelobe_dynamic = 0.4;
		inst->sidelobe_static = 0.3;
		

		cfarConfig->inputType		=	RADARDEMO_DETECTIONCFAR_INPUTTYPE_SP; 
		cfarConfig->pfa				=	(float)1e-6; //hardcoded, not used for now;
		cfarConfig->rangeRes		=	config->cfarConfig.rangeRes * (float)config->numAdcSamplePerChirp * _rcpsp((float)inst->fftSize1D);
		inst->rangeRes              =   cfarConfig->rangeRes;
		cfarConfig->dopplerRes		=	config->cfarConfig.velocityRes * (float)config->numChirpPerFrame * _rcpsp((float)inst->fftSize2D);
        inst->dopplerRes            =   cfarConfig->dopplerRes;
		cfarConfig->maxNumDetObj	=	(uint16_t)DOA_OUTPUT_MAXPOINTS;    
		cfarConfig->searchWinSizeRange	=	(uint8_t)config->cfarConfig.refWinSize[0];
		cfarConfig->guardSizeRange		=	(uint8_t)config->cfarConfig.guardWinSize[0];
		//hard coded for now for CASO
		cfarConfig->searchWinSizeDoppler=	(uint8_t)config->cfarConfig.refWinSize[1];
		cfarConfig->guardSizeDoppler	=	(uint8_t)config->cfarConfig.guardWinSize[1];
		cfarConfig->K0					=	config->cfarConfig.rangeThre; 
		cfarConfig->dopplerSearchRelThr	=	config->cfarConfig.azimuthThre; 
		cfarConfig->leftSkipSize		=	(uint8_t)config->cfarConfig.cfarDiscardRangeLeft;
		cfarConfig->rightSkipSize		=	(uint8_t)config->cfarConfig.cfarDiscardRangeRight;
		cfarConfig->leftSkipSizeAzimuth		=	(uint8_t)config->cfarConfig.cfarDiscardAngleLeft;
		cfarConfig->rightSkipSizeAzimuth	=	(uint8_t)config->cfarConfig.cfarDiscardAngleRight;
		cfarConfig->log2MagFlag			=	config->cfarConfig.log2MagFlag; 
		cfarConfig->sidelobe_dynamic	=	inst->sidelobe_dynamic;
		cfarConfig->sidelobe_static		=	inst->sidelobe_static; 

		inst->detectionInstance = (void *) RADARDEMO_detectionCFAR_create(cfarConfig, &inst->cfarErrorCode);
		if (inst->cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INIT_FAILED;
		}

		inst->maxNumDetObj	=	config->maxNumDetObj;         
		inst->detectionCFAROutput		=	(RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_output), 1);
		inst->detectionCFAROutput->rangeInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		inst->detectionCFAROutput->dopplerInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		//not used for range-azimuth CFAR (type 6)
		//inst->detectionCFAROutput->rangeEst	=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		//inst->detectionCFAROutput->dopplerEst	=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->snrEst		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->noise		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);

		inst->staticCFAROutput				=	(RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_output), 1);
		inst->staticCFAROutput->rangeInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		inst->staticCFAROutput->dopplerInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(uint16_t), 1);
		inst->staticCFAROutput->snrEst		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);
		inst->staticCFAROutput->noise		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, cfarConfig->maxNumDetObj * sizeof(float), 1);


		inst->detectionCFARInput		=	(RADARDEMO_detectionCFAR_input *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_detectionCFAR_input), 1);
		inst->detectionCFARInput->heatmapInput = inst->localPDPPtr;
		inst->detectionCFARInput->azMaxPerRangeBin = inst->perRangeBinMax;
		//inst->detectionCFARInput->sidelobeThr = 0.3;
		//inst->detectionCFARInput->enableSecondPass = 1;
		

		if ( (inst->detectionCFAROutput == NULL)
			|| (inst->detectionCFAROutput->rangeInd == NULL)
			|| (inst->detectionCFAROutput->dopplerInd == NULL)
			|| (inst->detectionCFAROutput->snrEst == NULL)
			|| (inst->detectionCFAROutput->noise == NULL))
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED;
		}

	}

	//Capon BF
	{
		RADARDEMO_aoaEstCaponBF_config aoaBFConfigParam0;
		RADARDEMO_aoaEstCaponBF_config *aoaBFConfigParam = &aoaBFConfigParam0;
		uint8_t antSpacing[MAXANT];

		aoaBFConfigParam->antSpacing = &antSpacing[0];
		for (i = 0; i < (int32_t)config->numAntenna; i++ )
		{
			aoaBFConfigParam->antSpacing[i] = i;
		}

		aoaBFConfigParam->estAngleResolution = config->doaConfig.estAngleResolution; 
		aoaBFConfigParam->nRxAnt = config->numAntenna; 
		aoaBFConfigParam->estAngleRange = config->doaConfig.estAngleRange;
		aoaBFConfigParam->gamma = config->doaConfig.gamma;
		aoaBFConfigParam->useCFAR4DopDet = config->doaConfig.useCFAR4DopDet;
		aoaBFConfigParam->dopCfarGuardLen = config->doaConfig.dopCfarGuardLen;
		aoaBFConfigParam->dopCfarThr = config->doaConfig.dopCfarThr;

		aoaBFConfigParam->dopplerFFTSize = inst->dopplerOversampleFactor * inst->fftSize2D;
        inst->dopplerRes            =   inst->dopplerRes * _rcpsp((float)inst->dopplerOversampleFactor);

		aoaBFConfigParam->nRxAnt = inst->nRxAnt;
		aoaBFConfigParam->numInputRangeBins = inst->fftSize1D;

		inst->DoAInstance = (void *) RADARDEMO_aoaEstCaponBF_create(aoaBFConfigParam, &inst->aoaBFErrorCode);
		if (inst->aoaBFErrorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INIT_FAILED;
		}
		inst->aoaOutput	=	(RADARDEMO_aoAEstCaponBF_output *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoAEstCaponBF_output), 1);
		inst->aoaOutput->dopplerIdx	=	(uint16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 20 * sizeof(RADARDEMO_aoAEstCaponBF_output), 1);
		//inst->aoaOutput->steeringVec = ((RADARDEMO_aoaEstCaponBF_handle)(inst->DoAInstance))->steeringVec;		
		inst->aoaOutput->static_information = (cplxf_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(cplxf_t) * config->fftSize1D * config->numAntenna, 1);
		inst->aoaInput		=	(RADARDEMO_aoAEstCaponBF_input *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(RADARDEMO_aoAEstCaponBF_input), 1);
	    inst->aoaInput->fallBackToConvBFFlag = 0; //hardcoded to perform Capon only;
	    inst->aoaInput->nChirps = config->numChirpPerFrame;
	    inst->aoaInput->clutterRemovalFlag = config->doaConfig.clutterRemovalFlag;
		inst->angleRes		=	(float) aoaBFConfigParam->estAngleResolution;
		config->numAzimuthBin = (uint16_t)inst->numAzimuthBin;

		if ((inst->aoaOutput == NULL) || (inst->aoaInput == NULL))
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED;
		}
	}
	inst->scaleDopCfarOutCFAR = config->doaConfig.scaleDopCfarOutCFAR;

	inst->framePeriod  = config->framePeriod;
	inst->pFFT1DBuffer = (cplx16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, inst->numChirpsPerFrame * inst->nRxAnt * inst->fftSize1D * sizeof (cplx16_t), 8);

	inst->mimoModeFlag  	=  (uint8_t)config->mimoModeFlag;
	inst->localPDP  	=  (float * ) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, inst->numAzimuthBin * inst->fftSize1D * sizeof(float), 8);
	inst->localPDPPtr	=	(float **)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->numAzimuthBin*sizeof(float * ), 1);
	for (i = 0; i < (int32_t) inst->numAzimuthBin; i++)
	{
		inst->localPDPPtr[i]	=	(float *)&inst->localPDP[i * inst->fftSize1D];
	}
    config->heatMapMemSize = inst->fftSize1D * inst->numAzimuthBin * sizeof(float);
    config->heatMapMem = inst->localPDP;
	memset(inst->localPDP, 0, inst->numAzimuthBin * inst->fftSize1D * sizeof(float));
	

	//inst->scratchBuffer		=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, 2 * inst->fftSize1D * sizeof(float), 8);
	//inst->scratchBuffer = (float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 1, inst->numAzimuthBin * sizeof(float), 8); 
	inst->benchmarkPtr = (radarProcessBenchmarkObj *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, sizeof(radarProcessBenchmarkObj), 1);
	inst->benchmarkPtr->bufferLen = 20;
	inst->benchmarkPtr->bufferIdx = 0;
	inst->benchmarkPtr->buffer    = (radarProcessBenchmarkElem *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem), 1);

	memset(inst->benchmarkPtr->buffer, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem));

	if ((inst->localPDP == NULL) || (inst->localPDPPtr== NULL)|| (inst->pFFT1DBuffer == NULL)
		//||(inst->scratchBuffer == NULL) 
		||(inst->benchmarkPtr == NULL) || (inst->benchmarkPtr->buffer == NULL))
		errorCode 	=	PROCESS_ERROR_INIT_MEMALLOC_FAILED;


	config->benchmarkPtr = inst->benchmarkPtr;
	config->pFFT1DBuffer = inst->pFFT1DBuffer;

	*procErrorCode = errorCode;

#ifndef CCS
	System_printf("processCreate: (radarProcessInstance_t *)0x%x\n", (uint32_t)inst);
	System_printf("processCreate: (RADARDEMO_rangeProc_handle *)0x%x\n", (uint32_t)(inst->rangeFFTInstance));
	System_printf("processCreate: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->detectionInstance));
	System_printf("processCreate: (RADARDEMO_aoaEstCaponBF_handle *)0x%x\n", (uint32_t)(inst->DoAInstance));
	System_printf("processCreate: (radarProcessBenchmarkObj *)0x%x\n", (uint32_t)(inst->benchmarkPtr));
	System_printf("processCreate: heatmap (float *)0x%x\n", (uint32_t)(inst->localPDP));
#endif
	return (void *) inst;
}

uint8_t radarRangeProcessRun(void *handle, cplx16_t * pDataIn, cplx16_t * pDataOut)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;

	processInst->rangeProcInput->inputSignal = pDataIn;
	processInst->rangeProcInput->chirpNumber = 0;			//irrelavent here because 2D windowing is done in Doppler processing.
	processInst->rangeProcErrorCode = RADARDEMO_rangeProc_run(
							processInst->rangeFFTInstance,
							processInst->rangeProcInput,
							(void *)pDataOut);
	return((uint8_t)processInst->rangeProcErrorCode);
}

uint8_t radarRangeAzimuthProcessRun(void *handle, uint32_t frameNum, int32_t rangeIdx, cplx16_t * pDataIn, float *pDataOut)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;
	int32_t i, j;
	int64_t		lltemp;
	int64_t     * RESTRICT inputPtr;
	float peakval=0.0;
	
#if 0
    if((frameNum % 10) == 0) {
		processInst->aoaInput->clutterRemovalFlag = 0;
        processInst->aoaInput->fallBackToConvBFFlag = 1;
	}
	else {
        processInst->aoaInput->clutterRemovalFlag = 1;
        processInst->aoaInput->fallBackToConvBFFlag = 0;
	}
#endif

	/* phase error compensation*/
	for (i = 0; i < processInst->nRxAnt; i++)
	{
		inputPtr	=	(int64_t *) &pDataIn[i * processInst->fftSize1D];
		lltemp	=	_itoll(_amem4(&processInst->antPhaseCompCoeff[i]), _amem4(&processInst->antPhaseCompCoeff[i]));
		for (j = 0; j < (processInst->fftSize1D >> 1); j++)
		{
			_amem8(&inputPtr[j])		=	_dcmpyr1(_amem8(&inputPtr[j]), lltemp);
		}
	}
	processInst->aoaInput->rangeIndx = rangeIdx; 
	processInst->aoaInput->inputAntSamples =  pDataIn;
	processInst->aoaOutput->rangeAzimuthHeatMap = pDataOut;
	processInst->aoaInput->processingStepSelector = 0; //BF part
	processInst->aoaBFErrorCode =     RADARDEMO_aoaEstCaponBF_run(
							processInst->DoAInstance,
							processInst->aoaInput,
							processInst->aoaOutput);
	for (i = 0; i < processInst->numAzimuthBin; i++)
	{
		if(peakval<pDataOut[i])
			peakval = pDataOut[i];
	}
	processInst->perRangeBinMax[rangeIdx] = peakval;

	return((uint8_t)processInst->aoaBFErrorCode);
}


uint8_t radarFrameProcessRun(void *handle, void * outBuffer, uint8_t flag_chain)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;
    uint8_t status = PROCESS_OK;
	int32_t i, j, k, tempNDet, totalNpnt;
	uint32_t timeStart, timeStamp;
	radarProcessBenchmarkObj * benchmarks = processInst->benchmarkPtr;
    radarProcessOutputToTracker * outBuffCntxt;

	outBuffCntxt = (radarProcessOutputToTracker*)outBuffer;

	timeStart	=	ranClock();
	timeStamp 	=	timeStart;
	if (benchmarks->bufferIdx > benchmarks->bufferLen)
		benchmarks->bufferIdx = 0;

	processInst->detectionCFARInput->heatmapInput = processInst->localPDPPtr;
	processInst->detectionCFARInput->azMaxPerRangeBin = processInst->perRangeBinMax;
	if(flag_chain == DYNAMIC_CHAIN )
	{
		processInst->detectionCFARInput->sidelobeThr = processInst->sidelobe_dynamic;
		processInst->detectionCFARInput->enableSecondPass = 1;
		processInst->detectionCFARInput->enable_neighbour_check = 1;
	}
	else if(flag_chain == STATIC_CHAIN )
	{
		processInst->detectionCFARInput->sidelobeThr = processInst->sidelobe_static;
		processInst->detectionCFARInput->enableSecondPass = 0;		
		processInst->detectionCFARInput->enable_neighbour_check = 1;
	}
	// Detection
	processInst->cfarErrorCode	=	RADARDEMO_detectionCFAR_run(
                            processInst->detectionInstance,
							processInst->detectionCFARInput,
							processInst->detectionCFAROutput);

	timeStart	=	ranClock();
	benchmarks->buffer[benchmarks->bufferIdx].cfarDetectionCycles 	=	timeStart - timeStamp;
	benchmarks->buffer[benchmarks->bufferIdx].numDetPnts 	=	processInst->detectionCFAROutput->numObjDetected;
	timeStamp 	=	timeStart;

	
	if(flag_chain == DYNAMIC_CHAIN )
	{
		totalNpnt = 0;
		tempNDet	=	processInst->detectionCFAROutput->numObjDetected;

		if (tempNDet > processInst->maxNumDetObj)
		{
			tempNDet	=	processInst->maxNumDetObj;
			status		=	PROCESS_ERROR_CFARPROC_NDET_EXCEEDLIMIT;
		}
		for(i = 0; i < tempNDet; i++ )
		{
			processInst->aoaInput->processingStepSelector = 1; //Doppler estimation part

			processInst->aoaInput->rangeIndx = processInst->detectionCFAROutput->rangeInd[i]; 
			processInst->aoaInput->azimuthIndx = processInst->detectionCFAROutput->dopplerInd[i]; 
			processInst->aoaInput->inputAntSamples = &processInst->pFFT1DBuffer[processInst->aoaInput->rangeIndx * processInst->nRxAnt * processInst->numChirpsPerFrame];
			processInst->aoaInput->bwDemon = processInst->localPDPPtr[processInst->aoaInput->azimuthIndx][processInst->aoaInput->rangeIndx];
			processInst->aoaBFErrorCode =     RADARDEMO_aoaEstCaponBF_run(
									processInst->DoAInstance,
									processInst->aoaInput,
									processInst->aoaOutput);

		
			if (processInst->aoaOutput->numDopplerIdx == 1)
			{
				k		=	0;
				outBuffCntxt->angle[totalNpnt]		=	RAD_TO_DEG * processInst->aoaOutput->angleEst;
				outBuffCntxt->range[totalNpnt]		=	(float)processInst->detectionCFAROutput->rangeInd[i] * processInst->rangeRes;
			
				if ((uint32_t)processInst->aoaOutput->dopplerIdx[k] >= (((uint32_t)processInst->dopplerOversampleFactor * (uint32_t)processInst->fftSize2D) >> 1))
					j			=	(int32_t)processInst->aoaOutput->dopplerIdx[k] - (int32_t)(processInst->dopplerOversampleFactor * processInst->fftSize2D);
				else
					j			=	(int32_t)processInst->aoaOutput->dopplerIdx[k];

				outBuffCntxt->doppler[totalNpnt] = (float)j * processInst->dopplerRes;				
				if ((processInst->scaleDopCfarOutCFAR <= 1) || (processInst->aoaOutput->dopplerDetSNR == NULL))
					outBuffCntxt->snr[totalNpnt]	=	processInst->detectionCFAROutput->snrEst[i];
				else
					outBuffCntxt->snr[totalNpnt]	=	processInst->aoaOutput->dopplerDetSNR[k];
				totalNpnt++;
				if (totalNpnt > DOA_OUTPUT_MAXPOINTS)
					break;
			}
			else
			{
				float sumDopSNR;
			
				sumDopSNR	=	0.f;
				for (k = 0; k < processInst->aoaOutput->numDopplerIdx; k++)
				{
					sumDopSNR	+=	processInst->aoaOutput->dopplerDetSNR[k];
				}
				sumDopSNR	=	_rcpsp(sumDopSNR);
				for (k = 0; k < processInst->aoaOutput->numDopplerIdx; k++)
				{
					outBuffCntxt->angle[totalNpnt]		=	RAD_TO_DEG * processInst->aoaOutput->angleEst;
					outBuffCntxt->range[totalNpnt]		=	(float)processInst->detectionCFAROutput->rangeInd[i] * processInst->rangeRes;
				
					if ((uint32_t)processInst->aoaOutput->dopplerIdx[k] >= (((uint32_t)processInst->dopplerOversampleFactor * (uint32_t)processInst->fftSize2D) >> 1))
						j			=	(int32_t)processInst->aoaOutput->dopplerIdx[k] - (int32_t)(processInst->dopplerOversampleFactor * processInst->fftSize2D);
					else
						j			=	(int32_t)processInst->aoaOutput->dopplerIdx[k];

					outBuffCntxt->doppler[totalNpnt] = (float)j * processInst->dopplerRes;
				
					if (processInst->scaleDopCfarOutCFAR == 0)
						outBuffCntxt->snr[totalNpnt]	=	processInst->detectionCFAROutput->snrEst[i];
					else if (processInst->scaleDopCfarOutCFAR == 1)
						outBuffCntxt->snr[totalNpnt]	=	processInst->detectionCFAROutput->snrEst[i] * processInst->aoaOutput->dopplerDetSNR[k] * sumDopSNR;
					else 
						outBuffCntxt->snr[totalNpnt]	=	processInst->aoaOutput->dopplerDetSNR[k];
					totalNpnt++;
					if (totalNpnt > DOA_OUTPUT_MAXPOINTS)
						break;
				}
			}
		}
	}
	else if(flag_chain == STATIC_CHAIN)
	{
		totalNpnt = outBuffCntxt->object_count;
		tempNDet	=	processInst->detectionCFAROutput->numObjDetected;
		if ((tempNDet+totalNpnt) > processInst->maxNumDetObj)
		{
			tempNDet	=	processInst->maxNumDetObj-totalNpnt;
			status		=	PROCESS_ERROR_CFARPROC_NDET_EXCEEDLIMIT;
		}

		for(i = 0; i < tempNDet; i++ )
		{
			outBuffCntxt->angle[totalNpnt]		=	RAD_TO_DEG * (60 -processInst->detectionCFAROutput->dopplerInd[i] * processInst->angleRes);
			outBuffCntxt->range[totalNpnt]		=	(float)processInst->detectionCFAROutput->rangeInd[i] * processInst->rangeRes;
			outBuffCntxt->doppler[totalNpnt]		=	0;
			outBuffCntxt->snr[totalNpnt++]		=	processInst->detectionCFAROutput->snrEst[i];
		}


	}


	outBuffCntxt->object_count = totalNpnt;
    timeStart   =   ranClock();
    benchmarks->buffer[benchmarks->bufferIdx].dopplerEstCycles   =   timeStart - timeStamp;
    timeStamp   =   timeStart;
	
    benchmarks->bufferIdx++;
    if (benchmarks->bufferIdx > benchmarks->bufferLen)
        benchmarks->bufferIdx = 0;

    return status;

}


int32_t processDelete(void *handle)
{
    radarProcessInstance_t *inst = (radarProcessInstance_t *)handle;
    //EDMA3_DRV_Result edma3Result = EDMA3_DRV_SOK;

    RADARDEMO_rangeProc_delete(inst->rangeFFTInstance);
    RADARDEMO_detectionCFAR_delete(inst->detectionInstance);
	RADARDEMO_aoaEstCaponBF_delete(inst->DoAInstance);

    radarOsal_memFree(inst->rangeProcInput, sizeof(RADARDEMO_rangeProc_input));

    radarOsal_memFree(inst->localPDPPtr, inst->numAzimuthBin*sizeof(float * ));
    radarOsal_memFree(inst->localPDP, inst->numAzimuthBin * inst->fftSize1D * sizeof(float));

    radarOsal_memFree(inst->detectionCFAROutput->rangeInd, inst->maxNumDetObj *sizeof(uint16_t ));
    radarOsal_memFree(inst->detectionCFAROutput->dopplerInd, inst->maxNumDetObj *sizeof(uint16_t ));
    radarOsal_memFree(inst->detectionCFAROutput->rangeEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->dopplerEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->snrEst, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput->noise, inst->maxNumDetObj *sizeof(float ));
    radarOsal_memFree(inst->detectionCFAROutput, sizeof(RADARDEMO_detectionCFAR_output));

    radarOsal_memFree(inst->aoaOutput, inst->maxNumDetObj *sizeof(RADARDEMO_aoAEstCaponBF_output ));
    radarOsal_memFree(inst->aoaInput, sizeof(RADARDEMO_aoAEstCaponBF_input));
	radarOsal_memFree(inst->aoaOutput->static_information, sizeof(cplxf_t) * inst->fftSize1D * 2 * inst->nRxAnt);

    radarOsal_memFree(inst->pFFT1DBuffer, 2 * inst->nRxAnt * inst->fftSize2D * inst->fftSize1D * sizeof (float));
    radarOsal_memFree(inst->aoaInputSignal, 2 * inst->nRxAnt * sizeof (float));
    //radarOsal_memFree(inst->scratchBuffer, 2 * inst->fftSize1D * sizeof (float));

    radarOsal_memFree(inst->benchmarkPtr->buffer, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem));
    radarOsal_memFree(inst->benchmarkPtr, sizeof(radarProcessBenchmarkObj));

    radarOsal_memFree(handle, sizeof(radarProcessInstance_t));

    return PROCESS_OK;

}


uint8_t radarRangeAzimuthProcessRun_static(void *handle, uint32_t frameNum, int32_t rangeIdx, cplx16_t * pDataIn, float *pDataOut)
{
    radarProcessInstance_t *processInst = (radarProcessInstance_t *)handle;
	processInst->aoaInput->rangeIndx = rangeIdx;
	processInst->aoaInput->processingStepSelector = 0; //BF part
	processInst->aoaBFErrorCode =     RADARDEMO_aoaEstCaponBF_static_run(
							processInst->DoAInstance,
							processInst->aoaInput,
							processInst->aoaOutput,	
							pDataOut,
							processInst->perRangeBinMax);

	return((uint8_t)processInst->aoaBFErrorCode);
}


