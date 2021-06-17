/*!
 *  \file   radarProcess.c
 *
 *  \brief   radar signal processing chain.
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

#include <common/src/dpu/capon3d/include/radarProcess_internal.h>
#include <math.h>
#include <common/src/dpu/capon3d/modules/utilities/cycle_measure.h>
#include <common/src/dpu/capon3d/modules/utilities/radarOsal_malloc.h>
#include <common/src/dpu/capon3d/modules/utilities/radar_c674x.h>

#if (defined SOC_XWR16XX) || (defined SOC_XWR68XX)
#include <xdc/runtime/System.h>
#endif

#define ONEOVERFACTORIAL3 (1.f/6.f)
#define ONEOVERFACTORIAL5 (1.f/230.f)
#define ONEOVERFACTORIAL7 (1.f/5040.f)
#define MAXANT (12)
#define MAXWIN1DSize (128)
//user input configuration parameters

/***************************************************************************
 *************************** External API Functions ************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU init function. It allocates memory to store
 *		its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              radarProcess initialization parameters.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *
 *  @retval
 *      Success     - valid radarProcess handle
 *  @retval
 *      Error       - NULL
 */
DPU_radarProcess_Handle DPU_radarProcess_init
(
    DPU_radarProcessConfig_t	* initParams,
    DPU_ProcessErrorCodes			* errCode
)
{
    radarProcessInstance_t * inst;
	int32_t		i;
	int32_t		itemp, perRngbinHeatmapLen;
	DPU_ProcessErrorCodes errorCode = PROCESS_OK;
	
    inst								=	(radarProcessInstance_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(radarProcessInstance_t), 8);

	itemp								=	initParams->numChirpPerFrame; 
	if ((1 << (30 - _norm(itemp))) == itemp)
		inst->DopplerFFTSize			=	itemp;
	else
		inst->DopplerFFTSize			=	1 << (31 - _norm(itemp));

	inst->dopplerOversampleFactor		=	1;
	inst->scaleDopCfarOutCFAR			=	1;
	inst->nRxAnt						=	initParams->numAntenna;
	inst->numChirpsPerFrame				=	initParams->numChirpPerFrame; 


	itemp								=	initParams->numAdcSamplePerChirp; 
	if ((1 << (30 - _norm(itemp))) == itemp)
		inst->numRangeBins				=	itemp;
	else
		inst->numRangeBins				=	1 << (31 - _norm(itemp));
	initParams->numRangeBins			=	inst->numRangeBins;

	//update the interbin resolution for range and Doppler
	initParams->dynamicCfarConfig.rangeRes	=	divsp_i(initParams->dynamicCfarConfig.rangeRes * (float)initParams->numAdcSamplePerChirp, (float)inst->numRangeBins);
	inst->rangeRes							=	initParams->dynamicCfarConfig.rangeRes;
	initParams->dynamicCfarConfig.dopplerRes=	divsp_i(initParams->dynamicCfarConfig.dopplerRes * (float)initParams->numChirpPerFrame, (float)inst->DopplerFFTSize);
	inst->dopplerRes						=	initParams->dynamicCfarConfig.dopplerRes;


	/* rearrange phase compensation coeff from OOB to be able to use in 2D capon chain */
	{
		cplxf_t		* tempCmpVec;
		float		tempRe, tempIm, tempP, invsqrt, errorLimit;
		int32_t     compFlag;

		tempCmpVec			=	(cplxf_t *)&(initParams->doaConfig.phaseCompVect[0]);
		//check whether we need to compensate
		compFlag 			=	0;
		errorLimit 			=	0.0001f;
		for (i = 0; i < initParams->numAntenna; i++)
		{
			if (((_fabs(tempCmpVec[i].real) - 1.f) > errorLimit) || (_fabs(tempCmpVec[i].imag) > errorLimit))
				compFlag	=	1;
		}
		if (compFlag)
		{
			for (i = 0; i < initParams->numAntenna; i++)
			{
				tempRe				=	tempCmpVec[i].real;
				tempIm				=	tempCmpVec[i].imag;
				tempP				=	tempRe * tempRe + tempIm * tempIm;
				invsqrt				=	_rsqrsp(tempP);
				invsqrt				=	invsqrt * (1.5f - 0.5f * tempP * invsqrt * invsqrt);
				invsqrt				=	invsqrt * (1.5f - 0.5f * tempP * invsqrt * invsqrt);
				tempCmpVec[i].real	=	tempRe * invsqrt * (float)(initParams->doaConfig.phaseRot[i]);
				tempCmpVec[i].imag	=	-tempIm * invsqrt * (float)(initParams->doaConfig.phaseRot[i]);
			}
		}
	}


	inst->perRangeBinMax				=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, inst->numRangeBins * sizeof(float), 8);
	/* 2D Capon DoA init and config */
	{
		int32_t	heatmapSize, maxNumAngleEst;

		initParams->doaConfig.numInputRangeBins =	inst->numRangeBins;
		initParams->doaConfig.numInputChirps	=	initParams->numChirpPerFrame;
		initParams->doaConfig.dopperFFTSize		=	inst->DopplerFFTSize;
		initParams->doaConfig.nRxAnt			=	inst->nRxAnt;

	    inst->aoaInstance				=	(void *) RADARDEMO_aoaEst2DCaponBF_create(&initParams->doaConfig, &inst->aoaBFErrorCode);
		if (inst->aoaBFErrorCode > RADARDEMO_AOACAPONBF_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INIT_FAILED;
		}

		inst->numDynAngleBin			=	initParams->doaConfig.numRAangleBin;
		inst->aoaInput					=	(RADARDEMO_aoaEst2DCaponBF_input *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_aoaEst2DCaponBF_input), 1);
		inst->aoaOutput					=	(RADARDEMO_aoaEst2DCaponBF_output *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_aoaEst2DCaponBF_output), 1);

		heatmapSize						=   (int32_t)ceil(divsp_i ((float)initParams->doaConfig.numAzimBins, (float)initParams->doaConfig.staticEstCfg.staticAzimStepDeciFactor))
											* (int32_t)ceil(divsp_i ((float)initParams->doaConfig.numElevBins, (float)initParams->doaConfig.staticEstCfg.staticElevStepDeciFactor))
											* initParams->doaConfig.numInputRangeBins;
		if (heatmapSize         <   initParams->doaConfig.numRAangleBin * initParams->doaConfig.numInputRangeBins)
			heatmapSize         =   initParams->doaConfig.numRAangleBin * initParams->doaConfig.numInputRangeBins;
		if ( heatmapSize		<	initParams->doaConfig.numAzimBins * initParams->doaConfig.numElevBins)
			heatmapSize		    =	initParams->doaConfig.numAzimBins * initParams->doaConfig.numElevBins;
		inst->localHeatmap		=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0,  heatmapSize *sizeof(float), 8);
	    initParams->heatMapMemSize	=	heatmapSize;
		inst->heatMapMemSize	=	heatmapSize;

		if (initParams->doaConfig.rangeAngleCfg.detectionMethod <= 1)
			maxNumAngleEst		=	initParams->doaConfig.angle2DEst.azimElevAngleEstCfg.maxNpeak2Search * (initParams->doaConfig.angle2DEst.azimElevAngleEstCfg.peakExpSamples * 2 + 1) * (initParams->doaConfig.angle2DEst.azimElevAngleEstCfg.peakExpSamples * 2 + 1);
		else
			maxNumAngleEst		=	(initParams->doaConfig.angle2DEst.azimElevZoominCfg.peakExpSamples * 2 + 1) * (initParams->doaConfig.angle2DEst.azimElevZoominCfg.peakExpSamples * 2 + 1);

		inst->aoaOutput->azimEst	=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * sizeof(float), 1);
		inst->aoaOutput->elevEst	=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * sizeof(float), 1);
		inst->aoaOutput->peakPow	=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * sizeof(float), 1);
		inst->aoaOutput->bwFilter	=	(cplxf_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * initParams->doaConfig.nRxAnt * sizeof(cplxf_t), 8);
		//inst->aoaOutput->malValPerRngBin	=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, initParams->doaConfig.numInputRangeBins * sizeof(float), 1);
		inst->aoaOutput->static_information	=	(cplxf_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, initParams->doaConfig.numInputRangeBins * initParams->doaConfig.nRxAnt * sizeof(cplxf_t), 8);
		inst->aoaOutput->malValPerRngBin	=	inst->perRangeBinMax;

		inst->aoaOutput->invRnMatrices		=	(cplxf_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, initParams->doaConfig.numInputRangeBins * (initParams->doaConfig.nRxAnt >> 1) * (initParams->doaConfig.nRxAnt + 1) * sizeof(cplxf_t), 8);
		if (initParams->doaConfig.rangeAngleCfg.dopplerEstMethod == 1)
		{
			inst->aoaOutput->dopplerIdx		=	(uint16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * MAX_DOPCFAR_DET * sizeof(uint16_t), 1);
			inst->aoaOutput->dopplerDetSNR	=	(float *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * MAX_DOPCFAR_DET * sizeof(float), 1);
		}
		else
		{
			inst->aoaOutput->dopplerIdx		=	(uint16_t *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, maxNumAngleEst * sizeof(uint16_t), 1);
		}
#ifdef CAPON2DMODULEDEBUG
        inst->aoaOutput->cyclesLog          =   (RADARDEMO_aoaEst2DCaponBF_moduleCycles *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(RADARDEMO_aoaEst2DCaponBF_moduleCycles), 8);
		memset(inst->aoaOutput->cyclesLog, 0, sizeof(RADARDEMO_aoaEst2DCaponBF_moduleCycles));
#endif

		if ((inst->aoaOutput == NULL) || (inst->aoaInput == NULL))
		{
			errorCode 	=	PROCESS_ERROR_DOAPROC_INOUTALLOC_FAILED;
		}
	}


	/* dynamic Detection CFAR init and config */
	{
		inst->cfarRangeSkipLeft							=	initParams->dynamicCfarConfig.leftSkipSize;
		inst->cfarRangeSkipRight						=	initParams->dynamicCfarConfig.rightSkipSize;
		initParams->dynamicCfarConfig.fft2DSize			=	initParams->doaConfig.numRAangleBin;
		initParams->dynamicCfarConfig.fft1DSize			=	inst->numRangeBins; 
		initParams->dynamicCfarConfig.maxNumDetObj		=	MAX_DYNAMIC_CFAR_PNTS;
		initParams->dynamicCfarConfig.angleDim1			=	initParams->doaConfig.numAzimBins;		// not used, hardcoded to 0
		initParams->dynamicCfarConfig.angleDim2			=	initParams->doaConfig.numElevBins;		// not used, hardcoded to 0

		inst->dynamicCFARInstance = (void *) RADARDEMO_detectionCFAR_create(&initParams->dynamicCfarConfig, &inst->cfarErrorCode);
		if (inst->cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INIT_FAILED;
		}

		inst->maxNumDetObj						=	initParams->maxNumDetObj;         
		inst->detectionCFAROutput				=	(RADARDEMO_detectionCFAR_output *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_detectionCFAR_output), 1);
		inst->detectionCFAROutput->rangeInd		=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, initParams->dynamicCfarConfig.maxNumDetObj * sizeof(uint16_t), 1);
		inst->detectionCFAROutput->dopplerInd	=	(uint16_t *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, initParams->dynamicCfarConfig.maxNumDetObj * sizeof(uint16_t), 1);
		inst->detectionCFAROutput->snrEst		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, initParams->dynamicCfarConfig.maxNumDetObj * sizeof(float), 1);
		inst->detectionCFAROutput->noise		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, initParams->dynamicCfarConfig.maxNumDetObj * sizeof(float), 1);

		inst->detectionCFARInput				=	(RADARDEMO_detectionCFAR_input *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, sizeof(RADARDEMO_detectionCFAR_input), 1);
		inst->dynamicHeatmapPtr					=	(float **) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, initParams->dynamicCfarConfig.fft2DSize * sizeof(float *), 1);
		inst->dynamicSideLobeThr				=	initParams->dynamicSideLobeThr;
		inst->dynamicSideLobeThr				=	initParams->dynamicSideLobeThr;
		for (i = 0; i < (int32_t)initParams->dynamicCfarConfig.fft2DSize; i++ )
		{
			inst->dynamicHeatmapPtr[i]			=	(float *) &inst->localHeatmap[i * initParams->dynamicCfarConfig.fft1DSize];
		}

		if ( (inst->detectionCFAROutput == NULL)
			|| (inst->detectionCFAROutput->rangeInd == NULL)
			|| (inst->detectionCFAROutput->dopplerInd == NULL)
			|| (inst->detectionCFAROutput->snrEst == NULL)
			|| (inst->detectionCFAROutput->noise == NULL))
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INOUTALLOC_FAILED;
		}

	}

	inst->staticProcEnabled                         =   initParams->doaConfig.staticEstCfg.staticProcEnabled;
	perRngbinHeatmapLen								=	inst->numDynAngleBin;
	/* static Detection CFAR init and config */
	if (inst->staticProcEnabled)
	{

		inst->numStaticAngleBin						=	(int32_t)(ceil(divsp_i ((float)initParams->doaConfig.numAzimBins, (float)initParams->doaConfig.staticEstCfg.staticAzimStepDeciFactor)) 
														* ceil(divsp_i ((float)initParams->doaConfig.numElevBins, (float)initParams->doaConfig.staticEstCfg.staticElevStepDeciFactor)));

		perRngbinHeatmapLen							=	inst->numStaticAngleBin;
		if ( perRngbinHeatmapLen < inst->numStaticAngleBin )
			perRngbinHeatmapLen						=	inst->numStaticAngleBin;

		initParams->staticCfarConfig.fft2DSize		=	inst->numStaticAngleBin;
		initParams->staticCfarConfig.fft1DSize		=	inst->numRangeBins; 
		initParams->staticCfarConfig.maxNumDetObj	=	MAX_STATIC_CFAR_PNTS;
		initParams->staticCfarConfig.angleDim2		=	(uint32_t)ceil(divsp_i ((float)initParams->doaConfig.numAzimBins, (float)initParams->doaConfig.staticEstCfg.staticAzimStepDeciFactor));	
		initParams->staticCfarConfig.angleDim1		=	(uint32_t)ceil(divsp_i ((float)initParams->doaConfig.numElevBins, (float)initParams->doaConfig.staticEstCfg.staticElevStepDeciFactor));	
		initParams->staticCfarConfig.leftSkipSize	=	inst->cfarRangeSkipLeft;   //set the skip range bins the same as the dynamic scene
		initParams->staticCfarConfig.rightSkipSize	=	inst->cfarRangeSkipRight;  //set the skip range bins the same as the dynamic scene

		inst->staticCFARInstance = (void *) RADARDEMO_detectionCFAR_create(&initParams->staticCfarConfig, &inst->cfarErrorCode);
		if (inst->cfarErrorCode > RADARDEMO_DETECTIONCFAR_NO_ERROR)
		{
			errorCode 	=	PROCESS_ERROR_CFARPROC_INIT_FAILED;
		}

		inst->staticSideLobeThr				=	initParams->staticSideLobeThr;
		inst->staticHeatmapPtr				=	(float **) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL1, 0, initParams->dynamicCfarConfig.fft2DSize * sizeof(float *), 1);
		for (i = 0; i < (int32_t)initParams->staticCfarConfig.fft2DSize; i++ )
		{
			inst->staticHeatmapPtr[i]		=	(float *) &inst->localHeatmap[i * initParams->staticCfarConfig.fft1DSize];
		}

	}
	inst->tempHeatMapOut		=	(float *) radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_LL2, 0, perRngbinHeatmapLen * sizeof(float), 8);

	inst->framePeriod			=	initParams->framePeriod;

	inst->mimoModeFlag  		=  (uint8_t)initParams->mimoModeFlag;
    initParams->heatMapMem		=	inst->localHeatmap;
	
	inst->benchmarkPtr = (radarProcessBenchmarkObj *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, sizeof(radarProcessBenchmarkObj), 1);
	inst->benchmarkPtr->bufferLen = 20;
	inst->benchmarkPtr->bufferIdx = 0;
	inst->benchmarkPtr->buffer    = (radarProcessBenchmarkElem *)radarOsal_memAlloc(RADARMEMOSAL_HEAPTYPE_DDR_CACHED, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem), 1);
#ifdef CAPON2DMODULEDEBUG
	inst->benchmarkPtr->aoaCyclesLog    =   inst->aoaOutput->cyclesLog;
#endif

	memset(inst->benchmarkPtr->buffer, 0, inst->benchmarkPtr->bufferLen * sizeof(radarProcessBenchmarkElem));

	if ((inst->localHeatmap == NULL) 	||(inst->benchmarkPtr == NULL) || (inst->benchmarkPtr->buffer == NULL))
		errorCode 	=	PROCESS_ERROR_INIT_MEMALLOC_FAILED;


	initParams->benchmarkPtr = inst->benchmarkPtr;

	*errCode = errorCode;

#ifndef CCS
	System_printf("DPU_radarProcess_init - process handle: (radarProcessInstance_t *)0x%x\n", (uint32_t)inst);
	System_printf("DPU_radarProcess_init - dynamic CFAR handle: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->dynamicCFARInstance));
	System_printf("DPU_radarProcess_init - staic CFAR handle: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->staticCFARInstance));
	System_printf("DPU_radarProcess_init - 2D capon handle: (RADARDEMO_aoaEst2DCaponBF_handle *)0x%x\n", (uint32_t)(inst->aoaInstance));
	System_printf("DPU_radarProcess_init - benchmark obj: (radarProcessBenchmarkObj *)0x%x\n", (uint32_t)(inst->benchmarkPtr));
	System_printf("DPU_radarProcess_init - heatmap: (float *)0x%x\n", (uint32_t)(inst->localHeatmap));
#else
	printf("DPU_radarProcess_init - process handle: (radarProcessInstance_t *)0x%x\n", (uint32_t)inst);
	printf("DPU_radarProcess_init - dynamic CFAR handle: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->dynamicCFARInstance));
	printf("DPU_radarProcess_init - staic CFAR handle: (RADARDEMO_detectionCFAR_handle *)0x%x\n", (uint32_t)(inst->staticCFARInstance));
	printf("DPU_radarProcess_init - 2D capon handle: (RADARDEMO_aoaEst2DCaponBF_handle *)0x%x\n", (uint32_t)(inst->aoaInstance));
	printf("DPU_radarProcess_init - benchmark obj: (radarProcessBenchmarkObj *)0x%x\n", (uint32_t)(inst->benchmarkPtr));
	printf("DPU_radarProcess_init - heatmap: (float *)0x%x\n", (uint32_t)(inst->localHeatmap));
#endif
	return (void *) inst;
}

/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU config function. Currently not used.
 *
 *  @param[in]  handle                  radarProcess handle.
 *  @param[in]  sampleProcCfg           radarProcess configurations.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_SAMPLEPROC_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */
 
int32_t DPU_radarProcess_config
(
	DPU_radarProcess_Handle     hndle,
    DPU_radarProcessConfig_t	* initParams,
    DPU_ProcessErrorCodes		* errCode
)
{
    int32_t     retVal = 0;

    return (retVal);

}

/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU process function. It runs from the data path 
 *  processing chain, with configuration and buffers assiged at config time. 
 *
 *  @param[in]   handle                  radarProcess handle.
 *  @param[in]   pDataIn                 Input buffer for the processing -- range FFT output in radar cube
 *  @param[out]  pDataOut                3D point cloud data
 *  @param[out]  errCode                 Pointer to errCode generates from the API
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */

int32_t DPU_radarProcess_process (void *handle, cplx16_t * pDataIn, void * pDataOut, int32_t *errCode)
{
    radarProcessInstance_t * processInst = (radarProcessInstance_t *)handle;
#ifdef _TMS320C6X
	int32_t		t1;
#endif
	int32_t		i, cOutNumDectected, numDynamicPnts;
    //RADARDEMO_aoaEst2DCaponBF_errorCode aoaBFErrorCode;
	radarProcessOutput			*resultsPtr =	(radarProcessOutput *) pDataOut;
	radarProcessOutputToTracker	* output =	&resultsPtr->pointCloudOut;
	DPIF_DetMatrix				* heatmap =	&resultsPtr->heatMapOut;
	
	////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////       Calling modules -- Dynamic processing     ////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	{ //range-angle heatmap generation for dynamic scene processing
		processInst->aoaInput->fallBackToConvBFFlag			=	0; //capon only 
		processInst->aoaInput->processingStepSelector		=	0; //BF part
		processInst->aoaInput->clutterRemovalFlag			=	1; //clutter removal always on
		processInst->aoaInput->nChirps						=	processInst->numChirpsPerFrame;
		processInst->aoaInput->lastRB2Process				=	0;
#ifdef _TMS320C6X
		t1 = TSCL;
#endif

#ifdef CAPON2DMODULEDEBUG
		processInst->aoaOutput->cyclesLog->uniqueRngCnt  =   0;
		processInst->aoaOutput->cyclesLog->dopDetCnt     =   0;
		processInst->aoaOutput->cyclesLog->raDetCnt      =   0;
#endif

		for ( i = processInst->cfarRangeSkipLeft; i < processInst->numRangeBins - processInst->cfarRangeSkipRight; i++)
		{
			processInst->aoaInput->rangeIndx				=	i; 
			processInst->aoaInput->inputRangeProcOutSamples =	&pDataIn[i * processInst->nRxAnt * processInst->aoaInput->nChirps];
			processInst->aoaOutput->rangeAzimuthHeatMap		=	processInst->tempHeatMapOut;
			//has to be set to indicate the last rb to process, so that buffer init will be done correctly inside module, only needed for dynamic processing. 
			if ( i == (processInst->numRangeBins - processInst->cfarRangeSkipRight - 1)) 
				processInst->aoaInput->lastRB2Process		=	1;
			processInst->aoaBFErrorCode =     RADARDEMO_aoaEst2DCaponBF_run(
									processInst->aoaInstance,
									processInst->aoaInput,
									processInst->aoaOutput);

			//transpose and store
			copyTranspose((uint32_t *)&processInst->tempHeatMapOut[0], (uint32_t *)&processInst->localHeatmap[i], processInst->numDynAngleBin, 0, processInst->numRangeBins, 1);
		}
#ifdef _TMS320C6X
		processInst->benchmarkPtr->bufferIdx++;
		if (processInst->benchmarkPtr->bufferIdx >= processInst->benchmarkPtr->bufferLen)
			processInst->benchmarkPtr->bufferIdx = 0;
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].dynHeatmpGenCycles 	=	TSCL - t1;
#endif
	}

	{ //Dynamic CFAR

#ifdef _TMS320C6X
		t1 = TSCL;
#endif
		processInst->detectionCFARInput->azMaxPerRangeBin	=	processInst->perRangeBinMax;
		processInst->detectionCFARInput->sidelobeThr		=	processInst->dynamicSideLobeThr;
		processInst->detectionCFARInput->enableSecondPass	=	1;
		processInst->detectionCFARInput->enable_neighbour_check		=	1;
		processInst->detectionCFARInput->heatmapInput		=	processInst->dynamicHeatmapPtr;
		// Detection
		processInst->cfarErrorCode	=	RADARDEMO_detectionCFAR_run(
								processInst->dynamicCFARInstance,
								processInst->detectionCFARInput,
								processInst->detectionCFAROutput);

#ifdef _TMS320C6X
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].dynCfarDetectionCycles 	=	TSCL - t1;
#endif
	}
#ifdef CCS
	printf("CFAR pnt: %d\t", processInst->detectionCFAROutput->numObjDetected);
#endif
	cOutNumDectected		=	0;
	{ // angle and doppler estimation, per detected range-angle points -- for dynamic scene processing
		int32_t angleCount, dopplerCount, detIdx;
		int32_t dopplerIdx;

		processInst->aoaInput->processingStepSelector	=	1; 
#ifdef _TMS320C6X
		t1 = TSCL;
#endif
		processInst->aoaInput->nChirps					=	processInst->numChirpsPerFrame;
		processInst->aoaOutput->rangeAzimuthHeatMap		=	processInst->localHeatmap;
		for (detIdx = 0; detIdx < (int32_t)processInst->detectionCFAROutput->numObjDetected; detIdx++)
		{ 
			if (cOutNumDectected > DOA_OUTPUT_MAXPOINTS) break;
			processInst->aoaInput->rangeIndx				=	processInst->detectionCFAROutput->rangeInd[detIdx];
			processInst->aoaInput->angleIndx				=	processInst->detectionCFAROutput->dopplerInd[detIdx];
			processInst->aoaInput->inputRangeProcOutSamples =	&pDataIn[processInst->aoaInput->rangeIndx * processInst->nRxAnt * processInst->aoaInput->nChirps];
			processInst->aoaInput->noise					=	processInst->detectionCFAROutput->noise[detIdx];

			processInst->aoaBFErrorCode =     RADARDEMO_aoaEst2DCaponBF_run(
									processInst->aoaInstance,
									processInst->aoaInput,
									processInst->aoaOutput);

			dopplerCount						=	0;
			for ( angleCount = 0; angleCount < processInst->aoaOutput->numAngleEst; angleCount++ )
			{
				if (cOutNumDectected > DOA_OUTPUT_MAXPOINTS) break;
				for ( i = 0; i < processInst->aoaOutput->numDopplerIdx; i++ )
				{
					output->pointCloud[cOutNumDectected].range			=	(float)processInst->aoaInput->rangeIndx * processInst->rangeRes;
					output->pointCloud[cOutNumDectected].azimuthAngle	=	processInst->aoaOutput->azimEst[angleCount];
					output->pointCloud[cOutNumDectected].elevAngle		=	processInst->aoaOutput->elevEst[angleCount];
					dopplerIdx											=	(int32_t)processInst->aoaOutput->dopplerIdx[dopplerCount++];
					if ( dopplerIdx > (processInst->DopplerFFTSize >> 1))
						dopplerIdx	-=	processInst->DopplerFFTSize;
					output->pointCloud[cOutNumDectected].velocity		=	(float)dopplerIdx * processInst->dopplerRes;
					if (processInst->aoaOutput->numAngleEst == 1)
						output->snr[cOutNumDectected].snr					=	(int16_t)((float)processInst->detectionCFAROutput->snrEst[detIdx] * 8.f);
					else
						output->snr[cOutNumDectected].snr					=	(int16_t)(divsp_i ((float)processInst->aoaOutput->peakPow[angleCount], processInst->aoaInput->noise) * 8.f);
					cOutNumDectected++;
					if (cOutNumDectected > DOA_OUTPUT_MAXPOINTS) break;
				}
			}
		}
		numDynamicPnts		=	cOutNumDectected;
#ifdef _TMS320C6X
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].dynAngleDopEstCycles 	=	TSCL - t1;
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].dynNumDetPnts 	=	numDynamicPnts;
#endif

	}
#ifdef CCS
	printf("dyn pnt: %d\t", numDynamicPnts);
#endif
	///////////////////       end of Dynamic processing     ////////////////////////////////////


	////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////       Calling modules -- Static processing      ////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////

    if ((processInst->staticProcEnabled) && (cOutNumDectected < DOA_OUTPUT_MAXPOINTS))
	{ //range-angle heatmap generation for Static scene processing

		processInst->aoaInput->processingStepSelector	=	0; //BF part
#ifdef _TMS320C6X
		t1 = TSCL;
#endif
		for ( i = 0; i < processInst->numRangeBins; i++)
		{
			processInst->aoaInput->rangeIndx			= i; 
			processInst->aoaOutput->rangeAzimuthHeatMap = processInst->tempHeatMapOut;
			processInst->aoaBFErrorCode =     RADARDEMO_aoaEst2DCaponBF_static_run(
									processInst->aoaInstance,
									processInst->aoaInput,
									processInst->aoaOutput);
			copyTranspose((uint32_t *)&processInst->tempHeatMapOut[0], (uint32_t *)&processInst->localHeatmap[i], processInst->numStaticAngleBin, 0, processInst->numRangeBins, 1);
		}
#ifdef _TMS320C6X
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].staticHeatmpGenCycles 	=	TSCL - t1;
#endif
	}

    if (processInst->staticProcEnabled)
	{ // test CFAR -- for Static scene processing

#ifdef _TMS320C6X
		t1 = TSCL;
#endif
		processInst->detectionCFARInput->azMaxPerRangeBin	=	processInst->perRangeBinMax;
		processInst->detectionCFARInput->sidelobeThr		=	processInst->staticSideLobeThr;
		processInst->detectionCFARInput->enableSecondPass	=	0;
		processInst->detectionCFARInput->enable_neighbour_check		=	1;
		processInst->detectionCFARInput->heatmapInput		=	processInst->staticHeatmapPtr;
		// Detection
		processInst->cfarErrorCode	=	RADARDEMO_detectionCFAR_run(
								processInst->staticCFARInstance,
								processInst->detectionCFARInput,
								processInst->detectionCFAROutput);

#ifdef _TMS320C6X
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].staticCfarDetectionCycles 	=	TSCL - t1;
#endif
	}

    if (processInst->staticProcEnabled)
	{ // angle interpolation per detected range-angle points -- for Static scene processing
		int32_t detIdx, angleIdx;
		
#ifdef _TMS320C6X
		t1 = TSCL;
#endif
		processInst->aoaInput->processingStepSelector	=	1; 
		processInst->aoaOutput->rangeAzimuthHeatMap = &processInst->localHeatmap[0];
		processInst->aoaInput->nChirps					=	processInst->numChirpsPerFrame;
		processInst->aoaOutput->rangeAzimuthHeatMap		=	processInst->localHeatmap;
		for (detIdx = 0; detIdx < (int32_t)processInst->detectionCFAROutput->numObjDetected; detIdx++)
		{ 
            if (cOutNumDectected > DOA_OUTPUT_MAXPOINTS) break;
			processInst->aoaInput->rangeIndx				=	processInst->detectionCFAROutput->rangeInd[detIdx];
			processInst->aoaInput->angleIndx				=	processInst->detectionCFAROutput->dopplerInd[detIdx];
			processInst->aoaInput->inputRangeProcOutSamples =	&pDataIn[processInst->aoaInput->rangeIndx * processInst->nRxAnt * processInst->aoaInput->nChirps];
			processInst->aoaInput->noise					=	processInst->detectionCFAROutput->noise[detIdx];

			processInst->aoaBFErrorCode =     RADARDEMO_aoaEst2DCaponBF_static_run(
									processInst->aoaInstance,
									processInst->aoaInput,
									processInst->aoaOutput);

			for ( angleIdx = 0; angleIdx < processInst->aoaOutput->numAngleEst; angleIdx++ )
			{
				output->pointCloud[cOutNumDectected].range			=	(float)processInst->aoaInput->rangeIndx * processInst->rangeRes;
				output->pointCloud[cOutNumDectected].azimuthAngle	=	-processInst->aoaOutput->azimEst[angleIdx];
				output->pointCloud[cOutNumDectected].elevAngle		=	processInst->aoaOutput->elevEst[angleIdx];
				output->pointCloud[cOutNumDectected].velocity		=	0.f;
				output->snr[cOutNumDectected].snr					=	(int16_t)(divsp_i ((float)processInst->aoaOutput->peakPow[angleIdx], processInst->aoaInput->noise) * 8.f);
				cOutNumDectected++;
                if (cOutNumDectected > DOA_OUTPUT_MAXPOINTS) break;

			}
		}
#ifdef _TMS320C6X
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].staticAngleEstCycles 	=	TSCL - t1;
		processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx].staticNumDetPnts 		=	cOutNumDectected - numDynamicPnts;
#endif
	}		
#ifdef CCS
	printf("static pnt: %d\t", cOutNumDectected - numDynamicPnts);
#endif

#ifdef _TMS320C6X
    resultsPtr->benchmarkOut    =   &processInst->benchmarkPtr->buffer[processInst->benchmarkPtr->bufferIdx];
#endif

	heatmap->datafmt		=	0;
	heatmap->dataSize		=	processInst->heatMapMemSize;
	heatmap->data			=	(void *)processInst->localHeatmap;

	output->object_count	=	cOutNumDectected;

	*errCode	=	(int32_t)processInst->aoaBFErrorCode;

	return(-processInst->aoaBFErrorCode);
}


/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU control function. Currently not used.
 *
 *  @param[in]  handle                  radarProcess handle.
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */
int32_t DPU_radarProcess_control
(
    DPU_radarProcess_Handle   handle
)
{
    int32_t     retVal = 0;

    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function is radarProcess DPU deinit function. It release resources used for the DPU.
 *
 *  @param[in]  handle                  radarProcess handle.
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *
 *  @retval
 *      Success     =0
 *  @retval
 *      Error       <0
 */
int32_t DPU_radarProcess_deinit
(
    DPU_radarProcess_Handle		handle,
    int32_t						*errCode
)
{
    radarProcessInstance_t * processInst = (radarProcessInstance_t *)handle;

	RADARDEMO_aoaEst2DCaponBF_delete(processInst->aoaInstance);
    RADARDEMO_detectionCFAR_delete(processInst->dynamicCFARInstance);
	if (processInst->staticProcEnabled)
		RADARDEMO_detectionCFAR_delete(processInst->staticCFARInstance);

	//
	radarOsal_memFree(processInst, sizeof(radarProcessInstance_t));

	return 0;
}

