/**
 *   @file  data_path.c
 *
 *   @brief
 *      Implements Data path processing functionality.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/HeapMin.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>
#define DebugP_ASSERT_ENABLED 1
#include <ti/drivers/osal/DebugP.h>
#include <assert.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>

#include "config_edma_util.h"

#include "data_path.h"


void MmwDemo_dataPathHwaDoneIsrCallback(void * arg);

#pragma DATA_SECTION(gMmwHwaMemBuf, ".hwaBufs");
/*! HWA memory buffer to produce the M0,M1,M2,M3 partition addresses
    and link to the HWA section in the linker command file. */
mmwHwaBuf_t gMmwHwaMemBuf[MMW_HWA_NUM_MEM_BUFS];

/**
 *  @b Description
 *  @n
 *      Function to generate a single FFT window sample.
 *
 *  @param[out] win Pointer to output calculated window sample.
 *  @param[in]  winIndx Index of window to generate sample at.
 *  @param[in]  phi Pre-calculated constant by caller as (2*pi/(window length - 1)).
 *  @param[in]  winType Type of window, one of @ref MMW_WIN_BLACKMAN, @ref MMW_WIN_HANNING,
 *              or @ref MMW_WIN_RECT.
 *  @retval none.
 */
static inline MmwDemo_genWindow(uint32_t *win, uint32_t winIndx, float phi, uint32_t winType)
{
    if(winType == MMW_WIN_BLACKMAN)
    {
        //Blackman window
        float a0 = 0.42;
        float a1 = 0.5;
        float a2 = 0.08;
        *win = (uint32_t) ((ONE_Q17 * (a0 - a1*cos(phi * winIndx) +
            a2*cos(2 * phi * winIndx))) + 0.5); //in Q17
        if(*win >= ONE_Q17)
        {
            *win = ONE_Q17 - 1;
        }
    }
    else if(winType == MMW_WIN_HANNING)
    {
        //Hanning window
        *win = (uint32_t) ((ONE_Q17 * 0.5* (1 - cos(phi * winIndx))) + 0.5); //in Q17
        if(*win >= ONE_Q17)
        {
            *win = ONE_Q17 - 1;
        }
    }
    else if(winType == MMW_WIN_RECT)
    {
        //Rectangular window
        *win = (uint32_t) (ONE_Q17/16); //in Q17
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA transfer completion call back function as per EDMA API.
 *      Depending on the programmed transfer completion codes,
 *      posts the corresponding done/completion semaphore.
 *      Per current design, a single semaphore could have been used as the
 *      1D, 2D and CFAR stages are sequential, this code gives some flexibility
 *      if some design change in future.
 */
void MmwDemo_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint32_t transferCompletionCode)
{
    MmwDemo_DataPathObj *obj = (MmwDemo_DataPathObj *)arg;

    switch (transferCompletionCode)
    {
        case MMWDEMO_EDMA_TRANSFER_COMPLETION_CODE_1D_DONE:
            Semaphore_post(obj->EDMA_1Ddone_semHandle);
        break;

        default:
            DebugP_assert(0);
        break;
    }
}

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 */
void MmwDemo_dataPathHwaDoneIsrCallback(void * arg)
{
    Semaphore_Handle       semHandle;

    if (arg != NULL) {
        semHandle = (Semaphore_Handle)arg;
        Semaphore_post(semHandle);
    }
}

/**
 *  @b Description
 *  @n
 *      Common configuration of data path processing, required to be done only
 *      once. Configures all of EDMA and window RAM of HWA.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_dataPathConfigCommon(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;
    uint32_t fftWindow;
    uint32_t winLen, winIndx, hwaRamOffset;
    float phi;
    HWA_CommonConfig hwaCommonConfig;
    uint32_t paramsetIdx;
    uint8_t          hwaTriggerMode;

    /**********************************************/
    /* Disable HWA and reset to known state       */
    /**********************************************/

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0);
    if (errCode != 0)
    {
        System_printf("Error: MmwDemo_dataPathConfigCommon:HWA_enable(0) returned %d\n",errCode);
        return;
    }

    /* Reset the internal state of the HWA */
    errCode = HWA_reset(obj->hwaHandle);
    if (errCode != 0)
    {
        System_printf("Error: MmwDemo_dataPathConfigCommon:HWA_reset returned %d\n",errCode);
        return;
    }

    obj->zoomInFFTSize = 16384;
    obj->log2ZoomInFFTSize = log2Approx(obj->zoomInFFTSize);

    /***********************/
    /* CONFIG WINDOW RAM   */
    /***********************/
    /* if windowing is enabled, load the window coefficients in RAM */
    //1D-FFT window
    hwaRamOffset = 0;
    winLen = obj->numAdcSamples;
    phi = 2 * PI_ / ((float) winLen - 1);
    for(winIndx = 0; winIndx < winLen; winIndx++)
    {
        MmwDemo_genWindow(&fftWindow, winIndx, phi, MMW_WIN_BLACKMAN);
        errCode = HWA_configRam(obj->hwaHandle,
                                HWA_RAM_TYPE_WINDOW_RAM,
                                (uint8_t *)&fftWindow,
                                sizeof(uint32_t),   //size in bytes
                                hwaRamOffset); //offset in bytes
        if (errCode != 0)
        {
            System_printf("Error: HWA_configRam returned %d\n",errCode);
            return;
        }
        hwaRamOffset += sizeof(uint32_t);
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    errCode = HWA_enableDoneInterrupt(obj->hwaHandle,
        MmwDemo_dataPathHwaDoneIsrCallback,
        obj->HWA_done_semHandle);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enableDoneInterrupt returned %d\n",errCode);
        return;
    }

    /***********************************************/
    /* ENABLE FFT Twiddle coefficient dithering    */
    /***********************************************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                                 HWA_COMMONCONFIG_MASK_LFSRSEED;
    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/

    errCode = HWA_configCommon(obj->hwaHandle,&hwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }

    /*set up common parameters for range FFT and interpolation DFT */
    paramsetIdx     =   MMW_HWA_START_POS_PARAMSETS_1D;
    memset((void*)&obj->rangeHwaParamCfg[paramsetIdx],0,sizeof(HWA_ParamConfig));

    /***********************/
    /* PING DUMMY PARAMSET */
    /***********************/
    obj->rangeHwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; //Software triggered  - in demo this will be HWA_TRIG_MODE_DMA
    obj->rangeHwaParamCfg[paramsetIdx].dmaTriggerSrc = MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING; //in demo this will be first EDMA Src channel id
    obj->rangeHwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE; //dummy
    /***********************/
    /* PING PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    memset((void*)&obj->rangeHwaParamCfg[paramsetIdx],0,sizeof(HWA_ParamConfig));
    if(obj->dataPathMode == DATA_PATH_STANDALONE)
    {
        /* trigger manually and immediately */
        hwaTriggerMode = HWA_TRIG_MODE_SOFTWARE;
    }
    else
    {
        /* trigger done by ADC buffer */
        hwaTriggerMode = HWA_TRIG_MODE_DFE;
    }
    obj->rangeHwaParamCfg[paramsetIdx].triggerMode = hwaTriggerMode;
    obj->rangeHwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; //do FFT
    obj->rangeHwaParamCfg[paramsetIdx].source.srcAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_ADCBUF_INP); // address is relative to start of MEM0
    obj->rangeHwaParamCfg[paramsetIdx].source.srcAcnt = obj->numAdcSamples - 1; //this is samples - 1
    obj->rangeHwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t); // 16 bytes
    obj->rangeHwaParamCfg[paramsetIdx].source.srcBcnt = 0; //no iterations here
    obj->rangeHwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t); //should be dont care
    obj->rangeHwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    obj->rangeHwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; //no shift
    obj->rangeHwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    obj->rangeHwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    obj->rangeHwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    obj->rangeHwaParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    obj->rangeHwaParamCfg[paramsetIdx].source.srcScale = 4;
    obj->rangeHwaParamCfg[paramsetIdx].source.bpmEnable = 0; //bpm removal not enabled
    obj->rangeHwaParamCfg[paramsetIdx].source.bpmPhase = 0; //dont care
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PING); // address is relative to start of MEM0
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstAcnt = obj->numRangeBins-1; //this is samples - 1
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t); //
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t); //should be dont care
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL; //same as input - complex
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstScale = 0;
    obj->rangeHwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = obj->log2RangeBins;
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant - revisit this for all FFT size and data size
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; //enabled
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; //start of window RAM
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = 0; //non-symmetric - in demo do we make this symmetric
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED; //disabled
    obj->rangeHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; // output FFT samples
    obj->rangeHwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* Config Common Registers */
    obj->rangeHhwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
    obj->rangeHhwaCommonConfig.numLoops = 1;
    obj->rangeHhwaCommonConfig.paramStartIdx = MMW_HWA_START_POS_PARAMSETS_1D;
    obj->rangeHhwaCommonConfig.paramStopIdx = MMW_HWA_START_POS_PARAMSETS_1D + HWAUTIL_NUM_PARAM_SETS_1D - 1;
    if(obj->dataPathMode == DATA_PATH_STANDALONE)
    {
        /* HWA will input data from M0 memory*/
        obj->rangeHhwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    }
    else
    {
        /* HWA will input data from ADC buffer memory*/
        obj->rangeHhwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    }
    obj->rangeHhwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    paramsetIdx = 0;
    memset( (void*) &obj->peakHwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    obj->peakHwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    obj->peakHwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    obj->peakHwaParamCfg[paramsetIdx].source.srcAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_OUT_PING);
    obj->peakHwaParamCfg[paramsetIdx].source.srcAcnt = obj->numAdcSamples - 1;
    obj->peakHwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t);
    obj->peakHwaParamCfg[paramsetIdx].source.srcBcnt = 0; //no iterations here
    obj->peakHwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t); //should be dont care
    obj->peakHwaParamCfg[paramsetIdx].source.srcShift = 0; //no shift
    obj->peakHwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;
    obj->peakHwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL;
    obj->peakHwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    obj->peakHwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; //signed
    obj->peakHwaParamCfg[paramsetIdx].source.srcConjugate = 0;
    obj->peakHwaParamCfg[paramsetIdx].source.srcScale = 4;
    obj->peakHwaParamCfg[paramsetIdx].source.bpmEnable = 0;
    obj->peakHwaParamCfg[paramsetIdx].source.bpmPhase = 0;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_INTERP_OUT);
    obj->peakHwaParamCfg[paramsetIdx].dest.dstAcnt = obj->numRangeBins-1;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t);
    obj->peakHwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t);
    obj->peakHwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstConjugate = 0;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstScale = 0;
    obj->peakHwaParamCfg[paramsetIdx].dest.dstSkipInit = 0;
    obj->peakHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
    obj->peakHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = obj->log2RangeBins;
    obj->peakHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;
    obj->peakHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_MAX_STATS;
    obj->peakHwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* Config Common Registers */
    obj->peakHwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;
    obj->peakHwaCommonConfig.numLoops = 1;
    obj->peakHwaCommonConfig.paramStartIdx = MMW_HWA_START_POS_PARAMSETS_PEAK;
    obj->peakHwaCommonConfig.paramStopIdx =  MMW_HWA_START_POS_PARAMSETS_PEAK + HWAUTIL_NUM_PARAM_SETS_PEAK - 1;
    obj->peakHwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    obj->peakHwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    paramsetIdx = 0;
    memset( (void*) &obj->interpHwaParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    obj->interpHwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    obj->interpHwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    obj->interpHwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_SLOW_DFT;
    obj->interpHwaParamCfg[paramsetIdx].complexMultiply.cmpMulArgs.dft.freqIncrement = obj->log2ZoomInFFTSize;
    obj->interpHwaParamCfg[paramsetIdx].complexMultiply.cmpMulArgs.dft.startFreq = 0; //will need to overwrite later
    obj->interpHwaParamCfg[paramsetIdx].source.srcAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_1D_ADCBUF_INP); // MEM0, this should still hold the ADC samples.
    obj->interpHwaParamCfg[paramsetIdx].source.srcAcnt = obj->numRangeBins-1;
    obj->interpHwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    obj->interpHwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
    obj->interpHwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
    obj->interpHwaParamCfg[paramsetIdx].source.srcBIdx = 0;       //repeat from sample 0 of srcAddr for every iteration
    obj->interpHwaParamCfg[paramsetIdx].source.srcBcnt = 2 * SAMPLES_TO_ZOOM_IN_ONE_SIDE * (obj->zoomInFFTSize >> obj->log2RangeBins) - 1; //iterate for 2 * interpFactor times
    obj->interpHwaParamCfg[paramsetIdx].source.srcScale = 8;
    obj->interpHwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; //signed
    obj->interpHwaParamCfg[paramsetIdx].dest.dstAddr = ADDR_TRANSLATE_CPU_TO_HWA(MMW_HWA_INTERP_OUT);  //MEM3
    obj->interpHwaParamCfg[paramsetIdx].dest.dstAcnt = 0;// one DFT output sample at a time
    obj->interpHwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    obj->interpHwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_32BIT;
    obj->interpHwaParamCfg[paramsetIdx].dest.dstAIdx = 2 * sizeof(uint32_t);
    obj->interpHwaParamCfg[paramsetIdx].dest.dstBIdx = 2 * sizeof(uint32_t);
    obj->interpHwaParamCfg[paramsetIdx].dest.dstScale = 0;
    obj->interpHwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
    obj->interpHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = HWA_FEATURE_BIT_DISABLE;
    obj->interpHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = obj->log2ZoomInFFTSize;
    obj->interpHwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_SUM_STATS;

    /* Config Common Registers */
    obj->interpHhwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX|
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD;


    obj->interpHhwaCommonConfig.numLoops = 1;
    obj->interpHhwaCommonConfig.paramStartIdx = MMW_HWA_START_POS_PARAMSETS_INTERP;
    obj->interpHhwaCommonConfig.paramStopIdx =  MMW_HWA_START_POS_PARAMSETS_INTERP + HWAUTIL_NUM_PARAM_SETS_INTERP - 1;
    obj->interpHhwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    obj->interpHhwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

}

/**
 *  @b Description
 *  @n
 *      Init HWA.
 */
void MmwDemo_hwaInit(MmwDemo_DataPathObj *obj)
{
    /* Initialize the HWA */
    HWA_init();

    /* Debug Message: */
    System_printf ("Debug: HWA has been initialized\n");
}

/**
 *  @b Description
 *  @n
 *      Init EDMA.
 */
void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj)
{
    uint8_t edmaNumInstances, inst;
    Semaphore_Params       semParams;
    int32_t errorCode;

    edmaNumInstances = EDMA_getNumInstances();
    for (inst = 0; inst < edmaNumInstances; inst++)
    {
        errorCode = EDMA_init(inst);
        if (errorCode != EDMA_NO_ERROR)
        {
            System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
            return;
        }
        System_printf ("Debug: EDMA instance %d has been initialized\n", inst);
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));

    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->EDMA_1Ddone_semHandle = Semaphore_create(0, &semParams, NULL);

    /* Enable Done Interrupt */
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    obj->HWA_done_semHandle = Semaphore_create(0, &semParams, NULL);

}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    DebugP_assert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo)
{
    DebugP_assert(0);
}

/**
 *  @b Description
 *  @n
 *      Open HWA instance.
 */
void MmwDemo_hwaOpen(MmwDemo_DataPathObj *obj, SOC_Handle socHandle)
{
    int32_t             errCode;

    /* Open the HWA Instance */
    obj->hwaHandle = HWA_open(0, socHandle, &errCode);
    if (obj->hwaHandle == NULL)
    {
        System_printf("Error: Unable to open the HWA Instance err:%d\n",errCode);
        return;
    }

    System_printf("Debug: HWA Instance %p has been opened successfully\n", obj->hwaHandle);
}

/**
 *  @b Description
 *  @n
 *      Close HWA instance.
 */
void MmwDemo_hwaClose(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;

    /* Close the HWA Instance */
    errCode = HWA_close(obj->hwaHandle);
    if (errCode != 0)
    {
        System_printf("Error: Unable to close the HWA Instance err:%d\n",errCode);
        return;
    }

    System_printf("Debug: HWA Instance %p has been closed successfully\n", obj->hwaHandle);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA.
 */
void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;
    EDMA_instanceInfo_t edmaInstanceInfo;
    EDMA_errorConfig_t  errorConfig;

    obj->edmaHandle = EDMA_open(0, &errCode, &edmaInstanceInfo);

    if (obj->edmaHandle == NULL)
    {
        System_printf("Error: Unable to open the EDMA Instance err:%d\n",errCode);
        return;
    }
    System_printf("Debug: EDMA Instance %p has been opened successfully\n", obj->edmaHandle);

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = MmwDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MmwDemo_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        System_printf("Debug: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        return;
    }

}

/**
 *  @b Description
 *  @n
 *      Close EDMA.
 */
void MmwDemo_edmaClose(MmwDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
    System_printf("Debug: EDMA Instance %p has been closed successfully\n", obj->edmaHandle);
}


/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool structure.
 *
 *  @retval
 *      none.
 */
void MmwDemo_memPoolReset(MmwDemoMemPool_t *pool)
{
    pool->indx = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @pre It is assumed that no allocation from l3 will need better than 32-bit
 *      alignment (structures with 64-bit prohibited) and so for simplicity,
 *      size is assumed to be multiple of 4 bytes
 *
 *  @param[in]  pool Handle to pool structure.
 *  @param[in]  size Size in bytes to be allocated.
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could notice
 *      allocate.
 */
uint8_t *MmwDemo_memPoolAlloc(MmwDemoMemPool_t *pool, uint32_t size)
{
    uint8_t *ptr = NULL;

    if ((size % 4) != 0)
    {
        return(ptr);
    }

    if ((pool->indx + size) <= pool->size)
    {
        ptr = pool->base + pool->indx;
        pool->indx += size;
    }

    return(ptr);
}


/**
 *  @b Description
 *  @n
 *      Configure HWA for 1D processing.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_config1D_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t                 errCode;
    uint32_t paramsetIdx = MMW_HWA_START_POS_PARAMSETS_1D;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }
    errCode = HWA_configParamSet(obj->hwaHandle, paramsetIdx,&obj->rangeHwaParamCfg[0],NULL);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramsetIdx, errCode);
        return;
    }

    /***********************/
    /* PING PROCESS PARAMSET */
    /***********************/
    paramsetIdx++;
    errCode = HWA_configParamSet(obj->hwaHandle, paramsetIdx,&obj->rangeHwaParamCfg[1],NULL);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configParamSet(%d) returned %d\n",errCode,paramsetIdx);
        return;
    }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    errCode = HWA_configCommon(obj->hwaHandle,&obj->rangeHhwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for 1D processing.
 *  @param[in]  obj Pointer to data path obj.
 *  @retval none.
 */
void MmwDemo_dataPathTrigger1D(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }

    /* trigger the HWA since triggerMode is set to DMA */
    errCode = HWA_setDMA2ACCManualTrig(obj->hwaHandle, MMW_HWA_DMA_TRIGGER_SOURCE_1D_PING);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_setDMA2ACCManualTrig(0) returned %d\n",errCode);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Waits for 1D processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWait1D(MmwDemo_DataPathObj *obj)
{
    Bool       status;
//    Semaphore_Handle       paramSetSem;

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: Semaphore_pend returned %d\n",status);
    }

}
/**
 *  @b Description
 *  @n
 *      Configures HWA for peak search processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_configPeakSearch_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t                 errCode;
    uint32_t paramsetIdx = MMW_HWA_START_POS_PARAMSETS_PEAK;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }

    errCode = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &obj->peakHwaParamCfg[0], NULL);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramsetIdx, errCode);
        return;
    }

    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    errCode = HWA_configCommon(obj->hwaHandle,&obj->peakHwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }
}
/**
 *  @b Description
 *  @n
 *      Configures HWA for interpolation processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_configInterp_HWA(MmwDemo_DataPathObj *obj)
{
    int32_t                 errCode;
    uint32_t paramsetIdx = MMW_HWA_START_POS_PARAMSETS_INTERP;

    /* Disable the HWA */
    errCode = HWA_enable(obj->hwaHandle,0); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }

    /**************************************/
    /* Interpolation PROCESSING                    */
    /**************************************/
    if (obj->rangeProcStats.maxIndex < SAMPLES_TO_ZOOM_IN_ONE_SIDE)
    {
        obj->interpHwaParamCfg[0].complexMultiply.cmpMulArgs.dft.startFreq = 0;
    }
    else if(obj->rangeProcStats.maxIndex > (obj->numRangeBins-SAMPLES_TO_ZOOM_IN_ONE_SIDE))
    {
        obj->interpHwaParamCfg[0].complexMultiply.cmpMulArgs.dft.startFreq = (obj->numRangeBins - 2 * SAMPLES_TO_ZOOM_IN_ONE_SIDE) * (obj->zoomInFFTSize >> (obj->log2RangeBins));
    }
    else
    {
        obj->interpHwaParamCfg[0].complexMultiply.cmpMulArgs.dft.startFreq = (obj->rangeProcStats.maxIndex - SAMPLES_TO_ZOOM_IN_ONE_SIDE) * (obj->zoomInFFTSize >> (obj->log2RangeBins));
    }
    errCode = HWA_configParamSet(obj->hwaHandle, paramsetIdx, &obj->interpHwaParamCfg[0], NULL);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configParamSet(%d) returned %d\n", paramsetIdx, errCode);
        return;
    }



    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    errCode = HWA_configCommon(obj->hwaHandle,&obj->interpHhwaCommonConfig);
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_configCommon returned %d\n",errCode);
        return;
    }
}
extern highAccuRangeProc_config     gHighAccuConfig;
void MmwDemo_rangeLimit(MmwDemo_DataPathObj *obj)
{
    uint32_t i;
    uint16_t RangeBinfft1DSize,RangeBinMin,RangeBinMax;
    int16_t *fftOut1D = (int16_t *) MMW_HWA_1D_OUT_PING; //M2 data for 1Dfftoutput

    RangeBinfft1DSize                    = obj->numRangeBins;
    RangeBinMin                         = (uint16_t)(gHighAccuConfig.skipMin/obj->rangeResolution);
    RangeBinMax                        = (uint16_t)(gHighAccuConfig.skipMax/obj->rangeResolution);


    if(RangeBinMin > RangeBinfft1DSize)
        RangeBinMin = 0;
    if(RangeBinMax > RangeBinfft1DSize)
        RangeBinMax = RangeBinfft1DSize;

    for( i = 0; i < (int32_t)RangeBinMin;  i++ )
    {
        fftOut1D[ i ] = 0;
    }
    for( i = (int32_t)RangeBinMax; i < (int32_t)RangeBinfft1DSize; i++ )
    {
        fftOut1D[ i ] = 0;
    }

}
/**
 *  @b Description
 *  @n
 *    Compensation of DC range antenna signature
 *
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_dcRangeSignatureCompensation(MmwDemo_DataPathObj *obj)
{
}


/**
 *  @b Description
 *  @n
 *      Peak search using statistics block.
 */
void MmwDemo_peakSearch(MmwDemo_DataPathObj *obj)
{
    MmwDemo_configPeakSearch_HWA(obj);
    MmwDemo_dataPathTriggerPeakSearch(obj);
    MmwDemo_dataPathWaitPeakSearch(obj);
}
/**
 *  @b Description
 *  @n
 *      Trigger Interpolation processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathTriggerPeakSearch(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }
}
/**
 *  @b Description
 *  @n
 *      Trigger Interpolation processing.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathTriggerInterp(MmwDemo_DataPathObj *obj)
{
    int32_t errCode;

    /* Enable the HWA */
    errCode = HWA_enable(obj->hwaHandle,1); // set 1 to enable
    if (errCode != 0)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: HWA_enable(1) returned %d\n",errCode);
        return;
    }
}
/**
 *  @b Description
 *  @n
 *      Waits for peak search processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitPeakSearch(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    /* then wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: Semaphore_pend returned %d\n",status);
        return;
    }
}
/**
 *  @b Description
 *  @n
 *      Waits for interpolation processing to finish. This is a blocking function.
 *
 *  @param[in] obj  Pointer to data path object
 *
 *  @retval
 *      NONE
 */
void MmwDemo_dataPathWaitInterp(MmwDemo_DataPathObj *obj)
{
    Bool       status;

    /* then wait for the all paramSets done interrupt */
    status = Semaphore_pend(obj->HWA_done_semHandle, BIOS_WAIT_FOREVER);
    if (status != TRUE)
    {
        //retCode = HWA_TEST_ERROR;
        System_printf("Error: Semaphore_pend returned %d\n",status);
        return;
    }
}


/**
 *  @b Description
 *  @n
 *      Interpolation using complex multiplication module.
 */
void MmwDemo_processInterpolation(MmwDemo_DataPathObj *obj)
{
    int32_t i, max_ind, interp_factor, coarseIndStart;
    cmplx32ImRe_t * interpOutsAddr;
    float power, maxP;

    float fpower[3], interpIndx, maxIndexFine, fineFreqEst;


    interp_factor   =   (obj->zoomInFFTSize >> (obj->log2RangeBins));
    interpOutsAddr  =   (cmplx32ImRe_t *) MMW_HWA_INTERP_OUT;

    MmwDemo_configInterp_HWA(obj);
    MmwDemo_dataPathTriggerInterp(obj);
    MmwDemo_dataPathWaitInterp(obj);

    if (obj->rangeProcStats.maxIndex < SAMPLES_TO_ZOOM_IN_ONE_SIDE)
    {
        coarseIndStart = 0;
    }
    else if (obj->rangeProcStats.maxIndex > (obj->numRangeBins - SAMPLES_TO_ZOOM_IN_ONE_SIDE))
    {
        coarseIndStart  =   obj->numRangeBins - 2 * SAMPLES_TO_ZOOM_IN_ONE_SIDE;
    }
    else
    {
        coarseIndStart  =   obj->rangeProcStats.maxIndex  - SAMPLES_TO_ZOOM_IN_ONE_SIDE;
    }

    maxP    =   0.f;
    for (i = 0; i < 2 * SAMPLES_TO_ZOOM_IN_ONE_SIDE * interp_factor; i++)
    {
        power   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
        if (power > maxP)
        {
            maxP    =   power;
            max_ind =   i;
        }
    }
    obj->finePeakIndex = max_ind;

    i           =   max_ind - 1;
    fpower[0]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
    i           =   max_ind;
    fpower[1]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;
    i           =   max_ind + 1;
    fpower[2]   =   (float)interpOutsAddr[i].imag * (float)interpOutsAddr[i].imag + (float)interpOutsAddr[i].real * (float)interpOutsAddr[i].real;

    interpIndx  =   0.5f * (fpower[0] - fpower[2])/(fpower[0] + fpower[2] -2.f * fpower[1]);

    obj->interpIndex = interpIndx;
    maxIndexFine    =   (float)(interp_factor * coarseIndStart + max_ind) + interpIndx;
    fineFreqEst     =   maxIndexFine * obj->maxBeatFreq/(interp_factor * obj->numRangeBins);
    obj->rangeEst   =   (fineFreqEst * 3.0e8 * obj->chirpRampTime)/(2 * obj->chirpBandwidth);
}


/**
 *  @b Description
 *  @n
 *      Power of 2 round up function.
 */
uint32_t MmwDemo_pow2roundup (uint32_t x)
{
    uint32_t result = 1;
    while(x > result)
    {
        result <<= 1;
    }
    return result;
}

/**
 *  @b Description
 *  @n
 *      Delete semaphores.
 */
void MmwDemo_dataPathDeleteSemaphore(MmwDemo_DataPathObj *obj)
{
    Semaphore_delete(&obj->EDMA_1Ddone_semHandle);
    Semaphore_delete(&obj->HWA_done_semHandle);
}

uint32_t log2Approx(uint32_t x)
{
    uint32_t idx, detectFlag = 0;

    if ( x < 2)
    {
        return (0);
    }

    idx = 32U;
    while((detectFlag==0U) || (idx==0U))
    {
        if(x & 0x80000000U)
        {
            detectFlag = 1;
        }
        x <<= 1U;
        idx--;
    }

    if(x != 0)
    {
        idx = idx + 1;
    }

    return(idx);
}
