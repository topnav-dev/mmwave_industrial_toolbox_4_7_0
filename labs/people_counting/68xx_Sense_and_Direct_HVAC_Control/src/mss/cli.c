/*
 *   @file  cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
 *
 *  Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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

/* BIOS/XDC Include Files. */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/IHeap.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Memory.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/heaps/HeapBuf.h>
#include <ti/sysbios/heaps/HeapMem.h>
#include <ti/sysbios/knl/Event.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>

#if (defined (GTRACK_2D)) || (defined (GTRACK_3D))
#include <gtrack.h>
#else
#include <gtrack.h>
#endif

/* Demo Include Files */
#include "mss_mmw.h"
#include <common/mmw_messages.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIDoACfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIFrameStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIClassifierCfg (int32_t argc, char* argv[]);

/* Tracking configuration is done by application */
extern int32_t MmwDemo_CLITrackingCfg (int32_t argc, char* argv[]);
/* SceneryParam configuration is done by application */
extern int32_t MmwDemo_CLISceneryParamCfg (int32_t argc, char* argv[]);
/* GatingParam configuration is done by application */
extern int32_t MmwDemo_CLIGatingParamCfg (int32_t argc, char* argv[]);
/* StateParam configuration is done by application */
extern int32_t MmwDemo_CLIStateParamCfg (int32_t argc, char* argv[]);
/* AllocationParam configuration is done by application */
extern int32_t MmwDemo_CLIAllocationParamCfg (int32_t argc, char* argv[]);
/* Filter parameter configuration */
extern int32_t MmwDemo_FilterCfg (int32_t argc, char* argv[]);

/* Live config commands:
* CFAR - only thresholds
* DOA - only 1 param for now
* Allocation
* Gating
* State
*/
 //extern int32_t LiveCFG_CFAR (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Allocation (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Gating (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_State (int32_t argc, char* argv[]);
 extern int32_t LiveCFG_Scenery (int32_t argc, char* argv[]);


/**************************************************************************
 *************************** External Definitions *************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMssMCB;
extern int32_t MmwDemo_mboxWrite(MmwDemo_message     * message);
extern void MmwDemo_printHeapStats(void);

/**************************************************************************
 *************************** CLI  Function Definitions **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliSensorStartEvt ++;

    /* Get the configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionConfig (&gMmwMssMCB.cfg.ctrlCfg);

    /* Get the open configuration from the CLI mmWave Extension */
    CLI_getMMWaveExtensionOpenConfig (&gMmwMssMCB.cfg.openCfg);

    /* Post sensorSTart event to notify configuration is done */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTART_EVT);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the frame start command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIFrameStart (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliFrameStartEvt ++;

    /* Post sensorSTart event to notify configuration is done */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_FRAMESTART_EVT);
    return 0;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the sensor stop command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[])
{
    gMmwMssMCB.stats.cliSensorStopEvt ++;

    /* Post sensorSTOP event to notify sensor stop command */
    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_CLI_SENSORSTOP_EVT);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for gui monitoring configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[])
{
    MmwDemo_GuiMonSel   guiMonSel;
    MmwDemo_message     message;
	int32_t             pcSize;

    /* Sanity Check: Minimum argument check */
    if (argc < 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[1]);
    guiMonSel.trackerOutput               = atoi (argv[2]);
    guiMonSel.classificationOutput       = atoi (argv[3]);
	pcSize								=	500;
	if (argc == 6)
		pcSize       = atoi (argv[5]);
	if (atoi (argv[4]) > 0)
		guiMonSel.reducePointCloudOutputSize = pcSize;
	else
		guiMonSel.reducePointCloudOutputSize = 0;

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));
    
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_GUIMON_CFG;
    memcpy((void *)&message.body.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for CFAR configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICfarCfg (int32_t argc, char* argv[])
{
	mmwDemoCfarConfig     cfarCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc < 13)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&cfarCfg, 0, sizeof(mmwDemoCfarConfig));

    //System_printf("CFAR config\n");

    //cliCfg.tableEntry[4].helpString     = "<detMode> <discardLeft> <discardRight> <refWinSize1> <refWinSize2> <guardWinSize1> <guardWinSize2> <thre>";
    /* Populate configuration: */
    cfarCfg.cfarMethod       = (uint16_t) atoi (argv[1]);
    cfarCfg.cfarDiscardRangeLeft  = (uint16_t) atoi (argv[2]);
    cfarCfg.cfarDiscardRangeRight = (uint16_t) atoi (argv[3]);
    cfarCfg.cfarDiscardAngleLeft  = (uint16_t) atoi (argv[4]);
    cfarCfg.cfarDiscardAngleRight = (uint16_t) atoi (argv[5]);
    cfarCfg.refWinSize[0]    = (uint16_t) atoi (argv[6]);
    cfarCfg.refWinSize[1]    = (uint16_t) atoi (argv[7]);
    cfarCfg.guardWinSize[0]  = (uint16_t) atoi (argv[8]);
    cfarCfg.guardWinSize[1]  = (uint16_t) atoi (argv[9]);
    cfarCfg.rangeThre             = (float) atoi (argv[10]) * 0.1f;
    cfarCfg.azimuthThre     = (float) atoi (argv[11]) * 0.1f;
    cfarCfg.log2MagFlag        = (uint16_t) atoi (argv[12]);
	if (argc == 14)
	{
		cfarCfg.inputScaleDown = (uint16_t) atoi (argv[13]);
	}
	else
	{
		#ifdef SOC_XWR68XX
		cfarCfg.inputScaleDown = 3;
		#else
		cfarCfg.inputScaleDown = 0;
		#endif
	}
    //System_printf("CFAR config:method = %d\n", cfarCfg.cfarMethod);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.cfarCfg, (void *)&cfarCfg, sizeof(mmwDemoCfarConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CFAR_CFG;
    memcpy((void *)&message.body.cfar, (void *)&cfarCfg, sizeof(mmwDemoCfarConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DOA configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIDoACfg (int32_t argc, char* argv[])
{
	mmwDemoDoaConfig     doaCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc < 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&doaCfg, 0, sizeof(mmwDemoDoaConfig));

    //cliCfg.tableEntry[7].helpString     = "<doaMode> <doaGamma> <sideLobe_dB> <searchRange> <searchRes> <varThre>";
    /* Populate configuration: */
    doaCfg.estAngleRange       	= (float) atoi (argv[1]) * 0.1f;
    doaCfg.estAngleResolution      = (float) atoi (argv[2]) * 0.001f;
    doaCfg.gamma            = (float) atoi (argv[3]) * 0.001f;
    doaCfg.clutterRemovalFlag = (uint8_t) atoi (argv[4]);
	
	
	#ifdef SOC_XWR68XX
    doaCfg.dopplerOversampleFactor = 1;
    doaCfg.useCFAR4DopDet = 1;
	doaCfg.dopCfarThr = 30;
	doaCfg.dopCfarGuardLen = 4;
	doaCfg.scaleDopCfarOutCFAR = 2;
	#else
    doaCfg.dopplerOversampleFactor = 1;
    doaCfg.useCFAR4DopDet = 0;
	doaCfg.dopCfarThr = 30;
	doaCfg.dopCfarGuardLen = 4;
	doaCfg.scaleDopCfarOutCFAR = 0;
	#endif
	
	if (argc == 10)
	{
		doaCfg.scaleDopCfarOutCFAR = (uint8_t) atoi (argv[9]);
		doaCfg.dopCfarGuardLen = (uint8_t) atoi (argv[8]);
		doaCfg.dopCfarThr = (float) atoi (argv[7]) * 0.1f;
		doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
		doaCfg.useCFAR4DopDet = (uint8_t) atoi (argv[6]);
	}
	if (argc == 9)
	{
		doaCfg.dopCfarGuardLen = (uint8_t) atoi (argv[8]);
		doaCfg.dopCfarThr = (float) atoi (argv[7]) * 0.1f;
		doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
		doaCfg.useCFAR4DopDet = (uint8_t) atoi (argv[6]);
	}
	if (argc == 8)
	{
		doaCfg.dopCfarThr = (float) atoi (argv[7]) * 0.1f;
		doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
		doaCfg.useCFAR4DopDet = (uint8_t) atoi (argv[6]);
	}
	if (argc == 7)
	{
		doaCfg.useCFAR4DopDet = (uint8_t) atoi (argv[6]);
		doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
	}
	if (argc == 6)
	{
		doaCfg.dopplerOversampleFactor = (uint8_t) atoi (argv[5]);
	}

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.doaCfg, (void *)&doaCfg, sizeof(mmwDemoDoaConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_DOA_CFG;
    memcpy((void *)&message.body.doa, (void *)&doaCfg, sizeof(mmwDemoDoaConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for Classifier configuration
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIClassifierCfg (int32_t argc, char* argv[])
{
	mmwDemoClassifierConfig     classifierCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc < 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&classifierCfg, 0, sizeof(mmwDemoClassifierConfig));

    //mandatory field: cliCfg.tableEntry[].helpString     = "<enableFlag> <ClassifierSelector> <k> <CBLen>";
    /* Populate configuration: */

    gMmwMssMCB.mssDataPathObj.classifierPlatformSelector       	= (uint8_t) atoi (argv[1]);
    classifierCfg.classifierSelector      = (uint8_t) atoi (argv[2]);
    classifierCfg.k            = (uint8_t) atoi (argv[3]) ;
    classifierCfg.cbLen = (uint16_t) atoi (argv[4]);

    /* other optional/default config */
    if (gMmwMssMCB.mssDataPathObj.groupTrackerEnabled == 0)
    {
    	gMmwMssMCB.mssDataPathObj.classifierPlatformSelector	=	0;
    }
    classifierCfg.maxNumTracks		=	gMmwMssMCB.cfg.trackingCfg.config.maxNumTracks;
    classifierCfg.blockLen			=	30;
    classifierCfg.minNpntsPObj		=	10;
    classifierCfg.slideWinLen		=	5;
    classifierCfg.outBufLen			=	2;
    classifierCfg.gamma				=	0.6f;
    classifierCfg.gamma1			=	0.95f;
    classifierCfg.maxDisPosThr		=	3.f;
    classifierCfg.neighborDistSqrThr	=	1.f;
    classifierCfg.histRangeBinSize	=	0.784f;
    classifierCfg.histNumRangeBin	=	16;
    classifierCfg.histDopplerBinSize	=	0.33f;
    classifierCfg.histNumDopplerBin	=	32;

    switch (argc) // x at the location of the parameter means take the default
    {
    	case 17: //histNumDopplerBin
    		if (strcmp(argv[16], "x"))
    			classifierCfg.histNumDopplerBin = (uint16_t) atoi (argv[16]);
    	case 16:
    		if (strcmp(argv[15], "x"))
    			classifierCfg.histDopplerBinSize = (float) atof (argv[15]);
    	case 15:
    		if (strcmp(argv[14], "x"))
    			classifierCfg.histNumRangeBin = (uint16_t) atoi (argv[14]);
    	case 14:
    		if (strcmp(argv[13], "x"))
    			classifierCfg.histRangeBinSize = (float) atof (argv[13]);
    	case 13:
    		if (strcmp(argv[12], "x"))
				classifierCfg.maxDisPosThr = (float) atof (argv[12]);
    	case 12:
    		if (strcmp(argv[11], "x"))
				classifierCfg.outBufLen = (uint8_t) atoi (argv[11]);
    	case 11:
    		if (strcmp(argv[10], "x"))
				classifierCfg.slideWinLen = (uint32_t) atoi (argv[10]);
    	case 10:
    		if (strcmp(argv[9], "x"))
				classifierCfg.blockLen = (uint32_t) atoi (argv[9]);
    	case 9:
    		if (strcmp(argv[8], "x"))
				classifierCfg.minNpntsPObj = (uint8_t) atoi (argv[8]);
    	case 8:
    		if (strcmp(argv[7], "x"))
				classifierCfg.gamma1 = (float) atof (argv[7]);
    	case 7:
    		if (strcmp(argv[6], "x"))
				classifierCfg.neighborDistSqrThr = (float) atof (argv[6]);
    	case 6:
    		if (strcmp(argv[5], "x"))
    			classifierCfg.gamma = (float) atof (argv[5]);
    		break;
    }
		
		
    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.classifierCfg, (void *)&classifierCfg, sizeof(mmwDemoClassifierConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_CLASSIFIER_CFG;
    memcpy((void *)&message.body.classifier, (void *)&classifierCfg, sizeof(mmwDemoClassifierConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}



/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[])
{
    MmwDemo_ADCBufCfg   adcBufCfg;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&adcBufCfg, 0, sizeof(MmwDemo_ADCBufCfg));

    /* Populate configuration: */
    adcBufCfg.adcFmt          = (uint8_t) atoi (argv[1]);
    adcBufCfg.iqSwapSel       = (uint8_t) atoi (argv[2]);
    adcBufCfg.chInterleave    = (uint8_t) atoi (argv[3]);
    adcBufCfg.chirpThreshold  = (uint8_t) atoi (argv[4]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_ADCBUFCFG;
    memcpy((void *)&message.body.adcBufCfg, (void *)&adcBufCfg, sizeof(MmwDemo_ADCBufCfg));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for compensation of range bias and channel phase offsets
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg (int32_t argc, char* argv[])
{
    MmwDemo_compRxChannelBiasCfg_t   cfg;
    MmwDemo_message     message;
    int32_t argInd;
    int32_t i;

    /* Sanity Check: Minimum argument check */
    if (argc != (1+1+SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL*2))
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Populate configuration: */
    cfg.rangeBias          = (float) atof (argv[1]);

    argInd = 2;
    for (i=0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        cfg.rxChPhaseComp[2 * i + 0] = (float) atof (argv[argInd++]);
        cfg.rxChPhaseComp[2 * i + 1] = (float) atof (argv[argInd++]);
    }
    /* Save Configuration to use later */
    memcpy((void *) &gMmwMssMCB.cfg.antCompCfg, &cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));
    message.type = MMWDEMO_MSS2DSS_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE;
    memcpy((void *)&message.body.antCompCfg, (void *)&cfg, sizeof(MmwDemo_compRxChannelBiasCfg_t));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for data logger set command
 *
 *  @param[in] argc
 *      Number of arguments
 *  @param[in] argv
 *      Arguments
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[])
{
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 2)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }


    /* Save Configuration to use later */
    if (strcmp(argv[1], "mssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 0;
    else if (strcmp(argv[1], "dssLogger") == 0)  
        gMmwMssMCB.cfg.dataLogger = 1;
    else
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
       
    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_SET_DATALOGGER;
    message.body.dataLogger = gMmwMssMCB.cfg.dataLogger;

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Execution Task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_CLIInit (void)
{
    CLI_Cfg     cliCfg;
    char        demoBanner[256];
	int32_t     errorCode;

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], "******************************************\n" \
                       "MMW TM Demo %s\n"  \
                       "******************************************\n", MMW_VERSION);
    
    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMssMCB.commandUartHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
    cliCfg.socHandle                 	= gMmwMssMCB.socHandle;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.tableEntry[0].cmd            = "sensorStart";
    cliCfg.tableEntry[0].helpString     = "No arguments";
    cliCfg.tableEntry[0].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cliCfg.tableEntry[1].cmd            = "sensorStop";
    cliCfg.tableEntry[1].helpString     = "No arguments";
    cliCfg.tableEntry[1].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cliCfg.tableEntry[2].cmd            = "frameStart";
    cliCfg.tableEntry[2].helpString     = "No arguments";
    cliCfg.tableEntry[2].cmdHandlerFxn  = MmwDemo_CLIFrameStart;
    cliCfg.tableEntry[3].cmd            = "guiMonitor";
    cliCfg.tableEntry[3].helpString     = "<detectedObjects> <logMagRange> <rangeAzimuthHeatMap> <rangeDopplerHeatMap>";
    cliCfg.tableEntry[3].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cliCfg.tableEntry[4].cmd            = "cfarCfg";
    cliCfg.tableEntry[4].helpString     = "<detMode> <discardLeft> <discardRight> <refWinSize1> <refWinSize2> <guardWinSize1> <guardWinSize2> <thre>";
    cliCfg.tableEntry[4].cmdHandlerFxn  = MmwDemo_CLICfarCfg;
    cliCfg.tableEntry[5].cmd            = "doaCfg";
    cliCfg.tableEntry[5].helpString     = "<doaMode> <doaGamma> <sideLobe_dB> <searchRange> <searchRes> <varThre>";
    cliCfg.tableEntry[5].cmdHandlerFxn  = MmwDemo_CLIDoACfg;
    cliCfg.tableEntry[6].cmd            = "trackingCfg";
    cliCfg.tableEntry[6].helpString     = "<enable> <paramSet> <numPoints> <numTracks> <maxDoppler> <framePeriod>";
    cliCfg.tableEntry[6].cmdHandlerFxn  = MmwDemo_CLITrackingCfg;
    cliCfg.tableEntry[7].cmd            = "classifierCfg";
    cliCfg.tableEntry[7].helpString     = "<platformSelector> <CBSelector> <k> <CBLen>";
    cliCfg.tableEntry[7].cmdHandlerFxn  = MmwDemo_CLIClassifierCfg;
    cliCfg.tableEntry[8].cmd            = "dataLogger";
    cliCfg.tableEntry[8].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[8].cmdHandlerFxn  = MmwDemo_CLISetDataLogger;
    cliCfg.tableEntry[9].cmd            = "adcbufCfg";
    cliCfg.tableEntry[9].helpString     = "<adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[9].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cliCfg.tableEntry[10].cmd            = "compRangeBiasAndRxChanPhase";
    cliCfg.tableEntry[10].helpString     = "<rangeBias> <Re00> <Im00> <Re01> <Im01> <Re02> <Im02> <Re03> <Im03> <Re10> <Im10> <Re11> <Im11> <Re12> <Im12> <Re13> <Im13> ";
    cliCfg.tableEntry[10].cmdHandlerFxn  = MmwDemo_CLICompRangeBiasAndRxChanPhaseCfg;

    //Apps Quality of Life
    cliCfg.tableEntry[11].cmd            = "SceneryParam";// PC OFFICE Left Wall (-1.5), Right Wall (1.5), Bottom Exit Zone (1m), Upper Exit Zone (4.5m)
    cliCfg.tableEntry[11].helpString     = "<Left Wall> <Right Wall> <Bottom Exit Zone> <Upper Exit Zone>";
    cliCfg.tableEntry[11].cmdHandlerFxn  = MmwDemo_CLISceneryParamCfg;
    cliCfg.tableEntry[12].cmd            = "GatingParam";// PC: 4 gating volume, Limits are set to 3m in length, 2m in width, 0 no limit in doppler
    cliCfg.tableEntry[12].helpString     = "<gating volume> <length> <width> <doppler>";
    cliCfg.tableEntry[12].cmdHandlerFxn  = MmwDemo_CLIGatingParamCfg;
    cliCfg.tableEntry[13].cmd            = "StateParam";// PC: 10 frames to activate, 5 to forget, 10 active to free, 1000 static to free, 5 exit to free
    cliCfg.tableEntry[13].helpString     = "<det2act> <det2free> <act2free> <stat2free> <exit2free>";//det2act, det2free, act2free, stat2free, exit2free
    cliCfg.tableEntry[13].cmdHandlerFxn  = MmwDemo_CLIStateParamCfg;
    cliCfg.tableEntry[14].cmd            = "AllocationParam";// PC: 250 SNR, 0.1 minimal velocity, 5 points, 1m in distance, 2m/s in velocity
    cliCfg.tableEntry[14].helpString     = "<SNRs> <minimal velocity> <points> <in distance> <in velocity>";
    cliCfg.tableEntry[14].cmdHandlerFxn  = MmwDemo_CLIAllocationParamCfg;
    cliCfg.tableEntry[15].cmd            = "FilterParam";
    cliCfg.tableEntry[15].helpString     = "<Human Displacement> <Interference Uniqueness> <Interference Distance>";
    cliCfg.tableEntry[15].cmdHandlerFxn  = MmwDemo_FilterCfg;

    //Live Configuration
    cliCfg.tableEntry[16].cmd            = "LiveAllocation";
    cliCfg.tableEntry[16].helpString     = "<SNRs> <SNR Obscured Threshold> <minimal velocity> <points> <in distance> <in velocity>";
    cliCfg.tableEntry[16].cmdHandlerFxn  = LiveCFG_Allocation;
    cliCfg.tableEntry[17].cmd            = "LiveGating";
    cliCfg.tableEntry[17].helpString     = "<gating volume> <length> <width> <doppler>";
    cliCfg.tableEntry[17].cmdHandlerFxn  = LiveCFG_Gating;
    cliCfg.tableEntry[18].cmd            = "LiveState";
    cliCfg.tableEntry[18].helpString     = "<det2act> <det2free> <act2free> <stat2free> <exit2free>";
    cliCfg.tableEntry[18].cmdHandlerFxn  = LiveCFG_State;
    cliCfg.tableEntry[19].cmd            = "LiveScenery";
    cliCfg.tableEntry[19].helpString     = "<Left Wall> <Right Wall> <Bottom Exit Zone> <Upper Exit Zone>";
    cliCfg.tableEntry[19].cmdHandlerFxn  = LiveCFG_Scenery;

    /* Open the CLI: */
	errorCode = CLI_open (&cliCfg);
    if (errorCode < 0)
    {
        System_printf ("Error: Unable to open the CLI, error code %d\n", errorCode);
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}


