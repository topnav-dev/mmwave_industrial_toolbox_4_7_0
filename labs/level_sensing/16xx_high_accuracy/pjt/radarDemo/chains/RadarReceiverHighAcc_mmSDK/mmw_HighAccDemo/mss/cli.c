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
#include <ti/control/mmwavelink/mmwavelink.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cli/include/cli_internal.h>

/* Demo Include Files */
#include "mss_mmw.h"
/*#include <chains/RadarReceiverHighAcc_mmSDK/mmw_HighAccDemo/common/mmw_messages.h>*/
#include "mmw_messages.h"

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Extended Command Functions */
static int32_t MmwDemo_CLIHighAccuCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIFrameStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISetDataLogger (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIADCBufCfg (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMssMCB;
extern int32_t MmwDemo_mboxWrite(MmwDemo_message     * message);

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

    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&guiMonSel, 0, sizeof(MmwDemo_GuiMonSel));

    /* Populate configuration: */
    guiMonSel.detectedObjects           = atoi (argv[1]);
    guiMonSel.logRangeInput             = atoi (argv[2]);
    guiMonSel.dummy       				= atoi (argv[3]);
    guiMonSel.dummy       				= atoi (argv[4]);
    guiMonSel.dummy       				= atoi (argv[5]);
    guiMonSel.logStats       			= atoi (argv[6]);

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
static int32_t MmwDemo_CLIHighAccuCfg (int32_t argc, char* argv[])
{
	radarModuleHighAccuConfig     highAccuConfig;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&highAccuConfig, 0, sizeof(radarModuleHighAccuConfig));



    //System_printf("highAccuConfig config\n");

    //cliCfg.tableEntry[4].helpString     = "<numRangeBinZoomIn> <skipLeft> <skipRight> <enablePhaseEst> <enableLinearFit> <enableFilter>";
    /* Populate configuration: */
    highAccuConfig.numRangeBinZoomIn    = (uint16_t) atoi (argv[1]);
    highAccuConfig.skipLeft       		= (uint16_t) atoi (argv[2]);
    highAccuConfig.skipRight       		= (uint16_t) atoi (argv[3]);
    highAccuConfig.enablePhaseEst  		= (uint16_t) atoi (argv[4]);
    highAccuConfig.enableLinearFit 		= (uint16_t) atoi (argv[5]);
    highAccuConfig.enableFilter    		= (uint16_t) atoi (argv[6]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.highAccuRangeCfg, (void *)&highAccuConfig, sizeof(radarModuleHighAccuConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_HIGHACCURANGE_CFG;
    memcpy((void *)&message.body.highAccuRangeCfg, (void *)&highAccuConfig, sizeof(radarModuleHighAccuConfig));

    if (MmwDemo_mboxWrite(&message) == 0)
        return 0;
    else
        return -1;    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for range limited configuration
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
static int32_t MmwDemo_CLIRangeLimitCfg (int32_t argc, char* argv[])
{
    radarModuleHighAccuConfig     highAccuConfig;
    MmwDemo_message     message;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&highAccuConfig, 0, sizeof(radarModuleHighAccuConfig));

    /* Read Configuration */
    memcpy( (void *)&highAccuConfig, (void *)&gMmwMssMCB.cfg.highAccuRangeCfg, sizeof(radarModuleHighAccuConfig));


    //System_printf("highAccuConfig config\n");

    //"<numRangeBinZoomIn> <enabled> <min_range> <max_range> ";
    /* Populate configuration: */
    highAccuConfig.numRangeBinZoomIn    = (uint16_t) atoi (argv[1]);
    highAccuConfig.enableRangeLimit    = (uint8_t) atoi (argv[2]);
    highAccuConfig.skipMin              = (float) atof (argv[3]);
    highAccuConfig.skipMax              = (float) atof (argv[4]);

    printf("numRangeBinZoomIn=%d,left=%d,right=%d\n",highAccuConfig.numRangeBinZoomIn,highAccuConfig.skipLeft,highAccuConfig.skipRight);
    printf("enableRangeLimit=%d,skipMin=%f,skipMax=%f\n",highAccuConfig.enableRangeLimit,highAccuConfig.skipMin,highAccuConfig.skipMax);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMssMCB.cfg.highAccuRangeCfg, (void *)&highAccuConfig, sizeof(radarModuleHighAccuConfig));

    /* Send configuration to DSS */
    memset((void *)&message, 0, sizeof(MmwDemo_message));

    message.type = MMWDEMO_MSS2DSS_HIGHACCURANGE_CFG;
    memcpy((void *)&message.body.highAccuRangeCfg, (void *)&highAccuConfig, sizeof(radarModuleHighAccuConfig));

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
    cliCfg.socHandle                    = gMmwMssMCB.socHandle;
    cliCfg.taskPriority                 = 3;
    cliCfg.mmWaveHandle                 = gMmwMssMCB.ctrlHandle;
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
    cliCfg.tableEntry[4].cmd            = "highAccCfg";
    cliCfg.tableEntry[4].helpString     = "<numRangeBinZoomIn> <skipLeft> <skipRight> <enablePhaseEst> <enableLinearFit> <enableFilter>";
    cliCfg.tableEntry[4].cmdHandlerFxn  = MmwDemo_CLIHighAccuCfg;
    cliCfg.tableEntry[5].cmd            = "dataLogger";
    cliCfg.tableEntry[5].helpString     = "<mssLogger | dssLogger>";
    cliCfg.tableEntry[5].cmdHandlerFxn  = MmwDemo_CLISetDataLogger;
    cliCfg.tableEntry[6].cmd            = "adcbufCfg";
    cliCfg.tableEntry[6].helpString     = "<adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
    cliCfg.tableEntry[6].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cliCfg.tableEntry[7].cmd            = "RangeLimitCfg";
    cliCfg.tableEntry[7].helpString     = "<numRangeBinZoomIn> <enabled> <min_range> <max_range> "; //"<numRangeBinZoomIn> <enabled> <min_range> <max_range> ";
    cliCfg.tableEntry[7].cmdHandlerFxn  = MmwDemo_CLIRangeLimitCfg;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}


