/*
 *   @file  cli.c
 *
 *   @brief
 *      Mmw (Milli-meter wave) DEMO CLI Implementation
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
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/control/mmwavelink/mmwavelink.h>

/* Demo Include Files */
#include "mmw.h"


extern uint32_t log2Approx(uint32_t x);

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* CLI Command Functions */
static int32_t MmwDemo_CLIHighAccuCfg (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStart (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLISensorStop (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLIGuiMonSel (int32_t argc, char* argv[]);
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[]);

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern MmwDemo_MCB    gMmwMCB;

/**************************************************************************
 **************************** MMW CLI Functions ***************************
 **************************************************************************/

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
    guiMonSel.logMagRange               = atoi (argv[2]);
    guiMonSel.noiseProfile              = atoi (argv[3]);
    guiMonSel.rangeAzimuthHeatMap       = atoi (argv[4]);
    guiMonSel.rangeDopplerHeatMap       = atoi (argv[5]);
    guiMonSel.statsInfo                     = atoi (argv[6]);

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMCB.cliCfg.guiMonSel, (void *)&guiMonSel, sizeof(MmwDemo_GuiMonSel));
    return 0;
}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for DC range calibration
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
static int32_t MmwDemo_CLICalibDcRangeSig (int32_t argc, char* argv[])
{
    MmwDemo_CalibDcRangeSigCfg cfg;
    uint32_t log2NumAvgChirps;

    /* Sanity Check: Minimum argument check */
    if (argc != 5)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&cfg, 0, sizeof(MmwDemo_CalibDcRangeSigCfg));

    /* Populate configuration: */
    cfg.enabled          = (uint16_t) atoi (argv[1]);
    cfg.negativeBinIdx   = (int16_t)  atoi (argv[2]);
    cfg.positiveBinIdx   = (int16_t)  atoi (argv[3]);
    cfg.numAvgChirps     = (uint16_t)  atoi (argv[4]);

    if (cfg.negativeBinIdx > 0)
    {
        CLI_write ("Error: Invalid negative bin index\n");
        return -1;
    }
    if ((cfg.positiveBinIdx - cfg.negativeBinIdx + 1) > DC_RANGE_SIGNATURE_COMP_MAX_BIN_SIZE)
    {
        CLI_write ("Error: Number of bins exceeds the limit\n");
        return -1;
    }
    log2NumAvgChirps = (uint32_t) log2Approx (cfg.numAvgChirps);
    if (cfg.numAvgChirps != (1 << log2NumAvgChirps))
    {
        CLI_write ("Error: Number of averaged chirps is not power of two\n");
        return -1;
    }

    /* Save Configuration to use later */
    memcpy((void *)&gMmwMCB.dataPathObj.cliCfg->calibDcRangeSigCfg, (void *)&cfg, 
        sizeof(MmwDemo_CalibDcRangeSigCfg));
    gMmwMCB.dataPathObj.dcRangeSigCalibCntr = 0;
    gMmwMCB.dataPathObj.log2NumAvgChirps = log2NumAvgChirps;

    return 0;
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

highAccuRangeProc_config     gHighAccuConfig;
static int32_t MmwDemo_CLIRangeLimitCfg (int32_t argc, char* argv[])
{


    /* Sanity Check: Minimum argument check */
    if (argc != 4)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    memset ((void *)&gHighAccuConfig, 0, sizeof(highAccuRangeProc_config));

    //System_printf("highAccuConfig config\n");


    gHighAccuConfig.enableRangeLimit    = (uint8_t) atoi (argv[1]);
    gHighAccuConfig.skipMin              = (float) atof (argv[2]);
    gHighAccuConfig.skipMax              = (float) atof (argv[3]);

//    gHighAccuConfig.skipMinRangeBin      = (uint16_t)(gHighAccuConfig.skipMin/gMmwMCB.dataPathObj.rangeResolution);
//    gHighAccuConfig.skipMaxRangeBin      = (uint16_t)(gHighAccuConfig.skipMax/gMmwMCB.dataPathObj.rangeResolution);
//    printf("highAccuConfig config %f,%f\n",gHighAccuConfig.skipMin,gHighAccuConfig.skipMax);
//    printf("highAccuConfig rangebin %d,%d\n",gHighAccuConfig.skipMinRangeBin,gHighAccuConfig.skipMaxRangeBin);
//
//
//    if(gHighAccuConfig.skipMinRangeBin > gMmwMCB.dataPathObj.numRangeBins)
//    {
//            CLI_write ("Error: skipMin should be less than total range bins\n");
//            return -1;
//    }
//    if(gHighAccuConfig.skipMaxRangeBin > gMmwMCB.dataPathObj.numRangeBins)
//    {
//            CLI_write ("Error: skipMax should be less than total range bins\n");
//            return -1;
//    }


    return 0;
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for the ADCBUF configuration
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
    MmwDemo_ADCBufCfg    adcBufCfg;

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
    memcpy((void *)&gMmwMCB.dataPathObj.cliCfg->adcBufCfg, (void *)&adcBufCfg, 
        sizeof(MmwDemo_ADCBufCfg));
    return 0;
}

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
    
    bool doReconfig = true;
    if (argc==2)
    {
        doReconfig = (bool) atoi (argv[1]);
    }
    /* Notify the sensor management module to start the sensor */
    MmwDemo_notifySensorStart (doReconfig);
    /* Pend for completion */
    return (MmwDemo_waitSensorStartComplete());
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
    /* Notify the sensor management module to stop the sensor */
    MmwDemo_notifySensorStop ();
    /* Pend for completion */
    MmwDemo_waitSensorStopComplete();
    
    return 0;
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
    uint32_t    cnt = 0;
    char        demoBanner[256];

    /* Create Demo Banner to be printed out by CLI */
    sprintf(&demoBanner[0], 
                       "******************************************\n" \
                       "xWR14xx MMW Demo %02d.%02d.%02d.%02d\n"  \
                       "******************************************\n", 
                       MMWAVE_SDK_VERSION_MAJOR,
                       MMWAVE_SDK_VERSION_MINOR,
                       MMWAVE_SDK_VERSION_BUGFIX,
                       MMWAVE_SDK_VERSION_BUILD
            );


    /* Initialize the CLI configuration: */
    memset ((void *)&cliCfg, 0, sizeof(CLI_Cfg));

    /* Populate the CLI configuration: */
    cliCfg.cliPrompt                    = "mmwDemo:/>";
    cliCfg.cliBanner                    = demoBanner;
    cliCfg.cliUartHandle                = gMmwMCB.commandUartHandle;
    cliCfg.socHandle                    = gMmwMCB.socHandle;

    cliCfg.taskPriority                 = 3;
    cliCfg.enableMMWaveExtension        = 1U;
    cliCfg.usePolledMode                = true;
    cliCfg.mmWaveHandle                 = gMmwMCB.ctrlHandle;
    cliCfg.tableEntry[cnt].cmd            = "sensorStart";
    cliCfg.tableEntry[cnt].helpString     = NULL; //"[doReconfig(optional, default:enabled)]";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStart;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "sensorStop";
    cliCfg.tableEntry[cnt].helpString     = NULL; //"No arguments";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLISensorStop;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "guiMonitor";
    cliCfg.tableEntry[cnt].helpString     = NULL; //"<detectedObjects> <logMagRange> <noiseProfile> <rangeAzimuthHeatMap> <rangeDopplerHeatMap> <statsInfo>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIGuiMonSel;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "calibDcRangeSig";
    cliCfg.tableEntry[cnt].helpString     = NULL; //"<enabled> <negativeBinIdx> <positiveBinIdx> <numAvgFrames>";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLICalibDcRangeSig;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "adcbufCfg";
#if 0
    cliCfg.tableEntry[cnt].helpString     = "<adcOutputFmt> <SampleSwap> <ChanInterleave> <ChirpThreshold>";
#else
    cliCfg.tableEntry[cnt].helpString     = NULL;
#endif
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIADCBufCfg;
    cnt++;

    cliCfg.tableEntry[cnt].cmd            = "RangeLimitCfg";
    cliCfg.tableEntry[cnt].helpString     = NULL; //"<enabled> <min_range> <max_range> ";
    cliCfg.tableEntry[cnt].cmdHandlerFxn  = MmwDemo_CLIRangeLimitCfg;
    cnt++;

    /* Open the CLI: */
    if (CLI_open (&cliCfg) < 0)
    {
        System_printf ("Error: Unable to open the CLI\n");
        return;
    }
    System_printf ("Debug: CLI is operational\n");
    return;
}

