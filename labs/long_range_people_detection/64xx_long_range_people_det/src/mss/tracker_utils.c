/**
 *   @file  task_mbox.c
 *
 *   @brief
 *     MSS main implementation of the millimeter wave Demo
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
#include <math.h>


/* BIOS/XDC Include Files. */
#include <xdc/runtime/System.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/alg/gtrack/gtrack.h>
#include <ti/drivers/osal/MemoryP.h>
#include "float.h"

/* Demo Include Files */
#include <mmw.h>


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

typedef enum {
    TRACKING_DEFAULT_PARAM_SET = 0,
    TRACKING_TRAFFIC_MONITORING_PARAM_SET,
    TRACKING_PEOPLE_COUNTING_PARAM_SET,
    TRACKING_OUTDOOR_PARAM_SET,	
    TRACKING_CEILING_MOUNT_PARAM_SET	
} TRACKING_ADVANCED_PARAM_SET;

typedef enum {
    TRACKING_PARAM_SET_TM = 0,
    TRACKING_PARAM_SET_PC,
    TRACKING_PARAM_SET_OUTDOOR,
    TRACKING_PARAM_SET_CEILING_MOUNT
} TRACKING_ADVANCED_PARAM_SET_TABLE;
#if 0
/* Scenery parameters includes up to two boundary boxes and up to two static boxes */
/* Each box is in format {x1,x2, y1,y2, z1,z2}. In 2D cases, the z parameters are ignored */
GTRACK_sceneryParams appSceneryParamTable[4] = {
	/* TM: 1 boundary box: {-1,12, 15,75, 0,0}, and 1 static box {0,14, 19,50, 0,0} */
	1,{{-1.f,12.f, 15.f,75.f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}},1,{{0.f,11.f, 19.f,50.f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f}},
	/* PEOPLE COUNTING: 1 boundary box: {-4,4, 0.5,7.5, 0,0}, and 1 static box {-3,3, 2,6, 0,0} */
	1,{{-4.f,4.f, 0.5f,7.5f, 0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-3.f,3.f,2.f,6.f,0.f,0.f},{0.f,0.f,0.f,0.f,0.f,0.f}},
 	/* OUTDOOR: 1 boundary box: {-39,19, 2,50, 0,0}, and 1 static box {-30,16, 4,44, 0,0} */
	1,{{-29.f,39.f, 2.f,59.f, -1.f,3.f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-20.f,20.f, 12.f,40.f, 0.f, 2.f},{0.f,0.f,0.f,0.f,0.f,0.f}},
	/* CEILING MOUNT: 1 boundary box: {-4,4, 0.5,7.5, -1,3}, and 1 static box {-3,3, 2,6, -0.5,2.5} */
	1,{{-4.f,4.f, 0.5f,7.5f, -1.f,3.0f},{0.f,0.f,0.f,0.f,0.f,0.f}}, 1,{{-3.f,3.f,2.f,6.f,-0.5,2.5f},{0.f,0.f,0.f,0.f,0.f,0.f}}
};

/* Gating Volume 2 "liters", Limits are set to 2m in depth and width, no limit in height and doppler */
GTRACK_gatingParams appGatingParamTable[4] = {
	/* TM: Gating volume = 16, Limits are set to 12m in depth, 8m in width, ignore the height, no limit in doppler */
    {4.f, {12.f, 6.f, 4.f, 12.f}},
	/* PEOPLE COUNTING: Gating gain = 3, Limits are 2m in depth, 2m in width, ignore the height, 12m/s in doppler */
	{3.f,  {2.f,  2.f, 2.f, 12.f}},
	/* OUTDOOR: Gating gain = 4, Limits are set to 6m in depth, 6m in width, ignore the height, 10m/s limit in doppler */
    {4.f, {6.f,  6.f, 4.f, 10.f}},
	/* CEILING MOUNT: Gating volume = 2, Limits are 2m in depth, 2m in width, 2m the height, no limit in doppler */
	{2.f,  {2.f,  2.f, 2.f, 10.f}}
};
GTRACK_stateParams appStateParamTable[4] = {
	{3U,	3U,  	5U,		100U, 	5U}, 	/* TM: det2act, det2free, act2free, stat2free, exit2free */
	{10U,	5U, 	50U,	100U, 	5U}, 	/* PC: det2act, det2free, act2free, stat2free, exit2free */
	{4U, 	10U, 	60U, 	600U,	20U},	/* OUTDOOR: det2act, det2free, act2free, stat2free, exit2free */
	{10U,	5U, 	10U, 	100U,	5U} 	/* CEILING MOUNT: det2act, det2free, act2free, stat2free, exit2free */
};
GTRACK_allocationParams appAllocationParamTable[4] = {
	{100.f, 100.f, 1.f,   3U, 4.f,   2.f},	/* TM: 100 (100) SNRs, 1m/s minimal velocity, 3 points with 4m in distance, 2m/c in velocity  separation */
	{60.f, 	200.f, 0.1f,  5U, 1.5f,  2.f},	/* PC: 150 (250 obscured), 0.1 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
	{40.f,  200.f, 0.5f,  3U, 2.f,   2.f},  /* OUTDOOR: 50 (200 obscured), 0.5 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
	{60.f,  200.f, 0.1f,  5U, 1.5f,  2.f}  	/* CEILING MOUNT: 150 (200 obscured), 0.5 m/s minimal velocity, 5 points, with 1m in distance, 2m/c in velocity in separation */
};
/* This parameter is ignored in 2D/3D tracker versions */
GTRACK_varParams appVariationParamTable[4] = {
	 {0.f, 0.f, 0.f},
	 {0.f, 0.f, 0.f},
	 {0.f, 0.f, 0.f},
	 {0.f, 0.f, 0.f}
};

float maxAccelerationParams[3] = {1, 1, 1};
#endif
/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
extern MmwDemo_MCB  gMmwMCB;

unsigned int gGtrackMemoryUsed = 0;

/* @TODO: These functions need to be abstracted to the DPC */
void *gtrack_alloc(unsigned int numElements, unsigned int sizeInBytes)
{
	gGtrackMemoryUsed += numElements*sizeInBytes;
    return MemoryP_ctrlAlloc(numElements*sizeInBytes, 0);
}
void gtrack_free(void *pFree, unsigned int sizeInBytes)
{
	gGtrackMemoryUsed -= sizeInBytes;
	MemoryP_ctrlFree(pFree,sizeInBytes);
}
void gtrack_log(GTRACK_VERBOSE_TYPE level, const char *format, ...)
{
#if 0
	va_list args;
    va_start(args, format);
	vprintf(format, args);
	va_end(args);
#endif
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for tracking configuration
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
int32_t MmwDemo_CLITrackingCfg (int32_t argc, char* argv[])
{
    GTRACK_moduleConfig         config;


    TRACKING_ADVANCED_PARAM_SET trackingParamSet;

    //Memory_Stats            startMemoryStats;
    //Memory_Stats            endMemoryStats;

    if (argc >= 1) {
        gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.trackerEnabled = (uint16_t) atoi (argv[1]);
    }

    if(gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.trackerEnabled != 1) {
        return 0;
    }

    /* Sanity Check: Minimum argument check */
    if (argc != 8)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
#if 0
    System_printf("Debug: Heap before creating a tracker\n");
    MmwDemo_printHeapStats();
#endif
    /* Initialize CLI configuration: */
    memset ((void *)&config, 0, sizeof(GTRACK_moduleConfig));

    trackingParamSet = (TRACKING_ADVANCED_PARAM_SET) atoi (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.trackingParamSet = trackingParamSet;

    switch(trackingParamSet)
    {
        case TRACKING_DEFAULT_PARAM_SET:
            // Do not configure advanced parameters, use library default parameters
            config.advParams = 0;
            break;

        case TRACKING_TRAFFIC_MONITORING_PARAM_SET:
            /* Initialize CLI configuration: */
            config.initialRadialVelocity = -8.0f; 	// for TM, detected targets are approaching
            config.maxAcceleration[0] = 2.0f;     	// for TM, maximum acceleration in lateral direction is set to 2m/s2
            config.maxAcceleration[1] = 20.0f;     	// for TM, maximum acceleration is longitudinal direction set to 20m/s2
            config.maxAcceleration[2] = 0.f;		// ignored
            break;

        case TRACKING_PEOPLE_COUNTING_PARAM_SET:
            /* Initialize CLI configuration: */
            config.initialRadialVelocity = 0;	//For PC, detected target velocity is unknown
            config.maxAcceleration[0] = 1.0f;
            config.maxAcceleration[1] = 1.0f;
            config.maxAcceleration[2] = 1.0f;
            break;

        case TRACKING_OUTDOOR_PARAM_SET:
            /* Initialize CLI configuration: */
            config.initialRadialVelocity = 0; 	// for OUTDOOR, detected targets velocity is unknown 
            config.maxAcceleration[0] = 1.0f;
            config.maxAcceleration[1] = 1.0f;
            config.maxAcceleration[2] = 1.0f;
            break;			

        case TRACKING_CEILING_MOUNT_PARAM_SET:
            /* Initialize CLI configuration: */
            config.initialRadialVelocity = 0;
            config.maxAcceleration[0] = 0.1f;
            config.maxAcceleration[1] = 0.1f;
            config.maxAcceleration[2] = 0.1f;
            break;


        default:
            CLI_write ("Error: Invalid usage of the CLI command\n");
            return -1;
    }
#ifdef GTRACK_3D
    config.stateVectorType      	= GTRACK_STATE_VECTORS_3DA; // Track three dimensions with acceleration
#else
    config.stateVectorType      	= GTRACK_STATE_VECTORS_2DA; // Track two dimensions with acceleration
#endif
    config.verbose              	= GTRACK_VERBOSE_NONE;
    config.maxNumPoints         	= (uint16_t) atoi(argv[3]);
    config.maxNumTracks         	= (uint16_t) atoi(argv[4]);
    config.maxRadialVelocity   		= (float) atoi(argv[5]) *0.1f;
#ifndef GTRACK_3D
    config.radialVelocityResolution	= (float) atoi(argv[6]) *0.001f;
#endif
    config.deltaT               	= (float) atoi(argv[7]) *0.001f;


    /* Save Configuration to use later */
    memcpy((void *)&gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gtrackModuleConfig, (void *)&config, sizeof(GTRACK_moduleConfig));
    //memcpy((void *)&gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gTrackAdvParams, (void *)&advParams, sizeof(GTRACK_advancedParameters));

    return 0;
}

int32_t MmwDemo_CLIStaticBoundaryBoxCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
#ifdef GTRACK_3D
    if (argc != 7)
#else
    if (argc != 5)
#endif
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    //memset ((void *)&sceneryParams, 0, sizeof(GTRACK_sceneryParams));
    
    /* Populate configuration: */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.numStaticBoxes = 1;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].x1 = (float) atof (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].x2 = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].y1 = (float) atof (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].y2 = (float) atof (argv[4]);
    
#ifdef GTRACK_3D
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].z1 = (float) atof (argv[5]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.staticBox[0].z2 = (float) atof (argv[6]);
#endif

    return 0;
    
}
int32_t MmwDemo_CLISensorPositionCfg(int32_t argc, char* argv[])
{
    if (argc != 4)
    {
        CLI_write("Error: Invalid usage of the CLI Command\n");
        return -1;
    }

    /* Assume sensor position as the origin in the xy plane so x=0, y=0*/
    /*populate sensor position configuration*/
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.sensorPosition.x = 0;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.sensorPosition.y = 0;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.sensorPosition.z = (float) atof (argv[1]);
    /*populate sensor orientation configuration*/
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.sensorOrientation.azimTilt = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.sensorOrientation.elevTilt = (float) atof (argv[3]);
    /*demo parameters*/
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorAzimuthTilt = atoi(argv[2])*3.14f/180;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorElevationTilt = atoi(argv[3])*3.14f/180;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sensorHeight = (float)atof(argv[1]);

    return 0;
}
int32_t MmwDemo_CLIBoundaryBoxCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
#ifdef GTRACK_3D
    if (argc != 7)
#else
    if (argc != 5)
#endif
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    //memset ((void *)&sceneryParams, 0, sizeof(GTRACK_sceneryParams));
    
    /* Populate configuration: */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.numBoundaryBoxes = 1;
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].x1 = (float) atof (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].x2 = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].y1 = (float) atof (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].y2 = (float) atof (argv[4]);
#ifdef GTRACK_3D
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].z1 = (float) atof (argv[5]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.boundaryBox[0].z2 = (float) atof (argv[6]);
#endif
    
    return 0;
    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for GatingParam configuration
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
int32_t MmwDemo_CLIGatingParamCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
#ifdef GTRACK_3D
    if (argc != 6)
#else
    if (argc != 5)
#endif
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    /* Initialize the ADC Output configuration: */
    //memset ((void *)&gatingParams, 0, sizeof(GTRACK_gatingParams));
    
    /* Populate configuration: */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.gain = (float) atof (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.limits.width = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.limits.depth = (float) atof (argv[3]);
#ifdef GTRACK_3D
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.limits.height = (float) atof (argv[4]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.limits.vel = (float) atof (argv[5]);
#else
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.gatingParams.limits.vel = (float) atof (argv[4]);
#endif

    return 0;
    
}


/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for StateParam configuration
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
int32_t MmwDemo_CLIStateParamCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    //memset ((void *)&stateParams, 0, sizeof(GTRACK_stateParams));
    
    /* Populate configuration: */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.det2actThre = (uint16_t) atoi (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.det2freeThre= (uint16_t) atoi (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.active2freeThre= (uint16_t) atoi (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.static2freeThre= (uint16_t) atoi (argv[4]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.exit2freeThre= (uint16_t) atoi (argv[5]);
	gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.stateParams.sleep2freeThre = (uint16_t) atoi (argv[6]);
			  

    return 0;
    
}

/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for AllocationParam configuration
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
int32_t MmwDemo_CLIAllocationParamCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
#ifdef GTRACK_3D
    if (argc != 7)
#else
    if (argc != 6)
#endif
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Initialize the ADC Output configuration: */
    //memset ((void *)&allocationParams, 0, sizeof(GTRACK_allocationParams));

    /* Populate configuration: */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.snrThre = (float) atof (argv[1]);
#ifdef GTRACK_3D
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.snrThreObscured = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.velocityThre = (float) atof (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.pointsThre = (uint16_t) atoi (argv[4]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.maxDistanceThre = (float) atof (argv[5]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.maxVelThre = (float) atof (argv[6]);
#else
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.velocityThre = (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.pointsThre = (uint16_t) atoi (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.maxDistanceThre = (float) atof (argv[4]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.allocationParams.maxVelThre = (float) atof (argv[5]);
#endif

    return 0;
    
}
/**
 *  @b Description
 *  @n
 *      This is the CLI Handler for PresenceParams configuration
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
int32_t MmwDemo_CLIPresenceParamCfg (int32_t argc, char* argv[])
{
    /* Sanity Check: Minimum argument check */
    if (argc != 7)
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }

    /* Hardcode presence detection thresholds */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.pointsThre= 3;//(uint16_t) atoi (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.velocityThre= 0.5;//(uint16_t) atoi (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.on2offThre= 10;//(uint16_t) atoi (argv[3]);
    /* Only one presence detection boundary box supported for now */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.numOccupancyBoxes= 1;

    /* Set presence detection enabled flag */
    if( ( gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.sceneryParams.numBoundaryBoxes > 0)
            && ( gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.pointsThre > 0))
    {
        //gMmwMCB.presenceDetEnabled = true;
    }
    
    /* Boundary Box configuration */
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].x1= (float) atof (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].x2= (float) atof (argv[2]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].y1= (float) atof (argv[3]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].y2= (float) atof (argv[4]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].z1= (float) atof (argv[5]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.presenceParams.occupancyBox[0].z2= (float) atof (argv[6]);

    return 0;
    
}

int32_t MmwDemoCLIMaxAccelerationParamCfg(int32_t argc, char* argv[])
{
#ifdef GTRACK_3D
    if (argc != 4)
#else
    if (argc != 3)
#endif
    {
        CLI_write ("Error: Invalid usage of the CLI command\n");
        return -1;
    }
    
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.accelerationParams[0] = (float) atof (argv[1]);
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.accelerationParams[1] = (float) atof (argv[2]);
#ifdef GTRACK_3D    
    gMmwMCB.trackerCfg.trackerDpuCfg.staticCfg.accelerationParams[2] = (float) atof (argv[3]);
#endif
    
    return 0;

}
