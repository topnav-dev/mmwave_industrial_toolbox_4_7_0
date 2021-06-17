/**
 *   @file  pcount3D_mss.h
 *
 *   @brief
 *      This is the main header file for the 3D people counting demo
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
#ifndef MMW_MSS_H
#define MMW_MSS_H

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/common/mmwave_error.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>

#include <ti/demo/utils/mmwdemo_adcconfig.h>
//#include <ti/demo/utils/mmwdemo_monitor.h>

#include <common/src/dpc/objdetrangehwa_overhead/objdetrangehwa.h>
#include <people_counting/overhead_3d_people_counting/src/common/pcount3D_output.h>
#include <people_counting/overhead_3d_people_counting/src/common/pcount3D_config.h>


#ifdef __cplusplus
extern "C" {
#endif

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define PCOUNT3DDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/**
 * @defgroup configStoreOffsets     Offsets for storing CLI configuration
 * @brief    Offsets of config fields within the parent structures, note these offsets will be
 *           unique and hence can be used to differentiate the commands for processing purposes.
 * @{
 */
#define PCOUNT3DDEMO_ADCBUFCFG_OFFSET                 (offsetof(Pcount3DDemo_SubFrameCfg, adcBufCfg))

#define PCOUNT3DDEMO_SUBFRAME_DSPDYNCFG_OFFSET        (offsetof(Pcount3DDemo_SubFrameCfg, objDetDynCfg) + \
                                                  offsetof(Pcount3DDemo_DPC_ObjDet_DynCfg, dspDynCfg))

#define PCOUNT3DDEMO_SUBFRAME_R4FDYNCFG_OFFSET        (offsetof(Pcount3DDemo_SubFrameCfg, objDetDynCfg) + \
                                                  offsetof(Pcount3DDemo_DPC_ObjDet_DynCfg, r4fDynCfg))

#define PCOUNT3DDEMO_CALIBDCRANGESIG_OFFSET           (PCOUNT3DDEMO_SUBFRAME_R4FDYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetectionRangeHWA_DynCfg, calibDcRangeSigCfg))

#define PCOUNT3DDEMO_CAPONCHAINCFG_OFFSET              (PCOUNT3DDEMO_SUBFRAME_DSPDYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, caponChainCfg))

#define PCOUNT3DDEMO_DYNRACFARCFG_OFFSET            (PCOUNT3DDEMO_CAPONCHAINCFG_OFFSET + \
                                                  offsetof(caponChainCfg, dynamicCfarConfig))

#define PCOUNT3DDEMO_STATICRACFARCFG_OFFSET              (PCOUNT3DDEMO_CAPONCHAINCFG_OFFSET + \
                                                  offsetof(caponChainCfg, staticCfarConfig))

#define PCOUNT3DDEMO_DOACAPONCFG_OFFSET                (PCOUNT3DDEMO_CAPONCHAINCFG_OFFSET + \
                                                  offsetof(caponChainCfg, doaConfig))

#define PCOUNT3DDEMO_DOACAPONRACFG_OFFSET                    (PCOUNT3DDEMO_DOACAPONCFG_OFFSET + \
                                                  offsetof(doaConfig, rangeAngleCfg))

#define PCOUNT3DDEMO_DOA2DESTCFG_OFFSET                 (PCOUNT3DDEMO_DOACAPONCFG_OFFSET + \
                                                  offsetof(doaConfig, angle2DEst))

#define PCOUNT3DDEMO_DOAFOVCFG_OFFSET                 (PCOUNT3DDEMO_DOACAPONCFG_OFFSET + \
                                                  offsetof(doaConfig, fovCfg))

#define PCOUNT3DDEMO_STATICANGESTCFG_OFFSET       (PCOUNT3DDEMO_DOACAPONCFG_OFFSET + \
                                                  offsetof(doaConfig, staticEstCfg))

#define PCOUNT3DDEMO_DOPCFARCFG_OFFSET       	(PCOUNT3DDEMO_DOACAPONCFG_OFFSET + \
                                                  offsetof(doaConfig, dopCfarCfg))



/** @}*/ /* configStoreOffsets */

/**
 * @brief
 *  3D people counting Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in 3D people counting Demo
 */
typedef enum Pcount3DDemo_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    Pcount3DDemo_SensorState_INIT = 0,

    /*!  @brief Indicates sensor is started */
    Pcount3DDemo_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    Pcount3DDemo_SensorState_STOPPED
}Pcount3DDemo_SensorState;

/**
 * @brief
 *  3D people counting Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  3D people counting Demo
 */
typedef struct Pcount3DDemo_MSS_Stats_t
{
    /*! @brief   Counter which tracks the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;
    
    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     calibrationReports;

     /*! @brief   Counter which tracks the number of sensor stop events received
      *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     sensorStopped;
}Pcount3DDemo_MSS_Stats;

/**
 * @brief
 *  3D people counting Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct Pcount3DDemo_SubFrameCfg_t
{
    /*! @brief ADC buffer configuration storage */
    MmwDemo_ADCBufCfg adcBufCfg;

    /*! @brief Flag indicating if @ref adcBufCfg is pending processing. */
    uint8_t isAdcBufCfgPending : 1;

    /*! @brief Dynamic configuration storage for object detection DPC */
    Pcount3DDemo_DPC_ObjDet_DynCfg objDetDynCfg;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
    uint32_t    adcBufChanDataSize;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of chirps per sub-frame */
    uint16_t    numChirpsPerSubFrame;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
} Pcount3DDemo_SubFrameCfg;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct Pcount3DDemo_SubFrameStats_t
{
    /*! @brief   Frame processing stats */
    Pcount3DDemo_output_message_stats    outputStats;

    /*! @brief   Dynamic CLI configuration time in usec */
    uint32_t                        pendingConfigProcTime;

    /*! @brief   SubFrame Preparation time on MSS in usec */
    uint32_t                        subFramePreparationTime;
} Pcount3DDemo_SubFrameStats;

/**
 * @brief Task handles storage structure
 */
typedef struct Pcount3DDemo_TaskHandles_t
{
    /*! @brief   MMWAVE Control Task Handle */
    Task_Handle mmwaveCtrl;

    /*! @brief   ObjectDetection DPC related dpmTask */
    Task_Handle objDetDpmTask;

    /*! @brief   Demo init task */
    Task_Handle initTask;
} Pcount3DDemo_taskHandles;

typedef struct Pcount3DDemo_DataPathObj_t
{
    /*! @brief Handle to hardware accelerator driver. */
    HWA_Handle          hwaHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle         edmaHandle;

    /*! @brief   Radar cube memory information from range DPC */
    DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem radarCubeMem;

    /*! @brief   Memory usage after the preStartCfg range DPC is applied */
    DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage memUsage;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t    EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;

} Pcount3DDemo_DataPathObj;

/**
 * @brief
 *  3D people counting Demo  MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  3D people counting Demo
 */
typedef struct Pcount3DDemo_MSS_MCB_t
{
    /*! @brief      Configuration which is used to execute the demo */
    Pcount3DDemo_Cfg                 cfg;

    /*! * @brief    Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief      UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      ADCBuf driver handle */
    ADCBuf_Handle               adcBufHandle;

    /*! @brief      DSP chain DPM Handle */
    DPM_Handle                  objDetDpmHandle;

    /*! @brief      Object Detection DPC common configuration */
    Pcount3DDemo_DPC_ObjDet_CommonCfg objDetCommonCfg;

    /*! @brief      Data path object */
    Pcount3DDemo_DataPathObj         dataPathObj;

    /*! @brief   Tracker DPU Static Configuration */
    DPC_ObjectDetection_TrackerConfig trackerCfg;

    /*! @brief      Object Detection DPC subFrame configuration */
    Pcount3DDemo_SubFrameCfg         subFrameCfg[RL_MAX_SUBFRAMES];

    /*! @brief      sub-frame stats */
    Pcount3DDemo_SubFrameStats       subFrameStats[RL_MAX_SUBFRAMES];

    /*! @brief      Demo Stats */
    Pcount3DDemo_MSS_Stats           stats;

    Pcount3DDemo_output_message_UARTpointCloud   pointCloudToUart;
    DPIF_DetMatrix                      heatMapOutFromDSP;
    DPIF_PointCloudSpherical            *pointCloudFromDSP;
    DPIF_PointCloudSideInfo             *pointCloudSideInfoFromDSP;
    DPC_ObjectDetection_Stats           *frameStatsFromDSP;
    uint32_t                            currSubFrameIdx;

    trackerProc_TargetDescrHandle       trackerOutput;
    uint8_t                             numTargets;
    uint16_t                            numIndices;
    bool                                presenceDetEnabled;
    uint32_t                            presenceInd;
    uint16_t                            numDetectedPoints;
    uint32_t                            trackerProcessingTimeInUsec;
    uint32_t                            uartProcessingTimeInUsec;


    /*! @brief      Task handle storage */
    Pcount3DDemo_taskHandles         taskHandles;

    /*! @brief   RF frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
    double                      rfFreqScaleFactor;

    /*! @brief   Semaphore handle to signal DPM started from DPM report function */
    Semaphore_Handle            DPMstartSemHandle;

    /*! @brief   Semaphore handle to signal DPM stopped from DPM report function. */
    Semaphore_Handle            DPMstopSemHandle;

    /*! @brief   Semaphore handle to signal DPM ioctl from DPM report function. */
    Semaphore_Handle            DPMioctlSemHandle;

    /*! @brief   Semaphore handle to run UART DMA task. */
    Semaphore_Handle            uartTxSemHandle;

    /*! @brief   Semaphore handle to trigger tracker DPU. */
    Semaphore_Handle            trackerDPUSemHandle;

    /*! @brief    Sensor state */
    Pcount3DDemo_SensorState         sensorState;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

} Pcount3DDemo_MSS_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

/* Functions to handle the actions need to move the sensor state */
extern int32_t Pcount3DDemo_openSensor(bool isFirstTimeOpen);
extern int32_t Pcount3DDemo_configSensor(void);
extern int32_t Pcount3DDemo_startSensor(void);
extern void Pcount3DDemo_stopSensor(void);

/* functions to manage the dynamic configuration */
extern uint8_t Pcount3DDemo_isAllCfgInPendingState(void);
extern uint8_t Pcount3DDemo_isAllCfgInNonPendingState(void);
extern void Pcount3DDemo_resetStaticCfgPendingState(void);
extern void Pcount3DDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);

extern void Pcount3DDemo_CLIInit (uint8_t taskPriority);

/* Debug Functions */
extern void _Pcount3DDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define Pcount3DDemo_debugAssert(expression) {                                      \
                                         _Pcount3DDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

#ifdef __cplusplus
}
#endif

#endif /* MMW_MSS_H */

