/**
 *   @file  mss_main.c
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
#include <ti/sysbios/family/arm/v7a/Pmu.h>
#include <ti/sysbios/family/arm/v7r/vim/Hwi.h>

/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/utils/cli/cli.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

/* Demo Include Files */
#include "mss_mmw.h"
/*#include <chains/RadarReceiverHighAcc_mmSDK/mmw_HighAccDemo/common/mmw_messages.h>*/
#include "mmw_messages.h"
#include "mmw_output.h"
#include <ti/common/mmwave_sdk_version.h>
#include <ti/demo/io_interface/mmw_config.h>
#include <ti/demo/io_interface/detected_obj.h>


/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MCB    gMmwMssMCB;


#define SOC_XWR16XX_MSS_MAXNUMHEAPS (RADARMEMOSAL_HEAPTYPE_MAXNUMHEAPS)
#define SOC_XWR16XX_MSS_L1_SCRATCH_SIZE           0x2100U
#define SOC_XWR16XX_MSS_L2_BUFF_SIZE              0x12000U
#define SOC_XWR16XX_MSS_L3RAM_BUFF_SIZE           0x0000U

/*! L2 RAM buffer */
#pragma DATA_SECTION(gMmwL2, ".l2data");
#pragma DATA_ALIGN(gMmwL2, 8);
uint8_t gMmwL2[SOC_XWR16XX_MSS_L2_BUFF_SIZE];

/*! L2 RAM scratch */
#pragma DATA_SECTION(gMmwL1Scratch, ".l2data");
#pragma DATA_ALIGN(gMmwL1Scratch, 8);
uint8_t gMmwL1Scratch[SOC_XWR16XX_MSS_L1_SCRATCH_SIZE];




/**************************************************************************
 *************************** Extern Definitions *******************************
 **************************************************************************/
/* CLI Init function */
extern void MmwDemo_CLIInit (void);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions Prototype**************
 **************************************************************************/

/* Data path functions */
int32_t MmwDemo_mssDataPathConfig(void);
int32_t MmwDemo_mssDataPathStart(void);
int32_t MmwDemo_mssDataPathStop(void);

/* mmwave library call back fundtions */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg);
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg);
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg);
static void MmwDemo_mssMmwaveCloseCallbackFxn(void);

void MmwDemo_mssMmwaveStopcallbackFxn(void);
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* MMW demo Task */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1);
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1);
void MmwDemo_transmitProcessedOutput();


/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/


/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and icluding the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is set, pbjOut structure containing range,
*       doppler, and X,Y,Z location for detected objects,
*       size = sizeof(objOut_t) * number of detected objects
*    3. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    4. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    7. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    8. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    9. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] obj          Pointer data path object MmwDemo_DataPathObj
*/

void MmwDemo_transmitProcessedOutput()
{
    MmwDemo_output_message_header header;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint32_t packetLen;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
	MmwDemo_detOutputHdr           *ptrDetOutputHdr;
	radarProcessOutput_t * outputData;
    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];
	uint32_t fft1D_length;
	MmwDemo_MSS_DataPathObj     *obj = &gMmwMssMCB.mssDataPathObj;
	MmwDemo_GuiMonSel   guiMonSel = gMmwMssMCB.cfg.guiMonSel;
	UART_Handle uartHandle = gMmwMssMCB.loggingUartHandle;

    ptrDetOutputHdr = (MmwDemo_detOutputHdr *)&obj->inputInfo.header;
    outputData      = (radarProcessOutput_t *)&obj->inputInfo.pointCloudBuf;

    fft1D_length	=	outputData->fft1DSize;

    /* Clear message header */
    memset((void *)&header, 0, sizeof(MmwDemo_output_message_header));
    /* Header: */
    header.platform = 0xA1642;
    header.magicWord[0] = 0x0102;
    header.magicWord[1] = 0x0304;
    header.magicWord[2] = 0x0506;
    header.magicWord[3] = 0x0708;
    header.numDetectedObj = 1;
    header.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
                        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
                        (MMWAVE_SDK_VERSION_MINOR << 16) |
                        (MMWAVE_SDK_VERSION_MAJOR << 24);

    packetLen = sizeof(MmwDemo_output_message_header);
    //detectedObjects
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(MmwDemo_detectedObj) * 1 +
                            sizeof(MmwDemo_output_message_dataObjDescr);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    //Range FFT input
    if(guiMonSel.logRangeInput)
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_RANGE_PROFILE;
        tl[tlvIdx].length = 2 * sizeof(float) * fft1D_length;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }
    //statsInfo
    {
        tl[tlvIdx].type = MMWDEMO_OUTPUT_MSG_STATS;
        tl[tlvIdx].length = sizeof(MmwDemo_output_message_stats);
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        tlvIdx++;
    }

    header.numTLVs = tlvIdx;
    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    header.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);
    header.timeCpuCycles =  Pmu_getCount(0);
    header.frameNumber = 0;


    UART_writePolling (uartHandle,
                       (uint8_t*)&header,
                       sizeof(MmwDemo_output_message_header));

    tlvIdx = 0;
    /* Send detected Objects */
    {
        MmwDemo_output_message_dataObjDescr descr;
        MmwDemo_detectedObj dummyDetectionOut; //work around the current format
        int32_t tempRange;

        memset((void *)&dummyDetectionOut, 0, sizeof(MmwDemo_detectedObj));

        tempRange                       =   (int32_t)(outputData->rangeEst * 1048576.f);

        dummyDetectionOut.dopplerIdx    =   0;
        dummyDetectionOut.peakVal       =   0;
        dummyDetectionOut.rangeIdx      =   (uint16_t) tempRange & 0xFFFF;
        dummyDetectionOut.x             =   tempRange >> 16;
        dummyDetectionOut.y             =   0;
        dummyDetectionOut.z             =   0;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        /* Send objects descriptor */
        descr.numDetetedObj = 1;
        descr.xyzQFormat = 20;
        UART_writePolling (uartHandle, (uint8_t*)&descr, sizeof(MmwDemo_output_message_dataObjDescr));

        /*Send array of objects */
        UART_writePolling (uartHandle, (uint8_t*)&dummyDetectionOut, sizeof(MmwDemo_detectedObj) * 1);
        tlvIdx++;
    }

    /* Send Range FFT input */
    if(guiMonSel.logRangeInput)
    {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));

        UART_writePolling (uartHandle, (uint8_t*)outputData->fft1Dinput, 2 * fft1D_length * sizeof(float));
        tlvIdx++;
    }

    /* Send stats information */
    {
        MmwDemo_output_message_stats stats;
        stats.interChirpProcessingMargin = (uint32_t) ptrDetOutputHdr->chirpProcessingMarginInUsec;
        stats.interFrameProcessingMargin = (uint32_t) ptrDetOutputHdr->frameProcessingMarginInUsec;
        stats.interFrameProcessingTime = 0;
        stats.transmitOutputTime = (uint32_t) obj->cycleLog.sendingToUARTTimeCurrInusec;
        stats.activeFrameCPULoad = (uint32_t) ptrDetOutputHdr->chirpProcessingLoading;
        stats.interFrameCPULoad = (uint32_t) ptrDetOutputHdr->frameProcessingLoading;

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        UART_writePolling (uartHandle,
                           (uint8_t*)&stats,
                           tl[tlvIdx].length);
        tlvIdx++;
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }


#if 0
                	gMmwMssMCB.mssDataPathObj.inputInfoBuffSize = message.body.detObj.detObjOutsize;

					timeStart = Cycleprofiler_getTimeStamp();
					memcpy((void *)&(gMmwMssMCB.mssDataPathObj.inputInfo),
                			(uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress, SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
							message.body.detObj.detObjOutsize);
                    /*UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress,
                                                           SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                            message.body.detObj.detObjOutsize);*/
					gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec;

					timeStart = Cycleprofiler_getTimeStamp();
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)&(gMmwMssMCB.mssDataPathObj.inputInfo),
							gMmwMssMCB.mssDataPathObj.inputInfoBuffSize);
					gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec;

                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(MmwDemo_message));

                    message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                    if (MmwDemo_mboxWrite(&message) != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }
#endif

}


/**
 *  @b Description
 *  @n
 *      Registered event function which is invoked when an event from the
 *      BSS is received.
 *
 *  @param[in]  msgId
 *      Message Identifier
 *  @param[in]  sbId
 *      Subblock identifier
 *  @param[in]  sbLen
 *      Length of the subblock
 *  @param[in]  payload
 *      Pointer to the payload buffer
 *
 *  @retval
 *      Always returns 0 [Continue passing the event to the peer domain]
 */
int32_t MmwDemo_mssMmwaveEventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

#if 0
    System_printf ("Debug: BSS Event MsgId: %d [Sub Block Id: %d Sub Block Length: %d]\n",
                    msgId, sbId, sbLen);
#endif

    /* Process the received message: */
    switch (msgId)
    {
        case RL_RF_ASYNC_EVENT_MSG:
        {
            /* Received Asychronous Message: */
            switch (asyncSB)
            {
                case RL_RF_AE_CPUFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_CPUFAULT_EVT);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    /* Post event to datapath task notify BSS events */
                    Event_post(gMmwMssMCB.eventHandle, MMWDEMO_BSS_ESMFAULT_EVT);
                    break;
                }
                case RL_RF_AE_INITCALIBSTATUS_SB:
                {
                    rlRfInitComplete_t*  ptrRFInitCompleteMessage;
                    uint32_t            calibrationStatus;

                    /* Get the RF-Init completion message: */
                    ptrRFInitCompleteMessage = (rlRfInitComplete_t*)payload;
                    calibrationStatus = ptrRFInitCompleteMessage->calibStatus & 0xFFFU;

                    /* Display the calibration status: */
                    CLI_write ("Debug: Init Calibration Status = 0x%x\n", calibrationStatus);
                    break;
                }

                case RL_RF_AE_FRAME_TRIGGER_RDY_SB:
                {
                    /* This event is not handled on MSS */
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    /* Increment the statistics for the number of failed reports */
                    gMmwMssMCB.stats.numFailedTimingReports++;

                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    /* Increment the statistics for the number of received calibration reports */
                    gMmwMssMCB.stats.numCalibrationReports++;

                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    /*Received Frame Stop async event from BSS.
                      No further action required on MSS as it will
                      wait for a message from DSS when it is done (MMWDEMO_DSS2MSS_STOPDONE)*/
                    break;
                }
				default:
                {
                    System_printf ("Error: Asynchronous Event SB Id %d not handled\n", asyncSB);
                    break;
                }
            }
            break;
        }
        default:
        {
            System_printf ("Error: Asynchronous message %d is NOT handled\n", msgId);
            break;
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked after the configuration
 *      has been used to configure the mmWave link and the BSS. This is applicable only for
 *      the XWR16xx. The BSS can be configured only by the MSS *or* DSS. The callback API is
 *      triggered on the remote execution domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCtrlCfg
 *      Pointer to the control configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveConfigCallbackFxn(MMWave_CtrlCfg* ptrCtrlCfg)
{
    /* For mmw Demo, mmwave_config() will always be called from MSS, 
       due to the fact CLI is running on MSS, hence this callback won't be called */

    gMmwMssMCB.stats.datapathConfigEvt ++;
}
/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been opened.
 *
 *
 *  @param[in]  ptrOpenCfg
 *      Pointer to the open configuration
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveOpenCallbackFxn(MMWave_OpenCfg* ptrOpenCfg)
{
    return;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been closed.
 *
 *  @retval
 *      Not applicable
 */
static void MmwDemo_mssMmwaveCloseCallbackFxn(void)
{
    return;
}
/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been started. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @param[in]  ptrCalibrationCfg
 *      Pointer to the calibration configuration
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStartCallbackFxn(MMWave_CalibrationCfg* ptrCalibrationCfg)
{
    /* Post an event to main data path task. 
       This function in only called when mmwave_start() is called on DSS */
    gMmwMssMCB.stats.datapathStartEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Application registered callback function which is invoked the mmWave link on BSS
 *      has been stopped. This is applicable only for the XWR16xx. The BSS can be configured
 *      only by the MSS *or* DSS. The callback API is triggered on the remote execution
 *      domain (which did not configure the BSS)
 *
 *  @retval
 *      Not applicable
 */
void MmwDemo_mssMmwaveStopCallbackFxn(void)
{
    /* Possible sceanarios:
       1. CLI sensorStop command triggers mmwave_stop() to be called from MSS 
       2. In case of Error, mmwave_stop() will be triggered either from MSS or DSS
     */
    gMmwMssMCB.stats.datapathStopEvt ++;
}

/**
 *  @b Description
 *  @n
 *      Function to send a message to peer through Mailbox virtural channel 
 *
 *  @param[in]  message
 *      Pointer to the MMW demo message.  
 *
 *  @retval
 *      Success    - 0
 *      Fail       < -1 
 */
int32_t MmwDemo_mboxWrite(MmwDemo_message     * message)
{
    int32_t                  retVal = -1;
    
    retVal = Mailbox_write (gMmwMssMCB.peerMailbox, (uint8_t*)message, sizeof(MmwDemo_message));
    if (retVal == sizeof(MmwDemo_message))
    {
        retVal = 0;
    }
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The Task is used to handle  the mmw demo messages received from
 *      Mailbox virtual channel.
 *
 *  @param[in]  arg0
 *      arg0 of the Task. Not used
 *  @param[in]  arg1
 *      arg1 of the Task. Not used
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_mboxReadTask(UArg arg0, UArg arg1)
{
    MmwDemo_message      message;
    int32_t              retVal = 0;
    uint32_t             timeStart;

    /* wait for new message and process all the messages received from the peer */
    while(1)
    {
        Semaphore_pend(gMmwMssMCB.mboxSemHandle, BIOS_WAIT_FOREVER);

        /* Read the message from the peer mailbox: We are not trying to protect the read
         * from the peer mailbox because this is only being invoked from a single thread */
        retVal = Mailbox_read(gMmwMssMCB.peerMailbox, (uint8_t*)&message, sizeof(MmwDemo_message));
        if (retVal < 0)
        {
            /* Error: Unable to read the message. Setup the error code and return values */
            System_printf ("Error: Mailbox read failed [Error code %d]\n", retVal);
        }
        else if (retVal == 0)
        {
            /* We are done: There are no messages available from the peer execution domain. */
            continue;
        }
        else
        {
            /* Flush out the contents of the mailbox to indicate that we are done with the message. This will
             * allow us to receive another message in the mailbox while we process the received message. */
            Mailbox_readFlush (gMmwMssMCB.peerMailbox);

            /* Process the received message: */
            switch (message.type)
            {
                case MMWDEMO_DSS2MSS_DETOBJ_READY:
                    /* Got detetced objectes , shipped out through UART */

                	gMmwMssMCB.mssDataPathObj.inputInfoBuffSize = message.body.detObj.detObjOutsize;

					memcpy((void *)&(gMmwMssMCB.mssDataPathObj.inputInfo),
                			(uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress, SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
							message.body.detObj.detObjOutsize);

                	timeStart = Cycleprofiler_getTimeStamp();
                	gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = 1.f;
                	MmwDemo_transmitProcessedOutput();
                	gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = 2.f;
                	gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() -  timeStart)/(float)R4F_CLOCK_MHZ);

#if 0
                    /*UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)SOC_translateAddress(message.body.detObj.detObjOutAddress,
                                                           SOC_TranslateAddr_Dir_FROM_OTHER_CPU,NULL),
                            message.body.detObj.detObjOutsize);*/
					gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.copyResultsTimeCurrInusec;

					timeStart = Cycleprofiler_getTimeStamp();
                    UART_writePolling (gMmwMssMCB.loggingUartHandle,
                            (uint8_t*)&(gMmwMssMCB.mssDataPathObj.inputInfo),
							gMmwMssMCB.mssDataPathObj.inputInfoBuffSize);
					gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec = ((float)(Cycleprofiler_getTimeStamp() - timeStart))/(float)R4F_CLOCK_MHZ;
					if ((gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > 0) && (gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec > gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec))
						gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeMaxInusec = gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec;
#endif

                    /* Send a message to MSS to log the output data */
                    memset((void *)&message, 0, sizeof(MmwDemo_message));

                    message.type = MMWDEMO_MSS2DSS_DETOBJ_SHIPPED;

                    if (MmwDemo_mboxWrite(&message) != 0)
                    {
                        System_printf ("Error: Mailbox send message id=%d failed \n", message.type);
                    }
                    break;
                default:
                {
                    /* Message not support */
                    System_printf ("Error: unsupport Mailbox message id=%d\n", message.type);
                    break;
                }
            }
        }
    }
}


/**
 *  @b Description
 *  @n
 *      This function is a callback funciton that invoked when a message is received from the peer.
 *
 *  @param[in]  handle
 *      Handle to the Mailbox on which data was received
 *  @param[in]  peer
 *      Peer from which data was received
 
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mboxCallback
(
    Mbox_Handle  handle,
    Mailbox_Type    peer
)
{
    /* Message has been received from the peer endpoint. Wakeup the mmWave thread to process
     * the received message. */
    Semaphore_post (gMmwMssMCB.mboxSemHandle);
}

void MmwDemo_printHeapStats()
{
    Memory_Stats            startMemoryStats;

       HeapMem_getStats (heap0, &startMemoryStats);
       System_printf ("Debug: System Heap (TCM): Size: %d, Used = %d, Free = %d bytes\n", startMemoryStats.totalSize, startMemoryStats.totalSize - startMemoryStats.totalFreeSize, startMemoryStats.totalFreeSize);
}


/**
 *  @b Description
 *  @n
 *      Function to do Data Path Configuration on MSS. After received Configuration from
 *    CLI, this function will start the system configuration process, inclucing mmwaveLink, BSS
 *    and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathConfig(void)
{
    int32_t  errCode;
    radarOsal_heapConfig heapconfig[SOC_XWR16XX_MSS_MAXNUMHEAPS];
    /* Setup the calibration frequency: */    
  //  gMmwMssMCB.cfg.ctrlCfg.freqLimitLow  = 760U;
   // gMmwMssMCB.cfg.ctrlCfg.freqLimitHigh = 810U;
    
    /* Configure the mmWave module: */
    /*if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
        return -1;
    }
*/
    if (gMmwMssMCB.isMMWaveOpen == false)
        {
            /* NO: Setup the calibration frequency: */
            gMmwMssMCB.cfg.openCfg.freqLimitLow  = 760U;
            gMmwMssMCB.cfg.openCfg.freqLimitHigh = 810U;

            /* Open the mmWave module: */
            if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
            {
                System_printf ("Error: MMWDemoMSS mmWave open configuration failed [Error code %d]\n", errCode);
                return -1;
            }

            /* mmWave module has been opened. */
            gMmwMssMCB.isMMWaveOpen = true;
        }
        /* Configure the mmWave module: */
        if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
        {
            System_printf ("Error: MMWDemoMSS mmWave Configuration failed [Error code %d]\n", errCode);
            return -1;
        }

    // heap init
	memset(heapconfig, 0, sizeof(heapconfig));
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapType 	= 	RADARMEMOSAL_HEAPTYPE_DDR_CACHED;
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapAddr   = 	NULL; //(int8_t *) &gMmwL3[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].heapSize   = 	SOC_XWR16XX_MSS_L3RAM_BUFF_SIZE;
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchAddr= 	NULL; 	/* not DDR scratch for TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_DDR_CACHED].scratchSize= 	0; 	/* not DDR scratch for TM demo  */

	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL2;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapAddr   		= 	(int8_t *) &gMmwL2[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].heapSize   		= 	SOC_XWR16XX_MSS_L2_BUFF_SIZE;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchAddr   	= 	NULL;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL2].scratchSize   	= 	0;

	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapType 			= 	RADARMEMOSAL_HEAPTYPE_LL1;
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapAddr   		= 	NULL; /* not used as L1 heap in TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].heapSize   		= 	0;
 	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchAddr   	= 	(int8_t *)&gMmwL1Scratch[0];
	heapconfig[RADARMEMOSAL_HEAPTYPE_LL1].scratchSize   	= 	SOC_XWR16XX_MSS_L1_SCRATCH_SIZE;

	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapType 		= 	RADARMEMOSAL_HEAPTYPE_HSRAM;
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapAddr   		= 	NULL;
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].heapSize   		= 	0;
 	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchAddr   	= 	NULL; 	/* not HSRAM scratch for TM demo  */
	heapconfig[RADARMEMOSAL_HEAPTYPE_HSRAM].scratchSize   	= 	0;    /* not HSRAM scratch for TM demo  */

	if(radarOsal_memInit(&heapconfig[0], SOC_XWR16XX_MSS_MAXNUMHEAPS) == RADARMEMOSAL_FAIL)
	{
		System_printf("Error: radarOsal_memInit fail\n");
        return -1;
	}
	memset(&(gMmwMssMCB.mssDataPathObj.cycleLog), 0, sizeof(gMmwMssMCB.mssDataPathObj.cycleLog));

	MmwDemo_printHeapStats();

    radarOsal_memDeInit();

    System_printf ("Debug: MMWDemoMSS mmWave  config succeeded \n");    

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will 
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStart(void)
{
    int32_t    errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    /* Initialize the calibration configuration: */
    memset ((void*)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));

    /* Populate the calibration configuration: */
  /*  calibrationCfg.enableCalibration    = true;
    calibrationCfg.enablePeriodicity    = true;
    calibrationCfg.periodicTimeInFrames = 10U;*/

    /* Start the mmWave module: The configuration has been applied successfully. */
  //  if (MMWave_start (gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
   // {
        /* Error: Unable to start the mmWave control */
   //     System_printf ("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
  //      return -1;
  //  }

    /* Populate the calibration configuration: */
   // calibrationCfg.dfeDataOutputMode                          = MMWave_DFEDataOutputMode_FRAME;
    calibrationCfg.dfeDataOutputMode                          = 
        gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

        /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start (gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
            /* Error: Unable to start the mmWave control */
        System_printf ("Error: MMWDemoMSS mmWave Start failed [Error code %d]\n", errCode);
        return -1;
    }
    System_printf ("Debug: MMWDemoMSS mmWave Start succeeded \n");
    return 0;
}


/**
 *  @b Description
 *  @n
 *      Function to do Data Path Start on MSS. After received SensorStart command, MSS will 
 *    start all data path componets including mmwaveLink, BSS and DSS.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssDataPathStop(void)
{
    int32_t    errCode;
    
    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to start the mmWave control */
        System_printf ("Error: MMWDemoMSS mmWave Stop failed [Error code %d]\n", errCode);
        return -1;
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      The task is used to provide an execution context for the mmWave
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
            System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
    }
}

/**
 *  @b Description
 *  @n
 *      The task is used to process data path events
 *      control task
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssCtrlPathTask(UArg arg0, UArg arg1)
{
    UInt          event;

    /* Data Path management task Main loop */
    while (1)
    {
        event = Event_pend(gMmwMssMCB.eventHandle, 
                          Event_Id_NONE, 
                          MMWDEMO_CLI_EVENTS | MMWDEMO_BSS_FAULT_EVENTS,
                          BIOS_WAIT_FOREVER); 

        /************************************************************************
         * CLI event:: SensorStart
         ************************************************************************/

        if(event & MMWDEMO_CLI_SENSORSTART_EVT)
        {
            System_printf ("Debug: MMWDemoMSS Received CLI sensorStart Event\n");
        
            /* Setup the data path: */
            if(MmwDemo_mssDataPathConfig () < 0)
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: SensorStop
         ************************************************************************/
        if(event & MMWDEMO_CLI_SENSORSTOP_EVT)
        {
            if (MmwDemo_mssDataPathStop() < 0 )
            {
                continue;
            }
        }

        /************************************************************************
         * CLI event:: Framestart
         ************************************************************************/
        if(event & MMWDEMO_CLI_FRAMESTART_EVT)
        {
            if (MmwDemo_mssDataPathStart() < 0 )
            {
                continue;
            }
        }

        /************************************************************************
         * BSS event:: CPU fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_CPUFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }

        /************************************************************************
         * BSS event:: ESM fault
         ************************************************************************/
        if(event & MMWDEMO_BSS_ESMFAULT_EVT)
        {
            DebugP_assert(0);
            break;
        }
    }

    System_printf("Debug: MMWDemoDSS Data path exit\n");
}

/**
 *  @b Description
 *  @n
 *      System Initialization Task which initializes the various
 *      components in the system.
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_mssInitTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    Mailbox_Config      mboxCfg;
    Error_Block         eb;

    /* Debug Message: */
    System_printf("Debug: MMWDemoMSS Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Pinmux setting */

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN5_PADBE, SOC_XWR16XX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINN4_PADBD, SOC_XWR16XX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINF14_PADAJ, SOC_XWR16XX_PINF14_PADAJ_MSS_UARTB_TX);

    /* Setup the PINMUX to bring out the DSS UART */
    Pinmux_Set_OverrideCtrl(SOC_XWR16XX_PINP8_PADBM, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWR16XX_PINP8_PADBM, SOC_XWR16XX_PINP8_PADBM_DSS_UART_TX);

    /* Initialize the UART */
    UART_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /*****************************************************************************
     * Open & configure the drivers:
     *****************************************************************************/

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency  = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate        = gMmwMssMCB.cfg.commandBaudRate;
    uartParams.isPinMuxDone    = 1U;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Command UART Instance\n");
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: MMWDemoMSS Unable to open the Logging UART Instance\n");
        return;
    }

    /*****************************************************************************
     * Creating communication channel between MSS & DSS
     *****************************************************************************/

    /* Create a binary semaphore which is used to handle mailbox interrupt. */
    Semaphore_Params_init(&semParams);
    semParams.mode             = Semaphore_Mode_BINARY;
    gMmwMssMCB.mboxSemHandle = Semaphore_create(0, &semParams, NULL);

    /* Setup the default mailbox configuration */
    Mailbox_Config_init(&mboxCfg);

    /* Setup the configuration: */
    mboxCfg.chType       = MAILBOX_CHTYPE_MULTI;
    mboxCfg.chId         = MAILBOX_CH_ID_0;
    mboxCfg.writeMode    = MAILBOX_MODE_BLOCKING;
    mboxCfg.readMode     = MAILBOX_MODE_CALLBACK;
    mboxCfg.readCallback = &MmwDemo_mboxCallback;

    /* Initialization of Mailbox Virtual Channel  */
    gMmwMssMCB.peerMailbox = Mailbox_open(MAILBOX_TYPE_DSS, &mboxCfg, &errCode);
    if (gMmwMssMCB.peerMailbox == NULL)
    {
        /* Error: Unable to open the mailbox */
        System_printf("Error: Unable to open the Mailbox to the DSS [Error code %d]\n", errCode);
        return;
    }

    /* Create task to handle mailbox messges */
    Task_Params_init(&taskParams);
    taskParams.stackSize = 16*1024;
    Task_create(MmwDemo_mboxReadTask, &taskParams, NULL);

    /*****************************************************************************
     * Create Event to handle mmwave callback and system datapath events 
     *****************************************************************************/
    /* Default instance configuration params */
    Error_init(&eb);
    gMmwMssMCB.eventHandle = Event_create(NULL, &eb);
    if (gMmwMssMCB.eventHandle == NULL) 
    {
        DebugP_assert(0);
        return ;
    }

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration for mmwave library: */
    initCfg.domain               = MMWave_Domain_MSS;
    initCfg.socHandle            = gMmwMssMCB.socHandle;
    initCfg.eventFxn             = MmwDemo_mssMmwaveEventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel  = CRC_Channel_CH1;
    initCfg.cfgMode                = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode         = MMWave_ExecutionMode_COOPERATIVE;
    initCfg.cooperativeModeCfg.cfgFxn = MmwDemo_mssMmwaveConfigCallbackFxn;
    initCfg.cooperativeModeCfg.openFxn  = MmwDemo_mssMmwaveOpenCallbackFxn;
    initCfg.cooperativeModeCfg.closeFxn = MmwDemo_mssMmwaveCloseCallbackFxn;
    initCfg.cooperativeModeCfg.startFxn = MmwDemo_mssMmwaveStartCallbackFxn;
    initCfg.cooperativeModeCfg.stopFxn = MmwDemo_mssMmwaveStopCallbackFxn;        
        
    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf("Error: MMWDemoMSS mmWave Control Initialization failed [Error code %d]\n", errCode);
        return;
    }
    System_printf("Debug: MMWDemoMSS mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = MMWave_sync (gMmwMssMCB.ctrlHandle , &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the mmWave control module */
            System_printf ("Error: MMWDemoMSS mmWave Control Synchronization failed [Error code %d]\n", errCode);
            return;
        }
        if (syncStatus == 1)
        {
            /* Synchronization acheived: */
            break;
        }
        /* Sleep and poll again: */
        Task_sleep(1);
    }  

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 6;
    taskParams.stackSize = 5*1024;
    Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Create a data path management task to handle data Path events
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority = 4;
    taskParams.stackSize = 3*1024;
    Task_create(MmwDemo_mssCtrlPathTask, &taskParams, NULL);
    
    /*****************************************************************************
     * At this point, MSS and DSS are both up and synced. Configuration is ready to be sent.
     * Start CLI to get configuration from user
     *****************************************************************************/
    MmwDemo_CLIInit();

    /*****************************************************************************
     * Benchmarking Count init
     *****************************************************************************/
    /* Configure banchmark counter */
    Pmu_configureCounter(0, 0x11, FALSE);
    Pmu_startCounter(0);
   
    return;
}

/**
 *  @b Description
 *  @n
 *      Entry point into the Millimeter Wave Demo
 *
 *  @retval
 *      Not Applicable.
 */
int main (void)
{
    Task_Params     taskParams;
    int32_t         errCode;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: */
    ESM_init(0U); //dont clear errors as TI RTOS does it

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MCB));

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    gMmwMssMCB.socHandle   = SOC_init (&socCfg, &errCode);
    if (gMmwMssMCB.socHandle  == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        return -1;
    }

    /* Initialize the DEMO configuration: */
    gMmwMssMCB.cfg.sysClockFrequency = MSS_SYS_VCLK;
    gMmwMssMCB.cfg.loggingBaudRate   = 921600;
    gMmwMssMCB.cfg.commandBaudRate   = 115200;

    Cycleprofiler_init();

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the Millimeter Wave Demo\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    taskParams.priority = 3;
    Task_create(MmwDemo_mssInitTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}
