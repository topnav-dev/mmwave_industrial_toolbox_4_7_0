/*
 * gtrack_filter.h
 *
 *  Created on: Jul 3, 2019
 *      Author: a0232274
 */

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
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/HwiP.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>

#include <common/mmw_output.h>


#ifndef GTRACK_FILTER_H_
#define GTRACK_FILTER_H_

typedef struct trackHist_t {
    int32_t tid;
    float initX;
    float initY;
    float initZ;
    float maxNetDisplacement;
    int8_t classHumanCount;
    int8_t classNHCount;
    int32_t finalDecision;
    uint8_t locked;
    MmwDemo_output_message_target * target;
}trackHist;

float calculateUniqueness(uint32_t tid, uint8_t* indexes, uint8_t* unique, uint16_t numPoints);

void calculateDisplacement(uint32_t tid, trackHist * trackList);

void checkTrackSwap(uint32_t tid, trackHist * trackList, MmwDemo_output_message_target * target, uint16_t numTracks);

uint16_t classifierLowPassFilter(uint32_t tid, MmwDemo_output_message_classifierResults * classOut, trackHist * trackList, uint16_t numTracks);

void updateClassifierOutput(MmwDemo_output_message_classifierResults * classOut, MmwDemo_output_message_target * targets, uint8_t * indexes, uint8_t * unique, trackHist * trackList, uint16_t numPoints, uint16_t numTracks);

#endif /* GTRACK_FILTER_H_ */
