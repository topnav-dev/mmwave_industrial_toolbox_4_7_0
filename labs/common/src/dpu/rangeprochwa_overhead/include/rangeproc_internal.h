/*
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
/**
 *   @file  rangeproc_internal.h
 *
 *   @brief
 *      Includes common definitions for rangeProcHWA and rangeProcDSP.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROC_INTERNAL_H
#define RANGEPROC_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Rangeproc supported Radar cube layout format
 *
 * @details
 *  The enumeration describes the radar cube layout format
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 */
typedef enum rangeProcRadarCubeLayoutFmt_e
{
    /*! @brief  Data layout: range-Doppler-TxAnt - RxAnt */
    rangeProc_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt,

    /*! @brief  Data layout: TxAnt->doppler->RxAnt->range */
    rangeProc_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE
}rangeProcRadarCubeLayoutFmt;

/**
 * @brief
 *  Data path common parameters needed by RangeProc
 *
 * @details
 *  The structure is used to hold the data path parameters used by both rangeProcHWA and rangeProdDSP DPUs.
 *
 *  \ingroup DPU_RANGEPROC_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct rangeProc_dpParams_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief  Number of chirps for Doppler computation purposes. */
    uint16_t    numDopplerChirps;
}rangeProc_dpParams;

#ifdef __cplusplus
}
#endif

#endif
