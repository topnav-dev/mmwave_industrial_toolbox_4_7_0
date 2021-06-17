/**
 *   @file  pcount3D_output.h
 *
 *   @brief
 *      This is the interface/message header file for the 3D people counting Demo on DSS.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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
#ifndef PCOUNT3DDEMO_OUTPUT_H
#define PCOUNT3DDEMO_OUTPUT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/common/sys_common.h>
#include <common/src/dpc/capon3d/objectdetection.h>

/** @brief Output packet length is a multiple of this value, must be power of 2*/
#define MMWDEMO_OUTPUT_MSG_SEGMENT_LEN 32

/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR18xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum Pcount3DDemo_output_message_type_e
{
    /*! @brief   List of detected points */
    MMWDEMO_OUTPUT_MSG_DETECTED_POINTS = 1,

    /*! @brief   Range profile */
    MMWDEMO_OUTPUT_MSG_RANGE_PROFILE,

    /*! @brief   Noise floor profile */
    MMWDEMO_OUTPUT_MSG_NOISE_PROFILE,

    /*! @brief   Samples to calculate static azimuth  heatmap */
    MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP,

    /*! @brief   Range/Doppler detection matrix */
    MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP,

    /*! @brief   Point Cloud - Array of detected points (range/angle/doppler) */
    MMWDEMO_OUTPUT_MSG_POINT_CLOUD,

    /*! @brief   Target List - Array of detected targets (position, velocity, error covariance) */
    MMWDEMO_OUTPUT_MSG_TARGET_LIST,

    /*! @brief   Target List - Array of target indices */
    MMWDEMO_OUTPUT_MSG_TARGET_INDEX,

    /*! @brief   Classifier Output -- Array of target indices and tags */
    MMWDEMO_OUTPUT_MSG_CLASSIFIER_OUTPUT,

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    /*! @brief   Presence information */
    MMWDEMO_OUTPUT_PRESENCE_IND,

    MMWDEMO_OUTPUT_MSG_MAX
} Pcount3DDemo_output_message_type;

/*!
 * @brief
 *  Message header for reporting detection information from data path.
 *
 * @details
 *  The structure defines the message header.
 */
typedef struct Pcount3DDemo_output_message_header_t
{
    /*! @brief   Output buffer magic word (sync word). It is initialized to  {0x0102,0x0304,0x0506,0x0708} */
    uint16_t    magicWord[4];

    /*! brief   Version: : MajorNum * 2^24 + MinorNum * 2^16 + BugfixNum * 2^8 + BuildNum   */
    uint32_t     version;

    /*! @brief   Total packet length including header in Bytes */
    uint32_t    totalPacketLen;

    /*! @brief   platform type */
    uint32_t    platform;

    /*! @brief   Frame number */
    uint32_t    frameNumber;

    /*! @brief   For Advanced Frame config, this is the sub-frame number in the range
     * 0 to (number of subframes - 1). For frame config (not advanced), this is always
     * set to 0. */
    uint32_t    subFrameNumber;

    /*! @brief Detection Layer timing */
    uint32_t    chirpProcessingMargin;
    uint32_t    frameProcessingTimeInUsec;

    /*! @brief Localization Layer Timing */
    uint32_t    trackingProcessingTimeInUsec;
    uint32_t    uartSendingTimeInUsec;


    /*! @brief   Number of TLVs */
    uint16_t    numTLVs;

    /*! @brief   check sum of the header */
    uint16_t    checkSum;

} Pcount3DDemo_output_message_header;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct Pcount3DDemo_output_message_stats_t
{
    /*! @brief   Interframe processing time in usec */
    uint32_t     interFrameProcessingTime;

    /*! @brief   Transmission time of output detection information in usec */
    uint32_t     transmitOutputTime;

    /*! @brief   Interframe processing margin in usec */
    uint32_t     frameProcessingTimeInUsec;

    /*! @brief   Interchirp processing margin in usec */
    uint32_t     interChirpProcessingMargin;

    /*! @brief   CPU Load (%) during active frame duration */
    uint32_t     activeFrameCPULoad;

    /*! @brief   CPU Load (%) during inter frame duration */
    uint32_t     interFrameCPULoad;
} Pcount3DDemo_output_message_stats;


/**
 * @brief
 *  DSS stores demo output and stats in shared memory.
 */
// typedef struct Pcount3DDemo_OUTPUT_t
//{
    /*! @brief   DPC execution result */
//    DPC_ObjectDetection_ExecuteResult result;

//} Pcount3DDemo_OUTPUT;


/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path.
 */
typedef struct Pcount3DDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

} Pcount3DDemo_output_message_tl;


/*!
 * @brief
 * Structure holds the message body to UART for the  Point Cloud
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler
 */
typedef struct Pcount3DDemo_output_message_UARTpoint_t
{
    /*! @brief Detected point elevation, in number of azimuthUnit */
    int8_t      elevation;
    /*! @brief Detected point azimuth, in number of azimuthUnit */
    int8_t      azimuth;
    /*! @brief Detected point doppler, in number of dopplerUnit */
    int16_t      doppler;
    /*! @brief Detected point range, in number of rangeUnit */
    uint16_t        range;
    /*! @brief Range detection SNR, in number of snrUnit */
    uint16_t       snr;

} Pcount3DDemo_output_message_UARTpoint;


/*!
 * @brief
 * Structure holds the message body for the  Point Cloud shared by LLSPC, tracker and classifier
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler
 */
typedef struct Pcount3DDemo_output_message_point_t
{
    /*! @brief Detected point range, in m */
    float       range;
    /*! @brief Detected point azimuth, in radians */
    float       azimuth;
    /*! @brief Detected point elevation, in radians */
    float       elevation;
    /*! @brief Detected point doppler, in m/s */
    float       doppler;
    /*! @brief Range detection SNR, linear */
    float       snr;

} Pcount3DDemo_output_message_point;

/*!
 * @brief
 * Structure holds the message body for the  Point Cloud units
 *
 * @details
 * Reporting units for range, azimuth, and doppler
 */
typedef struct Pcount3DDemo_output_message_point_uint_t
{
    /*! @brief elevation  reporting unit, in radians */
    float       elevationUnit;
    /*! @brief azimuth  reporting unit, in radians */
    float       azimuthUnit;
    /*! @brief Doppler  reporting unit, in m/s */
    float       dopplerUnit;
    /*! @brief range reporting unit, in m */
    float       rangeUnit;
    /*! @brief SNR  reporting unit, linear */
    float       snrUint;

} Pcount3DDemo_output_message_point_unit;

typedef struct MmwDemo_output_message_UARTpointCloud_t
{
    Pcount3DDemo_output_message_tl       header;
    Pcount3DDemo_output_message_point_unit pointUint;
    Pcount3DDemo_output_message_UARTpoint    point[MAX_RESOLVED_OBJECTS_PER_FRAME];
} Pcount3DDemo_output_message_UARTpointCloud;

typedef struct MmwDemo_output_message_pointCloud_t
{
    Pcount3DDemo_output_message_tl       header;
    Pcount3DDemo_output_message_point    *point;
} Pcount3DDemo_output_message_pointCloud;

#ifdef __cplusplus
}
#endif

#endif /* PCOUNT3DDEMO_OUTPUT_H */
