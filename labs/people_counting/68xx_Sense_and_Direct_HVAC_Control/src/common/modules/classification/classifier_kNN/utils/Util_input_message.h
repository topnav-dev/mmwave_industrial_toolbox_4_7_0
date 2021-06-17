/**
 *   @file  Util_input_message.h
 *
 *   @brief
 *      Header file for input message definition. If on target, use the existing defs.
 *
 * Copyright (C) 2017 Texas Instruments Incorporated - http://www.ti.com/ 
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
#ifndef UTIL_INPUT_MESSAGE_H
#define UTIL_INPUT_MESSAGE_H


#ifdef CCS
/*!
 * @brief
 *  Message types used in Millimeter Wave Demo for the communication between
 *  target and host, and also for Mailbox communication
 *  between MSS and DSS on the XWR16xx platform. Message types are used to indicate
 *  different type detection information sent out from the target.
 *
 */
typedef enum MmwDemo_output_message_type_e
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

    /*! @brief   Stats information */
    MMWDEMO_OUTPUT_MSG_STATS,

    MMWDEMO_OUTPUT_MSG_MAX
} MmwDemo_output_message_type;

/**
 * @brief
 *  Message for reporting detected objects from data path.
 *
 * @details
 *  The structure defines the message body for detected objects from from data path.
 */
typedef struct MmwDemo_output_message_tl_t
{
    /*! @brief   TLV type */
    uint32_t    type;

    /*! @brief   Length in bytes */
    uint32_t    length;

} MmwDemo_output_message_tl;

/*!
 * @brief
 * Structure holds the message body for the  Point Cloud
 *
 * @details
 * For each detected point, we report range, azimuth, and doppler
 */
typedef struct MmwDemo_output_message_point_t
{
    /*! @brief Detected point range, in m */
    float		range;
    /*! @brief Detected point azimuth, in rad */
    float		azimuth;
    /*! @brief Detected point doppler, in m/s */
    float		doppler;
    /*! @brief Range detection SNR, linear */
    float       snr;

} MmwDemo_output_message_point;

typedef struct MmwDemo_output_message_pointCloud_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_point    point[1];
} MmwDemo_output_message_pointCloud;


/*!
 * @brief
 * Structure holds the target features
 *
 * @details
 * For each detected target, we report position, velocity, and measurement error covariance
 */
typedef struct MmwDemo_output_message_target_t
{
    /*! @brief   tracking ID */
    uint32_t    tid;
    /*! @brief   Detected target X coordinate, in m */
    float		posX;
    /*! @brief   Detected target Y coordinate, in m */
    float		posY;
    /*! @brief   Detected target X velocity, in m/s */
    float		velX;
    /*! @brief   Detected target Y velocity, in m/s */
    float		velY;
    /*! @brief   Detected target X acceleration, in m/s2 */
    float       accX;
    /*! @brief   Detected target Y acceleration, in m/s2 */
    float       accY;

    /*! @brief   Target Error covarience matrix, [3x3 float], in row major order, range, azimuth, doppler */
    float		ec[9];
    float       g;

} MmwDemo_output_message_target;

/*!
 * @brief
 * Structure holds the message body for the Target List
 *
 * @details
 * For each detected target, we report position, velocity, and measurement error covariance
 */
typedef struct MmwDemo_output_message_targetList_t
{
    MmwDemo_output_message_tl       header;
    MmwDemo_output_message_target   target[1];
} MmwDemo_output_message_targetList;

/*!
 * @brief
 * Structure holds the message body for the Target Index.
 * This TLV is used to correlate each point in the point cloud with target
 *
 * @details
 * For each detected point, we report target index
 */
typedef struct MmwDemo_output_message_targetIndex_t
{
    MmwDemo_output_message_tl       header;
    uint8_t                         index[1];
} MmwDemo_output_message_targetIndex;
#else
#include <common/mmw_output.h>
#endif

#endif //UTIL_INPUT_MESSAGE_H
