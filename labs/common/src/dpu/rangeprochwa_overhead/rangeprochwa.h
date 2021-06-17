/*
 *  
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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
 *   @file  rangeprochwa.h
 *
 *   @brief
 *      Implements range processing functionality using HWA.
 */
/**
 *  @page dpu_rangehwa RangeProcHWA
 * [TOC]
 *  @section toplevel_hwa Top Level Design
 *
 *  Range FFT processing is done by HWA hardware. Based on the RF parameters, rangeProcHWA configures
 *  hardware accelerator FFT engine accordingly. It also configures data input and output EDMA channels to
 *  bring data in and out of range Processing memory.
 *
 *  HWA FFT process is triggered by hardware based trigger -"chirp data available" which is hooked up to HWA internally in hardware.
 *
 *  After FFT processing is done, HWA generates interrupt to rangeProcHWA DPU, at the same time triggers EDMA data
 *  output channel to copy FFT results to radarCube in configured format. EDMA interrupt done interrupt is triggered by EDMA hardware
 *  after the copy is completed.
 *
 *  DC antenna signal calibration and removal is built into the rangeProcHWA Module.
 *  If the feature is enabled, it is done after all chirps (in the frame) FFT
 *  processing is completed.
 *
 *  The following diagram shows the top level design for rangeProcHWA.
 *
 *  @image html rangeprochwa_toplevel.png "rangeProcHWA Top Level"
 *
 *  The following diagram shows a general timing diagram for a 3 tx antenna MIMO
 *  case but can be imagined to hold for 1 or 2 tx cases as well. All use cases
 *  described in later sections have this general timing flow.
 *
 *  @image html datapath_1d_timing.png "Timing Diagram"
 *
 *  @section config_hwa Data Interface Parameter Range
 *
 *  Here are supported ADCBuf and radarCube interface configurations:
 *
 *  ADCBuf Data Interface
 *----------------------
 *
 *   Parameter | Supported value
 *  :----------|:----------------:
 *   dataFmt | @ref DPIF_DATAFORMAT_COMPLEX16_IMRE  only
 *   interleave|interleave (@ref DPIF_RXCHAN_INTERLEAVE_MODE) and non-interleave (@ref DPIF_RXCHAN_NON_INTERLEAVE_MODE)
 *   numChirpsPerChirpEvent|1 ONLY
 *   numRxAntennas|1, 2 or 4
 *   numAdcSamples|64 - 1024
 *
 *
 *  Radar Cube Data Interface
 *----------------------
 *
 *   Parameter | Supported value
 *  :----------|:----------------:
 *   datafmt | @ref DPIF_RADARCUBE_FORMAT_1 and @ref DPIF_RADARCUBE_FORMAT_2 only
 *   numTxAntennas|1, 2 and 3
 *   numRangeBins|64 - 1024
 *   numChirpsPerFrame|As AdcBuf and HWA memory permit, numChirpsPerFrame/numTxAntennas should be an even number
 *
 *  @note  For 1024 Range bins with 1 TX , 4 RX antenna and @ref DPIF_RADARCUBE_FORMAT_1,
 *         it is not currently supported due to EDMA address jump limitation (jump < 32768).
 *  @note  For 1024 Range bins with 3 TX , 4 RX antenna and @ref DPIF_RADARCUBE_FORMAT_1,
 *         it is not currently supported due to EDMA address jump limitation (jump < 32768).
 *  @note  Due to the Ping/Pong scheme implemented in rangeProcHWA DPU, numChirpsPerFrame/numTxAntennas should be an even number.
 *
 *  @section input_hwa Data Input
 *
 *  There are two HWA input mode supported (@ref DPU_RangeProcHWA_InputMode_e),
 *  -  @ref DPU_RangeProcHWA_InputMode_ISOLATED : \n
 *     ADCBuf buffer and HWA memory are isolated in physical memory space.
 *     Data input EDMA channel is configured to transfer data from ADCBuf to HWA M0/M1 memory
 *     in ping/pong alternate order.
 *  -  @ref DPU_RangeProcHWA_InputMode_MAPPED :\n
 *     ADCBuf buffer and HWA memory are mapped. HWA can read ADC data directly. No copy is needed.
 *
 *  Depending on the RF configuration, the data in ADCBuf can be either interleaved or non-interleaved mode.
 *
 *  @section output_hwa Data Output
 *
 *  RangeProcHWA configures data output EDMA channels to transfer FFT results from HWA M2/M3 to radarCube memory.
 *
 *  The following two radarCube formats are supported in rangeProcHWA.
 *  - \ref DPIF_RADARCUBE_FORMAT_1
 *  - \ref DPIF_RADARCUBE_FORMAT_2
 *
 *  The following table describes the suppoted data input and output combinations:
 *
 *   Interleave Mode | DPIF_RADARCUBE_FORMAT_1 |  DPIF_RADARCUBE_FORMAT_2
 *  :----------|:----------------:|:-----------:
 *   interleave | NOT supported  | Supported
 *  non-interleave|Supported | Supported
 *
 *
 *
 *  @section process_hwa Data Processing
 *
 *  Range FFT processing is done by HWA hardware. As shown in the following diagram, after @ref DPU_RangeProcHWA_config is completed,
 *  in every frame, rangeProcHWA is triggered through @ref DPU_RangeProcHWA_control \n
 *  using command @ref DPU_RangeProcHWA_Cmd_triggerProc. If the hardware resources are overlaid with other modules,
 *  @ref DPU_RangeProcHWA_config can be called before the next frame start. \n
 *
 *  DC signal removal configuration  can be updated at inter-frame time before the next frame starts.
 *
 * @image html hwa_callflow.png "rangeProcHWA call flow"
 *
 *
 *  @subsection hwa_calibDC_Range Antenna coupling signature removal
 *
 *  This feature is controlled through configuration @ref DPU_RangeProc_CalibDcRangeSigCfg_t.
 *  The configuration can be sent to rangeProcHWA DPU through API @ref DPU_RangeProcHWA_config. \n
 *  The configuration can also be updated at runtime through control command @ref DPU_RangeProcHWA_Cmd_dcRangeCfg.
 *
 *  DC signal calibration and compensation is operated on radarCube directly by the CPU.
 *
 * @image html hwa_dcremoval.png "rangeProcHWA Antenna DC signal removal"
 *
 *
 *  @section hwa_usecase Use Cases and Implementation
 *
 *  This Section describes some of the internal implementation for a few use cases with combination of input/output configurations. \n
 *
 *  @subsection hwa_impl HWA param set Implementation
 *  RangeProcHWA always use 4 consecutive HWA param sets(@ref DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS)  starting from paramSetStartIdx,
 *  these params will be labeled PARAM_0, PARAM_1, PARAM_2 and PARAM_3 from now onwards for convenience
 *  of explanation and picture representation.
 *  Param sets are used in two parallel processing chain in ping/pong alternate manner.
 *  PARAM_0 and PARAM_1 are used for PING and PARAM_2 and PARAM_3 are used for PONG.
 *
 *  PARAM_0 and PARAM_2 need to be activated by triggers to protect internal data memory integrity. There are initially triggered
 *  by software when starting the very first chirp. Consequently they are triggered by data out hot signature channel.
 *  PARAM_1 and PARAM_3 are Range FFT param sets for PING and PONG respectively. In directly mapped input mode, they are triggered by
 *  frontend hardware when chirp data is available. In isolated input mode, they are triggered by data in hot signature.
 *  After range FFT processing is done in HWA, HWA param sets automatically trigger data out EDMA channel to copy data to radar Cube in desired
 *  radarCube format.
 *
 *  @subsection hwa_dataIn EDMA Data In Implementation
 *  When enabled, data input EDMA is constructed utilizing two EDMA channels. The first EDMA channel copies data from ADCBuf buffer to HWA memory.
 *  The second EDMA channel is chained to the first channel. It copies hot signature to trigger range FFT param sets.\n
 *  In summary, the data in EDMA requires the following resources:
 *
 *       - 1 EDMA channel with 1 shadow channel
 *       - 1 hot Signature channel with 1 shadow channel
 *
 *  @subsection hwa_dataOut EDMA Data Out Implementation
 *  Data out EDMA is responsible for copying data from HWA memory to radar Cube memory.
 *  It has two formats (@ref DPU_RangeProcHWA_EDMAOutputConfig_t).\n
 *  @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2 is only for the following configuration: \n
 *
 *   Setting | Note
 *  :------------|:-----------:
 *   @ref DPIF_RADARCUBE_FORMAT_1|  Radar Cube format 1
 *   numTxAntenna| 3 TX antenna
 *   @ref DPIF_RXCHAN_NON_INTERLEAVE_MODE|ADCBuf non interleave mode
 *
 *  All other configurations should use @ref DPU_RangeProcHWA_EDMAOutputConfigFmt1_t.
 *
 *  For EDMA dataOut Fmt1, it needs three EDMA channels, shown as follows:
 *
 *       Ping:
 *       - 1 EDMA channel with 1 shadow channel
 *
 *       Pong:
 *       - 1 EDMA channel with 1 shadow channel
 *
 *       Signature channel:
 *       - 1 HWA hot signature channel with 1 shadow channel
 *
 *  Ping and Pong EDMA channels are triggered by PARAM_1 and PARAM_3 FFT automatically. Both Ping/pong EDMA channels are chained to data out
 *  Hot Signature channel, which triggers HWA PARAM_0 AND PARAM_2 when EDMA channel copy is completed.
 *
 *  For EDMA dataOut Fmt2, it requires the following resources:\n
 *
 *       Ping:
 *       - 1 dummy EDMA channel with 3 shadow channels
 *       - 3 dataOut channel, each has a shadow channel
 *
 *       Pong:
 *       - 1 dummy EDMA channel with 3 shadow channels
 *       - 3 dataOut channel, each has a shadow channel
 *
 *       Signature channel:
 *       - 1 HWA hot signature channel with 1 shadow channel
 *
 *   The dummy EDMA channel is triggered by the HWA PARAM_1(trigger data out Ping EDMA channel) and PARAM_3(trigger data out pong EDMA channel)
 *   after FFT operation is completed.
 *   The dummy channel is linked to 3 shadow channels. The shadow channel is loaded to dummy channel in round robin order. Each dummy channel
 *   is chained to one of the 3 dataOut channel that have different source and destination address. Details are as follows:
 *
 *   Index | Source Address | Destination Address | sequence order in time
 *  :------:|:------------:|:-----------:|:------------------------:
 *   PING 1|  HWA M2  | TX1 | 1
 *   PING 2|  HWA M2  | TX3 | 3
 *   PING 3|  HWA M2  | TX2 | 5
 *   PONG 1|  HWA M3  | TX2 | 2
 *   PONG 2|  HWA M3  | TX1 | 4
 *   PONG 3|  HWA M3  | TX3 | 6
 *
 *
 *  @subsection hwa_usecase1 Interleaved RX channel data -> DPIF_RADARCUBE_FORMAT_2
 *
 *  This use case is for configuration with a directly mapped input buffer with interleaved ADC data. After Range FFT, data is saved in radar cube
 *  with format:@ref DPIF_RADARCUBE_FORMAT_2.
 *
 *  DataIn EDMA is NOT required, DataOut EDMA should be configured with format @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2.\n
 *
 *   Input params | Setting
 *  :------------|:-----------:
 *   InterleaveMode | Interleave
 *  HWA input mode|Mapped
 *  RadarCube format|DPIF_RADARCUBE_FORMAT_2
 *  numTxAnt| 1, 2, 3
 *
 * @image html interleave_to_radarcubefmt2.png "Interleaved data input to DPIF_RADARCUBE_FORMAT_2"
 *
 *   Above picture illustrates a case of three transmit antennas, chirping within the frame with
 *   repeating pattern of (Tx1,Tx3,Tx2).
 *   This is the 3D profile (velocity and x,y,z) case. There are 4 rx
 *   antennas, the samples of which are color-coded and labeled as 1,2,3,4 with
 *   unique coloring for each of chirps that are processed in ping-pong manner
 *   to parallelize accelerator and EDMA processing with sample acquisition from ADC.
 *   The hardware accelerator's parameter RAMs are setup to do FFT
 *   which operates on the input ADC ping and pong buffers to produce output in M2 and M3
 *   memories of the HWA.
 *   The processing needs to be given a kick every frame, this is done by the range DPU user
 *   by issuing the IOCTL command code @ref DPU_RangeProcHWA_Cmd_triggerProc (this
 *   invokes @ref rangeProcHWA_TriggerHWA) which activates the HWA's dummy
 *   params PARAM_0 (ping) and PARAM_2 (pong) which in turn activate the processing PARAMs
 *   PARAM_1 (ping) and PARAM_3 (pong), these are waiting on the ADC full signal.
 *   When ADC has samples to process in the ADC buffer Ping or Pong memories, the corresponding
 *   processing PARAM will trigger FFT calculation and transfer the FFT output into the M2 or M3 memories.
 *   Before ADC samples are sent to FFT engine, the configured FFT window is applied to them in the HWA.
 *   The completion of FFT triggers the Data Out Ping/Pong EDMAs which have been
 *   setup to do a copy with
 *   transposition from the M2/M3 memories to the L3 RAM (Radar Cube) as shown in the picture.
 *   This HWA-EDMA ping-pong processing is done @ref DPU_RangeProcHWA_StaticConfig_t::numChirpsPerFrame/2(ping/pong) times so
 *   that all chirps of the frame are processed. The EDMA
 *   is setup such that Data Out EDMAs are both chained to the Data Out Signature EDMA.
 *   So the Data Out Signature EDMA will trigger when the FFT result in M2/M3 memories
 *   is transferred to the radar cube. This signature EDMA is setup to trigger the
 *   dummy PARAMs. Even though the signature EDMA is a single EDMA channel, it is
 *   setup to alternate between the two dummy PARAMs so in effect when ping data out transfer
 *   is done, the ping dummy PARAM (PARAM_0) will be activated and when pong is done,
 *   the pong dummy PARAM (PARAM_2) will be activated.
 *   The signature EDMA is setup to give a completion interrupt after the last chirp
 *   which notifies software that range DPU processing is complete and user can trigger
 *   processing again for the next chirping period when the time comes.
 *   The shadow (link) PaRAMs of EDMA are used for reloading the PaRAMs so reprogramming
 *   is avoided. The blue arrows between EDMA blocks indicate linking and red arrows
 *   indicate chaining.
 *
 *   In further use cases, we do not describe the diagrams in such detail but
 *   the the general flow is similar. Details differ mostly in the programming
 *   of HWA and the amount of EDMA resources and their programming to
 *   handle the desired input and output formats.
 *
 *  @subsection hwa_usecase2 Non-Interleaved RX channel data -> DPIF_RADARCUBE_FORMAT_2
 *
 *  This use case is for configuration with a directly mapped input buffer with non-interleaved ADC data. After Range FFT, data is saved in radar cube
 *  with format:@ref DPIF_RADARCUBE_FORMAT_2.\n
 *  A conversion of data from non-interleaved format to interleaved format is done by PARAM_1 AND PARAM_3.
 *
 *  DataIn EDMA is NOT required, DataOut EDMA should be configured with format @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2.\n
 *
 *   Input params | Setting
 *  :------------|:-----------:
 *   InterleaveMode | Non-Interleave
 *  HWA input mode|Mapped
 *  RadarCube format|DPIF_RADARCUBE_FORMAT_2
 *  numTxAnt| 1, 2, 3
 *
 * @image html non-interleave_to_radarcubefmt2.png "Non-interleaved data input to DPIF_RADARCUBE_FORMAT_2"
 *
 *
 *
 *  @subsection hwa_usecase3 Non-Interleaved RX channel data(1 or 2 TX Antenna) -> DPIF_RADARCUBE_FORMAT_1
 *
 *  This use case is for configuration with a directly mapped input buffer with non-interleaved ADC data. After RangeFFT, data is saved in radar cube
 *  with format:@ref DPIF_RADARCUBE_FORMAT_1.
 *
 *  DataIn EDMA is NOT required, DataOut EDMA should be configured with format @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2.\n
 *  The diagram shows radarCube in 2 TX Antenna format. The output of Ping will be saved in TX1 section and output of Pong will be saved in TX2 section of the radar cube.
 *  For one TX antenna case, data from ping and pong data will be saved in TX1 radar cube in alternate order.
 *
 *   Input params | Setting
 *  :------------|:-----------:
 *   InterleaveMode | Non-Interleave
 *  HWA input mode|Mapped
 *  RadarCube format|DPIF_RADARCUBE_FORMAT_1
 *  numTxAnt| 1, 2
 *
 * @image html non-interleave_to_radarcubefmt1.png "Non-interleaved data input to DPIF_RADARCUBE_FORMAT_1"
 *
 *
 *
 *  @subsection hwa_usecase4 Isolated Non-Interleaved RX channel data(1 or 2 TX Antenna) -> DPIF_RADARCUBE_FORMAT_1
 *
 *  This use case is for configuration with isolated input buffer with non-interleaved ADC data. After RangeFFT, data is saved in radar cube
 *  with format:@ref DPIF_RADARCUBE_FORMAT_1. \n
 *
 *  DataIn EDMA is required, DataOut EDMA should be configured with format @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2  \n
 *  The diagram shows radarCube in 2 TX Antenna format. The output of Ping will be saved in TX1 section and output of Pong will be saved in TX2 section of the radar cube.
 *  For one TX antenna case, data from ping and pong will be saved in TX1 radar cube in alternate order.
 *
 *   Input params | Setting
 *  :------------|:-----------:
 *   InterleaveMode | Non-Interleave
 *  HWA input mode|Isolated
 *  RadarCube format|DPIF_RADARCUBE_FORMAT_1
 *  numTxAnt| 1, 2
 *
 * @image html isolated_non-interleave_to_radarcubefmt1.png "Isolated Non-interleaved data input to DPIF_RADARCUBE_FORMAT_1"
 *
 *
 *
 *  @subsection hwa_usecase5 Isolated Non-Interleaved RX channel data(3 TX Antenna) -> DPIF_RADARCUBE_FORMAT_1
 *
 *  This use case is for configuration with isolated input buffer with non-interleaved ADC data. After RangeFFT, data is saved in radar cube
 *  with format:@ref DPIF_RADARCUBE_FORMAT_1.\n
 *  DataIn EDMA is required, DataOut EDMA should be configured with format @ref DPU_RangeProcHWA_EDMAOutputConfigFmt1.\n
 *
 *  The output of Ping/Pong FFT results will be saved in radar cube as follows:
 *          Ping 1 (first dataOut EDMA channel) data will goto odd number doppler chirps in TX1, as shown in dark grey arrow)
 *          Ping 2 (second dataOut EDMA channel) data will goto odd number doppler chirps in TX3, as shown in light blue arrow)
 *          Ping 3 (third dataOut EDMA channel) data will goto even number doppler chirps in TX2, as shown in peach arrow)
 *
 *          Pong 1 (first dataOut EDMA channel) data will goto odd number doppler chirps in TX2, as shown in dark green arrow)
 *          Pong 2 (second dataOut EDMA channel) data will goto even number doppler chirps in TX1, as shown in light green arrow)
 *          Pong 3 (third dataOut EDMA channel) data will goto even number doppler chirps in TX3, as shown in blue arrow)
 *
 *   Input params | Setting
 *  :------------|:-----------:
 *   InterleaveMode | Non-Interleave
 *  HWA input mode|Isolated
 *  RadarCube format|DPIF_RADARCUBE_FORMAT_1
 *  numTxAnt| 3
 *
 * @image html isolated_non-interleave_to_radarcubefmt1_3tx.png "3 TX Isolated Non-interleaved data input to DPIF_RADARCUBE_FORMAT_1"
 *
 *
 *
 *
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROCHWA_H
#define RANGEPROCHWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <ti/drivers/hwa/hwa.h>

/* mmWave SDK Data Path Include Files */
#include <ti/datapath/dpif/dpif_adcdata.h>
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dp_error.h>
#include <ti/datapath/dpedma/dpedmahwa.h>
#include <ti/datapath/dpedma/dpedma.h>
#include <common/src/dpu/rangeprochwa_overhead/rangeproc_common.h>

#ifdef __cplusplus
extern "C" {
#endif
 
/*! Number of HWA parameter sets */
#define DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS             4U

/*! Alignment for DC range signal mean buffer - if DPU is running on DSP(C674) */
#define DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_DSP 8U

/*! Alignment for DC range signal mean buffer - if DPU is running on R4F */
#define DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_R4F 4U

/** @addtogroup DPU_RANGEPROC_ERROR_CODE
 *  Base error code for the rangeProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument
 */
#define DPU_RANGEPROCHWA_EINVAL                  (DP_ERRNO_RANGE_PROC_BASE-1)

/**
 * @brief   Error Code: Out of memory
 */
#define DPU_RANGEPROCHWA_ENOMEM                  (DP_ERRNO_RANGE_PROC_BASE-2)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_RANGEPROCHWA_EINTERNAL               (DP_ERRNO_RANGE_PROC_BASE-3)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_RANGEPROCHWA_ENOTIMPL                (DP_ERRNO_RANGE_PROC_BASE-4)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_RANGEPROCHWA_EINPROGRESS             (DP_ERRNO_RANGE_PROC_BASE-5)

/**
 * @brief   Error Code: Invalid control command
 */
#define DPU_RANGEPROCHWA_ECMD                    (DP_ERRNO_RANGE_PROC_BASE-6)

/**
 * @brief   Error Code: Semaphore error
 */
#define DPU_RANGEPROCHWA_ESEMA                   (DP_ERRNO_RANGE_PROC_BASE-7)

/**
 * @brief   Error Code: DC range signal removal configuration error
 */
#define DPU_RANGEPROCHWA_EDCREMOVAL              (DP_ERRNO_RANGE_PROC_BASE-8)

/**
 * @brief   Error Code: ADCBuf data interface configuration error
 */
#define DPU_RANGEPROCHWA_EADCBUF_INTF            (DP_ERRNO_RANGE_PROC_BASE-9)

/**
 * @brief   Error Code: ADCBuf data interface configuration error
 */
#define DPU_RANGEPROCHWA_ERADARCUBE_INTF         (DP_ERRNO_RANGE_PROC_BASE-10)

/**
 * @brief   Error Code: HWA windowing configuration error
 */
#define DPU_RANGEPROCHWA_EWINDOW                 (DP_ERRNO_RANGE_PROC_BASE-11)


/**
@}
*/

/**
 * @brief
 *  RangeProc data input mode
 *
 * @details
 *  This enum defines if the rangeProc input data is from RF front end or it is in M0 but 
 *  standalone from RF.
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef enum DPU_RangeProcHWA_InputMode_e
{
    /*! @brief     Range input is integrated with DFE input 
                    ADC buffer is mapped to HWA memory 
                    DMA data from ADC buffer to HWA is NOT required */
    DPU_RangeProcHWA_InputMode_MAPPED,

    /*! @brief     Range input is integrated with DFE input 
                    ADC buffer is not mapped to HWA memory,
                    DMA data from ADCBuf to HWA memory is 
                    needed in range processing */
    DPU_RangeProcHWA_InputMode_ISOLATED
} DPU_RangeProcHWA_InputMode;

/**
 * @brief
 *  rangeProc control command
 *
 * @details
 *  The enum defines the rangeProc supported run time command
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef enum DPU_RangeProcHWA_Cmd_e
{
    /*! @brief     Command to update DC Signature removal configuration */
    DPU_RangeProcHWA_Cmd_dcRangeCfg,

    /*! @brief     Command to trigger rangeProcHWA process */
    DPU_RangeProcHWA_Cmd_triggerProc,
}DPU_RangeProcHWA_Cmd;

/**
 * @brief
 *  RangeProc HWA configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_HwaConfig_t
{
    /*! @brief     HWA paramset Start index */
    uint8_t         paramSetStartIdx;

    /*! @brief     Number of HWA param sets must be @ref DPU_RANGEPROCHWA_NUM_HWA_PARAM_SETS */
    uint8_t         numParamSet;

    /*! @brief     Flag to indicate if HWA windowing is symmetric
                    see HWA_WINDOW_SYMM definitions in HWA driver's doxygen documentation
     */
    uint8_t         hwaWinSym;

    /*! @brief     HWA windowing RAM offset in number of samples */
    uint16_t        hwaWinRamOffset;

    /*! @brief     Data Input Mode, */
    DPU_RangeProcHWA_InputMode dataInputMode;
}DPU_RangeProcHWA_HwaConfig;

/**
 * @brief
 *  RangeProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAInputConfig_t
{
    /*! @brief     EDMA configuration for rangeProc data Input
                    This is needed only in @ref DPU_RangeProcHWA_InputMode_ISOLATED
     */
    DPEDMA_ChanCfg        dataIn;

    /*! @brief     EDMA configuration for rangeProc data Input Signature */
    DPEDMA_ChanCfg        dataInSignature;
}DPU_RangeProcHWA_EDMAInputConfig;

/**
 * @brief
 *  RangeProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAOutputConfigFmt1_t
{
    /*! @brief     EDMA configuration for rangeProc data Out- ping 
                    It must be a HWACC triggered EDMA channel.
     */
    DPEDMA_ChanCfg        dataOutPing;

    /*! @brief     EDMA configuration for rangeProc data Out- pong 
                    It must be a HWACC triggered EDMA channel
     */
    DPEDMA_ChanCfg        dataOutPong;
}DPU_RangeProcHWA_EDMAOutputConfigFmt1;

/**
 * @brief
 *  RangeProc EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAOutputConfigFmt2_t
{
    /*! @brief     EDMA configuration for rangeProc data Out- ping 
                    It must be a HWACC triggered EDMA channel
     */
    DPEDMA_3LinkChanCfg   dataOutPing;
    DPEDMA_ChanCfg        dataOutPingData[3];

    /*! @brief     EDMA configuration for rangeProc data Out- pong 
                    It must be a HWACC triggered EDMA channel
     */
    DPEDMA_3LinkChanCfg   dataOutPong;
    DPEDMA_ChanCfg        dataOutPongData[3];
}DPU_RangeProcHWA_EDMAOutputConfigFmt2;

/**
 * @brief
 *  RangeProc output EDMA configuration
 *
 * @details
 *  The structure is used to hold the EDMA configuration needed for Range FFT
 *
 *  Fmt1: Generic EDMA ping/pong output mode
 *       - 1 ping/pong EDMA channel, 
 *       - 1 ping/pong HWA signature channel
 *
 *  Fmt2: Specific EDMA ping/pong output mode used ONLY for 3 TX anntenna for radar cube
 *        layout format: @ref DPIF_RADARCUBE_FORMAT_1, ADCbuf interleave mode 
 *        @ref DPIF_RXCHAN_NON_INTERLEAVE_MODE
 *       - 1 ping/pong dummy EDMA channel with 3 shadow channels
         - 3 ping/pong dataOut channel
 *       - 1 ping/pong HWA signature channel 
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_EDMAOutputConfig_t
{
    /*! @brief     EDMA data output Signature */
    DPEDMA_ChanCfg        dataOutSignature;

    union
    {
        /*! @brief     EDMA data output fmt1 @ref DPU_RangeProcHWA_EDMAOutputConfigFmt1 */
        DPU_RangeProcHWA_EDMAOutputConfigFmt1     fmt1;

        /*! @brief     EDMA data output fmt2 @ref DPU_RangeProcHWA_EDMAOutputConfigFmt2 */
        DPU_RangeProcHWA_EDMAOutputConfigFmt2     fmt2;
    }u;
}DPU_RangeProcHWA_EDMAOutputConfig;

/**
 * @brief
 *  RangeProcHWA hardware resources
 *
 * @details
 *  The structure is used to hold the hardware resources needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_HW_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     HWA configuration */
    DPU_RangeProcHWA_HwaConfig      hwaCfg;

    /*! @brief     EDMA configuration for rangeProc data Input */
    DPU_RangeProcHWA_EDMAInputConfig edmaInCfg;

    /*! @brief     EDMA configuration for rangeProc data Output */ 
    DPU_RangeProcHWA_EDMAOutputConfig edmaOutCfg;

    /*! @brief     Pointer to Calibrate DC Range signature buffer 
                    The size of the buffer = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
                                        numTxAntenna * numRxAntenna * sizeof(cmplx32ImRe_t)
        For R4F:\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_R4F \n
        For DSP (C674X):\n
        Byte alignment Requirement = @ref DPU_RANGEPROCHWA_DCRANGESIGMEAN_BYTE_ALIGNMENT_DSP \n
     */
    cmplx32ImRe_t       *dcRangeSigMean;

    /*! @brief     DC range calibration scratch buffer size */
    uint32_t            dcRangeSigMeanSize;

    /*! @brief      Radar cube data interface */
    DPIF_RadarCube      radarCube;
}DPU_RangeProcHWA_HW_Resources;

/**
 * @brief
 *  RangeProcHWA static configuration
 *
 * @details
 *  The structure is used to hold the static configuraiton used by rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief  Range FFT window coefficients, Appliation provided windows coefficients
                After @ref DPU_RangeProcHWA_config(), windowing buffer is not used by rangeProcHWA DPU,
                Hence memory can be released
     */
    int32_t    *window;

    /*! @brief     Range FFT window coefficients size in bytes 
                    non-symmetric window, size = sizeof(uint32_t) * numADCSamples
                    symmetric window, size = sizeof(uint32_t)*(numADCSamples round up to even number )/2
     */
    uint32_t    windowSize;

    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData     ADCBufData;

    /*! @brief      Flag to reset dcRangeSigMean buffer
                     1 - to reset the dcRangeSigMean buffer and counter
                     0 - do not reset
     */
    uint8_t     resetDcRangeSigMeanBuffer;
}DPU_RangeProcHWA_StaticConfig;

/**
 * @brief
 *  RangeProcHWA dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuraiton used by rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_DynamicConfig_t
{
    /*! @brief      Pointer to Calibrate DC Range signature configuration */
    DPU_RangeProc_CalibDcRangeSigCfg *calibDcRangeSigCfg;
}DPU_RangeProcHWA_DynamicConfig;

/**
 * @brief
 *  Range FFT configuration
 *
 * @details
 *  The structure is used to hold the configuration needed for Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_Config_t
{
    /*! @brief     rangeProc hardware resources */
    DPU_RangeProcHWA_HW_Resources   hwRes;

    /*! @brief     rangeProc static configuration */
    DPU_RangeProcHWA_StaticConfig   staticCfg;

    /*! @brief     rangeProc dynamic configuration */
    DPU_RangeProcHWA_DynamicConfig  dynCfg;
}DPU_RangeProcHWA_Config;

/**
 * @brief
 *  rangeProcHWA output parameters populated during rangeProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_InitParams_t
{
    /*! @brief     HWA Handle */
    HWA_Handle          hwaHandle;
}DPU_RangeProcHWA_InitParams;

/**
 * @brief
 *  rangeProcHWA output parameters populated during rangeProc Processing time
 *
 * @details
 *  The structure is used to hold the output parameters for rangeProcHWA
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_OutParams_t
{
    /*! @brief      End of Chirp indication for rangeProcHWA */
    bool                endOfChirp;

    /*! @brief     rangeProcHWA stats */
    DPU_RangeProc_stats  stats;
}DPU_RangeProcHWA_OutParams;

/**
 * @brief
 *  rangeProc DPU Handle
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef void* DPU_RangeProcHWA_Handle ;

/*================================================================
               rangeProcHWA DPU exposed APIs            
 ================================================================*/
DPU_RangeProcHWA_Handle DPU_RangeProcHWA_init
(
    DPU_RangeProcHWA_InitParams     *initParams,
    int32_t*                        errCode
);

int32_t DPU_RangeProcHWA_config
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_Config*    rangeHwaCfg
);

int32_t DPU_RangeProcHWA_process
(
    DPU_RangeProcHWA_Handle     handle,
    DPU_RangeProcHWA_OutParams* outParams
);

int32_t DPU_RangeProcHWA_control
(
    DPU_RangeProcHWA_Handle handle,
    DPU_RangeProcHWA_Cmd    cmd,
    void*                   arg,
    uint32_t                argSize
);

int32_t DPU_RangeProcHWA_deinit
(
    DPU_RangeProcHWA_Handle     handle
);

#ifdef __cplusplus
}
#endif

#endif
