/**
 *   @file  staticdetproccommon.h
 *
 *   @brief
 *      Implements Common definition across dopplerProc DPU.
 *
 *  \par
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef STATICDETPROC_COMMON_H
#define STATICDETPROC_COMMON_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <ti/datapath/dpedma/dpedma.h>

/** @mainpage Doppler DPU
 *
 * This DPU implements the Doppler FFT (2D FFT).
 *
 *  @section intro_section Introduction
 *
 *  Description
 *  ----------------
 *
 *  This DPU expects as input the radar cube with 1D FFT data as described in @ref DPIF_RADARCUBE_FORMAT_1. 
 *  The output of this DPU is a detection matrix of format described in @ref DPIF_DETMATRIX_FORMAT_1. 
 *  These are the only formats supported by this DPU. 
 *
 *  The Doppler DPU is available in two distinct implementations:
 *
 *   DPU Implementation   |  Runs on cores  | Uses HWA   
 *  :---------------------|:----------------|:---------
 *  dopplerProcHWA        |  R4F or DSP     |  Yes
 *  dopplerProcDSP        |     DSP         |  No
 *\n\n
 *  @section dpu1 DopplerProcHWA
 *
 * Data from radar cube is moved into HWA memory using EDMA,
 * HWA performs the required computations and data is moved by EDMA from HWA internal memory to the detection matrix.\n\n
 * 
 * The following figure shows a high level block diagram of the DPU implementation.\n
 *
 * @image html hwa_doppler_toplevel.png "HWA based Doppler DPU High Level block diagram"
 *
 * A list of resources required by this DPU is described in @ref DPU_DopplerProcHWA_HW_Resources_t.
 * In particular, the number of EDMA channels required is constant and does not depend on the DPU configuration. 
 * On the other hand, the number of required HWA paramsets is a function of the number of TX antennas configured in
 * the data path as described in @ref DPU_DopplerProcHWA_HwaCfg_t.\n
 * 
 * Besides the resources described above, other parameters required for the DPU configuration are listed
 * in @ref DPU_DopplerProcHWA_StaticConfig_t.
 * In particular, the DPU takes as input the number of Doppler chirps (numDopplerChirps), which does not
 * need to be a power of two. It produces a detection matrix of Doppler dimension equal to numDopplerBins
 * which must be a power of 2 greater or equal than numDopplerChirps.
 * Another DPU input is the number of range bins, which must be even but does not need to be a power of two.\n\n
 * 
 *
 *   Parameter         |  Restriction  
 *  :------------------|:--------------
 *  numDopplerChirps   |  None.   
 *  numDopplerBins     |  Must be power of two greater or equal than numDopplerChirps.      
 *  numRangeBins       |  Must be even.     
 *
 * Below are details of the DPU implementation:\n
 * 
 * EDMA is used to move data from the radar cube into the HWA internal memory and after processing is done
 * to move data from HWA internal memory to the detection matrix.
 * All 4 HWA memory banks are required by this DPU, regardless of the DPU configuration. In this document, the
 * HWA memory banks are called M0, M1, M2, M3.\n 
 * 
 * A ping/pong buffer scheme is used where:\n
 * 
 * M0 and M2 are used for ping input/processing.\n 
 * M1 and M3 are used for pong input/processing.\n
 * 
 * In each iteration (ping/pong), a column of the radar cube matrix is brought into HWA for processing. 
 * Such column consists of all samples for a fixed range, that is, all received virtual antennas for all
 * Doppler chirps for a fixed range bin.\n
 * 
 * The steps below are executed by HWA on data (one column of the radar cube as described above) sitting 
 * in its M memory. The description below is given for processing the ping buffer. 
 * Pong buffer processing is identical to ping, except that M0 memory is replaced by M1 and M2 memory is replaced by M3.\n\n
 * 
 * **Windowing**\n
 * Before FFT operation, input samples are multiplied by a window function. Window size and coefficients are 
 * defined in @ref DPU_DopplerProcHWA_HwaCfg_t.\n
 * Window coefficients must be provided by application.\n\n
 * 
 * **FFT and Log2|.|**\n
 * In this step 2D FFT and Log2 of the absolute value of each sample is computed. Input samples are in M0
 * and output samples are in M2. Input sample is of type cmplx16ImRe_t and output is of type uint16_t.
 * This step also converts the number of input samples from N = numDopplerChirps 
 * to a number of output samples K = numDopplerBins, which is the size of the Doppler FFT.
 * Both numDopplerBins and numDopplerChirps are input parameters for this DPU and numDopplerBins must be a power
 * of 2 greater or equal than numDopplerChirps. \n\n 
 * 
 * **Summation**\n
 * Summation of all virtual antennas is computed for each Doppler bin. \n
 * The output of the previous step (HWA Log2 magnitude) is in Q11 format.
 * The sum is done using FFT in HWA, the sum is obtained in the DC (0th) bin. 
 * Input of the summation is on M2 and output is on M0. This FFT programming has srcScale of 3, meaning 3
 * redundant bits (sign extension, in this case unsigned) are added to MSB and 5 LSBs are padded with 0, 
 * so input before computation is in Q[11 + 5] format. The dstScale is set to 8, so summation output will
 * have 8-bits dropped, giving a result in Q[11 + 5 - 8] = Q8 format. The FFT size is the next power of 2
 * of number of virtual antennas and the FFT is programmed to enable all butterfly scaling stages, hence the
 * FFT output will be 1/N' * sum(.), where N' = 2^Ceil(Log2(N)), where N = numVirtualAntennas.
 * The sum output which is input to CFAR is in Q8 format, so the CFAR threshold-scale
 * also needs to be in Q8 format. If CFAR threshold is originally intended to be expressed in dB
 * (say for user friendliness), then we need to do some translation before feeding to the CFAR
 * algorithm. This can be derived as follows: \n
 * Let N be the number of virtual antennas and N' = 2^ceil(log2(N))
 * and user friendly CFAR threshold in dB is TdB (= 20*log10(|.|)).
 * CFAR needs to do in general: \n
 * CUT (Cell Under Test) = 1/N * sum(log10(|.|)) > TdB/20 + average of noise terms [similar looking to CUT on LHS] \n
 * Given log10(|.|) = log2(|.|)/log2(10), and further adjusting the terms to do computation similar to match what is the
 * sum output described above : \n
 * 2^8 * 1/ N' * sum(log2(|.|)) > TdB/20*log2(10)*2^8 * (N/N') + average of noise terms [similar looking to CUT on LHS] \n
 * So the CFAR threshold to be provided to the CFAR algorithm operating on this doppler DPU output is \n
 * T = 256 * TdB / 6 * N / N'
 *
 * Once the summation is computed for all Doppler bins, EDMA transfers the result from M0 to the detection matrix.\n\n
 * 
 * **HWA memory bank size limitation**\n
 * The following 2 conditions must be satisfied for the parameters in this DPU configuration:\n
 * 4 x numRxAntennas x numTxAntennas x numDopplerChirps <= 16384\n
 * 2 x numRxAntennas x numTxAntennas x numDopplerBins <= 16384\n\n
 *
 * The reason for this limitation is as follows:\n
 * The size of the data that is brought in for processing in HWA (per ping/pong iteration) is a column of the radar cube.
 * The size of radar cube column is\n 
 * 4 x numRxAntennas x  numTxAntennas x numDopplerChirps (I),\n
 * where 4 bytes is sizeof(cmplx16ImRe_t), the sample size.\n
 * 
 * After the 2D FFT and Log2|.| , we go from Doppler chirps to Doppler bins and from cmplx16ImRe_t to uint16_t. \n
 * 
 * Therefore the size after 2D FFT and Log2|.| is\n 
 * 2 x numRxAntennas x numTxAntennas x numDopplerBins  (II),\n
 * where 2 bytes is sizeof(uint16_t).\n\n
 * 
 * Both quantities above (I and II) should fit (independently) in one of the HWA M memory partitions 
 * which has a size of 16KB.\n\n
 * 
 * **Exported APIs**\n
 * DPU initialization is done through @ref DPU_DopplerProcHWA_init.\n\n
 * 
 * DPU configuration is done by @ref DPU_DopplerProcHWA_config. The configuration can only be done after
 * the DPU has been initialized. The configuration parameters are described in @ref DPU_DopplerProcHWA_Config. \n\n
 *
 * The DPU is executed by calling @ref DPU_DopplerProcHWA_process. \n
 * This will trigger the first ping/pong EDMA transfers and from there on, the processing of the Doppler DPU for the
 * full radar cube is driven by hardware: EDMA will move data in and trigger HWA, which will process the data and 
 * trigger EDMA to move data out (to detection matrix) and trigger next EDMA to move data in – and so on. All columns of the radar cube
 * matrix will be processed in this loop and no CPU intervention is needed. When HWA finishes processing all columns of the
 * radar cube it will generate an interrupt. When the last EDMA transfer to the detection matrix has landed, EDMA will generate
 * an interrupt. The DPU processing is done when both interrupts are received.
 * Both interrupts are depicted in green boxes in the figure below.\n\n
 * 
 * 
 * **Detailed block diagram for 3 TX 4 RX**\n
 * 
 * The following figure depicts in detail the DPU implementation for the case of 3 TX and 4RX antennas.
 * The blue boxes connected by blue arrows represent different HWA paramsets.\n\n
 * 
 * @image html hwa_doppler_fft.png "Doppler DPU implementation for 3 TX and 4 RX antennas"
 * 
 * \n\n\n 
 *  @section dpu2 DopplerProcDSP
 * In this version of the Doppler DPU HWA is not used. All computation is done by the DSP.\n
 * Data from radar cube is moved into scratch memory using EDMA,
 * DSP performs the required computations and data is moved by EDMA from scratch memory to the detection matrix.\n\n
 * 
 * The following figure shows a high level block diagram of the DPU implementation.\n
 *
 * @image html dsp_doppler_fft_toplevel.png "DSP Doppler DPU High Level block diagram"
 *
 * A list of resources required by this DPU is described in @ref DPU_DopplerProcDSP_HW_Resources_t.
 * In particular, the number of EDMA channels required is constant and does not depend on the DPU configuration. 
 * 
 * Besides the resources described above, other parameters required for the DPU configuration are listed
 * in @ref DPU_DopplerProcDSP_StaticConfig_t.
 * In particular, the DPU takes as input the number of Doppler chirps (numDopplerChirps), which does not
 * need to be a power of two but must be a multiple of 4. It produces a detection matrix of Doppler dimension equal to numDopplerBins
 * which must be a power of 2 greater or equal than numDopplerChirps. Also, due to restrictions on the FFT implementation by DSPLIB,
 * numDopplerBins must be at least 16.
 *
 *   Parameter         |  Restriction  
 *  :------------------|:--------------
 *  numDopplerChirps   |  Must be a multiple of 4.   
 *  numDopplerBins     |  Must be at least 16. Must be power of two greater or equal than numDopplerChirps.      
 * \n
 * Below are details of the DPU implementation:\n\n
 *
 * **Input data**\n
 * EDMA is used to move data from the radar cube into the DPU scratch buffers.\n 
 * A ping/pong buffer scheme is used where in each iteration (ping/pong), data pertinent to one virtual antenna (for a fixed range bin) is brought 
 * from the radar cube matrix for processing. \n
 * For a fixed range bin, the order in which the virtual antennas are processed must assure that BPM can be decoded
 * and all virtual antennas can be summed up. In order to achieve this goal with minimum temporary buffering of virtual
 * antennas data, the sequence of virtual antennas is processed in the following order:\n\n
 * 1) Next TX antennas for same range bin and same RX antenna. This assures that BPM can be decoded.\n
 * 2) Once 1 is exhausted for all TX antennas for a fixed range bin and fixed RX antenna, move to the next RX antenna
 *    for the same range bin (to assure that the sum of all virtual antennas can be computed) and repeat 1.\n
 * 3) Once all virtual antennas for this range bin have been processed, move to the next range bin.\n\n
 *
 * Illustration of the ping/pong data pattern is shown in the next sections for the [3TX,4RX] and [2TX,4RX] cases.\n\n
 *
 * **Static Clutter Removal**\n
 * When Static Clutter Removal is enabled, the mean value of the input samples to the
 * Doppler FFT is subtracted from the samples.\n\n
 *
 * **Windowing**\n
 * Before FFT operation, input samples are multiplied by a real symmetric window function. Window size and coefficients are 
 * defined in @ref DPU_DopplerProcDSP_HW_Resources_t. Window coefficients must be provided by application.\n
 * Note that the windowing function also executes IQ swap. Before windowing the samples are in the same format as in the radar cube,
 * which is cmplx16ImRe_t. After windowing the output is in cmplx32ReIm_t format.\n\n
 *
 * **FFT**\n
 * FFT is computed the output has numDopplerBins samples of type cmplx32ReIm_t.\n\n
 *
 * **BPM decoding**\n
 * If BPM is enabled, when the FFT output is available for the 2 TX antennas (for the same RX antenna and range bin),
 * BPM is decoded. More details on BPM implementation in the following section.\n\n
 *
 * **Log2|.|**\n
 *  Log2 of the absolute value of each sample is computed and the output is a 16 bit number in Q8 format.\n\n
 *
 * **Summation and Output**\n
 * Accumulation is done for every Doppler bin for every virtual antenna. Note that before accumulating, each
 * factor in the sum is divided by 2^Ceil(Log2(numVirtualAntennas)). The purpose of doing this
 * is to have the same CFAR threshold scale interpretation between the HWA and DSP versions of the DPU,
 * this same interpretation allows for the convenience of configuring the same threshold scale
 * for a downstream CFAR algorithm whether operating on the HWA or the DSP version of the doppler DPU.
 * Refer to the description related to the conversion of CFAR threshold scale from user-friendly dB units
 * to what is required for CFAR algorithm under **Summation** section in @ref dpu1 \n
 * Once all virtual antennas are processed for a given range bin, the accumulated array is transferred by 
 * EDMA to the detection matrix.\n\n
 * 
 * **Detailed block diagram for 3 TX 4 RX TDM-MIMO (no BPM)**\n
 * 
 * The following figure depicts in detail the DPU implementation for the case of 3 TX and 4RX antennas (no BPM).
 * 
 * @image html dsp_doppler_fft_3TX_4RX.png "Doppler DPU implementation for 3 TX and 4 RX antennas"
 *
 *
 * The following picture illustrates the ping/pong pattern for the case of 3 TX and 4 RX antennas.                
 * @image html dsp_doppler_fft_3TX_4RX_pingpong.png "Ping/pong pattern for the case of 3 TX and 4 RX antennas"
 * 
 * 
 *  @subsection bpmCfgNotes BPM Scheme
 *    Similar to TDM-MIMO, in BPM scheme a frame consists of multiple blocks, each
 *    block consisting of 2 chirp intervals. However, unlike in TDM-MIMO where only
 *    one TX antenna active per chirp interval, two TX antennas are active in each
 *    chirp interval. This DPU only supports a BPM scheme with two
 *    TX antennas say A and B. In the even time slots (0, 2,...), both transmit antennas
 *    should be configured to transmit with positive phase i.e 
 *    @verbatim (A,B) = (+,+) @endverbatim      
 *    In the odd time slots (1, 3,...), the transmit antennas should be configured to 
 *    transmit with phase\n 
 *    @verbatim (A,B) = (+,-) @endverbatim      
 *    The BPM scheme is shown in the figure below where TX0 is used as antenna A and TX1 as
 *    antenna B.
 *
 * @image html bpm_antenna_cfg.png "BPM Scheme Antenna configuration"
 *
 * **BPM decoding**\n
 *    Let S1 and S2 represent chirp signals from two TX antennas. In time slot zero
 *    a combined signal Sa=S1+S2 is transmitted. Similarly in time slot one a
 *    combined signal Sb=S1-S2 is transmitted. Using the corresponding received signals,
 *    (S'a and S'b), at a specific received RX antenna, the components from the individual
 *    transmitters are separated out using\n
 *    @verbatim S'1=(S'a+S'b)/2 @endverbatim
 *    and
 *    @verbatim S'2=(S'a-S'b)/2 @endverbatim
 *    With simultaneous transmission on both TX antennas the total transmitted power per
 *    chirp interval is increased, and it can be shown that this translates to an SNR 
 *    improvement of 3dB.\n  
 *
 * **Order of the TX antennas in the BPM scheme**\n
 *    The BPM decoding will produce the virtual antenna 
 *    array in the order (A, B), not (B, A), which will be used for AoA processing. 
 *    Therefore, user must make sure that the (A, B) mapping to the physical transmit antennas
 *    corresponds to the intended virtual antenna order. 
 *    For example, take the 6843 EVM antenna arrangement as shown in the figure below.
 *    Note that the DPU indexes the TX antennas as (TX0, TX1, TX2) which corresponds
 *    in the device to (TX1, TX2, TX3). \n
 *
 *    @image html coordinate_geometry_6843.png "6843 EVM antenna arrangement"
 *
 *    Here two antennas TX1 and TX3 can create a virtual array of 8 antennas in the azimuth direction. 
 *    In order to ensure correct virtual antenna processing for AoA calculations, after BPM decoding, 
 *    the TX1 virtual antennas should precede the TX3 because the direction of spatial progression of 
 *    RX antennas matches the direction of spatial progression of TX antennas when TX1 is before TX3 
 *    (post BPM decoding, the result should essentially be like TDM-MIMO). Therefore A=TX1 and B=TX3. 
 *    In other words, BPM must be configured to do\n
 *    @verbatim (TX1,TX3) = (+,+) in even slots @endverbatim      
 *    and\n
 *    @verbatim (TX1,TX3) = (+,-) in odd slots @endverbatim      
 *    The opposite arrangement i.e\n
 *    @verbatim (TX3,TX1) = (+,+) in even slots @endverbatim      
 *    and\n
 *    @verbatim (TX3,TX1) = (+,-) in odd slots @endverbatim      
 *    will not work.\n\n
 *
 *  @subsection bpmNotes Doppler DPU changes when BPM is enabled
 *    When BPM is enabled the following changes are done in the Doppler DPU processing:\n
 *    Doppler compensation and BPM decoding are done after the Doppler FFT. 
 *    Note that the decoded data is not stored in the radar cube, therefore
 *    BPM decoding needs to be done again (on a much smaller set of samples)
 *    during the direction of arrival computation.
 *    The following figure shows the required changes in the Doppler processing.
 *    When BPM is enabled the fftOutBuf buffer is doubled in size to accommodate both Ping (TX0+TX1)
 *    and Pong (TX0-TX1) so that BPM can be decoded. The decoded data is written back to the fftOutBuf. 
 *
 * @image html dsp_doppler_fft_2TX_4RX_BPM.png "Doppler DPU implementation for 2 TX and 4 RX antennas and BPM enabled"
 * \n\n
 * The following picture shows the ping/pong pattern for the case of 2 TX and 4 RX antennas (regardless if BPM is enabled or not).                
 * @image html dsp_doppler_fft_2TX_4RX_BPM_pingpong.png "Ping/pong pattern for the case of 2 TX and 4 RX antennas"
 *
 * **Exported APIs**\n
 * DPU initialization is done through @ref DPU_DopplerProcDSP_init.\n\n
 * 
 * DPU configuration is done by @ref DPU_DopplerProcDSP_config. The configuration can only be done after
 * the DPU has been initialized. The configuration parameters are described in @ref DPU_DopplerProcDSP_Config. \n\n
 *
 * The DPU is executed by calling @ref DPU_DopplerProcDSP_process. \n
 * 
 * 
 */



/**
@defgroup DPU_DOPPLERPROC_EXTERNAL_FUNCTION            dopplerProc DPU External Functions
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all the exported API which the applications need to
*   invoke in order to use the dopplerProc DPU
*/
/**
@defgroup DPU_DOPPLERPROC_EXTERNAL_DATA_STRUCTURE      dopplerProc DPU External Data Structures
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all the data structures which are exposed to the application
*/
/**
@defgroup DPU_DOPPLERPROC_ERROR_CODE                   dopplerProc DPU Error Codes
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all the error codes which are generated by the dopplerProc DPU
*/
/**
@defgroup DPU_DOPPLERPROC_INTERNAL_FUNCTION            dopplerProc DPU Internal Functions
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup DPU_DOPPLERPROC_INTERNAL_DATA_STRUCTURE      dopplerProc DPU Internal Data Structures
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all internal data structures which are used internally
*   by the dopplerProc DPU module.
*/
/**
@defgroup DPU_DOPPLERPROC_INTERNAL_DEFINITION          dopplerProc DPU Internal Definitions
@ingroup DOPPLER_PROC_DPU
@brief
*   The section has a list of all internal definitions which are used internally
*   by the dopplerProc DPU.
*/



#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  dopplerProc DPU EDMA configuration parameters
 *
 * @details
 *  The structure is used to hold the EDMA configuration parameters
 *  for the Doppler Processing DPU
 *
 *  \ingroup DPU_DOPPLERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProc_Edma_t
{
    /*! @brief  EDMA Ping channel. */
    DPEDMA_ChanCfg  ping;
    /*! @brief  EDMA Pong channel. */
    DPEDMA_ChanCfg  pong;    
}DPU_StaticDetProc_Edma;


/**
 * @brief
 *  dopplerProc DPU statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU 
 *
 *  \ingroup DPU_DOPPLERPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_StaticDetProc_Stats_t
{
    /*! @brief total number of DPU processing */
    uint32_t            numProcess;

    /*! @brief For HWA version of the DPU: total processing time including EDMA transfers.\n
               For DSP version of the DPU: total processing time excluding EDMA transfers.*/
    uint32_t            processingTime;
    
    /*! @brief time spent waiting for EDMA transfers. Valid only for DSP version of DPU.*/
    uint32_t            waitTime;
}DPU_StaticDetProc_Stats;

#ifdef __cplusplus
}
#endif

#endif 
