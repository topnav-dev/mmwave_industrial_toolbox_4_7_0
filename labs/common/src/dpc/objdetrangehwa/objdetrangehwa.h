/**
 *   @file  objdetrangehwa.h
 *
 *   @brief
 *      Object Detection DPC with range HWA only DPU
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

/** @mainpage Object Detection Data-path Processing Chain (DPC) for range HWA DPU only
 * [TOC]
 *  @section objdetrangehwa_intro Overview
 *
 *  The Object detection DPC provides the functionality of processing ADC samples during
 *  the frame acquisition periods. It can be used by an application by registering with the DPM
 *  framework and invoked using DPM APIs. The external interface of this Object detection
 *  DPC can be seen at @ref DPC_OBJDETRANGEHWA_EXTERNAL
 *
 *   @image html object_detection_datapath.png "Object Detection Data Path Processing Chain"
 *   \n
 *   \n
 *   This data path chain processing consists of:
 *   - Processing during the chirps as seen in the timing diagram:
 *     - This consists of 1D (range) FFT processing using HWA that takes input from multiple
 *     receive antennas from the ADC buffer for every chirp (corresponding to the
 *     chirping pattern on the transmit antennas) and performs FFT on it and generates 
 *     radar cube output into the L3 RAM in the format defined by @ref DPIF_RADARCUBE_FORMAT_1. 
 *   For more details of range processing, see the doxygen
 *     documentation of range processing DPU (Data Path Unit) located at \ref dpu_range_intro
 *
 *  @section objdetrangehwa_appdpcFlow Application-DPC Execution Flow
 *   Following diagram shows the application-DPC execution Flow.
 *
 *  @image html dpc_flow.png "Application-DPC Execution Flow"
 *
 *   The flow above shows the sequencing of initialization, configuration, execution and
 *   dynamic control operations of the DPC and some level of detail of
 *   what happens under these operations.
 *   Most of the hardware resource (e.g HWA and EDMA related) configuration
 *   for the DPUs that is issued by the DPC as part of processing
 *   @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG commands is provided by the
 *   application at build time using a resource file (DPC sources are built
 *   as part of building the application, there is no separate DPC library object).
 *   This file is passed as a compiler command line define
 *   @verbatim --define=APP_RESOURCE_FILE="fileName" @endverbatim The "fileName"
 *   above includes the path as if to include the file when building the the DPC sources
 *   as part of building the application, and any DPC source that needs to refer
 *   to this file (currently objdetrangehwa.c) has the following code
 *    @verbatim #include APP_RESOURCE_FILE @endverbatim
 *   One of the demos that uses this DPC is located at ti/demo/xwr68xx/mmw. The
 *   resource file in this demo is mmw_res.h, this file shows all the definitions
 *   that are needed by the DPC from the application. This file is provided
 *   on compiler command line when building as follows:
 *   @verbatim --define=APP_RESOURCE_FILE="<ti/demo/xwr68xx/mmw/mmw_res.h>" @endverbatim
 *
 *   
 *  @section objdetrangehwa_memory Data Memory
 *
 *  @subsection objdetrangehwa_memCfg Memory configuration
 *   The configuration of L3 and Core Local RAM (hitherto referred in short as LRAM)
 *   memories are provided by the application 
 *      - @ref DPC_ObjectDetectionRangeHWA_InitParams_t::L3ramCfg,
 *      - @ref DPC_ObjectDetectionRangeHWA_InitParams_t::CoreLocalRamCfg
 *
 *   during @ref DPC_ObjectDetection_init (invoked by application through @ref DPM_init). 
 *
 *   This DPC reports L3RAM and radarCube memory usage back to application to be used by other DPCs
 *   (for details, refer to @ref DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem_t and 
 *   @ref DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage_t)
 *
 *   @subsection objdetrangehwa_memPar Memory partition
 *   The L3 and LRAM partition happens during the processing of @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG command
 *   and is shown in the following figure. The allocation from application system heap
 *   (typically in LRAM) using the MemoryP_osal API is done during @ref DPC_ObjectDetection_init
 *   (object instances of DPC and DPUs for all sub-frames) and during the
 *   processing of @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG command (range DPUs
 *   dc antenna coupling signature buffer that is unique for each sub-frame)
 *   is also shown in the figure.
 *
 *
 *   @image html memory_allocation.png "Data memory allocation"
 *
 *   The buffers labeled "windowBuffer" in the picture are allocated/generated during DPU configue time.
 *   The windowing coefficients will be copied to HWA internal memory, hence this is temporary use of the local memory.
 *
 *   @subsection objdetrangehwa_reconfig DPU reconfiguration 
 *   DPU reconfiguration is related to data path processing across sub-frames for advanced frame configuraiton.
 *   In such cases, reconfiguration is required when switching sub-frames because HWA and EDMA resources
 *   are overlapped across sub-frames. Note that the DPU's xxx_config API
 *   is a full configuration API beyond the HWA and EDMA resources configuration (e.g static
 *   and dynamic configuration) so restricting to the full configuration
 *   would imply that no sub-frame specific DPU instantiation is necessary. However, the code illustrates separate 
 *   instances of DPUs for each sub-frame to demonstrate generality of the sub-frame solution,
 *   in the case where there may be specialized (partial) configuration APIs in an
 *   optimized implementation (that only configured the overlapped HWA and EDMA resources).
 *   The limiting to full configuration also means that all the code required to build the
 *   configuration structures of the DPUs during the pre-start config time either
 *   has to be repeated or alternatively, the configurations that were created
 *   during the pre-start config processing be saved and reused later.
 *   The latter path has been taken, all DPU configuration that is built
 *   during the pre-start processing is stored in separate storage, this can be
 *   located at @ref SubFrameObj_t::rangeCfg. However, parts of this reconfiguration that
 *   cannot be captured in this storage need to be repeated, namely, window generation
 *   (@ref DPC_ObjDetRangeHwa_GenRangeWindow).
 *
 *  @section objdetrangehwa_designNotes Data Path Design Notes
 *
 *    @subsection objdetrangehwa_scaling Scaling
 *      -# HWA FFT Scaling: The HWA uses 24-bit fixed point arithmetic for the data path processing.
 *      In order to prevent overflows in the FFT processing, the scaling factors have to be
 *      set appropriately in the HWA configuration. The HWA has up to 10 stages of processing
 *      with ability to scale by 1/2 for each stage.
 *          -# 1D processing: If the HWA's FFT scale is set to \f$\frac{1}{2^k}\f$
 *          where \f$k\f$ is the combined value for (number of stages for which the scaling is enabled) 
 *          and (right shift applied in output formatter) and input to the FFT were a pure tone at one of the bins, then
 *          the output magnitude of the FFT at that bin will be \f$\frac{N}{2^k}\f$ (\f$N\f$ is the FFT order) times
 *          the input tone amplitude (because tone is complex, this implies that the individual real and
 *          imaginary components will also be amplified by a maximum of this scale).
 *          Because we do a Blackman window before the FFT, the overall scale is about 1/2.4 of the FFT scale.
 *          This means for example for 256 point FFT, the windowing + FFT scale will be \f$\frac{106.7}{2^k}\f$. 
 *          For example k=2, this will be 26.7. Therefore, the ADC output when it is a pure tone should not exceed 
 *          +/-2^15/26.7 = 1228 for the I and Q components (even though HWA is internally 24-bit, the FFT output is stored
 *          as 16-bit before 2D processing, hence 2^15). For example k=3, this will be 13.33. 
 *          Therefore, the ADC output when it is a pure tone should not exceed 
 *          +/-2^15/13.33  = 2458 for the I and Q components (even though HWA is internally 24-bit, the FFT output is stored
 *          as 16-bit before 2D processing, hence 2^15). So chosen value of 'k' can now be distributed between 
 *          butterfly scaling and output divisor (right) shift using via \ref DPC_ObjectDetectionRangeHWA_StaticCfg_t::rangeFFTtuning. \n
 *          Recommendation is to allow butterfly stages to grow to 24-bit without using scaling and apply scaling
 *          at appropriate butterfly stages using \ref DPU_RangeProcHWA_FFTtuning::numLastButterflyStagesToScale 
 *          only if there is a possibility of overflowing 24-bit at that particular stage and beyond. 
 *          Next, to convert 24-bit to 16-bit, use \ref DPU_RangeProcHWA_FFTtuning::fftOutputDivShift appropriately.\n
 *          The XWR68xx EVM when presented with a strong single reflector
 *          reasonably close to it (with Rx dB gain of 30 dB in the chirp profile and ADC samples = 256)
 *          shows ADC samples to be a max of about 2000 and hence application could chose value of '3'
 *          for fftOutputDivShift and value of '0' for numLastButterflyStagesToScale.
 *      -# Scaling factors for the chain: See ti/datapath/dpc/objectdetection/common/docs/object_detection_data_scaling.xlsx
 *
 */
#ifndef DPC_OBJDETRANGEHWA_H
#define DPC_OBJDETRANGEHWA_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/hwa/hwa.h>
#include <ti/control/dpm/dpm.h>

#include <ti/datapath/dpu/rangeproc/rangeprochwa.h>
#include <common/src/dpu/trackerproc_overhead/trackerproc.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup DPC_OBJDETRANGEHWA_EXTERNAL        Object Detection DPC (Data-path Processing Chain) External
 *
 * This DPC performs processes ADC samples and generates radar Cube data.
 */
/**
@defgroup DPC_OBJDETRANGEHWA__GLOBAL                             Object Detection DPC Globals
@ingroup DPC_OBJDETRANGEHWA_EXTERNAL
@brief
*   This section has a list of all the globals exposed by the Object detection DPC.
*/
/**
@defgroup DPC_OBJDETRANGEHWA_IOCTL__DATA_STRUCTURES               Object Detection DPC Data Structures
@ingroup DPC_OBJDETRANGEHWA_EXTERNAL
@brief
*   This section has a list of all the data structures which are a part of the DPC module
*   and which are exposed to the application
*/
/**
@defgroup DPC_OBJDETRANGEHWA_IOCTL__DEFINITIONS                   Object Detection DPC Definitions
@ingroup DPC_OBJDETRANGEHWA_EXTERNAL
@brief
*   This section has a list of all the definitions which are used by the Object Detection DPC.
*/
/**
@defgroup DPC_OBJDETRANGEHWA_IOCTL__COMMAND                       Object Detection DPC Configuration Commands
@ingroup DPC_OBJDETRANGEHWA_EXTERNAL
@brief
*   This section has a list of all the commands which are supported by the DPC.
*   All commands of the type IOCTL__STATIC_<...> can only be issued either before the
*   first call to @ref DPM_start (DPC_ObjectDetection_start) or after the @ref DPM_stop (DPC_ObjectDetection_stop)
*   All commands of the type IOCTL__DYNAMIC_<...> can be issued between at
*   the inter-frame boundary i.e when the result is available from @ref DPM_execute (DPC_ObjectDetection_execute).
*   All commands of the type IOCTL__STATIC_<..> must be issued
*   before @ref DPM_start (DPC_ObjectDetection_start) because there are no defaults.
*/
/**
@defgroup DPC_OBJECTDETECTION_ERROR_CODE                   Object Detection DPC Error Codes
@ingroup DPC_OBJDETRANGEHWA_EXTERNAL
@brief
*   This section has a list of all the error codes returned when calling Object Detection DPC functions
*   during error conditions.
*/


/** @addtogroup DPC_OBJDETRANGEHWA_IOCTL__COMMAND
 @{ */

/**
 * @brief Command associated with @ref DPC_ObjectDetectionRangeHWA_PreStartCfg_t. In this IOCTL, the sub-frame's
 *        configurations will be processed by configuring individual DPUs.
 *        The @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG must be issued
 *        before issuing this IOCTL.
 */
#define DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG                            (DPM_CMD_DPC_START_INDEX + 100U)

/**
 * @brief Command associated with @ref DPC_ObjectDetectionRangeHWA_PreStartCommonCfg_t. Must be issued before
 *        issuing @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG
 */
#define DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG                     (DPM_CMD_DPC_START_INDEX + 101U)

/**
 * @brief Command associated with @ref DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg_t
 */
#define DPC_OBJDETRANGEHWA_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG                  (DPM_CMD_DPC_START_INDEX + 102U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_TrackerConfig_t
 */
#define DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG                              (DPM_CMD_DPC_START_INDEX + 103U)

/**
@}
*/

/** @addtogroup DPC_OBJECTDETECTION_ERROR_CODE
 *  Base error code for the cfarcaProc DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument general (such as NULL argument pointer)
 */
#define DPC_OBJDETRANGEHWA_EINVAL                           (DP_ERRNO_OBJDETRANGEHWA_BASE-1)

/**
 * @brief   Error Code: Invalid argSize in DPM_InitCfg provided to @ref DPC_ObjectDetection_init,
 *          does not match the expected size of @ref DPC_ObjectDetectionRangeHWA_InitParams_t
 */
#define DPC_OBJDETRANGEHWA_EINVAL__INIT_CFG_ARGSIZE         (DP_ERRNO_OBJDETRANGEHWA_BASE-2)

/**
 * @brief   Error Code: Invalid argument bad command argument in DPM_ProcChainIoctlFxn for
 *                      Object detection DPC.
 */
#define DPC_OBJDETRANGEHWA_EINVAL__COMMAND                  (DP_ERRNO_OBJDETRANGEHWA_BASE-3)

/**
 * @brief   Error Code: Out of general heap memory
 */
#define DPC_OBJDETRANGEHWA_ENOMEM                           (DP_ERRNO_OBJDETRANGEHWA_BASE-10)

/**
 * @brief   Error Code: Out of L3 RAM during radar cube allocation.
 */
#define DPC_OBJDETRANGEHWA_ENOMEM__L3_RAM_RADAR_CUBE            (DP_ERRNO_OBJDETRANGEHWA_BASE-11)

/**
 * @brief   Error Code: Out of L3 RAM during detection matrix allocation.
 */
#define DPC_OBJDETRANGEHWA_ENOMEM__L3_RAM_DET_MATRIX            (DP_ERRNO_OBJDETRANGEHWA_BASE-12)

/**
 * @brief   Error Code: Out of HWA Window RAM
 */
#define DPC_OBJDETRANGEHWA_ENOMEM_HWA_WINDOW_RAM                (DP_ERRNO_OBJDETRANGEHWA_BASE-13)

/**
 * @brief   Error Code: Out of Core Local RAM for generating window coefficients
 *          for HWA when doing range DPU Config.DP_ERRNO_OBJECTDETECTION_BASE
 */
#define DPC_OBJDETRANGEHWA_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW    (DP_ERRNO_OBJDETRANGEHWA_BASE-14)


/**
 * @brief   Error Code: Pre-start config was received before pre-start common config.
 */
#define DPC_OBJDETRANGEHWA_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG  (DP_ERRNO_OBJDETRANGEHWA_BASE-30)

/**
 * @brief   Error Code: Internal error
 */
#define DPC_OBJDETRANGEHWA_EINTERNAL               (DP_ERRNO_OBJDETRANGEHWA_BASE-40)

/**
 * @brief   Error Code: Not implemented
 */
#define DPC_OBJDETRANGEHWA_ENOTIMPL                (DP_ERRNO_OBJDETRANGEHWA_BASE-50)
/**
@}
*/

/** @addtogroup DPC_OBJDETRANGEHWA_IOCTL__DEFINITIONS
 @{ */
/* Currently nothing here */
/**
@}
*/

/** @addtogroup DPC_OBJDETRANGEHWA_IOCTL__DATA_STRUCTURES
 @{ */

/**
 * @brief
 *  DC range signature compensation
 *
 * @details
 *  The structure contains the DC range antenna signature removeal configuration used in data path
 */
typedef struct DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! @brief   Calibration DC Range configuration */
    DPU_RangeProc_CalibDcRangeSigCfg cfg;
}DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg;


/*
 * @brief Memory Configuration used during init API
 */
typedef struct DPC_ObjectDetectionRangeHWA_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} DPC_ObjectDetectionRangeHWA_MemCfg;

/*!
*  @brief      Call back function type for calling back during process
*  @param[out] subFrameIndx Sub-frame indx [0..(numSubFrames-1)]
*/
typedef void (*DPC_ObjectDetectionRangeHWA_processCallBackFxn_t)(uint8_t subFrameIndx);

/*! @brief  Process call backs configuration */
typedef struct DPC_ObjectDetectionRangeHWA_ProcessCallBackCfg_t
{
    /*! @brief  Call back function that will be called at the beginning of frame
     *          processing (beginning of 1D) */
    DPC_ObjectDetectionRangeHWA_processCallBackFxn_t processFrameBeginCallBackFxn;

    /*! @brief  Call back function that will be called at the beginning of inter-frame
     *          processing (beginning of 2D) */
    DPC_ObjectDetectionRangeHWA_processCallBackFxn_t processInterFrameBeginCallBackFxn;
} DPC_ObjectDetectionRangeHWA_ProcessCallBackCfg;

/*
 * @brief Configuration for DPM's init API.
 *        DPM_init's arg = pointer to this structure.
 *        DPM_init's argLen = size of this structure.
 *
 */
typedef struct DPC_ObjectDetectionRangeHWA_InitParams_t
{
    /*! @brief   Handle to the hardware accelerator */
    HWA_Handle hwaHandle;

    /*! @brief   Handle of the EDMA driver. */
    EDMA_Handle edmaHandle;

    /*! @brief L3 RAM configuration. DPC will allocate memory from this
    *         as needed and report the amount of memory consumed through 
    *         @ref DPC_ObjectDetectionRangeHWA_PreStartCfg to application */
    DPC_ObjectDetectionRangeHWA_MemCfg L3ramCfg;

    /*! @brief Core Local RAM configuration (e.g data TCM for R4F).
    *         DPC will allocate memory from this as needed and report the amount
    *         of memory consumed through @ref DPC_ObjectDetectionRangeHWA_PreStartCfg
    *         to the application */
    DPC_ObjectDetectionRangeHWA_MemCfg CoreLocalRamCfg;

    /*! @brief  Call back function that will be called at the beginning of inter frame
    *          processing (end of 1D) */
    DPC_ObjectDetectionRangeHWA_ProcessCallBackCfg processCallBackFxn;
} DPC_ObjectDetectionRangeHWA_InitParams;


/*
 * @brief Configuration related to IOCTL API for command
 *        @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG. This is independent
 *        of sub frame.
 */
typedef struct DPC_ObjectDetectionRangeHWA_PreStartCommonCfg_t
{
    /*! @brief   Number of sub-frames */
    uint8_t numSubFrames;
} DPC_ObjectDetectionRangeHWA_PreStartCommonCfg;

/*
 * @brief Configuration related to IOCTL API for command
 *        @ref DPC_OBJDET_IOCTL__TRACKER_CFG. This is independent
 *        of sub frame.
 */
#ifdef TRACKERPROC_EN
typedef struct DPC_ObjectDetection_TrackerConfig_t
{
    /*! @brief   Tracker DPU configuration */
    DPU_TrackerProc_Config trackerDpuCfg;
} DPC_ObjectDetection_TrackerConfig;
#endif
/*
 * @brief Stats structure to convey to Application timing and related information.
 */
typedef struct DPC_ObjectDetectionRangeHWA_Stats_t
{
    /*! @brief   interChirpProcess margin in CPU cyctes */
    uint32_t      interChirpProcessingMargin;

    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStamp;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      interFrameStartTimeStamp;
} DPC_ObjectDetectionRangeHWA_Stats;

/*
 * @brief Static Configuration that is part of the pre-start configuration.
 */
typedef struct DPC_ObjectDetectionRangeHWA_StaticCfg_t
{
   /*! @brief      ADCBuf buffer interface */
   DPIF_ADCBufData  ADCBufData;

   /*! @brief  Number of transmit antennas */
   uint8_t     numTxAntennas;

   /*! @brief  Number of virtual antennas */
   uint8_t     numVirtualAntennas;

   /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
               @ref DPIF_ADCBufProperty_t::numAdcSamples. If range zoom is supported, this can be bigger than
               the minimum. */
   uint16_t    numRangeBins;

   /*! @brief  Number of chirps per frame */
   uint16_t    numChirpsPerFrame;

   /*! @brief  Number of chirps for Doppler computation purposes.
              For example, in TDM/BPM-MIMO scheme, this is the physical chirps
              in a frame per transmit antenna
              i.e numDopplerChirps = @ref numChirpsPerFrame / @ref numTxAntennas */
   uint16_t    numDopplerChirps;

   /*! @brief   Range FFT Tuning Params */
    DPU_RangeProcHWA_FFTtuning    rangeFFTtuning;

   /*! @brief   radar cube format See \ref DPIF_RADARCUBE_FORMAT. \n
                Values supported: DPIF_RADARCUBE_FORMAT_1 or DPIF_RADARCUBE_FORMAT_2.\n
                This should match the format expected by the processing unit that
                consumes the output of this DPC
    */
   uint8_t     radarCubeFormat;

} DPC_ObjectDetectionRangeHWA_StaticCfg;

/*
 * @brief Dynamic Configuration that is part of the pre-start configuration.
 */
typedef struct DPC_ObjectDetectionRangeHWA_DynCfg_t
{
    /*! @brief   Calibration DC Range configuration */
    DPU_RangeProc_CalibDcRangeSigCfg calibDcRangeSigCfg;
} DPC_ObjectDetectionRangeHWA_DynCfg;

/*
 * @brief  Structure related to radarCube memory usage.
 *        Since this is Range only DPC, radar.cube memory is allocated in 
 *        shared memory and its usage need to be reported back to be used in other DPC
 */
typedef struct DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem_t
{
    /*! @brief   Indicates address(current core) of radar cube in shared memory */
    void         *addr;

    /*! @brief   Indicates number of bytes of radar cube */
    uint32_t     size;
} DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem;

/*
 * @brief  Structure related to @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG
 *        IOCTL command. When the pre-start IOCTL is processed, it will report
 *        memory usage as part of @DPC_ObjectDetectionRangeHWA_PreStartCfg.
 */
typedef struct DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage_t
{
    /*! @brief   Indicates number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief   Indicates number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetectionRangeHWA_InitParams */
    uint32_t L3RamUsage;

    /*! @brief   Indicates number of bytes of Core Local memory allocated to be used by DPC */
    uint32_t CoreLocalRamTotal;

    /*! @brief   Indicates number of bytes of Core Local memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetectionRangeHWA_InitParams */
    uint32_t CoreLocalRamUsage;

    /*! @brief   Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief   Indicates number of bytes of system heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief   Indicates number of bytes of system heap used by DPC at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;
} DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage;

/*
 * @brief Configuration related to IOCTL API for command
 *        @ref DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG.
 *
 */
typedef struct DPC_ObjectDetectionRangeHWA_PreStartCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! Static configuration */
    DPC_ObjectDetectionRangeHWA_StaticCfg staticCfg;

    /*! Dynamic configuration */
    DPC_ObjectDetectionRangeHWA_DynCfg dynCfg;

    /*! Radar cube memroy information */
    DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem radarCubeMem;

    /*! Memory usage after the preStartCfg is applied */
    DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage memUsage;
} DPC_ObjectDetectionRangeHWA_PreStartCfg;

/**
@}
*/

/** @addtogroup DPC_OBJDETRANGEHWA__GLOBAL
 @{ */

/*! Application developers: Use this configuration to load the Object Detection DPC
 *  within the DPM. */
extern DPM_ProcChainCfg  gDPC_ObjDetRangeHWACfg;

/**
@}
*/

#ifdef __cplusplus
}
#endif

#endif /* DPC_OBJDETRANGEHWA_H */
