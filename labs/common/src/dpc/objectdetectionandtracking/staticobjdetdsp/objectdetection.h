/**
 *   @file  objectdetection.h
 *
 *   @brief
 *      Object Detection DPC (DSP chain) Header File
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

/** @mainpage Object Detection Data-path Processing Chain (DPC) with DSP based DPUs
 * [TOC]
 *  @section objdetdsp_intro Overview
 *
 *  The Object detection DPC provides the functionality of processing ADC samples
 *  to detect objects during the frame acquisition and inter-frame processing
 *  periods. It can be used by an application by registering with the DPM
 *  framework and invoked using DPM APIs. The external interface of Object detection
 *  DPC can be seen at @ref DPC_OBJDET_EXTERNAL
 *
 *   @image html object_detection_datapath.png "Object Detection Data Path Processing Chain"
 *   \n
 *   \n
 *   This data path chain processing consists of:
 *   - Processing during the chirps as seen in the timing diagram(optional):
 *     - This consists of 1D (range) FFT processing that takes input from multiple
 *     receive antennae from the ADC buffer for every chirp (corresponding to the
 *     chirping pattern on the transmit antennae)
 *     and performs FFT on it and generates output into the L3 RAM in the format
 *     defined by @ref DPIF_RADARCUBE_FORMAT_1. For more details, see the doxygen
 *     documentation of range processing DPU (Data Path Unit) located at:
 *       @verbatim
          ti/datapath/dpu/rangeproc/docs/doxygen/html/index.html
         @endverbatim
 *   - Processing during the time between the end of chirps until the beginning of the
 *     next chirping period, shown as "Inter frame Period" in the timing diagram.
 *     This processing consists of:
 *     - Doppler processing that takes input from 1D output in L3 RAM and performs
 *       2D FFT to give range-velocity detection matrix in the L3 RAM. For more details, see:
 *       @verbatim
          ti/datapath/dpc/dpu/dopplerproc/docs/doxygen/html/index.html
         @endverbatim
 *     - CFAR processing and peak grouping on detection matrix output of doppler processing.
 *       For more details, see:
 *       @verbatim
          ti/datapath/dpc/dpu/cfarcaproc/docs/doxygen/html/index.html
         @endverbatim
 *     - Angle (Azimuth, Elevation) of Arrival processing to produce a final list of
 *       detected objects with position coordinates (x,y,z) and velocity.
 *       For more details, see:
 *       @verbatim
          ti/datapath/dpc/dpu/aoaproc/docs/doxygen/html/index.html
         @endverbatim
 *
 *  @section objdetdsp_appdpcFlow Application-DPC Execution Flow
 *   Following diagram shows the application-DPC execution Flow.
 *
 *  @image html dpc_flow.png "Application-DPC Execution Flow"
 *
 *   The flow above shows the sequencing of initialization, configuration, execution and
 *   dynamic control operations of the DPC and some level of detail of
 *   what happens under these operations.
 *   Most of the hardware resource (e.g EDMA related) configuration
 *   for the DPUs that is issued by the DPC as part of processing
 *   @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG commands is provided by the
 *   application at build time using a resource file (DPC sources are built
 *   as part of building the application, there is no separate DPC library object).
 *   This file is passed as a compiler command line define
 *   @verbatim --define=APP_RESOURCE_FILE="fileName" @endverbatim The "fileName"
 *   above includes the path as if to include the file when building the the DPC sources
 *   as part of building the application, and any DPC source that needs to refer
 *   to this file (currently objectdetection.c) has the following code
 *    @verbatim #include APP_RESOURCE_FILE @endverbatim
 *   One of the demos that uses this DPC is located at ti/demo/xwr16xx/mmw. The
 *   resource file in this demo is mmw_res.h, this file shows all the definitions
 *   that are needed by the DPC from the application. This file is provided
 *   on compiler command line when building as follows:
 *   @verbatim --define=APP_RESOURCE_FILE="<ti/demo/xwr16xx/mmw/mmw_res.h>" @endverbatim
 *
 *   As seen in application execution flow, 2 options are given in objdetdsp DPC chain.
 *   - **Full DPC chain** with Range DPU, Doppler DPU , CFARCA DPU and AoA DPU.
 *        This is the default configuration. DPC accepts frameStart and chirpEvent(trigger DPC execution) from DPM. All DPUs
 *        will be excuted during DPC execution(@ref DPC_ObjectDetection_execute).
 *
 *   - **DPC without Range DPU**
 *        This option is enabled by compiler option defined as follows:
 *        @verbatim --define=OBJDET_NO_RANGE @endverbatim
 *        DPC accepts frameStart and data injection(trigger DPC execution) from DPM. All doppler DPU, cfarca DPU and AoA DPU will be
 *        executed during DPC execution(@ref DPC_ObjectDetection_execute).
 *   
 *  @section objdetdsp_memory Data Memory
 *
 *  @subsection objdetdsp_memCfg Memory configuration
 *   The configuration of L3 and Core Local L2 and L1 RAM (hitherto referred in short as LRAM)
 *   memories are provided by the application 
 *      - @ref DPC_ObjectDetection_InitParams_t::L3ramCfg,
 *      - @ref DPC_ObjectDetection_InitParams_t::CoreL1RamCfg and 
 *      - @ref DPC_ObjectDetection_InitParams_t::CoreL2RamCfg 
 *
 *   during @ref DPC_ObjectDetection_init (invoked by application through @ref DPM_init). 
 *   This configuration is the default memory configuration for the DPC.
 *
 *   This DPC can also accept share memory(L3 RAM) configuraiton during processing of 
 *   @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG command. This is used for option 2 of execution flow when range DPU is
 *   disabled and radarCube memory is allocated outside of this DPC. 
 *   
 *   @subsection objdetdsp_memPar Memory partition
 *   The L3 and LRAM partition happens during the processing of @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG command
 *   and is shown in the following figure. The allocation from application system heap
 *   (typically in LRAM) using the MemoryP_osal API is done during @ref DPC_ObjectDetection_init
 *   (object instances of DPC and DPUs for all sub-frames) and during the
 *   processing of @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG command (range DPUs
 *   dc antenna coupling signature buffer that is unique for each sub-frame)
 *   is also shown in the figure.
 *
 *
 *   @image html memory_allocation.png "Data memory allocation"
 *
 *   In the above picture, the L2 RAM shows allocation
 *   of the "cfarRngDopSnrList" (@ref DPIF_CFARDetList_t) outside of scratch usage
 *   as this is shared buffer between CFAR and AoA in the processing flow and
 *   therefore needs to persist within the sub-frame until AoA is executed at the
 *   end of the processing chain.
 *
 *   The buffers labeled "windowBuffer" and "twiddle Buffer" in the picture
 *   for range, doppler and AoA DPUs are allocated/generated during DPU configue time.
 *   AoA DPU needs the same "windowBuffer" and "twiddle Buffer" as used in doppler DPU to recompute the 2D
 *   doppler FFT, hence the same 2D windowing and twiddle buffers are provided to AoA DPU as well.
 *   
 *   In DPC, these "windowBuffer" and "twiddle buffer" are allocated in persistent memory(not overlapping with scratch
 *   buffers). This arrangement in memory makes the 2D window/twiddle sharable between Doppler and AoA DPUs.
 *   It also prevents window/twiddle buffer re-generation across frame boundary in non-advanced
 *   frame scenarios.
 *
 *   The AoA DPU API has been designed to require 2D-FFT window and twiddle buffer configuration
 *   (i.e configuration is not optional) because it may be used in contexts (unit test, other DPC flavors) where doppler 
 *   processing may not exist.
 *   The AoA interface buffers are consumed by the application at the end of the
 *   DPC execute API.
 * 
 *
 *   @subsection objdetdsp_reconfig DPU reconfiguration 
 *   DPU reconfiguration is related to data path processing across sub-frames for advanced frame configuraiton.
 *   In such cases, reconfiguration is required when switching sub-frames because scratch buffers and EDMA resources 
 *   are overlapped across sub-frames. Note that the DPU's xxx_config API
 *   is a full configuration API beyond the EDMA resources configuration (e.g static
 *   and dynamic configuration) so restricting to the full configuration
 *   would imply that no sub-frame specific DPU instantiation is necessary. However, the code illustrates separate 
 *   instances of DPUs for each sub-frame to demonstrate generality of the sub-frame solution,
 *   in the case where there may be specialized (partial) configuration APIs in an
 *   optimized implementation (that only configured the overlapped EDMA resources).
 *   The limiting to full configuration also means that all the code required to build the
 *   configuration structures of the DPUs during the pre-start config time either
 *   has to be repeated or alternatively, the configurations that were created
 *   during the pre-start config processing be saved and reused later.
 *   The latter path has been taken, all DPU configuration that is built
 *   during the pre-start processing is stored in separate storage, this can be
 *   located at @ref SubFrameObj_t::dpuCfg. However, parts of this reconfiguration that
 *   cannot be captured in this storage need to be repeated, namely, window generation
 *   (@ref DPC_ObjDetDSP_GenRangeWindow) and rx phase compensation
 *   (@ref DPC_ObjDetDSP_GetRxChPhaseComp).
 *
 *   The DPU top-level dynamic configuration structure contains
 *   pointers to the individual configurations (e.g see @ref DPU_AoAProc_DynamicConfig_t)
 *   so DPC stores the dynamic configuration in pre-start config (@ref DPC_ObjectDetection_DynCfg_t)
 *   in permanent storage (@ref SubFrameObj_t::dynCfg) and DPUs are passed from this storage area
 *   so that their pointers point to this permanent storage during reconfiguration.
 *   The static configuration during pre-start config (@ref DPC_ObjectDetection_StaticCfg_t)
 *   is also stored (@ref SubFrameObj_t::staticCfg) for convenient referencing when doing
 *   range bias and rx phase measurement (@ref DPC_ObjDetDSP_rangeBiasRxChPhaseMeasure)
 *   and the above referred rx ch phase computation during reconfiguration (@ref DPC_ObjDetDSP_GetRxChPhaseComp).
 *   Saving this entire static configuration is probably more than necessary (versus
 *   storing only the parameters that need to be referred in the above mentioned functions)
 *   but it keeps implementation simpler/cleaner and may be useful in debugging.
 *
 *  @section objdetdsp_calibration Range Bias and Rx Channel Gain/Offset Measurement and Compensation
 *
 *    Because of imperfections in antenna layouts on the board, RF delays in SOC, etc,
 *    there is need to calibrate the sensor to compensate for bias in the range estimation and
 *    receive channel gain and phase imperfections. The DPC provides the ability to
 *    do the measurement and compensation.
 *
 *    @anchor Figure_calibration
 *    @image html calibration.png "Range Bias and Rx Channel Gain/Offset Measurement and Compensation"
 *
 *   @subsection objdetdsp_measurement Measurement
 *     If the measurement (@ref DPC_ObjectDetection_MeasureRxChannelBiasCfg_t::enabled) is enabled in pre-start
 *     common config (@ref DPC_ObjectDetection_PreStartCommonCfg_t::measureRxChannelBiasCfg)
 *     or dynamic config (@ref DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE),
 *     it is assumed that a strong target like a corner reflector is kept at boresight at
 *     a distance of X = @ref DPC_ObjectDetection_MeasureRxChannelBiasCfg_t::targetDistance.
 *     Peak search is done after the 2D FFT in the 0th Doppler
 *     of the detection matrix within distance D = @ref DPC_ObjectDetection_MeasureRxChannelBiasCfg_t::searchWinSize
 *     around X i.e peak is searched between -D/2 + X and X + D/2.
 *     The peak position is then used to compute the
 *     square root of the sum of the magnitude squares of the virtual antennas
 *     (taken from 1D-FFT Radar Cube matrix) for the
 *     peak and its two nearest neighbors. These three magnitudes and their positions
 *     are used to do parabolic interpolation to find the more accurate peak location from
 *     which the range bias is estimated as this interpolated peak position minus
 *     the configured target distance X.
 *     The rx channel phase and gain estimation is done by finding the minimum
 *     of the magnitude squared of the virtual antennas and this minimum is used
 *     to scale the individual antennas so that the magnitude of the coefficients is
 *     always less than or equal to 1. Rx channel compensation coefficients are
 *     calculated according to following equations. The radar cube matrix is
 *     laid out as four dimensional matrix defined as @ref DPIF_RADARCUBE_FORMAT_1, i.e. as
 *       \f[
 *           \mbox{RadarCube}[\mbox{numTxPatterns}][\mbox{numDopplerChirps}][\mbox{numRxAnt}][\mbox{numRangeBins}]
 *       \f]
 *
 *     The received symbols are extracted from the 1D-FFT radar cube from the first chirp
 *     at the range index \f$i_{Max}\f$corresponding to the maximum in the searched range as
 *
 *       \f[
 *           x(i_{Tx},i_{Rx}) = \mbox{RadarCube}(i_{Tx},0,i_{Rx},i_{Max}), \;\;\;\; i_{Tx}=0,...,N_{TxAnt}-1, \;\;\;\; i_{Rx}=0,...,N_{RxAnt}-1
 *       \f]
 *
 *     The coefficients are calculated as
 *
 *       \f[
 *           c(i_{Tx},i_{Rx}) = \frac{x^*(i_{Tx},i_{Rx})}{|x(i_{Tx},i_{Rx})|^2}\;x_{Min}, \;\;\;\; i_{Tx}=0,...,N_{TxAnt}-1, \;\;\;\; i_{Rx}=0,...,N_{RxAnt}-1
 *       \f]
 *
 *     where
 *
 *       \f[
 *           x_{Min} = \min{|x(i_{Tx},i_{Rx})|}
 *       \f]
 *
 *     In the above calculations, the division by magnitude squared for each element
 *     achieves the gain compensation part i.e equalizes the different tx-rx path gains --
 *     during compensation (@ref objdetdsp_compensation), when the input antenna samples
 *     are multiplied with their corresponding above coefficients,
 *     the result has the same magnitude for all virtual antennas. Similarly,
 *     the multiplication by complex conjugate achieves phase compensation. The use of minimum
 *     of the data set allows for maximal utilization of the 16-bits of fixed point precision
 *     to represent the coefficients -- the compensation vector will have unity amplitude for the
 *     minimum element and less than unity for the rest.
 *     The compensation rx coefficients are in Q15 format.
 *     Refer to the function @ref DPC_ObjDetDSP_rangeBiasRxChPhaseMeasure
 *     which performs the measurements, the measurement
 *     result (@ref DPU_AoAProc_compRxChannelBiasCfg_t) is given out of the DPC
 *     to the application as part of the result structure produced by DPC's execute
 *     API (@ref DPC_ObjectDetection_ExecuteResult_t::compRxChanBiasMeasurement).
 *
 *   @subsection objdetdsp_compensation Compensation
 *     When compensation configuration (DPU_AoAProc_compRxChannelBiasCfg_t) is provided as part of
 *     pre-start common config (@ref DPC_ObjectDetection_PreStartCommonCfg_t::compRxChanCfg) or
 *     dynamic config (@ref DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE),
 *     as seen in figure @ref Figure_calibration, for each sub-frame the DPC
 *     determines the rx compensation vector (@ref  DPC_ObjDetDSP_GetRxChPhaseComp) to apply
 *     for that sub-frame (the resulting compensation vector is such that
 *     all antennas enabled for that configuration are contiguous in storage)
 *     depending on the antenna configuration for that sub-frame and passes this to the
 *     AoA DPU. During processing, AoA DPU will apply this contiguously stored vector
 *     on the virtual antenna array before performing angle computation.
 *
 *     @note Although the range bias estimation is immune to motion because
 *       the bias estimation is based on searching peak position after 2D FFT,
 *       the rx compensation coefficients are calculated from 1D FFT output,
 *       so this will not be immune to motion in the scene. Hence, during calibration
 *       care must exercised to ensure that there are no moving objects in the search
 *       window range.
 *
 *
 */
#ifndef DPC_OBJECTDETECTION_H
#define DPC_OBJECTDETECTION_H

/* MMWAVE Driver Include Files */
#include <ti/common/mmwave_error.h>
#include <ti/drivers/edma/edma.h>
#include <ti/control/dpm/dpm.h>

#ifndef OBJDET_NO_RANGE
#include <ti/datapath/dpu/rangeproc/rangeprocdsp.h>
#endif
#include <ti/datapath/dpif/dpif_adcdata.h>
#include <ti/datapath/dpc/dpu/staticclutterproc/staticclutterproc.h>
#include <ti/datapath/dpc/dpu/cfarcaproc/cfarcaprocdsp.h>
#include <ti/datapath/dpc/dpu/dopplerproc/dopplerprocdsp.h>
#include <common/src/dpu/staticdetproc/staticdetprocdsp.h>
#include <common/src/dpu/trackerproc/trackerproc.h>
#include <common/src/dpu/aoaproc/aoaprocdsp.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup DPC_OBJDET_EXTERNAL        Object Detection DPC (Data-path Processing Chain) External
 *
 * This DPC performs processes ADC samples and generates detected object list.
 */
/**
@defgroup DPC_OBJDET__GLOBAL                             Object Detection DPC Globals
@ingroup DPC_OBJDET_EXTERNAL
@brief
*   This section has a list of all the globals exposed by the Object detection DPC.
*/
/**
@defgroup DPC_OBJDET_IOCTL__DATA_STRUCTURES               Object Detection DPC Data Structures
@ingroup DPC_OBJDET_EXTERNAL
@brief
*   This section has a list of all the data structures which are a part of the DPC module
*   and which are exposed to the application
*/
/**
@defgroup DPC_OBJDET_IOCTL__COMMAND                       Object Detection DPC Configuration Commands
@ingroup DPC_OBJDET_EXTERNAL
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
@ingroup DPC_OBJDET_EXTERNAL
@brief
*   This section has a list of all the error codes returned when calling Object Detection DPC functions
*   during error conditions.
*/


/** @addtogroup DPC_OBJDET_IOCTL__DATA_STRUCTURES
 @{ */

#ifndef OBJDET_NO_RANGE
/**
 * @brief
 *  DC range signature compensation
 *
 * @details
 *  The structure contains the DC range antenna signature removeal configuration used in data path
 */
typedef struct DPC_ObjectDetection_CalibDcRangeSigCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! @brief   Calibration DC Range configuration */
    DPU_RangeProc_CalibDcRangeSigCfg cfg;
}DPC_ObjectDetection_CalibDcRangeSigCfg;
#endif

/**
 * @brief
 *  CFAR Configuration
 *
 * @details
 *  The structure contains the cfar configuration used in data path
 */
typedef struct DPC_ObjectDetection_CfarCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! @brief   CFAR Configuration */
    DPU_CFARCAProc_CfarCfg cfg;
}DPC_ObjectDetection_CfarCfg;

/*! @brief      Field of view configuration in range domain */
typedef struct DPC_ObjectDetection_fovRangeCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief      Field of view configuration in range domain */
   DPU_CFARCAProc_FovCfg cfg;
} DPC_ObjectDetection_fovRangeCfg;

/*! @brief      Field of view configuration in Doppler domain */
typedef struct DPC_ObjectDetection_fovDopplerCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief      Field of view configuration in Doppler domain */
   DPU_CFARCAProc_FovCfg cfg;
} DPC_ObjectDetection_fovDopplerCfg;

/**
 * @brief
 *  Multi-object beam forming Configuration.
 */
typedef struct DPC_ObjectDetection_MultiObjBeamFormingCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! @brief   Multi Object Beam Forming configuration */
    DPU_AoAProc_MultiObjBeamFormingCfg cfg;
}DPC_ObjectDetection_MultiObjBeamFormingCfg;

/*
 * @brief Field of view configuration for AoA.
 *
 */
typedef struct DPC_ObjectDetection_fovAoaCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief    Field of view configuration for AoA */
   DPU_AoAProc_FovAoaCfg cfg;
} DPC_ObjectDetection_fovAoaCfg;

/*
 * @brief heatmap generation for static detection.
 *
 */
typedef struct DPC_ObjectDetection_staticHeatmapGenCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief    Field of view configuration for AoA */
   DPU_StaticDetProc_heatmapGenCfg cfg;
} DPC_ObjectDetection_staticHeatmapGenCfg;

/*
 * @brief detection control for static detection.
 *
 */
typedef struct DPC_ObjectDetection_staticPeakDetectionCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief    Field of view configuration for AoA */
   DPU_StaticDetProc_peakDetectionCfg cfg;
} DPC_ObjectDetection_staticPeakDetectionCfg;

/*
 * @brief Extended maximum velocity configuration
 *
 */
typedef struct DPC_ObjectDetection_extMaxVelCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief    Field of view configuration for AoA */
   DPU_AoAProc_ExtendedMaxVelocityCfg cfg;
} DPC_ObjectDetection_extMaxVelCfg;

/*
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 */
typedef struct DPC_ObjectDetection_RangeAzimuthHeatMapCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
   uint8_t subFrameNum;

   /*! @brief     Flag indicates to prepare data for azimuth heat-map */
   bool  prepareRangeAzimuthHeatMap;
} DPC_ObjectDetection_RangeAzimuthHeatMapCfg;

/**
 * @brief
 *  Static clutter removal base (subframe agnostic) configuration
 *
 * @details
 *  The structure contains Static clutter removal configuration
 */
typedef struct DPC_ObjectDetection_StaticClutterRemovalCfg_Base_t
{
    /*! @brief   enabled flag: 1-enabled 0-disabled */
    uint8_t enabled;
} DPC_ObjectDetection_StaticClutterRemovalCfg_Base;

/**
 * @brief
 *  Static clutter removal configuration
 *
 * @details
 *  The structure contains static clutter removal configuration
 */
typedef struct DPC_ObjectDetection_StaticClutterRemovalCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t     subFrameNum;

    /*! @brief   Static clutter Removal Cfg */
    DPC_ObjectDetection_StaticClutterRemovalCfg_Base cfg;
} DPC_ObjectDetection_StaticClutterRemovalCfg;

/**
 * @brief Range Bias and rx channel gain/phase measurement configuration.
 *
 */
typedef struct DPC_ObjectDetection_MeasureRxChannelBiasCfg_t
{
    /*! @brief  1-enabled 0-disabled */
    uint8_t enabled;

    /*! @brief  Target distance during measurement (in meters) */
    float targetDistance;

    /*! @brief  Search window size (in meters), the search is done in range
     *          [-searchWinSize/2 + targetDistance, targetDistance + searchWinSize/2] */
    float searchWinSize;
} DPC_ObjectDetection_MeasureRxChannelBiasCfg;

/*!
*  @brief      Call back function type for calling back during process
*  @param[out] subFrameIndx Sub-frame indx [0..(numSubFrames-1)]
*/
typedef void (*DPC_ObjectDetection_processCallBackFxn_t)(uint8_t subFrameIndx);

/*! @brief  Process call backs configuration */
typedef struct DPC_ObjectDetection_ProcessCallBackCfg_t
{
    /*! @brief  Call back function that will be called at the beginning of frame
     *          processing (beginning of 1D) */
    DPC_ObjectDetection_processCallBackFxn_t processFrameBeginCallBackFxn;

    /*! @brief  Call back function that will be called at the beginning of inter-frame
     *          processing (beginning of 2D) */
    DPC_ObjectDetection_processCallBackFxn_t processInterFrameBeginCallBackFxn;
} DPC_ObjectDetection_ProcessCallBackCfg;

/*
 * @brief Memory Configuration used during init API
 */
typedef struct DPC_ObjectDetection_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} DPC_ObjectDetection_MemCfg;

/*
 * @brief Configuration for DPM's init API.
 *        DPM_init's arg = pointer to this structure.
 *        DPM_init's argLen = size of this structure.
 *
 */
typedef struct DPC_ObjectDetection_InitParams_t
{
   /*! @brief   Handle of the EDMA driver. */
   EDMA_Handle edmaHandle[EDMA_NUM_CC];

   /*! @brief L3 RAM configuration. DPC will allocate memory from this
    *         as needed and report the amount of memory consumed through 
    *         @ref DPC_ObjectDetection_PreStartCfg to application */
   DPC_ObjectDetection_MemCfg L3ramCfg;

   /*! @brief Core L2 RAM configuration (DSP L2 memory).
    *         DPC will allocate memory from this as needed and report the amount
    *         of memory consumed through @ref DPC_ObjectDetection_PreStartCfg
    *         to the application */
   DPC_ObjectDetection_MemCfg CoreL2RamCfg;

   /*! @brief Core L1 RAM configuration (DSP L1 memory).
    *         DPC will allocate memory from this as needed and report the amount
    *         of memory consumed through @ref DPC_ObjectDetection_PreStartCfg
    *         to the application */
   DPC_ObjectDetection_MemCfg CoreL1RamCfg;

   /*! @brief   Process call back function configuration */
   DPC_ObjectDetection_ProcessCallBackCfg processCallBackCfg;
} DPC_ObjectDetection_InitParams;

/*
 * @brief Static Configuration that is part of the pre-start configuration.
 */
typedef struct DPC_ObjectDetection_StaticCfg_t
{
    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData  ADCBufData;

    /*! @brief  Rx Antenna order */
    uint8_t     rxAntOrder[SYS_COMMON_NUM_RX_CHANNEL];

    /*! @brief  Tx Antenna order */
    uint8_t     txAntOrder[SYS_COMMON_NUM_TX_ANTENNAS];

    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Number of virtual azimuth antennas */
    uint8_t     numVirtualAntAzim;

    /*! @brief  Number of virtual elevation antennas */
    uint8_t     numVirtualAntElev;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
               @ref DPIF_ADCBufProperty_t::numAdcSamples. If range zoom is supported, this can be bigger than
               the minimum. */
    uint16_t    numRangeBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief Number of chirps for Doppler computation purposes.
              For example, in TDM/BPM-MIMO scheme, this is the physical chirps
              in a frame per transmit antenna
              i.e numDopplerChirps = @ref numChirpsPerFrame / @ref numTxAntennas */
    uint16_t    numDopplerChirps;

    /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
               @ref numDopplerChirps. If Doppler zoom is supported, this can be bigger
               than the minimum. */
    uint16_t    numDopplerBins;

    /*! @brief  Range conversion factor for FFT range index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;

    /*! @brief   1 if valid profile has one Tx per chirp else 0 */
    uint8_t     isValidProfileHasOneTxPerChirp;

    /*! @brief  Flag that indicates if BPM is enabled. 
                BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;
} DPC_ObjectDetection_StaticCfg;

/*
 * @brief Dynamic Configuration that is part of the pre-start configuration.
 */
typedef struct DPC_ObjectDetection_DynCfg_t
{
#ifndef OBJDET_NO_RANGE
    /*! @brief   Calibration DC Range configuration */
    DPU_RangeProc_CalibDcRangeSigCfg calibDcRangeSigCfg;
#endif
    /*! @brief      CFAR configuration in range direction */
    DPU_CFARCAProc_CfarCfg cfarCfgRange;

    /*! @brief      CFAR configuration in Doppler direction */
    DPU_CFARCAProc_CfarCfg cfarCfgDoppler;

    /*! @brief      Field of view configuration in range domain */
    DPU_CFARCAProc_FovCfg fovRange;

    /*! @brief      Field of view configuration in Doppler domain */
    DPU_CFARCAProc_FovCfg fovDoppler;

    /*! @brief   Multi Object Beam Forming configuration */
    DPU_AoAProc_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

    /*! @brief     Flag indicates to prepare data for azimuth heat-map */
    bool  prepareRangeAzimuthHeatMap;

    /*! @brief      Field of view configuration for AoA */
    DPU_AoAProc_FovAoaCfg fovAoaCfg;

    /*! @brief      Extended maximum velocity configuration */
    DPU_AoAProc_ExtendedMaxVelocityCfg extMaxVelCfg;
	
	/*! @brief      Static Detection heatmap generation configuration */
	DPU_StaticDetProc_heatmapGenCfg heatmapGenCfg;
		
    /*! @brief      Static Detection configuration */
    DPU_StaticDetProc_peakDetectionCfg staticDetectCfg; 

    /*! @brief   Static Clutter Removal Cfg */
    DPC_ObjectDetection_StaticClutterRemovalCfg_Base staticClutterRemovalCfg;
} DPC_ObjectDetection_DynCfg;

/*
 * @brief Configuration related to share memory allocation at run-time.
 *      These configuration should overwrite init time configuration once enabled.
 *      It is used to share memory across DPCs
 */
typedef struct DPC_ObjectDetection_ShareMemCfg_t
{
    /*! @brief   Enable run-time share memory configuration */
    bool                            shareMemEnable;

    /*! @brief   L3RAM run-time configuration */
    DPC_ObjectDetection_MemCfg      L3Ram;

    /*! @brief   L3RAM run-time configuration */
    DPC_ObjectDetection_MemCfg      radarCubeMem;
} DPC_ObjectDetection_ShareMemCfg;

/*
 * @brief Configuration related to IOCTL API for command
 *        @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG. This is independent
 *        of sub frame.
 */
typedef struct DPC_ObjectDetection_PreStartCommonCfg_t
{
    /*! @brief   Number of sub-frames */
    uint8_t numSubFrames;

    /*! @brief   Range Bias and rx channel gain/phase measurement configuration */
    DPC_ObjectDetection_MeasureRxChannelBiasCfg measureRxChannelBiasCfg;

    /*! @brief   Range Bias and rx channel gain/phase compensation configuration */
    DPU_AoAProc_compRxChannelBiasCfg compRxChanCfg;
} DPC_ObjectDetection_PreStartCommonCfg;

/*
 * @brief  Structure related to @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG
 *        IOCTL command. When the pre-start IOCTL is processed, it will report
 *        memory usage as part of @DPC_ObjectDetection_PreStartCfg.
 */
typedef struct DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage_t
{
    /*! @brief   Indicates number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief   Indicates number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t L3RamUsage;

    /*! @brief   Indicates number of bytes of Core Local memory allocated to be used by DPC */
    uint32_t CoreL2RamTotal;

    /*! @brief   Indicates number of bytes of Core Local memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t CoreL2RamUsage;

    /*! @brief   Indicates number of bytes of Core Local memory allocated to be used by DPC */
    uint32_t CoreL1RamTotal;

    /*! @brief   Indicates number of bytes of Core Local memory used by DPC from the allocated
     *           amount indicated through @ref DPC_ObjectDetection_InitParams */
    uint32_t CoreL1RamUsage;

    /*! @brief   Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief   Indicates number of bytes of system heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief   Indicates number of bytes of system heap used by DCP at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;
} DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage;

/*
 * @brief Configuration related to IOCTL API for command
 *        @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG.
 *
 */
typedef struct DPC_ObjectDetection_PreStartCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! @brief   Run-time share memory configuraiton. This configuation
     *           should be used to overwrite the init time configuration if available.
     */
    DPC_ObjectDetection_ShareMemCfg    shareMemCfg;

    /*! Static configuration */
    DPC_ObjectDetection_StaticCfg staticCfg;

    /*! Dynamic configuration */
    DPC_ObjectDetection_DynCfg dynCfg;

    /*! Memory usage after the preStartCfg is applied */
    DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage memUsage;
} DPC_ObjectDetection_PreStartCfg;

/*
 * @brief Stats structure to convey to Application timing and related information.
 */
typedef struct DPC_ObjectDetection_Stats_t
{
    /*! @brief   interChirpProcess margin in CPU cyctes */
    uint32_t      interChirpProcessingMargin;

    /*! @brief   Counter which tracks the number of frame start interrupt */
    uint32_t      frameStartIntCounter;

    /*! @brief   Frame start CPU time stamp */
    uint32_t      frameStartTimeStamp;

    /*! @brief   Inter-frame start CPU time stamp */
    uint32_t      interFrameStartTimeStamp;

    /*! @brief   Inter-frame end CPU time stamp */
    uint32_t      interFrameEndTimeStamp;

    /*! @brief Sub frame preparation cycles. Note when this is reported as part of
     *         the process result reporting, then it indicates the cycles that took
     *         place in the previous sub-frame/frame for preparing to switch to
     *         the sub-frame that is being reported because switching happens
     *         in the processing of DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
     *         which is after the DPC process. */
    uint32_t      subFramePreparationCycles;
} DPC_ObjectDetection_Stats;

typedef struct DPC_PointCloud_Results_t
{
  /*! @brief      Number of detected objects */
    uint32_t        numObjOut;

    /*! @brief      Detected objects output list of @ref numObjOut elements */
    DPIF_PointCloudCartesian *objOut;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              of @ref numObjOut elements */
    DPIF_PointCloudSideInfo *objOutSideInfo;
}DPC_PointCloud_Results;

/*
 * @brief This is the result structure reported from DPC's registered processing function
 *        to the application through the DPM_Buffer structure. The DPM_Buffer's
 *        first fields will be populated as follows:
 *        pointer[0] = pointer to this structure.
 *        size[0] = size of this structure i.e sizeof(DPC_ObjectDetection_Result)
 *
 *        pointer[1..3] = NULL and size[1..3] = 0.
 */
typedef struct DPC_ObjectDetection_ExecuteResult_t
{
    /*! @brief      Sub-frame index, this is in the range [0..numSubFrames - 1] */
    uint8_t         subFrameIdx;

 	/*! @brief  Dynamic Point Cloud results */
    DPC_PointCloud_Results  dynamicPC;
	/*! @brief  Static Point Cloud results */
	DPC_PointCloud_Results  staticPC;
	
    /*! @brief      Detected objects output list (Spherical format) of @ref numObjOut elements */
    DPIF_PointCloudSpherical *objOutSph;

    /*! @brief      Number of targets detected by the tracker */
    uint32_t        numTargets;    
    
    /*! @brief      List of targets detected by the tracker */
    trackerProc_Target  *tList;
    
    /*! @brief      Number of indices for detected targets */
    uint32_t        numIndices;                              
    
    /*! @brief      List of indexes for detected targets */
    trackerProc_TargetIndex *tIndex;
    
    /*! @brief      Valid side (ping/pong) of target descriptor buffer */
    bool            trackerCurrDesc;         

    /*! @brief      Detection Matrix structure */
    DPIF_DetMatrix  detMatrix;
	
    /*! @brief      Radar Cube structure */
    DPIF_RadarCube  radarCube;
	

    /*! @brief      Pointer to range-azimuth static heat map, this is a 2D FFT
     *              array in range direction (cmplx16ImRe_t x[numRangeBins][numVirtualAntAzim]),
     *              at doppler index 0 */
    cmplx16ImRe_t   *azimuthStaticHeatMap;

    /*! @brief      Number of elements of @ref azimuthStaticHeatMap, this will be
     *              @ref DPC_ObjectDetection_StaticCfg_t::numVirtualAntAzim *
     *              @ref DPC_ObjectDetection_StaticCfg_t::numRangeBins */
    uint32_t        azimuthStaticHeatMapSize;

    /*! @brief      Pointer to DPC stats structure */
    DPC_ObjectDetection_Stats *stats;

    /*! @brief   Pointer to Range Bias and rx channel gain/phase compensation measurement
     *           result. Note the contents of this pointer are independent of sub-frame
     *           i.e all sub-frames will report the same result although it is
     *           expected that when measurement is enabled,
     *           the number of sub-frames will be 1 (i.e advanced frame
     *           feature will be disabled). If measurement
     *           was not enabled, then this pointer will be NULL. */
    DPU_AoAProc_compRxChannelBiasCfg *compRxChanBiasMeasurement;
	
#if 0 	
    /*! @brief      The flag to see whether the heatmap generation is ready */
    uint32_t        recordingHeatmapReady;

    /*! @brief      The flag to see whether the heatmap is already processed */
    //uint32_t        recordingHeatmapDone;

    /*! @brief      The pointer to the recorded heatmap location */
    uint32_t           *rangeAzimuthHeatmap;

    /*! @brief      The pointer to the recorded heatmap location */
    uint32_t           *rangeElevationHeatmap;

    uint32_t        recordingHeatmapSize;	
#endif	
	
} DPC_ObjectDetection_ExecuteResult;

/*
 * @brief This is the informational structure related to the IOCTL command
 *        @ref DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED.
 */
typedef struct DPC_ObjectDetection_ExecuteResultExportedInfo_t
{
    /*! @brief      Sub-frame index, this is in the range [0..numSubFrames - 1].
     *              This is the sub-frame whose results have been exported.
     *              Although this DPC implementation knows what sub-frame to expect as the exports
     *              are expected to be sequential in sub-frames, this field helps
     *              in error checking when for example the application could miss
     *              exporting/consuming a sub-frame in a timely manner or have out of order
     *              export/consumption. */
    uint8_t         subFrameIdx;
} DPC_ObjectDetection_ExecuteResultExportedInfo;

/**
@}
*/

/** @addtogroup DPC_OBJDET_IOCTL__COMMAND
 @{ */

/**
 * @brief Command associated with @ref DPC_ObjectDetection_PreStartCfg_t. In this IOCTL, the sub-frame's
 *        configurations will be processed by configuring individual DPUs.
 *        The @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG must be issued
 *        before issuing this IOCTL.
 */
#define DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG                            (DPM_CMD_DPC_START_INDEX + 0U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_PreStartCommonCfg_t. Must be issued before
 *        issuing @ref DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG
 */
#define DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG                     (DPM_CMD_DPC_START_INDEX + 1U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_CfarCfg_t for range dimension.
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_CFAR_RANGE_CFG                          (DPM_CMD_DPC_START_INDEX + 2U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_CfarCfg_t for doppler dimension.
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_CFAR_DOPPLER_CFG                        (DPM_CMD_DPC_START_INDEX + 3U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_MultiObjBeamFormingCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_MULTI_OBJ_BEAM_FORM_CFG                 (DPM_CMD_DPC_START_INDEX + 4U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_CalibDcRangeSigCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG                  (DPM_CMD_DPC_START_INDEX + 5U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_StaticClutterRemovalCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_STATICCLUTTER_REMOVAL_CFG                     (DPM_CMD_DPC_START_INDEX + 6U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_MeasureRxChannelBiasCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE   (DPM_CMD_DPC_START_INDEX + 7U)

/**
 * @brief Command associated with @ref DPU_AoAProc_compRxChannelBiasCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE       (DPM_CMD_DPC_START_INDEX + 8U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_fovRangeCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_FOV_RANGE                               (DPM_CMD_DPC_START_INDEX + 9U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_fovDopplerCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_FOV_DOPPLER                             (DPM_CMD_DPC_START_INDEX + 10U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_fovAoaCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_FOV_AOA                                 (DPM_CMD_DPC_START_INDEX + 11U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_RangeAzimuthHeatMapCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_RANGE_AZIMUTH_HEAT_MAP                  (DPM_CMD_DPC_START_INDEX + 12U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_extMaxVelCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_EXT_MAX_VELOCITY                        (DPM_CMD_DPC_START_INDEX + 13U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_heatmapGenCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_HEATMAPGEN_STATICDET                    (DPM_CMD_DPC_START_INDEX + 14U)

/**
 * @brief Command associated with @ref DPC_ObjectDetection_staticPeakDetectionCfg_t
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_DETECTION_STATICDET                    (DPM_CMD_DPC_START_INDEX + 15U)

/**
 * @brief This commands indicates to the DPC that the results DPC provided to the application
 *        through its execute API (which application will access through DPM_execute API)
 *        have been exported/consumed. The purpose of this command is for DPC to
 *        reclaim the memory resources associated with the results. The DPC may
 *        also perform sub-frame switching, and do error-checking to see
 *        if export was later than expected e.g the DPC design may be such that
 *        the previous frame/sub-frame's export notification may need to come
 *        after a new frame/sub-frame (this is the case currently with this
 *        object detection DPC). The DPC will also consider this command as a
 *        signal from the application that all its processing for the current frame/sub-frame
 *        has been done and so if a new frame/sub-frame interrupt (DPC has registered
 *        a frame interrupt with the DPM) comes before the last step in the
 *        processing of this command (which could be sub-frame switching and
 *        preparing for next sub-frame/frame), then the DPC will signal an assert
 *        to the application from its frame interrupt. The expected sequence is
 *        the following:
 *
 *        1. App consumes the process result of the DPC (e.g sending output on UART).
 *        2. App performs any dynamic configuration command processing by issuing DPC's
 *           IOCTL APIs for the next frame/sub-frame.
 *        3. App issues this result-exported IOCTL.
 *        4. DPC does its processing related to this IOCTL in the following sequence:
 *            a. May do error checking and preparing for next sub-frame/frame.
 *            b. Do book-keeping related to marking this as end of sub-frame/frame processing
 *               by the app. The DPC's registered frame start interrupt performs
 *               check on this information to see if next frame/sub-frame came before
 *               this end of processing in which case it will issue an assert to app.
 *
 *        An informational structure @ref DPC_ObjectDetection_ExecuteResultExportedInfo_t
 *        is associated with this command.
 */
#define DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED                     (DPM_CMD_DPC_START_INDEX + 14U)

/**
 * @brief This command is for non real-time (without RF) testing. When issued, it will simulate
 *        the trigger of frame start. No configuration structure is associated with this command.
 *        Must be issued between start and stop of DPC.
 */
#define DPC_OBJDET_IOCTL__TRIGGER_FRAME                                     (DPM_CMD_DPC_START_INDEX + 15U)

/**
 * @brief Command is for non real-time (without RF) testing. When issued, it will simulate the trigger
        of chirp event. No argument associated with this command.
 */
#define DPC_OBJDET_IOCTL__TRIGGER_CHIRP                                     (DPM_CMD_DPC_START_INDEX + 16U)
/**
 * @brief Command associated with @ref DPC_ObjectDetection_TrackerConfig_t
 */
#define DPC_OBJDET_IOCTL__STATIC_TRACKER_CFG                              (DPM_CMD_DPC_START_INDEX + 17U)

/**
 * @brief This is not a command, just to capture the last command supported in DPC.
        This definition is used to track if the commond is targed for this DPC, otherwise return an error.
 */
#define DPC_OBJDET_IOCTL__MAX                                               (DPC_OBJDET_IOCTL__STATIC_TRACKER_CFG + 1)

/**
@}
*/

/** @addtogroup DPC_OBJECTDETECTION_ERROR_CODE
 *  Base error code for the objdetdsp DPC is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument general (such as NULL argument pointer)
 */
#define DPC_OBJECTDETECTION_EINVAL                                          (DP_ERRNO_OBJDETDSP_BASE - 1)

/**
 * @brief   Error Code: Invalid argSize in DPM_InitCfg provided to @ref DPC_ObjectDetection_init,
 *          does not match the expected size of @ref DPC_ObjectDetection_InitParams_t
 */
#define DPC_OBJECTDETECTION_EINVAL__INIT_CFG_ARGSIZE                       (DP_ERRNO_OBJDETDSP_BASE - 2)

/**
 * @brief   Error Code: Invalid argument in configuration @ref DPC_ObjectDetection_MeasureRxChannelBiasCfg_t
 */
#define DPC_OBJECTDETECTION_EINVAL__MEASURE_RX_CHANNEL_BIAS_CFG            (DP_ERRNO_OBJDETDSP_BASE - 3)

/**
 * @brief   Error Code: Invalid argument bad command argument in DPM_ProcChainIoctlFxn for
 *                      Object detection DPC.
 */
#define DPC_OBJECTDETECTION_EINVAL__COMMAND                                 (DP_ERRNO_OBJDETDSP_BASE - 4)

/**
 * @brief   Error Code: Out of general heap memory
 */
#define DPC_OBJECTDETECTION_ENOMEM                                          (DP_ERRNO_OBJDETDSP_BASE - 10)

/**
 * @brief   Error Code: Out of L3 RAM during radar cube allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_RADAR_CUBE                       (DP_ERRNO_OBJDETDSP_BASE - 11)

/**
 * @brief   Error Code: Out of L3 RAM during detection matrix allocation.
 */
#define DPC_OBJECTDETECTION_ENOMEM__L3_RAM_DET_MATRIX                       (DP_ERRNO_OBJDETDSP_BASE - 12)

/**
 * @brief   Error Code: Pre-start config was received before pre-start common config.
 */
#define DPC_OBJECTDETECTION_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG  (DP_ERRNO_OBJDETDSP_BASE - 13)

/**
 * @brief   Error Code: Internal error
 */
#define DPC_OBJECTDETECTION_EINTERNAL                                       (DP_ERRNO_OBJDETDSP_BASE - 40)

/**
 * @brief   Error Code: Not implemented
 */
#define DPC_OBJECTDETECTION_ENOTIMPL                                        (DP_ERRNO_OBJDETDSP_BASE - 50)

/**
@}
*/

/** @addtogroup DPC_OBJDET__GLOBAL
 @{ */

/*! Application developers: Use this configuration to load the Object Detection DPC
 *  within the DPM. */
extern DPM_ProcChainCfg  gDPC_ObjectDetectionCfg;

/**
@}
*/
#ifdef __cplusplus
}
#endif

#endif /* DPC_OBJECTDETECTION_H */
