/**
 *   @file  mss_main.c
 *
 *   @brief
 *      This is the main file which implements the millimeter wave Demo
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

/** @mainpage Millimeter Wave (mmw) Demo for XWR68XX
 * [TOC]
 *  @section intro_sec Introduction
 *
 *  @image html toplevel.png
 *
 *  The millimeter wave demo shows some of the capabilities of the XWR68xx SoC
 *  using the drivers in the mmWave SDK (Software Development Kit).
 *  It allows user to specify the chirping profile and displays the detected
 *  objects and other information in real-time.
 *
 *  Following is a high level description of the features of this demo:
 *  - Be able to specify desired chirping profile through command line interface (CLI)
 *    on a UART port or through the TI Gallery App - **mmWave Demo Visualizer** -
 *    that allows user to provide a variety of profile configurations via the UART input port
 *    and displays the streamed detected output from another UART port in real-time,
 *    as seen in picture above.
 *  - Some sample profile configurations have been provided in the demo directory that can be
 *    used with CLI directly or via **mmWave Demo Visualizer** under following directory:
 *    @verbatim mmw/profiles @endverbatim
 *  - Do 1D, 2D, CFAR, Azimuth and Elevation processing and stream out velocity
 *    and three spatial coordinates (x,y,z) of the detected objects in real-time.
 *    The demo can also be configured to do 2D only detection (velocity and x,y coordinates).
 *  - Various display options besides object detection like azimuth heat map and
 *    Doppler-range heat map.
 *
 *  @section limit Limitations
 *  - Because of UART speed limit (< 1 Mbps), the frame time is more restrictive.
 *    For example, for the azimuth and Doppler heat maps for 256 FFT range and
 *    16 point FFT Doppler, it takes about 200 ms to transmit.
 *  - For most boards, a range bias of few centimeters has been observed. User can estimate
 *    the range bias on their board and correct using the calibration procedure
 *    described in @ref Calibration_section.
 *
 *  @section systemFlow System Execution Flow
 *  The millimeter wave demo runs on R4F (MSS) and DSP(DSS). Following diagram shows the system
 *  execution flow
 *
 *  @image html system_flow.png "System Execution Flow"
 *
 *  @section tasks Software Tasks
 *    The demo consists of the following (SYSBIOS) tasks:
 *
 *    **MSS**
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      MMWave module (MMWave_init), DPM module (DPM_init), open UART and data
 *      path related drivers (ADCBUF), and create/launch the following tasks
 *      (the @ref CLI_task is launched indirectly by calling @ref CLI_open).
 *    - @ref CLI_task. This command line interface task provides a simplified 'shell' interface
 *      which allows the configuration of the BSS via the mmWave interface (MMWave_config).
 *      It parses input CLI configuration commands like chirp profile and GUI configuration.
 *      When sensor start CLI command is parsed, all actions related to starting sensor and
 *      starting the processing the data path are taken.
 *      When sensor stop CLI command is parsed, all actions related to stopping the sensor and
 *      stopping the processing of the data path are taken
 *    - @ref MmwDemo_mmWaveCtrlTask. This task is used to provide an execution
 *      context for the mmWave control, it calls in an endless loop the MMWave_execute API.
 *    - @ref mmwDemo_mssDPMTask. This task is used to provide an execution context for 
 *      DPM (Data Path Manager) execution, it calls in an endless loop. There is no DPC registered
 *      with DPM.
 *
 *    **DSS**
 *    - @ref MmwDemo_initTask. This task is created/launched by @ref main and is a
 *      one-time active task whose main functionality is to initialize drivers (\<driver\>_init),
 *      DPM module (DPM_init), data path related drivers (EDMA, HWA), and
 *      create/launch the following tasks.
 *    - @ref MmwDemo_DPC_ObjectDetection_dpmTask. This task is used to provide an execution
 *      context for DPM (Data Path Manager) execution, it calls in an endless loop
 *      the DPM_execute API. In this context, all of the registered object detection
 *      DPC (Data Path Chain) APIs like configuration, control and execute will
 *      take place. In this task. When the DPC's execute API produces the detected objects and other
 *      results, they are reported to MSS where they are transmitted out of the UART port for display 
 *      using the visualizer.
 *
 *   @section datapath Data Path
 *   @image html datapath_overall.png "Top Level Data Path Processing Chain"
 *   \n
 *   \n
 *   @image html datapath_overall_timing.png "Top Level Data Path Timing"
 *
 *   The data path processing consists of taking ADC samples as input and producing
 *   detected objects (point-cloud and other information) to be shipped out of
 *   UART port to the PC. The algorithm processing is realized using
 *   the DPM registered Object Detection DPC. The details of the processing in DPC
 *   can be seen from the following doxygen documentation:
 *   **Chirp processing DPC(objdetrangehwa)**
 *   @verbatim
      ti/datapath/dpc/objectdetection/objdetrangehwa/docs/doxygen/html/index.html
     @endverbatim
 *   **inter frame processing DPC(objdetdsp)**
 *   @verbatim
      ti/datapath/dpc/objectdetection/objdetdsp/docs/doxygen/html/index.html
     @endverbatim
 *
 *  @section output Output information sent to host
 *      @subsection output_general Output Packet
 *      Output packets with the detection information are sent out every frame
 *      through the UART. Each packet consists of the header @ref MmwDemo_output_message_header_t
 *      and the number of TLV items containing various data information with
 *      types enumerated in @ref MmwDemo_output_message_type_e. The numerical values
 *      of the types can be found in @ref mmw_output.h. Each TLV
 *      item consists of type, length (@ref MmwDemo_output_message_tl_t) and payload information.
 *      The structure of the output packet is illustrated in the following figure.
 *      Since the length of the packet depends on the number of detected objects
 *      it can vary from frame to frame. The end of the packet is padded so that
 *      the total packet length is always multiple of 32 Bytes.
 *
 *      @image html output_packet_uart.png "Output packet structure sent to UART"
 *
 *      The following subsections describe the structure of each TLV.
 *
 *      @subsection tlv1 List of detected objects
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudCartesian_t)
 *
 *       Value: Array of detected objects. The information of each detected object
 *       is as per the structure @ref DPIF_PointCloudCartesian_t. When the number of
 *       detected objects is zero, this TLV item is not sent. The maximum number of
 *       objects that can be detected in a sub-frame/frame is @ref DPC_OBJDET_MAX_NUM_OBJECTS.
 *
 *       The orientation of x,y and z axes relative to the sensor is as per the
 *       following figure.
 *
 *        @image html coordinate_geometry.png "Coordinate Geometry"
 *
 *       The whole detected objects TLV structure is illustrated in figure below.
 *
 *       @image html detected_objects_tlv.png "Detected objects TLV"
 *
 *      @subsection tlv2 Range profile
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: Array of profile points at 0th Doppler (stationary objects).
 *       The points represent the sum of log2 magnitudes of received antennas
 *       expressed in Q9 format.
 *
 *      @subsection tlv3 Noise floor profile
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_NOISE_PROFILE)
 *
 *       Length: (Range FFT size) x (size of uint16_t)
 *
 *       Value: This is the same format as range profile but the profile
 *       is at the maximum Doppler bin (maximum speed
 *       objects). In general for stationary scene, there would be no objects or clutter at
 *       maximum speed so the range profile at such speed represents the
 *       receiver noise floor.
 *
 *      @subsection tlv4 Azimuth static heatmap
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Number of virtual antennas) (size of @ref cmplx16ImRe_t_)
 *
 *       Value: Array @ref DPU_AoAProcHWA_HW_Resources::azimuthStaticHeatMap. The antenna data
 *       are complex symbols, with imaginary first and real second in the following order:\n
 *       @verbatim
             Imag(ant 0, range 0), Real(ant 0, range 0),...,Imag(ant N-1, range 0),Real(ant N-1, range 0)
             ...
             Imag(ant 0, range R-1), Real(ant 0, range R-1),...,Imag(ant N-1, range R-1),Real(ant N-1, range R-1)
         @endverbatim
 *       Based on this data the static azimuth heat map is constructed by the GUI
 *       running on the host.
 *
 *      @subsection tlv5 Range/Doppler heatmap
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP)
 *
 *       Length: (Range FFT size) x (Doppler FFT size) (size of uint16_t)
 *
 *       Value: Detection matrix @ref DPIF_DetMatrix::data.
 *       The order is : \n
 *       @verbatim
            X(range bin 0, Doppler bin 0),...,X(range bin 0, Doppler bin D-1),
            ...
            X(range bin R-1, Doppler bin 0),...,X(range bin R-1, Doppler bin D-1)
         @endverbatim
 *
 *      @subsection tlv6 Stats information
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_STATS )
 *
 *       Length: (size of @ref MmwDemo_output_message_stats_t)
 *
 *       Value: Timing information as per @ref MmwDemo_output_message_stats_t.
 *       See timing diagram below related to the stats.
 *
 *      @image html processing_timing.png "Processing timing"
 *
 *       Note:
 *       -#  The @ref MmwDemo_output_message_stats_t::interChirpProcessingMargin is not
 *           computed (it is always set to 0). This is because there is no CPU involvement
 *           in the 1D processing (only HWA and EDMA are involved), and it is not possible to
 *           know how much margin is there in chirp processing without CPU being notified
 *           at every chirp when processing begins (chirp event) and when the HWA-EDMA
 *           computation ends. The CPU is intentionally kept free during 1D processing
 *           because a real application may use this time for doing some post-processing
 *           algorithm execution.
 *       -#  While the
 *           @ref MmwDemo_output_message_stats_t::interFrameProcessingTime reported
 *           will be of the current sub-frame/frame,
 *           the @ref MmwDemo_output_message_stats_t::interFrameProcessingMargin and
 *           @ref MmwDemo_output_message_stats_t::transmitOutputTime
 *           will be of the previous sub-frame (of the same
 *           @ref MmwDemo_output_message_header_t::subFrameNumber as that of the
 *           current sub-frame) or of the previous frame.
 *       -#  The @ref MmwDemo_output_message_stats_t::interFrameProcessingMargin excludes
 *           the UART transmission time (available as
 *           @ref MmwDemo_output_message_stats_t::transmitOutputTime). This is done
 *           intentionally to inform the user of a genuine inter-frame processing margin
 *           without being influenced by a slow transport like UART, this transport time
 *           can be significantly longer for example when streaming out debug information like
 *           heat maps. Also, in a real product deployment, higher speed interfaces (e.g LVDS)
 *           are likely to be used instead of UART. User can calculate the margin
 *           that includes transport overhead (say to determine the max frame rate
 *           that a particular demo configuration will allow) using the stats
 *           because they also contain the UART transmission time.
 *
 *      The CLI command \"guMonitor\" specifies which TLV element will
 *      be sent out within the output packet. The arguments of the CLI command are stored
 *      in the structure @ref MmwDemo_GuiMonSel_t.
 *
 *      @subsection tlv7 Side information of detected objects
 *       Type: (@ref MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO)
 *
 *       Length: (Number of detected objects) x (size of @ref DPIF_PointCloudSideInfo_t)
 *
 *       Value: Array of detected objects side information. The side information
 *       of each detected object is as per the structure @ref DPIF_PointCloudSideInfo_t).
 *       When the number of detected objects is zero, this TLV item is not sent.
 *
 *  @section Calibration_section Range Bias and Rx Channel Gain/Phase Measurement and Compensation
 *
 *     Because of imperfections in antenna layouts on the board, RF delays in SOC, etc,
 *     there is need to calibrate the sensor to compensate for bias in the range estimation and
 *     receive channel gain and phase imperfections. The following figure illustrates
 *     the calibration procedure.
 *
 *     @anchor Figure_calibration_ladder_diagram
 *     @image html calibration_ladder_diagram.png "Calibration procedure ladder diagram"
 *
 *      The calibration procedure includes the following steps:
 *     -# Set a strong target like corner reflector at the distance of X meter
 *     (X less than 50 cm is not recommended) at boresight.
 *     -# Set the following command
 *     in the configuration profile in .../profiles/profile_calibration.cfg,
 *     to reflect the position X as follows:
 *     @verbatim
       measureRangeBiasAndRxChanPhase 1 X D
       @endverbatim
 *     where D (in meters) is the distance of window around X where the peak will be searched.
 *     The purpose of the search window is to allow the test environment from not being overly constrained
 *     say because it may not be possible to clear it of all reflectors that may be stronger than the one used
 *     for calibration. The window size is recommended to be at least the distance equivalent of a few range bins.
 *     One range bin for the calibration profile (profile_calibration.cfg) is about 5 cm.
 *     The first argument "1" is to enable the measurement. The stated configuration
 *     profile (.cfg) must be used otherwise the calibration may not work as expected
 *     (this profile ensures all transmit and receive antennas are engaged among
 *     other things needed for calibration).
 *     -# Start the sensor with the configuration file.
 *     -# In the configuration file, the measurement is enabled because of which
 *     the DPC will be configured to perform the measurement and generate
 *     the measurement result (@ref DPU_AoAProc_compRxChannelBiasCfg_t) in its
 *     result structure (@ref DPC_ObjectDetection_ExecuteResult_t::compRxChanBiasMeasurement),
 *     the measurement results are written out on the CLI port (@ref MmwDemo_measurementResultOutput)
 *     in the format below:
 *     @verbatim
       compRangeBiasAndRxChanPhase <rangeBias> <Re(0,0)> <Im(0,0)> <Re(0,1)> <Im(0,1)> ... <Re(0,R-1)> <Im(0,R-1)> <Re(1,0)> <Im(1,0)> ... <Re(T-1,R-1)> <Im(T-1,R-1)>
       @endverbatim
       For details of how DPC performs the measurement, see the DPC documentation.
 *     -# The command printed out on the CLI now can be copied and pasted in any
 *     configuration file for correction purposes. This configuration will be
 *     passed to the DPC for the purpose  of applying compensation during angle
 *     computation, the details of this can be seen in the DPC documentation.
 *     If compensation is not desired,
 *     the following command should be given
 *     @verbatim
       compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
       @endverbatim
 *     Above sets the range bias to 0 and
 *     the phase coefficients to unity so that there is no correction. Note
 *     the two commands must always be given in any configuration file, typically
 *     the measure commmand will be disabled when the correction command is the
 *     desired one.
 *
 *  @section LVDSStreamingNotes Streaming data over LVDS
 *
 *    The LVDS streaming feature enables the streaming of HW data (a combination of ADC/CP/CQ data)
 *    and/or user specific SW data through LVDS interface.
 *    The streaming is done mostly by the CBUFF and EDMA peripherals with minimal CPU intervention.
 *    The streaming is configured through the @ref MmwDemo_LvdsStreamCfg_t CLI command which
 *    allows control of HSI header, enable/disable of HW and SW data and data format choice for the HW data.
 *    The choices for data formats for HW data are:
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC
 *      - @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ
 *
 *    In order to see the high-level data format details corresponding to the above
 *    data format configurations, refer to the corresponding slides in ti\drivers\cbuff\docs\CBUFF_Transfers.pptx
 *
 *    Note:
 *    -# Only single-chirp formats are allowed, multi-chirp is not supported.
 *    -# Currently this device does not support analog monitoring and is rejected at
 *       CLI level if any of the analog monitoring configurations are enabled.
 *       However, the implementation code is there to make it easier to enable
 *       functionality in a future release when device may start supporting the monitoring.
 *       Because of this limitation, the @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ
 *       format is also rejected during CLI configuration.
 *
 *    When HW data LVDS streaming is enabled, the ADC/CP/CQ data is streamed per
 *    chirp on every chirp event. When SW data
 *    streaming is enabled, it is streamed during inter-frame period after the
 *    list of detected objects for that frame is computed.
 *    The SW data streamed every frame/sub-frame is composed of the following in time:
 *    -# HSI header (@ref HSIHeader_t): refer to HSI module for details.
 *    -# User data header: @ref MmwDemo_LVDSUserDataHeader
 *    -# User data payloads:
 *      -# Point-cloud information as a list : @ref DPIF_PointCloudCartesian_t x number of detected objects
 *      -# Point-cloud side information as a list : @ref DPIF_PointCloudSideInfo_t x number of detected objects
 *
 *    The format of the SW data streamed is shown in the following figure:
 *
 *      @image html lvds_sw_data_format.png "LVDS SW Data format"
 *
 *    Note:
 *    -# When number of objects detected in frame/sub-frame is 0, there is no
 *       transmission beyond the user data header.
 *    -# For HW data, the inter-chirp duration should be sufficient to stream out the desired amount
 *       of data. For example, if the HW data-format is ADC and HSI header is enabled,
 *       then the total amount of data generated per chirp
 *       is:\n
 *        (numAdcSamples * numRxChannels * 4 (size of complex sample) +
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] )
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       The chirp time Tc in us = idle time + ramp end time in the profile configuration.
 *       For n-lane LVDS with each lane at a maximum of B Mbps,\n
 *       maximum number of bytes that can be send per chirp = Tc * n * B / 8 which
 *       should be greater than the total amount of data generated per chirp i.e\n
 *       Tc * n * B / 8 >= round-up(numAdcSamples * numRxChannels * 4 + 52, 256). \n
 *       E.g if n = 2, B = 600 Mbps, idle time = 7 us, ramp end time = 44 us, numAdcSamples = 512,
 *       numRxChannels = 4, then 7650 >= 8448 is violated so this configuration will not work.
 *       If the idle-time is doubled in the above example, then we have 8700 > 8448, so
 *       this configuration will work.
 *    -# For SW data, the number of bytes to transmit each sub-frame/frame is:\n
 *       52 [sizeof(HSIDataCardHeader_t) + sizeof(HSISDKHeader_t)] +
 *       sizeof(MmwDemo_LVDSUserDataHeader_t) [=8] + \n
 *       number of detected objects (Nd) * { sizeof(DPIF_PointCloudCartesian_t) [=16] + sizeof(DPIF_PointCloudSideInfo_t) [=4] }
 *       rounded up to multiples of 256 [=sizeof(HSIHeader_t)] bytes.\n
 *       or X = round-up(60 + Nd * 20, 256). So the time to transmit this data will be \n
 *       X * 8 / (n*B) us. The maximum number of objects (Ndmax) that can be
 *       detected is defined in the DPC (@ref DPC_OBJDET_MAX_NUM_OBJECTS). So if
 *       Ndmax = 500, then time to transmit SW data is 68 us. Because we parallelize
 *       this transmission with the much slower UART transmission, and because UART transmission
 *       is also sending at least the same amount of information as the LVDS, the
 *       LVDS transmission time will not add any burdens on the processing budget
 *       beyond the overhead of reconfiguring and activating the CBUFF session
 *       (this overhead is likely bigger than the time to transmit).
 *    -# The total amount of data to be transmitted in a HW or SW packet must be greater than the
 *       minimum required by CBUFF, which is 64 bytes or 32 CBUFF Units (this is the definition
 *       CBUFF_MIN_TRANSFER_SIZE_CBUFF_UNITS in the CBUFF driver implementation).
 *       If this threshold condition is violated, the CBUFF driver will return an error during
 *       configuration and the demo will generate a fatal exception as a result.
 *       When HSI header is enabled, the total transfer size is ensured to be at least
 *       256 bytes, which satisfies the minimum. If HSI header is disabled, for the HW session,
 *       this means that numAdcSamples * numRxChannels * 4 >= 64. Although mmwavelink
 *       allows minimum number of ADC samples to be 2, the demo is supported
 *       for numAdcSamples >= 64. So HSI header is not required to be enabled for HW only case.
 *       But if SW session is enabled, without the HSI header, the bytes in each
 *       packet will be 8 + Nd * 20. So for frames/sub-frames where Nd < 3, the
 *       demo will generate exception. Therefore HSI header must be enabled if SW is enabled,
 *       this is checked in the CLI command validation.
 *
 *   @subsection lvdsImpl Implementation Notes
 *
 *    -# The LVDS implementation is mostly present in mmw_lvds_stream.h and
 *      mmw_lvds_stream.c with calls in mss_main.c. Additionally HSI
 *      clock initialization is done at first time sensor start using @ref MmwDemo_mssSetHsiClk.
 *    -# EDMA channel resources for CBUFF/LVDS are in the global resource file
 *      (mmw_res.h, see @ref resourceAlloc) along with other EDMA resource allocation.
 *      The user data header and two user payloads are configured as three user buffers in the CBUFF driver.
 *      Hence SW allocation for EDMA provides for three sets of EDMA resources as seen in
 *      the SW part (swSessionEDMAChannelTable[.]) of @ref MmwDemo_LVDSStream_EDMAInit.
 *      The maximum number of HW EDMA resources are needed for the
 *      data-format @ref MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ,
 *      which as seen in the corresponding slide in ti\drivers\cbuff\docs\CBUFF_Transfers.pptx is
 *      12 channels (+ shadows) including the 1st special CBUFF EDMA event channel
 *      which CBUFF IP generates to the EDMA, hence the HW part
 *      (hwwSessionEDMAChannelTable[.]) of @ref MmwDemo_LVDSStream_EDMAInit
 *      has 11 table entries.
 *    -# Although the CBUFF driver is configured for two sessions (hw and sw),
 *      at any time only one can be active. So depending on the LVDS CLI configuration
 *      and whether advanced frame or not, there is logic to activate/deactivate
 *      HW and SW sessions as necessary.
 *    -# The CBUFF session (HW/SW) configure-create and delete
 *      depends on whether or not re-configuration is required after the
 *      first time configuration.
 *      -# For HW session, re-configuration is done during sub-frame switching to
 *        re-configure for the next sub-frame but when there is no advanced frame
 *        (number of sub-frames = 1), the HW configuration does not need to
 *        change so HW session does not need to be re-created.
 *      -# For SW session, even though the user buffer start addresses and sizes of
 *        headers remains same, the number of detected objects which determines the
 *        sizes of some user buffers changes from one sub-frame/frame to another sub-frame/frame.
 *        Therefore SW session needs to be recreated every sub-frame/frame.
 *    -# User may modify the application software to transmit different information than point-cloud
 *      in the SW data e.g radar cube data (output of range DPU). However the CBUFF also has
 *      a maximum link list entry size limit of 0x3FFF CBUFF units or 32766 bytes.
 *      This means it is the limit for each user buffer entry
 *      [there are maximum of 3 entries -1st used for user data header, 2nd
 *      for point-cloud and 3rd for point-cloud side information]. During session creation,
 *      if this limit is exceeded, the CBUFF will return an error (and demo will in turn
 *      generate an exception). A single physical buffer of say size 50000 bytes
 *      may be split across two user buffers by providing one user buffer with
 *      (address, size) = (start address, 25000) and 2nd user buffer with
 *      (address, size) = (start address + 25000, 25000), beyond
 *      this two (or three if user data header is also replaced) limit,
 *      the user will need to create and activate (and wait for completion)
 *      the SW session multiple times to accomplish the transmission.
 *
 *    The following figure shows a timing diagram for the LVDS streaming
 *    (the figure is not to scale as actual durations will vary based on configuration).
 *
 *      @image html lvdstiming.png "LVDS timing diagram"
 *
 *  @section bypassCLI How to bypass CLI
 *
 *    Re-implement the file mmw_cli.c as follows:
 *
 *    -# @ref MmwDemo_CLIInit should just create a task with input taskPriority.
 *       Lets say the task is called "MmwDemo_sensorConfig_task".
 *    -# All other functions are not needed
 *    -# Implement the MmwDemo_sensorConfig_task as follows:
 *       - Fill gMmwMssMCB.cfg.openCfg
 *       - Fill gMmwMssMCB.cfg.ctrlCfg
 *       - Add profiles and chirps using @ref MMWave_addProfile and @ref MMWave_addChirp functions
 *       - Call @ref MmwDemo_CfgUpdate for every offset in @ref configStoreOffsets
 *         (MMWDEMO_xxx_OFFSET in mmw_mss.h)
 *       - Fill gMmwMssMCB.objDetCommonCfg.preStartCommonCfg
 *       - Call @ref MmwDemo_openSensor
 *       - Call @ref MmwDemo_configSensor
 *       - Call @ref MmwDemo_startSensor (One can use helper function
 *         @ref MmwDemo_isAllCfgInPendingState to know if all dynamic config was provided)
 *
 *  @section resourceAlloc Hardware Resource Allocation
 *    The object detection DPCs(objdetrangehwa and objdetdsp) need to configure the DPUs 
 *    hardware resources (HWA, EDMA etc). The resource partitioning is shown to be in
 *    the ownership of the demo. This is to illustrate the general
 *    case of resource allocation across DPCs and/or demo's own
 *    processing that is post-DPC processing. This partitioning
 *    can be seen in the mmw_res.h file. This file is passed as a compiler command line
 *    define @verbatim "--define=APP_RESOURCE_FILE="<ti/demo/xwr68xx/mmw/mmw_res.h>" @endverbatim 
 *    in mmw_mss.mak and mmw_dss.mak when building the DPC sources as part of building the demo application and
 *    is referred in object detection DPC sources where needed as
 *    @verbatim #include APP_RESOURCE_FILE @endverbatim
 *
 *  @section demoDesignNotes Design Notes
 *  
 *  Due to the limitation of DPM local queue size, for certain DPM functions such as @ref DPM_start, @ref DPM_stop and
 *  some of the DPC control through @ref DPM_ioctl, semaphores are used to sync between calling task and function
 *  @ref MmwDemo_DPC_ObjectDetection_reportFxn. So that it won't cause DPM crash because of running out of
 *  DPM local queues. The following diagram demonstrates the example calling flow for blocking @ref DPM_ioctl() function call.
 *  Non-blocking @ref DPM_ioctl is also shown for comparison.
 *
 *  @image html dpm_ioctl_handling.png "DPM_ioctl calling flow"
 *
 *  There are DPM report functions on both MSS and DSS for the same DPM_Report. However the sequence is not guaranteed between the
 *  two cores.
 *
 *  @section memoryUsage Memory Usage
 *  @subsection memUsageSummary Memory usage summary
 *    The table below shows the usage of various memories available on the device across
 *    the demo application and other SDK components. The table is generated using the demo's
 *    map file and applying some mapping rules to it to generate a condensed summary.
 *    The numeric values shown here represent bytes.
 *
 *    **MSS:**
 *    For the mapping rules, please refer to <a href="../../demo_mss_mapping.txt">demo_mss_mapping.txt</a>.
 *    Refer to the <a href="../../xwr68xx_mmw_demo_mss_mem_analysis_detailed.txt">xwr68xx_mmw_demo_mss_mem_analysis_detailed.txt</a>
 *    for detailed analysis of the memory usage across drivers and control/alg components on MSS and to
 *    <a href="../../demo_mss_mapping_detailed.txt">demo_mss_mapping_detailed.txt</a>
 *    for detailed mapping rules .
 *
 *  \include xwr68xx_mmw_demo_mss_mem_analysis.txt
 *
 *    **DSS:**
 *    For the mapping rules, please refer to <a href="../../demo_dss_mapping.txt">demo_dss_mapping.txt</a>.
 *    Refer to the <a href="../../xwr68xx_mmw_demo_dss_mem_analysis_detailed.txt">xwr68xx_mmw_demo_dss_mem_analysis_detailed.txt</a>
 *    for detailed analysis of the memory usage across drivers and control/alg components on DSS and to
 *    <a href="../../demo_dss_mapping_detailed.txt">demo_dss_mapping_detailed.txt</a>
 *    for detailed mapping rules .
 *
 *  \include xwr68xx_mmw_demo_dss_mem_analysis.txt
 *
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
#include <ti/sysbios/utils/Load.h>


/* mmWave SDK Include Files: */
#include <ti/common/sys_common.h>
#include <ti/common/mmwave_sdk_version.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/crc/crc.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#include <ti/drivers/pinmux/pinmux.h>
#include <ti/control/mmwave/mmwave.h>
#include <ti/control/dpm/dpm.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/mathutils/mathutils.h>

/* Demo Include Files */
#include <mmw_config.h>
#include <ti/demo/utils/mmwdemo_monitor.h>
#include <mmw_output.h>
#include <ti/demo/xwr68xx/mmw/mmw_res.h>
#include <mmw_mss.h>
#include <ti/demo/utils/mmwdemo_rfparser.h>
#include <ti/demo/utils/mmwdemo_adcconfig.h>

/* Profiler Include Files */
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <tracker_output.h>
#include <common/src/dpc/objectdetectionandtracking/objdetrangehwa/include/objdetrangehwainternal.h>
#include <common/src/dpc/objectdetectionandtracking/objdetrangehwa/objdetrangehwa.h>

/**
 * @brief Task Priority settings:
 * Mmwave task is at higher priority because of potential async messages from BSS
 * that need quick action in real-time.
 *
 * CLI task must be at a lower priority than object detection
 * dpm task priority because the dynamic CLI command handling in the objection detection
 * dpm task assumes CLI task is held back during this processing. The alternative
 * is to use a semaphore between the two tasks.
 */
#define MMWDEMO_CLI_TASK_PRIORITY                 3
#define MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY      4
#define MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY         5

#if (MMWDEMO_CLI_TASK_PRIORITY >= MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY)
#error CLI task priority must be < Object Detection DPM task priority
#endif

#define DPC_OBJDET_R4F_INSTANCEID       (0xFEEDFEED)

/* These address offsets are in bytes, when configure address offset in hardware,
   these values will be converted to number of 128bits
   Buffer at offset 0x0U is reserved by BSS, hence offset starts from 0x800
 */
#define MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET          0x800U
#define MMW_DEMO_CQ_RXSAT_ADDR_OFFSET           0x1000U

/* CQ data is at 16 bytes alignment for mulitple chirps */
#define MMW_DEMO_CQ_DATA_ALIGNMENT            16U

/* SOC based defines */
#ifdef SOC_XWR68XX
#define SOC_XWRXXXX_GPIO_2                          SOC_XWR68XX_GPIO_2
#define SOC_XWRXXXX_MSS_ADCBUF_BASE_ADDRESS         SOC_XWR68XX_MSS_ADCBUF_BASE_ADDRESS
#define SOC_XWRXXXX_PINF14_PADAJ_MSS_UARTB_TX       SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX
#define SOC_XWRXXXX_PINF14_PADAJ                    SOC_XWR68XX_PINF14_PADAJ
#define SOC_XWRXXXX_PINK13_PADAZ_GPIO_2             SOC_XWR68XX_PINK13_PADAZ_GPIO_2
#define SOC_XWRXXXX_PINK13_PADAZ                    SOC_XWR68XX_PINK13_PADAZ
#define SOC_XWRXXXX_PINN4_PADBD_MSS_UARTA_RX        SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX
#define SOC_XWRXXXX_PINN4_PADBD                     SOC_XWR68XX_PINN4_PADBD
#define SOC_XWRXXXX_PINN5_PADBE_MSS_UARTA_TX        SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX
#define SOC_XWRXXXX_PINN5_PADBE                     SOC_XWR68XX_PINN5_PADBE
#define CALIB_FREQ_LIMIT_LOW                        600U 
#define CALIB_FREQ_LIMIT_HIGH                       640U 
#elif SOC_XWR18XX
#define SOC_XWRXXXX_GPIO_2                          SOC_XWR18XX_GPIO_2
#define SOC_XWRXXXX_MSS_ADCBUF_BASE_ADDRESS         SOC_XWR18XX_MSS_ADCBUF_BASE_ADDRESS
#define SOC_XWRXXXX_PINF14_PADAJ_MSS_UARTB_TX       SOC_XWR18XX_PINF14_PADAJ_MSS_UARTB_TX
#define SOC_XWRXXXX_PINF14_PADAJ                    SOC_XWR18XX_PINF14_PADAJ
#define SOC_XWRXXXX_PINK13_PADAZ_GPIO_2             SOC_XWR18XX_PINK13_PADAZ_GPIO_2
#define SOC_XWRXXXX_PINK13_PADAZ                    SOC_XWR18XX_PINK13_PADAZ
#define SOC_XWRXXXX_PINN4_PADBD_MSS_UARTA_RX        SOC_XWR18XX_PINN4_PADBD_MSS_UARTA_RX
#define SOC_XWRXXXX_PINN4_PADBD                     SOC_XWR18XX_PINN4_PADBD
#define SOC_XWRXXXX_PINN5_PADBE_MSS_UARTA_TX        SOC_XWR18XX_PINN5_PADBE_MSS_UARTA_TX
#define SOC_XWRXXXX_PINN5_PADBE                     SOC_XWR18XX_PINN5_PADBE
#define CALIB_FREQ_LIMIT_LOW                        760U
#define CALIB_FREQ_LIMIT_HIGH                       810U
#endif
/**************************************************************************
 *************************** Global Definitions ***************************
 **************************************************************************/

/**
 * @brief
 *  Global Variable for tracking information required by the mmw Demo
 */
MmwDemo_MSS_MCB    gMmwMssMCB;

/**
 * @brief
 *  Global Variable for HSRAM buffer used to share results to remote
 */
MmwDemo_HSRAM gHSRAM;

/*! HSRAM for processing results */
#pragma DATA_SECTION(gHSRAM, ".demoSharedMem");
#pragma DATA_ALIGN(gHSRAM, 4);

 /*! TCMB RAM buffer for object detection DPC */
#define MMWDEMO_OBJDET_LOCALRAM_SIZE (4U * 1024U)
uint8_t gDPCTCM[MMWDEMO_OBJDET_LOCALRAM_SIZE];
#pragma DATA_SECTION(gDPCTCM, ".dpcLocalRam");
#pragma DATA_ALIGN(gDPCTCM, 4);

/*! L3 RAM buffer for object detection DPC */
#define MMWDEMO_L3SRAM_SIZE (SOC_L3RAM_SIZE - MMWAVE_L3_CODEMEM_SIZE)

uint8_t gMmwL3[MMWDEMO_L3SRAM_SIZE];
#pragma DATA_SECTION(gMmwL3, ".l3ram");

#define DBG_DPC_OBJDETRANGEHWA
#ifdef DBG_DPC_OBJDETRANGEHWA
extern ObjDetObj     *gObjDetObj;
#endif

 /**
 * @brief
 *  Global Variable for LDO BYPASS config, PLease consult your 
 * board/EVM user guide before changing the values here
 */
rlRfLdoBypassCfg_t gRFLdoBypassCfg =
{
    .ldoBypassEnable   = 0, /* 1.0V RF supply 1 and 1.0V RF supply 2 */
    .supplyMonIrDrop   = 0, /* IR drop of 3% */
    .ioSupplyIndicator = 0, /* 3.3 V IO supply */
};  
/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern void MmwDemo_CLIInit(uint8_t taskPriority);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions prototype *************
 **************************************************************************/

/* MMW demo functions for datapath operation */
static void MmwDemo_dataPathOpen(void);
static int32_t MmwDemo_dataPathConfig (void);
int32_t MmwDemo_trackerConfig(void);
static void MmwDemo_dataPathStart (void);
static void MmwDemo_dataPathStop (void);
static void MmwDemo_handleObjectDetResult(DPM_Buffer  *ptrResult);
static void MmwDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
);
/*static void MmwDemo_transmitProcessedOutput
(
    UART_Handle     uartHandle,
    DPC_ObjectDetection_ExecuteResult   *result,
    MmwDemo_output_message_stats        *timingInfo
);*/

void TrackerDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                         DPC_ObjectDetection_ExecuteResult *result);

static void MmwDemo_measurementResultOutput(DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg);
static int32_t MmwDemo_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
);
static int32_t MmwDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx);

static void MmwDemo_initTask(UArg arg0, UArg arg1);
static void MmwDemo_platformInit(MmwDemo_platformCfg *config);

/* Mmwave control functions */
static void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1);
static int32_t MmwDemo_mmWaveCtrlStop (void);
static int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload);

/* external sleep function when in idle (used in .cfg file) */
void MmwDemo_sleep(void);

/* Edma related functions */
static void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj, uint8_t instance);
static void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj, uint8_t instance);
static void MmwDemo_EDMA_transferControllerErrorCallbackFxn(EDMA_Handle handle,
                EDMA_transferControllerErrorInfo_t *errorInfo);
static void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo);

static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg,
                                           uint8_t numChirpsPerChirpEvent,
                                           uint8_t validProfileIdx);

/**************************************************************************
 ************************* Millimeter Wave Demo Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      EDMA driver init
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] instance     EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaInit(MmwDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t errorCode;

    errorCode = EDMA_init(instance);
    if (errorCode != EDMA_NO_ERROR)
    {
        System_printf ("Debug: EDMA instance %d initialization returned error %d\n", errorCode);
        MmwDemo_debugAssert (0);
        return;
    }

    memset(&obj->EDMA_errorInfo, 0, sizeof(obj->EDMA_errorInfo));
    memset(&obj->EDMA_transferControllerErrorInfo, 0, sizeof(obj->EDMA_transferControllerErrorInfo));
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA CC (Channel controller) error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void MmwDemo_EDMA_errorCallbackFxn(EDMA_Handle handle, EDMA_errorInfo_t *errorInfo)
{
    gMmwMssMCB.dataPathObj.EDMA_errorInfo = *errorInfo;
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Call back function for EDMA transfer controller error as per EDMA API.
 *      Declare fatal error if happens, the output errorInfo can be examined if code
 *      gets trapped here.
 */
static void MmwDemo_EDMA_transferControllerErrorCallbackFxn
(
    EDMA_Handle handle,
    EDMA_transferControllerErrorInfo_t *errorInfo
)
{
    gMmwMssMCB.dataPathObj.EDMA_transferControllerErrorInfo = *errorInfo;
    MmwDemo_debugAssert(0);
}

/**
 *  @b Description
 *  @n
 *      Open EDMA driver instance
 *
 *  @param[in] obj           Pointer to data path object
 *  @param[in] instance      EDMA instance
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_edmaOpen(MmwDemo_DataPathObj *obj, uint8_t instance)
{
    int32_t             errCode;
    EDMA_instanceInfo_t edmaInstanceInfo;
    EDMA_errorConfig_t  errorConfig;

    obj->edmaHandle = EDMA_open(instance, &errCode, &edmaInstanceInfo);

    if (obj->edmaHandle == NULL)
    {
        MmwDemo_debugAssert (0);
        return;
    }

    errorConfig.isConfigAllEventQueues = true;
    errorConfig.isConfigAllTransferControllers = true;
    errorConfig.isEventQueueThresholdingEnabled = true;
    errorConfig.eventQueueThreshold = EDMA_EVENT_QUEUE_THRESHOLD_MAX;
    errorConfig.isEnableAllTransferControllerErrors = true;
    errorConfig.callbackFxn = MmwDemo_EDMA_errorCallbackFxn;
    errorConfig.transferControllerCallbackFxn = MmwDemo_EDMA_transferControllerErrorCallbackFxn;
    if ((errCode = EDMA_configErrorMonitoring(obj->edmaHandle, &errorConfig)) != EDMA_NO_ERROR)
    {
        //System_printf("Error: EDMA_configErrorMonitoring() failed with errorCode = %d\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close EDMA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_edmaClose(MmwDemo_DataPathObj *obj)
{
    EDMA_close(obj->edmaHandle);
}

/**
 *  @b Description
 *  @n
 *      HWA driver init
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_hwaInit(MmwDemo_DataPathObj *obj)
{
    /* Initialize the HWA */
    HWA_init();
}

/**
 *  @b Description
 *  @n
 *      Open HWA driver instance
 *
 *  @param[in] obj          Pointer to data path object
 *  @param[in] socHandle    SOC driver handle
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_hwaOpen(MmwDemo_DataPathObj *obj, SOC_Handle socHandle)
{
    int32_t             errCode;

    /* Open the HWA Instance */
    obj->hwaHandle = HWA_open(0, socHandle, &errCode);
    if (obj->hwaHandle == NULL)
    {
        //System_printf("Error: Unable to open the HWA Instance err:%d\n",errCode);
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Close HWA driver instance
 *
 *  @param[in] obj      Pointer to data path object
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_hwaClose(MmwDemo_DataPathObj *obj)
{
    int32_t             errCode;

    /* Close the HWA Instance */
    errCode = HWA_close(obj->hwaHandle);
    if (errCode != 0)
    {
        MmwDemo_debugAssert (0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Send assert information through CLI.
 *
 *  @param[in] expression           Expression for evaluation
 *  @param[in] file                 C file that caused assertion
 *  @param[in] line                 Line number in C fine that caused assertion
 *
 */
void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line)
{
    if (!expression) {
        CLI_write ("Exception: %s, line %d.\n",file,line);
    }
}

/**
 *  @b Description
 *  @n
 *      Utility function to set the pending state of configuration.
 *
 *  @param[in] subFrameCfg Pointer to Sub-frame specific configuration
 *  @param[in] offset       Configuration structure offset that uniquely identifies the
 *                          configuration to set to the pending state.
 *
 *  @retval None
 */
static void MmwDemo_setSubFramePendingState(MmwDemo_SubFrameCfg *subFrameCfg, uint32_t offset)
{
    switch (offset)
    {
        case MMWDEMO_GUIMONSEL_OFFSET:
            subFrameCfg->objDetDynCfg.isPrepareRangeAzimuthHeatMapPending = 1;
        break;
        case MMWDEMO_CFARCFGRANGE_OFFSET:
            subFrameCfg->objDetDynCfg.isCfarCfgRangePending = 1;
        break;
        case MMWDEMO_CFARCFGDOPPLER_OFFSET:
            subFrameCfg->objDetDynCfg.isCfarCfgDopplerPending = 1;
        break;
        case MMWDEMO_FOVRANGE_OFFSET:
            subFrameCfg->objDetDynCfg.isFovRangePending = 1;
        break;
        case MMWDEMO_FOVDOPPLER_OFFSET:
            subFrameCfg->objDetDynCfg.isFovDopplerPending = 1;
        break;
        case MMWDEMO_FOVAOA_OFFSET:
            subFrameCfg->objDetDynCfg.isFovAoaCfgPending = 1;
        break;
        case MMWDEMO_MULTIOBJBEAMFORMING_OFFSET:
            subFrameCfg->objDetDynCfg.isMultiObjBeamFormingCfgPending = 1;
        break;
        case MMWDEMO_CALIBDCRANGESIG_OFFSET:
            subFrameCfg->objDetDynCfg.isCalibDcRangeSigCfg = 1;
        break;
        case MMWDEMO_STATICCLUTTERREMOFVAL_OFFSET:
            subFrameCfg->objDetDynCfg.isStaticClutterRemovalCfgPending = 1;
        break;
        case MMWDEMO_EXTMAXVEL_OFFSET:
            subFrameCfg->objDetDynCfg.isExtMaxVelCfgPending = 1;
        break;
        case MMWDEMO_ADCBUFCFG_OFFSET:
            subFrameCfg->isAdcBufCfgPending = 1;
        break;
        case MMWDEMO_BPMCFG_OFFSET:
            subFrameCfg->isBpmCfgPending = 1;
        break;
        case MMWDEMO_LVDSSTREAMCFG_OFFSET:
            subFrameCfg->isLvdsStreamCfgPending = 1;
        break;
        default:
            MmwDemo_debugAssert(0);
        break;
    }
}


/**
 *  @b Description
 *  @n
 *      Utility function to find out if all dynamic common configuration is pending
 *
 *  @param[in] cfg      Pointer to Common configuration
 *
 *  @retval 1 if all common configuration is pending, else return 0.
 */
static uint8_t MmwDemo_isDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCompRxChannelBiasCfgPending == 1) &&
             (cfg->isMeasureRxChannelBiasCfgPending == 1);

    return(retVal);
}


/**
 *  @b Description
 *  @n
 *      Utility function to find out if all sub-frame specific dynamic
 *      configuration is pending
 *
 *  @param[in] cfg      Pointer to sub-frame specific configuration
 *
 *  @retval 1 if all sub-frame specific dynamic configuration is pending, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCalibDcRangeSigCfg    == 1) &&
             (cfg->isCfarCfgDopplerPending == 1) &&
             (cfg->isCfarCfgRangePending   == 1) &&
             (cfg->isFovDopplerPending     == 1) &&
             (cfg->isFovRangePending       == 1) &&
             (cfg->isMultiObjBeamFormingCfgPending     == 1) &&
             (cfg->isPrepareRangeAzimuthHeatMapPending == 1) &&
             (cfg->isStaticClutterRemovalCfgPending    == 1) &&
             (cfg->isFovAoaCfgPending                  == 1) &&
             (cfg->isExtMaxVelCfgPending               == 1);

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all dynamic common configuration is in non-pending (cleared)
 *      state.
 *
 *  @param[in] cfg      Pointer to common specific configuration
 *
 *  @retval 1 if all common configuration is in non-pending state, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCommonCfgInNonPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCompRxChannelBiasCfgPending    == 0) &&
             (cfg->isMeasureRxChannelBiasCfgPending == 0);

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all sub-frame specific dynamic configuration
 *      is in non-pending (cleared) state.
 *
 *  @param[in] cfg      Pointer to common specific configuration
 *
 *  @retval 1 if all sub-frame specific dynamic configuration is in non-pending
 *          state, else return 0
 */
static uint8_t MmwDemo_isDynObjDetCfgInNonPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    uint8_t retVal;

    retVal = (cfg->isCalibDcRangeSigCfg    == 0) &&
             (cfg->isCfarCfgDopplerPending == 0) &&
             (cfg->isCfarCfgRangePending   == 0) &&
             (cfg->isFovDopplerPending     == 0) &&
             (cfg->isFovRangePending       == 0) &&
             (cfg->isMultiObjBeamFormingCfgPending     == 0) &&
             (cfg->isPrepareRangeAzimuthHeatMapPending == 0) &&
             (cfg->isStaticClutterRemovalCfgPending    == 0) &&
             (cfg->isFovAoaCfgPending                  == 0) &&
             (cfg->isExtMaxVelCfgPending               == 0);

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending dynamic common configuration of Object Detection DPC
 *
 *  @param[in] cfg      Object Detection DPC common configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_resetDynObjDetCommonCfgPendingState(MmwDemo_DPC_ObjDet_CommonCfg *cfg)
{
    cfg->isCompRxChannelBiasCfgPending = 0;
    cfg->isMeasureRxChannelBiasCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending sub-frame specific dynamic configuration
 *      of Object Detection DPC
 *
 *  @param[in] cfg      Object Detection DPC sub-frame dynamic configuration
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_resetDynObjDetCfgPendingState(MmwDemo_DPC_ObjDet_DynCfg *cfg)
{
    cfg->isCalibDcRangeSigCfg    = 0;
    cfg->isCfarCfgDopplerPending = 0;
    cfg->isCfarCfgRangePending   = 0;
    cfg->isFovDopplerPending     = 0;
    cfg->isFovRangePending       = 0;
    cfg->isMultiObjBeamFormingCfgPending = 0;
    cfg->isPrepareRangeAzimuthHeatMapPending = 0;
    cfg->isStaticClutterRemovalCfgPending = 0;
    cfg->isFovAoaCfgPending = 0;
    cfg->isExtMaxVelCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Resets (clears) all pending static (non-dynamic) configuration
 *
 */
void MmwDemo_resetStaticCfgPendingState(void)
{
    uint8_t indx;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending = 0;
        gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending = 0;
        gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending = 0;
    }

    gMmwMssMCB.isAnaMonCfgPending = 0;
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in pending state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in pending state, else return 0
 */
uint8_t MmwDemo_isAllCfgInPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && MmwDemo_isDynObjDetCfgPendingState(&gMmwMssMCB.subFrameCfg[indx].objDetDynCfg);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 1);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending == 1);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 1);
    }

    flag = flag && MmwDemo_isDynObjDetCommonCfgPendingState(&gMmwMssMCB.objDetCommonCfg);
    flag = flag && (gMmwMssMCB.isAnaMonCfgPending == 1);
    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to find out if all configuration (common and sub-frame
 *      specific dynamic config) is in non-pending (cleared) state.
 *
 *  @retval 1 if all configuration (common and sub-frame specific dynamic config)
 *            is in non-pending state, else return 0
 */
uint8_t MmwDemo_isAllCfgInNonPendingState(void)
{
    uint8_t indx, flag = 1;

    for(indx = 0; indx < gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames; indx++)
    {
        flag = flag && MmwDemo_isDynObjDetCfgInNonPendingState(&gMmwMssMCB.subFrameCfg[indx].objDetDynCfg);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isAdcBufCfgPending == 0);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isBpmCfgPending == 0);
        flag = flag && (gMmwMssMCB.subFrameCfg[indx].isLvdsStreamCfgPending == 0);
    }

    flag = flag && (MmwDemo_isDynObjDetCommonCfgInNonPendingState(&gMmwMssMCB.objDetCommonCfg) && flag);
    flag = flag && (gMmwMssMCB.isAnaMonCfgPending == 0);
    return(flag);
}

/**
 *  @b Description
 *  @n
 *      Utility function to apply configuration to specified sub-frame
 *
 *  @param[in] srcPtr Pointer to configuration
 *  @param[in] offset Offset of configuration within the parent structure
 *  @param[in] size   Size of configuration
 *  @param[in] subFrameNum Sub-frame Number (0 based) to apply to, broadcast to
 *                         all sub-frames if special code MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum)
{    
    /* if subFrameNum undefined, broadcast to all sub-frames */
    if(subFrameNum == MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG)
    {
        uint8_t  indx;
        for(indx = 0; indx < RL_MAX_SUBFRAMES; indx++)
        {
            memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[indx] + offset), srcPtr, size);
            MmwDemo_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[indx], offset);
        }
    }
    else
    {
        /* Apply configuration to specific subframe (or to position zero for the legacy case
           where there is no advanced frame config) */
        memcpy((void *)((uint32_t) &gMmwMssMCB.subFrameCfg[subFrameNum] + offset), srcPtr, size);
        MmwDemo_setSubFramePendingState(&gMmwMssMCB.subFrameCfg[subFrameNum], offset);
    }
}

/**************************************************************************
 ******************** Millimeter Wave Demo Results Transmit Functions *************
 **************************************************************************/
static void MmwDemo_measurementResultOutput(DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg)
{
    /* Send the received DSS calibration info through CLI */
    CLI_write ("compRangeBiasAndRxChanPhase");
    CLI_write (" %.7f", compRxChanCfg->rangeBias);
    int32_t i;
    for (i = 0; i < SYS_COMMON_NUM_TX_ANTENNAS*SYS_COMMON_NUM_RX_CHANNEL; i++)
    {
        CLI_write (" %.5f", (float) compRxChanCfg->rxChPhaseComp[i].real/32768.);
        CLI_write (" %.5f", (float) compRxChanCfg->rxChPhaseComp[i].imag/32768.);
    }
    CLI_write ("\n");
}


/** @brief Transmits detection data over UART
*
*    The following data is transmitted:
*    1. Header (size = 32bytes), including "Magic word", (size = 8 bytes)
*       and including the number of TLV items
*    TLV Items:
*    2. If detectedObjects flag is 1 or 2, DPIF_PointCloudCartesian structure containing
*       X,Y,Z location and velocity for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    3. If detectedObjects flag is 1, DPIF_PointCloudSideInfo structure containing SNR
*       and noise for detected objects,
*       size = sizeof(DPIF_PointCloudCartesian) * number of detected objects
*    4. If logMagRange flag is set,  rangeProfile,
*       size = number of range bins * sizeof(uint16_t)
*    5. If noiseProfile flag is set,  noiseProfile,
*       size = number of range bins * sizeof(uint16_t)
*    6. If rangeAzimuthHeatMap flag is set, the zero Doppler column of the
*       range cubed matrix, size = number of Rx Azimuth virtual antennas *
*       number of chirps per frame * sizeof(uint32_t)
*    7. If rangeDopplerHeatMap flag is set, the log magnitude range-Doppler matrix,
*       size = number of range bins * number of Doppler bins * sizeof(uint16_t)
*    8. If statsInfo flag is set, the stats information
*   @param[in] uartHandle   UART driver handle
*   @param[in] result       Pointer to result from object detection DPC processing
*   @param[in] timingInfo   Pointer to sub-frame object stats that contains timing info
*/
void TrackerDemo_transmitProcessedOutput(UART_Handle uartHandle,
                                     DPC_ObjectDetection_ExecuteResult *result)
{

    uint32_t packetLen;
    uint32_t tlvIdx = 0;
    uint32_t numPaddingBytes;
    uint8_t padding[MMWDEMO_OUTPUT_MSG_SEGMENT_LEN];
    MmwDemo_output_message_tl   tl[MMWDEMO_OUTPUT_MSG_MAX];

    DPIF_PointCloudCartesian *objOut;
    DPIF_PointCloudSpherical *objOutSph;
    DPIF_PointCloudSideInfo *objOutSideInfo;
    DPC_ObjectDetection_Stats *stats;
    trackerProc_Target *tList;
    trackerProc_TargetIndex *tIndex;
    int32_t errCode;

    int             n;
    uint16_t        *headerPtr;
    uint32_t        sum;
    
    TrackerDemo_output_message_header outputMessage;

    /* Clear message header */
    memset((void *)&outputMessage, 0, sizeof(TrackerDemo_output_message_header));
    /* Header: */
    outputMessage.platform =  0xA6843;
    outputMessage.magicWord[0] = 0x0102;
    outputMessage.magicWord[1] = 0x0304;
    outputMessage.magicWord[2] = 0x0506;
    outputMessage.magicWord[3] = 0x0708;
    outputMessage.version =    MMWAVE_SDK_VERSION_BUILD |   //DEBUG_VERSION
        (MMWAVE_SDK_VERSION_BUGFIX << 8) |
        (MMWAVE_SDK_VERSION_MINOR << 16) |
        (MMWAVE_SDK_VERSION_MAJOR << 24);

    /******************************************************************
       Send out data that is enabled, Since processing results are from DSP,
       address translation is needed for buffer pointers
    *******************************************************************/
    {

        objOut = (DPIF_PointCloudCartesian *) SOC_translateAddress((uint32_t)result->objOut,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)objOut != SOC_TRANSLATEADDR_INVALID);


        objOutSph = (DPIF_PointCloudSpherical *) SOC_translateAddress((uint32_t)result->objOutSph,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)objOutSph != SOC_TRANSLATEADDR_INVALID);


        objOutSideInfo = (DPIF_PointCloudSideInfo *) SOC_translateAddress((uint32_t)result->objOutSideInfo,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)objOutSideInfo != SOC_TRANSLATEADDR_INVALID);

        stats = (DPC_ObjectDetection_Stats *) SOC_translateAddress((uint32_t)result->stats,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)stats != SOC_TRANSLATEADDR_INVALID);

#ifdef TRACKERPROC_ON_DSP
        tList = (trackerProc_Target *)SOC_translateAddress((uint32_t)result->tList,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)tList != SOC_TRANSLATEADDR_INVALID);


        tIndex = (trackerProc_TargetIndex *)SOC_translateAddress((uint32_t)result->tIndex,
                                                     SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                     &errCode);
        DebugP_assert ((uint32_t)tIndex != SOC_TRANSLATEADDR_INVALID);
#else
        tList = (trackerProc_Target *)(result->tList);
        tIndex = (trackerProc_TargetIndex *)(result->tIndex);
#endif

    }


    outputMessage.subFrameNumber = 0;

    /* Start: Tracker output code */
    outputMessage.frameNumber = stats->frameStartIntCounter;
    //outputMessage.chirpProcessingMargin = srcAddress->header.chirpProcessingMarginInUsec;
    //outputMessage.frameProcessingMargin = srcAddress->header.frameProcessingMarginInUsec;
    //outputMessage.trackingProcessingTime = gMmwMssMCB.mssDataPathObj.cycleLog.trackingTimeCurrInusec;
    //outputMessage.uartSendingTime = gMmwMssMCB.mssDataPathObj.cycleLog.sendingToUARTTimeCurrInusec;

    outputMessage.timeStamp = Cycleprofiler_getTimeStamp();
    outputMessage.numTLVs = 0;
    outputMessage.checksum = 0;

    packetLen = sizeof(TrackerDemo_output_message_header);
    
    /* Point cloud and side info */
    if (result->numObjOut > 0)
    {
        /* Point cloud */
        tl[tlvIdx].type = 6;//MMWDEMO_OUTPUT_MSG_DETECTED_POINTS;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudSpherical) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        outputMessage.numTLVs += 1;
        tlvIdx++;

        /* Side info */
        tl[tlvIdx].type = 9;//MMWDEMO_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO;
        tl[tlvIdx].length = sizeof(DPIF_PointCloudSideInfo) * result->numObjOut;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        outputMessage.numTLVs += 1;
        tlvIdx++;

    }
    
    if(result->numTargets) {

        /* Point cloud */
        tl[tlvIdx].type = 7;//TRACKERPROC_OUTPUT_TARGET_LIST;
        tl[tlvIdx].length = sizeof(trackerProc_Target) * result->numTargets;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        outputMessage.numTLVs += 1;
        tlvIdx++;
    }

    if(result->numIndices) {
 
        /* Point cloud */
        tl[tlvIdx].type = 8;//TRACKERPROC_OUTPUT_TARGET_INDEX;
        tl[tlvIdx].length = sizeof(trackerProc_TargetIndex) * result->numIndices;
        packetLen += sizeof(MmwDemo_output_message_tl) + tl[tlvIdx].length;
        outputMessage.numTLVs += 1;
        tlvIdx++;
    }

    /* Round up packet length to multiple of MMWDEMO_OUTPUT_MSG_SEGMENT_LEN */
    outputMessage.totalPacketLen = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN *
            ((packetLen + (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1))/MMWDEMO_OUTPUT_MSG_SEGMENT_LEN);

    /* Calculate header checksum */
    headerPtr = (uint16_t *)&outputMessage;
    for(n=0, sum = 0; n < sizeof(TrackerDemo_output_message_header)/sizeof(uint16_t); n++)
        sum += *headerPtr++;
    outputMessage.checksum = ~((sum >> 16) + (sum & 0xFFFF));

    /* Always send a packet header */
    UART_writePolling (uartHandle, (uint8_t *)&outputMessage, sizeof(TrackerDemo_output_message_header));

    tlvIdx = 0;
    /* Send detected objects and side info*/
    if(result->numObjOut > 0) {
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        Task_sleep(1);

        /*Send array of objects */
        UART_writePolling (uartHandle, (uint8_t*)objOutSph,
                           sizeof(DPIF_PointCloudSpherical) * result->numObjOut);
        tlvIdx++;

        Task_sleep(1);

        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        Task_sleep(1);

        /*Send array of objects */
        UART_writePolling (uartHandle, (uint8_t*)objOutSideInfo,
                           sizeof(DPIF_PointCloudSideInfo) * result->numObjOut);
        tlvIdx++;
        Task_sleep(1);

    }

    if(result->numTargets > 0) {
        /* If any targets tracked, send send target List TLV  */
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        Task_sleep(1);

        UART_writePolling (uartHandle, (uint8_t *)(tList),
                    sizeof(trackerProc_Target) * result->numTargets);
        
        tlvIdx++;
        
        Task_sleep(1);
    }

    if(result->numIndices > 0) {
        /* If exists, send target Index TLV  */
        UART_writePolling (uartHandle,
                           (uint8_t*)&tl[tlvIdx],
                           sizeof(MmwDemo_output_message_tl));
        Task_sleep(1);

        UART_writePolling (uartHandle, (uint8_t *)(tIndex),
                    sizeof(trackerProc_TargetIndex) * result->numIndices);
        
        tlvIdx++;
        
        Task_sleep(1);
    }

    /* Send padding bytes */
    numPaddingBytes = MMWDEMO_OUTPUT_MSG_SEGMENT_LEN - (packetLen & (MMWDEMO_OUTPUT_MSG_SEGMENT_LEN-1));
    if (numPaddingBytes<MMWDEMO_OUTPUT_MSG_SEGMENT_LEN)
    {
        UART_writePolling (uartHandle,
                            (uint8_t*)padding,
                            numPaddingBytes);
    }


    /* End: Tracker output code */

}

/**************************************************************************
 ******************** Millimeter Wave Demo control path Functions *****************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      The function is used to trigger the Front end to stop generating chirps.
 *
 *  @retval
 *      Not Applicable.
 */
static int32_t MmwDemo_mmWaveCtrlStop (void)
{
    int32_t                 errCode = 0;

    DebugP_log0("App: Issuing MMWave_stop\n");

    /* Stop the mmWave module: */
    if (MMWave_stop (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        if (errorLevel == MMWave_ErrorLevel_ERROR)
        {
            /* Error: Display the error message: */
            System_printf ("Error: mmWave Stop failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);

            /* Not expected */
            MmwDemo_debugAssert(0);
        }
        else
        {
            /* Warning: This is treated as a successful stop. */
            System_printf ("mmWave Stop error ignored [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
        }
    }

    return errCode;
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
static void MmwDemo_mmWaveCtrlTask(UArg arg0, UArg arg1)
{
    int32_t errCode;

    while (1)
    {
        /* Execute the mmWave control module: */
        if (MMWave_execute (gMmwMssMCB.ctrlHandle, &errCode) < 0)
        {
            //System_printf ("Error: mmWave control execution failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
        }
    }
}

/**************************************************************************
 ******************** Millimeter Wave Demo data path Functions *******************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Help function to make DPM_ioctl blocking until response is reported
 *
 *  @retval
 *      Success         -0
 *      Failed          <0
 */
static int32_t MmwDemo_DPM_ioctl_blocking
(
    DPM_Handle handle,
    uint32_t cmd,
    void* arg,
    uint32_t argLen
)
{
    int32_t retVal = 0;

    retVal = DPM_ioctl(handle,
                     cmd,
                     arg,
                     argLen);

    if(retVal == 0)
    {
        /* Wait until ioctl completed */
        Semaphore_pend(gMmwMssMCB.DPMioctlSemHandle, BIOS_WAIT_FOREVER);
    }

    return(retVal);
}

/**
 *  @b Description
 *  @n
 *    Function that configures the BPM chirps based on the stored BPM CLI commands.
 *    The MMW demo supports only a fixed BPM scheme and this scheme is implemented
 *    by this function.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_bpmConfig(void)
{
    uint8_t                subframe;
    uint8_t                numberOfSubframes;
    int32_t                errCode;
    rlBpmChirpCfg_t        bpmChirpCfg;
    MMWave_BpmChirpHandle  bpmChirpHandle;

    /* BPM configuration for all valid subframes */
    numberOfSubframes = MmwDemo_RFParser_getNumSubFrames(&gMmwMssMCB.cfg.ctrlCfg);

    /*    Note: The only BPM scheme supported by AoA DPU uses both Azimuth TX antennas
     *    (only 2 TX is allowed and the 2 TX must be the 2 azimuth antennas).
     *    If 3TX antennas are enabled in a given profile ,BPM can not be enabled.
     *
     *    Based on 68xx device antenna pattern layout, BPM can be enabled when only TX1 and TX3 are enabled.
     */
    for(subframe = 0; subframe < numberOfSubframes; subframe++)
    {
        /* Is BPM enabled*/
        if(gMmwMssMCB.subFrameCfg[subframe].bpmCfg.isEnabled)
        {
            /* Configure chirp0 (++) */
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            /* Phase configuration: Both TX antenna should be positive (for 68xx device, they are TX1 and TX3 ) 
              Note: rlBpmChirpCfg_t uses TX0 - TX2 which refers to TX1 - TX3 on 68xx devices
             */
            bpmChirpCfg.constBpmVal = 0U;
                        
            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg chirp 0. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }

            /*Configure chirp1 (+-) */
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            /* Phase configuration:, first Azimuth Tx antenna should be positive(for 68xx device, this is TX1)
                                 second Azimuth Tx antenna should be negtive(for 68xx device, this is TX3) */
            bpmChirpCfg.constBpmVal = 0x30U;
            
            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg chirp 1. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }
        }
        else
        {
            /*BPM is disabled.
              Configure the range of chirps [chirp0Idx..chirp1Idx]
              all to have zero phase.*/
            memset ((void *)&bpmChirpCfg, 0, sizeof(rlBpmChirpCfg_t));
            bpmChirpCfg.chirpStartIdx = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp0Idx;
            bpmChirpCfg.chirpEndIdx   = gMmwMssMCB.subFrameCfg[subframe].bpmCfg.chirp1Idx;
            bpmChirpCfg.constBpmVal = 0U;

            bpmChirpHandle = MMWave_addBpmChirp (gMmwMssMCB.ctrlHandle, &bpmChirpCfg, &errCode);
            if (bpmChirpHandle == NULL)
            {
                System_printf ("Error: Unable to add BPM cfg for BPM disabled. Subframe %d [Error code %d]\n",subframe, errCode);
                return -1;
            }
        }
    }

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Perform Data path driver open 
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dataPathOpen(void)
{
    gMmwMssMCB.adcBufHandle = MmwDemo_ADCBufOpen(gMmwMssMCB.socHandle);
    if(gMmwMssMCB.adcBufHandle == NULL)
    {
        MmwDemo_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Function to configure CQ.
 *
 *  @param[in] subFrameCfg Pointer to sub-frame config
 *  @param[in] numChirpsPerChirpEvent number of chirps per chirp event
 *  @param[in] validProfileIdx valid profile index
 *
 *  @retval
 *      0 if no error, else error (there will be system prints for these).
 */
static int32_t MmwDemo_configCQ(MmwDemo_SubFrameCfg *subFrameCfg,
                                uint8_t numChirpsPerChirpEvent,
                                uint8_t validProfileIdx)
{
    MmwDemo_AnaMonitorCfg*      ptrAnaMonitorCfg;
    ADCBuf_CQConf               cqConfig;
    rlRxSatMonConf_t*           ptrSatMonCfg;
    rlSigImgMonConf_t*          ptrSigImgMonCfg;
    int32_t                     retVal;
    uint16_t                    cqChirpSize;

    /* Get analog monitor configuration */
    ptrAnaMonitorCfg = &gMmwMssMCB.anaMonCfg;

    /* Config mmwaveLink to enable Saturation monitor - CQ2 */
    ptrSatMonCfg = &gMmwMssMCB.cqSatMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        if (ptrSatMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Saturation monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSaturationMonitor(ptrSatMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxIfSatMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSatMonCfg->profileIndx);
            goto exit;
        }
    }

    /* Config mmwaveLink to enable Saturation monitor - CQ1 */
    ptrSigImgMonCfg = &gMmwMssMCB.cqSigImgMonCfg[validProfileIdx];

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        if (ptrSigImgMonCfg->profileIndx != validProfileIdx)
        {
            System_printf ("Error: Sig/Image monitoring (globally) enabled but not configured for profile(%d)\n",
                           validProfileIdx);
            MmwDemo_debugAssert(0);
        }

        retVal = mmwDemo_cfgRxSigImgMonitor(ptrSigImgMonCfg);
        if(retVal != 0)
        {
            System_printf ("Error: rlRfRxSigImgMonConfig returns error = %d for profile(%d)\n",
                           retVal, ptrSigImgMonCfg->profileIndx);
            goto exit;
        }
    }

    retVal = mmwDemo_cfgAnalogMonitor(ptrAnaMonitorCfg);
    if (retVal != 0)
    {
        System_printf ("Error: rlRfAnaMonConfig returns error = %d\n", retVal);
        goto exit;
    }

    if(ptrAnaMonitorCfg->rxSatMonEn || ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* CQ driver config */
        memset((void *)&cqConfig, 0, sizeof(ADCBuf_CQConf));
        cqConfig.cqDataWidth = 0; /* 16bit for mmw demo */
        cqConfig.cq1AddrOffset = MMW_DEMO_CQ_SIGIMG_ADDR_OFFSET; /* CQ1 starts from the beginning of the buffer */
        cqConfig.cq2AddrOffset = MMW_DEMO_CQ_RXSAT_ADDR_OFFSET;  /* Address should be 16 bytes aligned */

        retVal = ADCBuf_control(gMmwMssMCB.adcBufHandle, ADCBufMMWave_CMD_CONF_CQ, (void *)&cqConfig);
        if (retVal < 0)
        {
            System_printf ("Error: MMWDemoDSS Unable to configure the CQ\n");
            MmwDemo_debugAssert(0);
        }
    }

    if (ptrAnaMonitorCfg->sigImgMonEn)
    {
        /* This is for 16bit format in mmw demo, signal/image band data has 2 bytes/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSigImgMonCfg->numSlices + 1) * sizeof(uint16_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->sigImgMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

    if (ptrAnaMonitorCfg->rxSatMonEn)
    {
        /* This is for 16bit format in mmw demo, saturation data has one byte/slice
           For other format, please check DFP interface document
         */
        cqChirpSize = (ptrSatMonCfg->numSlices + 1) * sizeof(uint8_t);
        cqChirpSize = MATHUTILS_ROUND_UP_UNSIGNED(cqChirpSize, MMW_DEMO_CQ_DATA_ALIGNMENT);
        subFrameCfg->satMonTotalSize = cqChirpSize * numChirpsPerChirpEvent;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Utility function to convert the CFAR threshold
 *      from a CLI encoded dB value to a linear value
 *      as expected by the CFAR DPU
 *
 *  @param[in] codedCfarVal CFAR threshold in dB as encoded in the CLI 
 *  @param[in] numVirtualAntennas Number of virtual antennas
 *
 *  @retval
 *      CFAR threshold in linear format
 */
static uint16_t MmwDemo_convertCfarToLinear(uint16_t codedCfarVal, uint8_t numVirtualAntennas)
{
    uint16_t linearVal;
    float    dbVal, linVal;   

    /* dbVal is a float value from 0-100dB. It needs to
    be converted to linear scale..
    First, recover float dbVal that was encoded in CLI. */
    dbVal = (float)(codedCfarVal / MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR);
    
    /* Now convert it to linear value according to the following:
    linear_value = dB_value * (256 / 6) * (numVirtualAntennas / (2^ ceil(log2(numVirtualAntennas)))) .
    */
    linVal = dbVal * (256.0 / 6.0) * ((float)numVirtualAntennas / (float)(1 << mathUtils_ceilLog2(numVirtualAntennas)));
    
    linearVal = (uint16_t) linVal;
    return (linearVal);
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the data path based on the chirp profile.
 *      After this function is executed, the data path processing will ready to go
 *      when the ADC buffer starts receiving samples corresponding to the chirps.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
static int32_t MmwDemo_dataPathConfig (void)
{
    int32_t                         errCode;
    MMWave_CtrlCfg                  *ptrCtrlCfg;
    MmwDemo_DPC_ObjDet_CommonCfg *objDetCommonCfg;
    MmwDemo_SubFrameCfg             *subFrameCfg;
    int8_t                          subFrameIndx;
    MmwDemo_RFParserOutParams       RFparserOutParams;
    DPC_ObjectDetectionRangeHWA_PreStartCfg objDetPreStartR4fCfg;
    DPC_ObjectDetectionRangeHWA_StaticCfg *staticCfg;
    DPC_ObjectDetection_PreStartCfg objDetPreStartDspCfg;
    DPC_ObjectDetectionRangeHWA_PreStartCommonCfg preStartCommonCfg;

    /* Get data path object and control configuration */
    ptrCtrlCfg = &gMmwMssMCB.cfg.ctrlCfg;

    objDetCommonCfg = &gMmwMssMCB.objDetCommonCfg;
    staticCfg = &objDetPreStartR4fCfg.staticCfg;

    /* Get RF frequency scale factor */
    gMmwMssMCB.rfFreqScaleFactor = SOC_getDeviceRFFreqScaleFactor(gMmwMssMCB.socHandle, &errCode);
    if (errCode < 0)
    {
        System_printf ("Error: Unable to get RF scale factor [Error:%d]\n", errCode);
        MmwDemo_debugAssert(0);
    }

    gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames =
        MmwDemo_RFParser_getNumSubFrames(ptrCtrlCfg);

    DebugP_log0("App: Issuing Pre-start Common Config IOCTL to R4F\n");

    /* DPC pre-start common config */
    preStartCommonCfg.numSubFrames = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    errCode = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &preStartCommonCfg,
                         sizeof (preStartCommonCfg));
    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }
    DebugP_log0("App: Issuing Pre-start Common Config IOCTL to DSP\n");

    /* DPC pre-start common config */
    errCode = MmwDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG,
                         &objDetCommonCfg->preStartCommonCfg,
                         sizeof (DPC_ObjectDetection_PreStartCommonCfg));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG [Error:%d]\n", errCode);
        goto exit;
    }

    MmwDemo_resetDynObjDetCommonCfgPendingState(&gMmwMssMCB.objDetCommonCfg);

    /* Reason for reverse loop is that when sensor is started, the first sub-frame
     * will be active and the ADC configuration needs to be done for that sub-frame
     * before starting (ADC buf hardware does not have notion of sub-frame, it will
     * be reconfigured every sub-frame). This cannot be alternatively done by calling
     * the MmwDemo_ADCBufConfig function only for the first sub-frame because this is
     * a utility API that computes the rxChanOffset that is part of ADC dataProperty
     * which will be used by range DPU and therefore this computation is required for
     * all sub-frames.
     */
    for(subFrameIndx = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames -1; subFrameIndx >= 0;
        subFrameIndx--)
    {
        subFrameCfg  = &gMmwMssMCB.subFrameCfg[subFrameIndx];

        /*****************************************************************************
         * Data path :: Algorithm Configuration
         *****************************************************************************/

        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        errCode = MmwDemo_RFParser_parseConfig(&RFparserOutParams, subFrameIndx,
                                         &gMmwMssMCB.cfg.openCfg, ptrCtrlCfg,
                                         &subFrameCfg->adcBufCfg,
                                         gMmwMssMCB.rfFreqScaleFactor,
                                         subFrameCfg->bpmCfg.isEnabled); 

        if (errCode != 0)
        {
            System_printf ("Error: MmwDemo_RFParser_parseConfig [Error:%d]\n", errCode);
            goto exit;
        }

        /* The following code is to enable processing for number of doppler chirps that are
         * less than 16 (the minimal numDopplerBins supported in doppler DPU DSP).
         * In this case, interpolate to detect better with CFAR tuning. E.g. a 2 -pt FFT will
         * be problematic in terms of distinguishing direction of motion */
        if (RFparserOutParams.numDopplerChirps <= 8)
        {
            RFparserOutParams.dopplerStep = RFparserOutParams.dopplerStep / (16 / RFparserOutParams.numDopplerBins);
            RFparserOutParams.numDopplerBins = 16;
        }

        subFrameCfg->numRangeBins = RFparserOutParams.numRangeBins;
        /* Workaround for range DPU limitation for FFT size 1024 and 12 virtual antennas case*/
        if ((RFparserOutParams.numVirtualAntennas == 12) && (RFparserOutParams.numRangeBins == 1024))
        {
            subFrameCfg->numRangeBins = 1022;
            RFparserOutParams.numRangeBins = 1022;
        }

        subFrameCfg->numDopplerBins = RFparserOutParams.numDopplerBins;
        subFrameCfg->numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
        subFrameCfg->adcBufChanDataSize = RFparserOutParams.adcBufChanDataSize;
        subFrameCfg->objDetDynCfg.dspDynCfg.prepareRangeAzimuthHeatMap = subFrameCfg->guiMonSel.rangeAzimuthHeatMap;
        subFrameCfg->numAdcSamples = RFparserOutParams.numAdcSamples;
        subFrameCfg->numChirpsPerSubFrame = RFparserOutParams.numChirpsPerFrame;
        subFrameCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;

        errCode = MmwDemo_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 subFrameCfg->numChirpsPerChirpEvent,
                                 subFrameCfg->adcBufChanDataSize,
                                 &subFrameCfg->adcBufCfg,
                                 &staticCfg->ADCBufData.dataProperty.rxChanOffset[0]);
        if (errCode < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", errCode);
            MmwDemo_debugAssert (0);
        }

        errCode = MmwDemo_configCQ(subFrameCfg, RFparserOutParams.numChirpsPerChirpEvent,
                                   RFparserOutParams.validProfileIdx);

        if (errCode < 0)
        {
            goto exit;
        }

        /* DPC pre-start config */
        {
            int32_t idx;

            objDetPreStartR4fCfg.subFrameNum = subFrameIndx;

            /* Fill static configuration */
            staticCfg->ADCBufData.data = (void *)SOC_XWRXXXX_MSS_ADCBUF_BASE_ADDRESS;
            staticCfg->ADCBufData.dataProperty.adcBits = 2; /* 16-bit */

            /* only complex format supported */
            MmwDemo_debugAssert(subFrameCfg->adcBufCfg.adcFmt == 0);

            if (subFrameCfg->adcBufCfg.iqSwapSel == 1)
            {
                staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_IMRE;
            }
            else
            {
                staticCfg->ADCBufData.dataProperty.dataFmt = DPIF_DATAFORMAT_COMPLEX16_REIM;
            }
            if (subFrameCfg->adcBufCfg.chInterleave == 0)
            {
                staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_INTERLEAVE_MODE;
            }
            else
            {
                staticCfg->ADCBufData.dataProperty.interleave = DPIF_RXCHAN_NON_INTERLEAVE_MODE;
            }
            staticCfg->ADCBufData.dataProperty.numAdcSamples = RFparserOutParams.numAdcSamples;
            staticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent = RFparserOutParams.numChirpsPerChirpEvent;
            staticCfg->ADCBufData.dataProperty.numRxAntennas = RFparserOutParams.numRxAntennas;
            staticCfg->ADCBufData.dataSize = RFparserOutParams.numRxAntennas * RFparserOutParams.numAdcSamples * sizeof(cmplx16ImRe_t);
            staticCfg->numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
            staticCfg->numDopplerChirps = RFparserOutParams.numDopplerChirps;
            staticCfg->numRangeBins = RFparserOutParams.numRangeBins;
            staticCfg->numTxAntennas = RFparserOutParams.numTxAntennas;
            staticCfg->numVirtualAntennas = RFparserOutParams.numVirtualAntennas;

			/* Current 68xx SOC has higher receive level as compared to 18xx and hence using higher value for 
             * fftOutputDivShift to avoid overflow when converting from 24-bit to 16-bit
             * TODO: Future RadarSS firmware should be evaluated to assess if these settings are correct
             */
            if (RFparserOutParams.numRangeBins >= 1022) 
            {        
                staticCfg->rangeFFTtuning.fftOutputDivShift = 1;
                /* scale only 2 stages */
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 2; 
            } 
            else if (RFparserOutParams.numRangeBins==512)
            {        
                staticCfg->rangeFFTtuning.fftOutputDivShift = 2;
                /* scale last stage */
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 1; 
            } 
            else    
            {        
                staticCfg->rangeFFTtuning.fftOutputDivShift = 3;
                /* no scaling needed as ADC data is 16-bit and we have 8 bits to grow */
                staticCfg->rangeFFTtuning.numLastButterflyStagesToScale = 0; 
            }
            
            /* objectdetection DSP DPC needs radacube in format DPIF_RADARCUBE_FORMAT_1 */
            staticCfg->radarCubeFormat = DPIF_RADARCUBE_FORMAT_1; 																			
            /* Fill dynamic configuration for the sub-frame */
            objDetPreStartR4fCfg.dynCfg = subFrameCfg->objDetDynCfg.r4fDynCfg;

            DebugP_log1("App: Issuing Pre-start Config IOCTL (subFrameIndx = %d)\n", subFrameIndx);

            /* send pre-start config to R4F chain, using blocking call here */
            errCode = MmwDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartR4fCfg,
                                 sizeof (DPC_ObjectDetectionRangeHWA_PreStartCfg));
            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
            DebugP_log0("App: DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG is processed \n");

            /***********************************************************************
              Pre-start preparation for objdetdsp
             ***********************************************************************/
            /* Reset preStart config */
            memset((void *)&objDetPreStartDspCfg, 0, sizeof(DPC_ObjectDetection_PreStartCfg));

            /* DPC configuration */
            objDetPreStartDspCfg.subFrameNum = subFrameIndx;

            /* Convert CFAR threshold value */
            subFrameCfg->objDetDynCfg.dspDynCfg.cfarCfgRange.thresholdScale = 
                MmwDemo_convertCfarToLinear(subFrameCfg->objDetDynCfg.dspDynCfg.cfarCfgRange.thresholdScale, 
                                            staticCfg->numVirtualAntennas);

            subFrameCfg->objDetDynCfg.dspDynCfg.cfarCfgDoppler.thresholdScale = 
                MmwDemo_convertCfarToLinear(subFrameCfg->objDetDynCfg.dspDynCfg.cfarCfgDoppler.thresholdScale, 
                                            staticCfg->numVirtualAntennas);
                                        
            objDetPreStartDspCfg.dynCfg = subFrameCfg->objDetDynCfg.dspDynCfg;

            memcpy((void *)&objDetPreStartDspCfg.staticCfg.ADCBufData, 
                    (void *)&staticCfg->ADCBufData,
                    sizeof(DPIF_ADCBufData));
            objDetPreStartDspCfg.staticCfg.dopplerStep = RFparserOutParams.dopplerStep;
            objDetPreStartDspCfg.staticCfg.isValidProfileHasOneTxPerChirp = RFparserOutParams.validProfileHasOneTxPerChirp;
            objDetPreStartDspCfg.staticCfg.numChirpsPerFrame = RFparserOutParams.numChirpsPerFrame;
            objDetPreStartDspCfg.staticCfg.numDopplerBins = RFparserOutParams.numDopplerBins;
            objDetPreStartDspCfg.staticCfg.numDopplerChirps = RFparserOutParams.numDopplerChirps;
            objDetPreStartDspCfg.staticCfg.numRangeBins = RFparserOutParams.numRangeBins;
            objDetPreStartDspCfg.staticCfg.numTxAntennas = RFparserOutParams.numTxAntennas;
            objDetPreStartDspCfg.staticCfg.numVirtualAntAzim = RFparserOutParams.numVirtualAntAzim;
            objDetPreStartDspCfg.staticCfg.numVirtualAntElev = RFparserOutParams.numVirtualAntElev;
            objDetPreStartDspCfg.staticCfg.numVirtualAntennas = RFparserOutParams.numVirtualAntennas;
            objDetPreStartDspCfg.staticCfg.rangeStep = RFparserOutParams.rangeStep;
            objDetPreStartDspCfg.staticCfg.isBpmEnabled = subFrameCfg->bpmCfg.isEnabled;
            for (idx = 0; idx < RFparserOutParams.numRxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.rxAntOrder[idx] = RFparserOutParams.rxAntOrder[idx];
            }
            for (idx = 0; idx < RFparserOutParams.numTxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.txAntOrder[idx] = RFparserOutParams.txAntOrder[idx];
            }

            for (idx = 0; idx < RFparserOutParams.numRxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.rxAntOrder[idx] = RFparserOutParams.rxAntOrder[idx];
            }
            for (idx = 0; idx < RFparserOutParams.numTxAntennas; idx++)
            {
                objDetPreStartDspCfg.staticCfg.txAntOrder[idx] = RFparserOutParams.txAntOrder[idx];
            }

            /* The L3 memory and radarCube memory usage are reported and saved in @ref MmwDemo_DPC_ObjectDetection_reportFxn.
               The memory information is configured here and passed to objdetdsp chain.
             */
            if(gMmwMssMCB.dataPathObj.radarCubeMem.addr != 0)
            {
                /* Update DPC radar cube configuration */
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr = gMmwMssMCB.dataPathObj.radarCubeMem.addr;
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.size = gMmwMssMCB.dataPathObj.radarCubeMem.size;

                /* Update DPC L3 RAM configuration */
                objDetPreStartDspCfg.shareMemCfg.L3Ram.addr = (void *)((uint32_t)(gMmwMssMCB.dataPathObj.radarCubeMem.addr) +
                                                                   gMmwMssMCB.dataPathObj.memUsage.L3RamUsage);
                objDetPreStartDspCfg.shareMemCfg.L3Ram.size =gMmwMssMCB.dataPathObj.memUsage.L3RamTotal - gMmwMssMCB.dataPathObj.memUsage.L3RamUsage;

                /* Convert address for DSP core */
                objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
                DebugP_assert ((uint32_t)objDetPreStartDspCfg.shareMemCfg.radarCubeMem.addr != SOC_TRANSLATEADDR_INVALID);

                objDetPreStartDspCfg.shareMemCfg.L3Ram.addr = (void *) SOC_translateAddress((uint32_t)objDetPreStartDspCfg.shareMemCfg.L3Ram.addr,
                                                 SOC_TranslateAddr_Dir_TO_OTHER_CPU,
                                                 &errCode);
                DebugP_assert ((uint32_t)objDetPreStartDspCfg.shareMemCfg.L3Ram.addr != SOC_TRANSLATEADDR_INVALID);

                /* Enable shared memory configuration */
                objDetPreStartDspCfg.shareMemCfg.shareMemEnable = true;
            }

            /* send pre-start config */
            errCode = MmwDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG,
                                 &objDetPreStartDspCfg,
                                 sizeof (DPC_ObjectDetection_PreStartCfg));
            DebugP_log0("App: DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG is processed \n");

            MmwDemo_resetDynObjDetCfgPendingState(&subFrameCfg->objDetDynCfg);

            if (errCode < 0)
            {
                System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG [Error:%d]\n", errCode);
                goto exit;
            }
        }
    }
exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      The function is used to configure the tracker DPU.
 *
 *  @retval
 *      0 if no error, error code otherwise.
 */
int32_t MmwDemo_trackerConfig (void)
{
    int32_t    errCode;

    DebugP_log0("App: Issuing Tracker Static Config IOCTL\n");

    /* DPC pre-start common config */
    //errCode =  DPM_ioctl (dataPathObj->objDetDpmHandle,
#ifdef TRACKERPROC_ON_DSP
    errCode =  MmwDemo_DPM_ioctl_blocking (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDET_IOCTL__STATIC_TRACKER_CFG,
                         &(gMmwMssMCB.trackerCfg),
                         sizeof (DPC_ObjectDetection_TrackerConfig));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDET_IOCTL__TRACKER_STATIC_CFG [Error:%d]\n", errCode);
        goto exit;
    }
#else
    errCode =  DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG,
                         &(gMmwMssMCB.trackerCfg),
                         sizeof (DPC_ObjectDetection_TrackerConfig));

    if (errCode < 0)
    {
        System_printf ("Error: Unable to send DPC_OBJDETRANGEHWA_IOCTL__TRACKER_STATIC_CFG [Error:%d]\n", errCode);
        goto exit;
    }
#endif

exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      This function is used to start data path to handle chirps from front end.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dataPathStart (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_start\n");

    /* Configure HW LVDS stream for the first sub-frame that will start upon
     * start of frame */
    if (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        MmwDemo_configLVDSHwData(0);
    }

    /* Start the DPM Profile: */
    if ((retVal = DPM_start(gMmwMssMCB.objDetDpmHandle)) < 0)
    {
        /* Error: Unable to start the profile */
        System_printf("Error: Unable to start the DPM [Error: %d]\n", retVal);
        MmwDemo_debugAssert(0);
    }

    /* Wait until start completed */
    Semaphore_pend(gMmwMssMCB.DPMstartSemHandle, BIOS_WAIT_FOREVER);

    DebugP_log0("App: DPM_start Done (post Semaphore_pend on reportFxn reporting start)\n");
}

/**
 *  @b Description
 *  @n
 *      This function is used to stop data path.
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_dataPathStop (void)
{
    int32_t retVal;

    DebugP_log0("App: Issuing DPM_stop\n");

    retVal = DPM_stop (gMmwMssMCB.objDetDpmHandle);
    if (retVal < 0)
    {
        System_printf ("DPM_stop failed[Error code %d]\n", retVal);
        MmwDemo_debugAssert(0);
    }
}

/**
 *  @b Description
 *  @n
 *      Registered event function to mmwave which is invoked when an event from the
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
 *      Always return 0
 */
static int32_t MmwDemo_eventCallbackFxn(uint16_t msgId, uint16_t sbId, uint16_t sbLen, uint8_t *payload)
{
    uint16_t asyncSB = RL_GET_SBID_FROM_UNIQ_SBID(sbId);

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
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ESMFAULT_SB:
                {
                    MmwDemo_debugAssert(0);
                    break;
                }
                case RL_RF_AE_ANALOG_FAULT_SB:
                {
                    MmwDemo_debugAssert(0);
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
                    gMmwMssMCB.stats.frameTriggerReady++;
                    break;
                }
                case RL_RF_AE_MON_TIMING_FAIL_REPORT_SB:
                {
                    gMmwMssMCB.stats.failedTimingReports++;
                    break;
                }
                case RL_RF_AE_RUN_TIME_CALIB_REPORT_SB:
                {
                    gMmwMssMCB.stats.calibrationReports++;
                    break;
                }
                case RL_RF_AE_FRAME_END_SB:
                {
                    gMmwMssMCB.stats.sensorStopped++;
                    DebugP_log0("App: BSS stop (frame end) received\n");

                    MmwDemo_dataPathStop();
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
        /* Async Event from MMWL */
        case RL_MMWL_ASYNC_EVENT_MSG:
        {
            switch (asyncSB)
            {
                case RL_MMWL_AE_MISMATCH_REPORT:
                {
                    /* link reports protocol error in the async report from BSS */
                    MmwDemo_debugAssert(0);
                    break;
                }            
                case RL_MMWL_AE_INTERNALERR_REPORT:
                {
                    /* link reports internal error during BSS communication */
                    MmwDemo_debugAssert(0);
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
 *      DPM Registered Report Handler. The DPM Module uses this registered function to notify
 *      the application about DPM reports.
 *
 *  @param[in]  reportType
 *      Report Type
 *  @param[in]  instanceId
 *      Instance Identifier which generated the report
 *  @param[in]  errCode
 *      Error code if any.
 *  @param[in] arg0
 *      Argument 0 interpreted with the report type
 *  @param[in] arg1
 *      Argument 1 interpreted with the report type
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_DPC_ObjectDetection_reportFxn
(
    DPM_Report  reportType,
    uint32_t    instanceId,
    int32_t     errCode,
    uint32_t    arg0,
    uint32_t    arg1
)
{
    /* Only errors are logged on the console: */
    if ((errCode != 0) )
    {
        /* Error: Detected log on the console and die all errors are FATAL currently. */
        System_printf ("Error: DPM Report %d received with error:%d arg0:0x%x arg1:0x%x\n",
                        reportType, errCode, arg0, arg1);
        DebugP_assert (0);
    }

    /* Processing further is based on the reports received: This is the control of the profile
     * state machine: */
    switch (reportType)
    {
        case DPM_Report_IOCTL:
        {
            /*****************************************************************
             * DPC has been configured without an error:
             * - This is an indication that the profile configuration commands
             *   went through without any issues.
             *****************************************************************/
            DebugP_log1("App: DPM Report IOCTL, command = %d\n", arg0);

            if (arg0 == DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG)
            {
                DPC_ObjectDetectionRangeHWA_PreStartCfg *cfg;
                DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage *memUsage;

                cfg = (DPC_ObjectDetectionRangeHWA_PreStartCfg*)arg1;

                /* Save radar cube memory information, it will be passed to objdetdsp chain for inter-frame processing */
                memcpy((void *)&gMmwMssMCB.dataPathObj.radarCubeMem, (void *)&cfg->radarCubeMem,
                    sizeof(DPC_ObjectDetectionRangeHWA_preStartCfg_radarCubeMem));

                /* Get memory usage and print the usage */
                memUsage = &cfg->memUsage;

                /* Save memory usage, it will be passed to objdetdsp chain for inter-frame processing */
                memcpy((void *)&gMmwMssMCB.dataPathObj.memUsage, (void *)memUsage,
                    sizeof(DPC_ObjectDetectionRangeHWA_preStartCfg_memUsage));

                System_printf("============ Heap Memory Stats ============\n");
                System_printf("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n", "System Heap(TCMB)",
                              memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                              memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                              memUsage->SystemHeapDPCUsed);

                System_printf("%20s %12d %12d %12d\n", "L3",
                              memUsage->L3RamTotal,
                              memUsage->L3RamUsage,
                              memUsage->L3RamTotal - memUsage->L3RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(TCMB)",
                              memUsage->CoreLocalRamTotal,
                              memUsage->CoreLocalRamUsage,
                              memUsage->CoreLocalRamTotal - memUsage->CoreLocalRamUsage);
            }
            if (arg0 == DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG) 
            {
                DPC_ObjectDetection_PreStartCfg *cfg;
                DPC_ObjectDetection_DPC_IOCTL_preStartCfg_memUsage *memUsage;

                cfg = (DPC_ObjectDetection_PreStartCfg*)arg1;

                memUsage = &cfg->memUsage;

                System_printf("============ Heap Memory Stats ============\n");
                System_printf("%20s %12s %12s %12s %12s\n", " ", "Size", "Used", "Free", "DPCUsed");
                System_printf("%20s %12d %12d %12d %12d\n", "System Heap(L2)",
                              memUsage->SystemHeapTotal, memUsage->SystemHeapUsed,
                              memUsage->SystemHeapTotal - memUsage->SystemHeapUsed,
                              memUsage->SystemHeapDPCUsed);

                System_printf("%20s %12d %12d %12d\n", "L3",
                              memUsage->L3RamTotal,
                              memUsage->L3RamUsage,
                              memUsage->L3RamTotal - memUsage->L3RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L2)",
                              memUsage->CoreL2RamTotal,
                              memUsage->CoreL2RamUsage,
                              memUsage->CoreL2RamTotal - memUsage->CoreL2RamUsage);

                System_printf("%20s %12d %12d %12d\n", "localRam(L1)",
                              memUsage->CoreL1RamTotal,
                              memUsage->CoreL1RamUsage,
                              memUsage->CoreL1RamTotal - memUsage->CoreL1RamUsage);
            }

            switch(arg0)
            {
                /* The following ioctls take longer time to finish. It causes DPM to queue IOCTL requests on DSS before
                 * they are handled. However DPM has limited pipe queues, hence adding sync points in demo to avoid 
                 * sending too many such ioctls to DSS at a time.
                 * The semaphore blocks CLI task to wait for the response from DSS before sending the next ioctl.
                 */
#ifdef TRACKERPROC_ON_DSP
                case DPC_OBJDET_IOCTL__STATIC_TRACKER_CFG:
#else
                case DPC_OBJDETRANGEHWA_IOCTL__STATIC_TRACKER_CFG:
#endif
                case DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG:
                case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
                /* The following ioctl returns memory information needs to be used in configuration follows the ioctl.
                 * The semaphore blocks CLI task to wiat for the response from DPC before further execution.
                 */
                case DPC_OBJDETRANGEHWA_IOCTL__STATIC_PRE_START_CFG:
                    Semaphore_post(gMmwMssMCB.DPMioctlSemHandle);
                    break;
                default:
                    break;
            }
            break;
        }
        case DPM_Report_DPC_STARTED:
        {
            /*****************************************************************
             * DPC has been started without an error:
             * - notify sensor management task that DPC is started.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Started\n");
            gMmwMssMCB.stats.dpmStartEvents++;
            /* every sensor start should cause 2 DPM start events due to distributed domain 
               Wait for both the events before proceeding with remaining steps */
            if (gMmwMssMCB.stats.dpmStartEvents % 2 == 0) 
            {
                Semaphore_post(gMmwMssMCB.DPMstartSemHandle);
            }
            break;
        }
        case DPM_Report_NOTIFY_DPC_RESULT:
        {
            /*****************************************************************
             * datapath has finished frame processing, results are reported
             *****************************************************************/
            DPM_Buffer*     ptrResult;

            /* Get the result: */
            ptrResult = (DPM_Buffer*)arg0;

            MmwDemo_handleObjectDetResult(ptrResult);
            break;
        }
        case DPM_Report_DPC_ASSERT:
        {
            DPM_DPCAssert*  ptrAssert;

            /*****************************************************************
             * DPC Fault has been detected:
             * - This implies that the DPC has crashed.
             * - The argument0 points to the DPC assertion information
             *****************************************************************/
            ptrAssert = (DPM_DPCAssert*)arg0;
            CLI_write("Obj Det DPC Exception: %s, line %d.\n", ptrAssert->fileName,
                       ptrAssert->lineNum);
            break;
        }
        case DPM_Report_DPC_STOPPED:
        {
            /*****************************************************************
             * DPC has been stopped without an error:
             * - This implies that the DPC can either be reconfigured or
             *   restarted.
             *****************************************************************/
            DebugP_log0("App: DPM Report DPC Stopped\n");
			gMmwMssMCB.stats.dpmStopEvents++;
            /* every sensor stop should cause 2 DPM stop events due to distributed domain 
               Wait for both the events before proceeding with remaining steps */
            if (gMmwMssMCB.stats.dpmStopEvents % 2 == 0) 
            {
                Semaphore_post(gMmwMssMCB.DPMstopSemHandle);
            }								 
            break;
        }
        case DPM_Report_DPC_INFO:
        case DPM_Report_NOTIFY_DPC_RESULT_ACKED:
        {
            /* Currently objDetDsp does not use this feature. */
            break;
        }
        default:
        {
            DebugP_assert (0);
            break;
        }
    }
    return;
}

/**
 *  @b Description
 *  @n
 *      Utility function to get next sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of next sub-frame.
 */
static uint8_t MmwDemo_getNextSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t nextIndx;

    if (currentIndx == (numSubFrames - 1))
    {
        nextIndx = 0;
    }
    else
    {
        nextIndx = currentIndx + 1;
    }
    return(nextIndx);
}

/**
 *  @b Description
 *  @n
 *      Utility function to get previous sub-frame index
 *
 *  @param[in] currentIndx      Current sub-frame index
 *  @param[in] numSubFrames     Number of sub-frames
 *
 *  @retval
 *      Index of previous sub-frame
 */
static uint8_t MmwDemo_getPrevSubFrameIndx(uint8_t currentIndx, uint8_t numSubFrames)
{
    uint8_t prevIndx;

    if (currentIndx == 0)
    {
        prevIndx = numSubFrames - 1;
    }
    else
    {
        prevIndx = currentIndx - 1;
    }
    return(prevIndx);
}

/**
 *  @b Description
 *  @n
 *      Processes any pending dynamic configuration commands for the specified
 *      sub-frame by fanning out to the respective DPUs using IOCTL interface, and
 *      resets (clears) the pending state after processing.
 *
 *  @param[in] subFrameIndx Sub-frame index of desired sub-frame to process
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t MmwDemo_processPendingDynamicCfgCommands(uint8_t subFrameIndx)
{
    int32_t retVal =0;

    MmwDemo_DPC_ObjDet_CommonCfg *commonCfg = &gMmwMssMCB.objDetCommonCfg;
    MmwDemo_DPC_ObjDet_DynCfg *subFrameCfg = &gMmwMssMCB.subFrameCfg[subFrameIndx].objDetDynCfg;
    uint8_t numVirtualAntennas = gMmwMssMCB.subFrameCfg[subFrameIndx].numVirtualAntennas;

    /* perform globals ones if first sub-frame */
    if (subFrameIndx == 0)
    {
        if (commonCfg->isMeasureRxChannelBiasCfgPending == 1)
        {
            retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE,
                                 &commonCfg->preStartCommonCfg.measureRxChannelBiasCfg,
                                 sizeof (DPC_ObjectDetection_MeasureRxChannelBiasCfg));
            if (retVal != 0)
            {
                goto exit;
            }
            commonCfg->isMeasureRxChannelBiasCfgPending = 0;
        }
        if (commonCfg->isCompRxChannelBiasCfgPending == 1)
        {
            retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                                 DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE,
                                 &commonCfg->preStartCommonCfg.compRxChanCfg,
                                 sizeof (DPU_AoAProc_compRxChannelBiasCfg));
            if (retVal != 0)
            {
                goto exit;
            }
            commonCfg->isCompRxChannelBiasCfgPending = 0;
        }
    }

    /* Perform sub-frame specific ones */
    if (subFrameCfg->isCalibDcRangeSigCfg == 1)
    {
        DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->r4fDynCfg.calibDcRangeSigCfg;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDETRANGEHWA_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG,
                             &cfg,
                             sizeof (DPC_ObjectDetectionRangeHWA_CalibDcRangeSigCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCalibDcRangeSigCfg = 0;
    }
    if (subFrameCfg->isCfarCfgDopplerPending == 1)
    {
        DPC_ObjectDetection_CfarCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        /* Update with correct threshold value based on number of virtual antennas */
        subFrameCfg->dspDynCfg.cfarCfgDoppler.thresholdScale = 
            MmwDemo_convertCfarToLinear(subFrameCfg->dspDynCfg.cfarCfgDoppler.thresholdScale, 
                                        numVirtualAntennas);

        cfg.cfg = subFrameCfg->dspDynCfg.cfarCfgDoppler;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_CFAR_DOPPLER_CFG,
                             &cfg,
                             sizeof (DPC_ObjectDetection_CfarCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCfarCfgDopplerPending = 0;
    }
    if (subFrameCfg->isCfarCfgRangePending == 1)
    {
        DPC_ObjectDetection_CfarCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        /* Update with correct threshold value based on number of virtual antennas */
        subFrameCfg->dspDynCfg.cfarCfgRange.thresholdScale = 
            MmwDemo_convertCfarToLinear(subFrameCfg->dspDynCfg.cfarCfgRange.thresholdScale, 
                                        numVirtualAntennas);
        
        cfg.cfg = subFrameCfg->dspDynCfg.cfarCfgRange;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_CFAR_RANGE_CFG,
                             &cfg,
                             sizeof (DPC_ObjectDetection_CfarCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isCfarCfgRangePending = 0;
    }
    if (subFrameCfg->isFovDopplerPending == 1)
    {
        DPC_ObjectDetection_fovDopplerCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.fovDoppler;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_FOV_DOPPLER,
                             &cfg,
                             sizeof (DPC_ObjectDetection_fovDopplerCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovDopplerPending = 0;
    }
    if (subFrameCfg->isFovRangePending == 1)
    {
        DPC_ObjectDetection_fovRangeCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.fovRange;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_FOV_RANGE,
                             &cfg,
                             sizeof (DPC_ObjectDetection_fovRangeCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovRangePending = 0;
    }
    if (subFrameCfg->isMultiObjBeamFormingCfgPending == 1)
    {
        DPC_ObjectDetection_MultiObjBeamFormingCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.multiObjBeamFormingCfg;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_MULTI_OBJ_BEAM_FORM_CFG,
                             &cfg,
                             sizeof (DPC_ObjectDetection_MultiObjBeamFormingCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isMultiObjBeamFormingCfgPending = 0;
    }
    if (subFrameCfg->isPrepareRangeAzimuthHeatMapPending == 1)
    {
        DPC_ObjectDetection_RangeAzimuthHeatMapCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.prepareRangeAzimuthHeatMap = subFrameCfg->dspDynCfg.prepareRangeAzimuthHeatMap;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_RANGE_AZIMUTH_HEAT_MAP,
                             &cfg,
                             sizeof (DPC_ObjectDetection_RangeAzimuthHeatMapCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isPrepareRangeAzimuthHeatMapPending = 0;
    }
    if (subFrameCfg->isStaticClutterRemovalCfgPending == 1)
    {
        DPC_ObjectDetection_StaticClutterRemovalCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.staticClutterRemovalCfg;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_STATICCLUTTER_REMOVAL_CFG,
                             &cfg,
                             sizeof (DPC_ObjectDetection_StaticClutterRemovalCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isStaticClutterRemovalCfgPending = 0;
    }
    if (subFrameCfg->isFovAoaCfgPending == 1)
    {
        DPC_ObjectDetection_fovAoaCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.fovAoaCfg;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                             DPC_OBJDET_IOCTL__DYNAMIC_FOV_AOA,
                             &cfg,
                             sizeof (DPC_ObjectDetection_fovAoaCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isFovAoaCfgPending = 0;
    }
    if (subFrameCfg->isExtMaxVelCfgPending == 1)
    {
        DPC_ObjectDetection_extMaxVelCfg cfg;

        cfg.subFrameNum = subFrameIndx;
        cfg.cfg = subFrameCfg->dspDynCfg.extMaxVelCfg;
        retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                            DPC_OBJDET_IOCTL__DYNAMIC_EXT_MAX_VELOCITY,
                            &cfg,
                            sizeof (DPC_ObjectDetection_extMaxVelCfg));
        if (retVal != 0)
        {
            goto exit;
        }
        subFrameCfg->isExtMaxVelCfgPending = 0;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Transmit user data over LVDS interface.
 *
 *  @param[in]  subFrameIndx Sub-frame index
 *  @param[in]  dpcResults   pointer to DPC result
 *
 */
void MmwDemo_transferLVDSUserData(uint8_t subFrameIndx,
                                  DPC_ObjectDetection_ExecuteResult *dpcResults)
{
    int32_t errCode;
    DPC_ObjectDetection_Stats *stats;
    DPIF_PointCloudCartesian *objOut;
    DPIF_PointCloudSideInfo *objOutSideInfo;

    stats = (DPC_ObjectDetection_Stats *) SOC_translateAddress((uint32_t)dpcResults->stats,
                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                 &errCode);
    DebugP_assert ((uint32_t)stats != SOC_TRANSLATEADDR_INVALID);

    objOut = (DPIF_PointCloudCartesian *) SOC_translateAddress((uint32_t)dpcResults->objOut,
                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                 &errCode);
    DebugP_assert ((uint32_t)objOut != SOC_TRANSLATEADDR_INVALID);

    objOutSideInfo = (DPIF_PointCloudSideInfo *) SOC_translateAddress((uint32_t)dpcResults->objOutSideInfo,
                                                 SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                                 &errCode);
    DebugP_assert ((uint32_t)objOutSideInfo != SOC_TRANSLATEADDR_INVALID);

    /* Delete previous SW session if it exists. SW session is being
       reconfigured every frame because number of detected objects
       may change from frame to frame which implies that the size of
       the streamed data may change. */
    if(gMmwMssMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }

    /* Configure SW session for this subframe */
    if (MmwDemo_LVDSStreamSwConfig(dpcResults->numObjOut,
                                   objOut,
                                   objOutSideInfo) < 0)
    {
        System_printf("Failed LVDS stream SW configuration for sub-frame %d\n", subFrameIndx);
        MmwDemo_debugAssert(0);
        return;
    }

    /* Populate user data header that will be streamed out*/
    gMmwMssMCB.lvdsStream.userDataHeader.frameNum  = stats->frameStartIntCounter;
    gMmwMssMCB.lvdsStream.userDataHeader.detObjNum = dpcResults->numObjOut;
    gMmwMssMCB.lvdsStream.userDataHeader.subFrameNum  = (uint16_t) dpcResults->subFrameIdx;

    /* If SW LVDS stream is enabled, start the session here. User data will immediately
       start to stream over LVDS.*/
    if(CBUFF_activateSession (gMmwMssMCB.lvdsStream.swSessionHandle, &errCode) < 0)
    {
        System_printf("Failed to activate CBUFF session for LVDS stream SW. errCode=%d\n",errCode);
        MmwDemo_debugAssert(0);
        return;
    }
}

/**
 *  @b Description
 *  @n
 *      Function to handle frame processing results from DPC
 *
 *  @param[in] ptrResult      Pointer to DPC result
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_handleObjectDetResult
(
    DPM_Buffer  *ptrResult
)
{
    int32_t     retVal;
    DPC_ObjectDetection_ExecuteResultExportedInfo exportInfo;
    DPC_ObjectDetection_ExecuteResult        *dpcResults;
    MmwDemo_output_message_stats            *frameStats;
    volatile uint32_t                        startTime;
    uint8_t                                  nextSubFrameIdx;
    uint8_t                                  numSubFrames;
    uint8_t                                  currSubFrameIdx;
    uint8_t                                  prevSubFrameIdx;
    MmwDemo_SubFrameStats                    *currSubFrameStats;
    MmwDemo_SubFrameStats                    *prevSubFrameStats;

#ifndef TRACKERPROC_ON_DSP
    DPU_TrackerProc_OutParams outTrackerProc;
    DPIF_PointCloudSpherical *objOutSph;
    DPIF_PointCloudSideInfo *objOutSideInfo;
    int32_t errCode;
#endif

    /*****************************************************************
     * datapath has finished frame processing, results are reported
     *****************************************************************/

    /* Validate DPC results buffer */
    DebugP_assert (ptrResult->size[0] == sizeof(DPC_ObjectDetection_ExecuteResult));

    /* Translate the address: */
    dpcResults = (DPC_ObjectDetection_ExecuteResult *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[0],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)dpcResults != SOC_TRANSLATEADDR_INVALID);

    /* Validate timing Info buffer */
    DebugP_assert (ptrResult->size[1] == sizeof(MmwDemo_output_message_stats));

    numSubFrames = gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames;
    currSubFrameIdx = dpcResults->subFrameIdx;
    prevSubFrameIdx = MmwDemo_getPrevSubFrameIndx(currSubFrameIdx, numSubFrames);
    currSubFrameStats = &gMmwMssMCB.subFrameStats[currSubFrameIdx];
    prevSubFrameStats = &gMmwMssMCB.subFrameStats[prevSubFrameIdx];

    /*****************************************************************
     * Transmit results
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();

    /* Send out of CLI the range bias and phase config measurement if it was enabled. */
    if (gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.measureRxChannelBiasCfg.enabled == 1)
    {
        if(dpcResults->compRxChanBiasMeasurement != NULL)
        {
            dpcResults->compRxChanBiasMeasurement =
                (DPU_AoAProc_compRxChannelBiasCfg *)SOC_translateAddress((uint32_t)dpcResults->compRxChanBiasMeasurement,
                SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                &retVal);
            DebugP_assert ((uint32_t)dpcResults->compRxChanBiasMeasurement != SOC_TRANSLATEADDR_INVALID);

            MmwDemo_measurementResultOutput(dpcResults->compRxChanBiasMeasurement);
        }
        else
        {
            /* DPC is not ready to ship the measurement results */
        }
    }

    /* Translate the address: */
    frameStats = (MmwDemo_output_message_stats *)SOC_translateAddress((uint32_t)ptrResult->ptrBuffer[1],
                                             SOC_TranslateAddr_Dir_FROM_OTHER_CPU,
                                             &retVal);
    DebugP_assert ((uint32_t)frameStats != SOC_TRANSLATEADDR_INVALID);

    /* Update current frame stats */
    currSubFrameStats->outputStats.interFrameCPULoad = frameStats->interFrameCPULoad;
    currSubFrameStats->outputStats.activeFrameCPULoad= gMmwMssMCB.subFrameStats[currSubFrameIdx].outputStats.activeFrameCPULoad;
    currSubFrameStats->outputStats.interChirpProcessingMargin = frameStats->interChirpProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingTime = frameStats->interFrameProcessingTime;
    prevSubFrameStats->outputStats.interFrameProcessingMargin = frameStats->interFrameProcessingMargin;
    currSubFrameStats->outputStats.interFrameProcessingMargin = currSubFrameStats->outputStats.interFrameProcessingMargin -
                                                         (currSubFrameStats->pendingConfigProcTime + currSubFrameStats->subFramePreparationTime);

    if (gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.dataFmt !=
             MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
    {
        /* Pend for completion of h/w session, generally this will not wait
         * because of time spent doing inter-frame processing is expected to
         * be bigger than the transmission of the h/w session */
        Semaphore_pend(gMmwMssMCB.lvdsStream.hwFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }

    /* Transfer data on LVDS if s/w session is enabled for the current sub-frame */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        MmwDemo_transferLVDSUserData(currSubFrameIdx, dpcResults);
    }

    /* Transmit processing results for the frame */
    /*MmwDemo_transmitProcessedOutput(gMmwMssMCB.loggingUartHandle,  
                                    dpcResults,
                                    &currSubFrameStats->outputStats);*/
#ifndef TRACKERPROC_ON_DSP

    /******************************************************************
       Processing results are from DSP so address translation is
       needed for buffer pointers
    *******************************************************************/

    objOutSph = (DPIF_PointCloudSpherical *) SOC_translateAddress((uint32_t)dpcResults->objOutSph,
            SOC_TranslateAddr_Dir_FROM_OTHER_CPU, &errCode);
    DebugP_assert ((uint32_t)objOutSph != SOC_TRANSLATEADDR_INVALID);


    objOutSideInfo = (DPIF_PointCloudSideInfo *) SOC_translateAddress((uint32_t)dpcResults->objOutSideInfo,
            SOC_TranslateAddr_Dir_FROM_OTHER_CPU, &errCode);
    DebugP_assert ((uint32_t)objOutSideInfo != SOC_TRANSLATEADDR_INVALID);

    retVal = DPU_TrackerProc_process(gObjDetObj->dpuTrackerObj, dpcResults->numObjOut,
                objOutSph, objOutSideInfo, &outTrackerProc);
    DebugP_assert(retVal == 0);

    dpcResults->trackerCurrDesc  = outTrackerProc.currentTargetDesc; 
    dpcResults->numTargets       = outTrackerProc.numTargets[outTrackerProc.currentTargetDesc]; 
    dpcResults->tList            = ((trackerProcObjType*)gObjDetObj->dpuTrackerObj)->targetDescrHandle->tList[dpcResults->trackerCurrDesc];   
    dpcResults->numIndices       = outTrackerProc.numIndices[outTrackerProc.currentTargetDesc]; 
    dpcResults->tIndex           = ((trackerProcObjType*)gObjDetObj->dpuTrackerObj)->targetDescrHandle->tIndex[dpcResults->trackerCurrDesc];   
#endif
    TrackerDemo_transmitProcessedOutput(gMmwMssMCB.loggingUartHandle, dpcResults);
    
    /* Wait until s/w session is complete. We expect the LVDS transmission of
     * s/w session to be completed by now because the UART transmission above is slower.
     * Doing the wait immediately after starting the transmission above MmwDemo_transferLVDSUserData
     * will serialize the LVDS and UART transfers so it is better to do after UART
     * transmission (which is blocking call i.e UART transmission is completed when we exit
     * out of above MmwDemo_transmitProcessedOutput). Note we cannot replace below code
     * with check for previous sub-frame s/w session completion before the
     * MmwDemo_transferLVDSUserData above because we need to ensure that
     * current sub-frame/frame's contents are not being read during the
     * next sub-frame/frame transmission, presently the data that is being
     * transmitted is not double buffered to allow this */
    if(gMmwMssMCB.subFrameCfg[currSubFrameIdx].lvdsStreamCfg.isSwEnabled == 1)
    {
        /* Pend completion of s/w session, no wait is expected here */
        Semaphore_pend(gMmwMssMCB.lvdsStream.swFrameDoneSemHandle, BIOS_WAIT_FOREVER);
    }

    /* Update current frame transmit time */
    currSubFrameStats->outputStats.transmitOutputTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ; /* In micro seconds */


    /*****************************************************************
     * Handle dynamic pending configuration
     * For non-advanced frame case:
     *   process all pending dynamic config commands.
     * For advanced-frame case:
     *  Process next sub-frame related pending dynamic config commands.
     *  If the next sub-frame was the first sub-frame of the frame,
     *  then process common (sub-frame independent) pending dynamic config
     *  commands.
     *****************************************************************/
    startTime = Cycleprofiler_getTimeStamp();

    nextSubFrameIdx = MmwDemo_getNextSubFrameIndx(currSubFrameIdx,   numSubFrames);
    retVal = MmwDemo_processPendingDynamicCfgCommands(nextSubFrameIdx);
    if (retVal != 0)
    {
        System_printf ("Error: Executing Pending Dynamic Configuration Commands [Error code %d]\n",
                       retVal);
        MmwDemo_debugAssert (0);
    }
    currSubFrameStats->pendingConfigProcTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;

    /*****************************************************************
     * Prepare for subFrame switch
     *****************************************************************/
    if(numSubFrames > 1)
    {
        MmwDemo_SubFrameCfg  *nextSubFrameCfg;
        uint16_t dummyRxChanOffset[SYS_COMMON_NUM_RX_CHANNEL];

        startTime = Cycleprofiler_getTimeStamp();

        nextSubFrameCfg = &gMmwMssMCB.subFrameCfg[nextSubFrameIdx];

        /* Configure ADC for next sub-frame */
        retVal = MmwDemo_ADCBufConfig(gMmwMssMCB.adcBufHandle,
                                 gMmwMssMCB.cfg.openCfg.chCfg.rxChannelEn,
                                 nextSubFrameCfg->numChirpsPerChirpEvent,
                                 nextSubFrameCfg->adcBufChanDataSize,
                                 &nextSubFrameCfg->adcBufCfg,
                                 &dummyRxChanOffset[0]);
        if(retVal < 0)
        {
            System_printf("Error: ADCBuf config failed with error[%d]\n", retVal);
            MmwDemo_debugAssert (0);
        }

        /* Configure HW LVDS stream for this subframe? */
        if(nextSubFrameCfg->lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED)
        {
            MmwDemo_configLVDSHwData(nextSubFrameIdx);
        }

        currSubFrameStats->subFramePreparationTime = (Cycleprofiler_getTimeStamp() - startTime)/R4F_CLOCK_MHZ;
    }
    else
    {
        currSubFrameStats->subFramePreparationTime = 0;
    }

    /*****************************************************************
     * Send notification to data path after results are handled
     *****************************************************************/
    /* Indicate result consumed and end of frame/sub-frame processing */
    exportInfo.subFrameIdx = currSubFrameIdx;
    retVal = DPM_ioctl (gMmwMssMCB.objDetDpmHandle,
                         DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED,
                         &exportInfo,
                         sizeof (DPC_ObjectDetection_ExecuteResultExportedInfo));
    if (retVal < 0) {
        System_printf ("Error: DPM DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED failed [Error code %d]\n",
                       retVal);
        MmwDemo_debugAssert (0);
    }
}

/**
 *  @b Description
 *  @n
 *      DPM Execution Task which executes the DPM Instance which manages the
 *      HL Profiles executing on the MSS.
 *
 *  @retval
 *      Not Applicable.
 */
static void mmwDemo_mssDPMTask(UArg arg0, UArg arg1)
{
    int32_t     errCode;
    DPM_Buffer  result;

    while (1)
    {
        /* Execute the DPM module: */
        errCode = DPM_execute (gMmwMssMCB.objDetDpmHandle, &result);
        if (errCode < 0)
        {
            System_printf ("Error: DPM execution failed [Error code %d]\n", errCode);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Function to Setup the HSI Clock. Required for LVDS streaming.
 *
 *  @retval
 *      0  - Success.
 *      <0 - Failed with errors
 */
int32_t MmwDemo_mssSetHsiClk(void)
{
    rlDevHsiClk_t                           hsiClkgs;
    int32_t                                 retVal;

    /*************************************************************************************
     * Setup the HSI Clock through the mmWave Link:
     *************************************************************************************/
    memset ((void*)&hsiClkgs, 0, sizeof(rlDevHsiClk_t));

    /* Setup the HSI Clock as per the Radar Interface Document:
     * - This is set to 600Mhz DDR Mode */
    hsiClkgs.hsiClk = 0x9;

    /* Setup the HSI in the radar link: */
    retVal = rlDeviceSetHsiClk(RL_DEVICE_MAP_CASCADED_1, &hsiClkgs);
    if (retVal != RL_RET_CODE_OK)
    {
        /* Error: Unable to set the HSI clock */
        System_printf ("Error: Setting up the HSI Clock Failed [Error %d]\n", retVal);
        return -1;
    }

    /*The delay below is needed only if the DCA1000EVM is being used to capture the data traces.
      This is needed because the DCA1000EVM FPGA needs the delay to lock to the
      bit clock before they can start capturing the data correctly. */
    Task_sleep(HSI_DCA_MIN_DELAY_MSEC);

    return 0;
}

/**************************************************************************
 ******************** Millimeter Wave Demo sensor management Functions **********
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to do one time sensor initialization. 
 *      User need to fill gMmwMssMCB.cfg.openCfg before calling this function
 *
 *  @param[in]  isFirstTimeOpen     If true then issues MMwave_open
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_openSensor(bool isFirstTimeOpen)
{
    int32_t             errCode;
    MMWave_ErrorLevel   errorLevel;
    int16_t             mmWaveErrorCode;
    int16_t             subsysErrorCode;
    int32_t             retVal;

    /*  Open mmWave module, this is only done once */
    if (isFirstTimeOpen == true)
    {
        
        System_printf ("Debug: Sending rlRfSetLdoBypassConfig with %d %d %d\n",
                                            gRFLdoBypassCfg.ldoBypassEnable,
                                            gRFLdoBypassCfg.supplyMonIrDrop,
                                            gRFLdoBypassCfg.ioSupplyIndicator);
        retVal = rlRfSetLdoBypassConfig(RL_DEVICE_MAP_INTERNAL_BSS, (rlRfLdoBypassCfg_t*)&gRFLdoBypassCfg);        
        if(retVal != 0)
        {            
            System_printf("Error: rlRfSetLdoBypassConfig retVal=%d\n", retVal);
            return -1;
        }

        /*  Open mmWave module, this is only done once */
        /* Setup the calibration frequency:
         * TODO: Presently DFP does not support these for 68xx platform,
         * need to change when DFP is updated with the support */

        gMmwMssMCB.cfg.openCfg.freqLimitLow = CALIB_FREQ_LIMIT_LOW;
        gMmwMssMCB.cfg.openCfg.freqLimitHigh = CALIB_FREQ_LIMIT_HIGH;


        /* start/stop async events */
        gMmwMssMCB.cfg.openCfg.disableFrameStartAsyncEvent = false;
        gMmwMssMCB.cfg.openCfg.disableFrameStopAsyncEvent  = false;

        /* No custom calibration: */
        gMmwMssMCB.cfg.openCfg.useCustomCalibration        = false;
        gMmwMssMCB.cfg.openCfg.customCalibrationEnableMask = 0x0;
        gMmwMssMCB.cfg.openCfg.calibMonTimeUnit = 1;

        /* Open the mmWave module: */
        if (MMWave_open (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.openCfg, NULL, &errCode) < 0)
        {
            /* Error: decode and Report the error */
            MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
            System_printf ("Error: mmWave Open failed [Error code: %d Subsystem: %d]\n",
                            mmWaveErrorCode, subsysErrorCode);
            return -1;
        }

        /*Set up HSI clock*/
        if(MmwDemo_mssSetHsiClk() < 0)
        {
            System_printf ("Error: MmwDemo_mssSetHsiClk failed.\n");
            return -1;
        }

        /* Open the datapath modules that runs on MSS */
        MmwDemo_dataPathOpen();
    }
    return 0;
}

/**
 *  @b Description
 *  @n
 *      MMW demo helper Function to configure sensor. User need to fill gMmwMssMCB.cfg.ctrlCfg and
 *      add profiles/chirp to mmWave before calling this function
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_configSensor(void)
{
    int32_t     errCode = 0;

    /* Prepare BPM configuration */
    if(MmwDemo_bpmConfig() < 0)
    {
        System_printf ("Error: MMWDemoMSS mmWave BPM Configuration failed\n");
        return -1;
    }

    /* Configure the mmWave module: */
    if (MMWave_config (gMmwMssMCB.ctrlHandle, &gMmwMssMCB.cfg.ctrlCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error: Report the error */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Config failed [Error code: %d Subsystem: %d]\n",
                        mmWaveErrorCode, subsysErrorCode);
    }
    else
    {
        errCode = MmwDemo_dataPathConfig();
        if(errCode < 0)
            goto exit;

        errCode = MmwDemo_trackerConfig();
        if(errCode < 0)
            goto exit;
    }

exit:
    return errCode;
}

/**
 *  @b Description
 *  @n
 *      mmw demo helper Function to start sensor.
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t MmwDemo_startSensor(void)
{
    int32_t     errCode;
    MMWave_CalibrationCfg   calibrationCfg;

    /*****************************************************************************
     * Data path :: start data path first - this will pend for DPC to ack
     *****************************************************************************/
    MmwDemo_dataPathStart();

    /*****************************************************************************
     * RF :: now start the RF and the real time ticking
     *****************************************************************************/
    /* Initialize the calibration configuration: */
    memset ((void *)&calibrationCfg, 0, sizeof(MMWave_CalibrationCfg));
    /* Populate the calibration configuration: */
    calibrationCfg.dfeDataOutputMode = gMmwMssMCB.cfg.ctrlCfg.dfeDataOutputMode;
    calibrationCfg.u.chirpCalibrationCfg.enableCalibration    = true;
    calibrationCfg.u.chirpCalibrationCfg.enablePeriodicity    = true;
    calibrationCfg.u.chirpCalibrationCfg.periodicTimeInFrames = 10U;

    DebugP_log0("App: MMWave_start Issued\n");

    System_printf("Starting Sensor (issuing MMWave_start)\n");

    /* Start the mmWave module: The configuration has been applied successfully. */
    if (MMWave_start(gMmwMssMCB.ctrlHandle, &calibrationCfg, &errCode) < 0)
    {
        MMWave_ErrorLevel   errorLevel;
        int16_t             mmWaveErrorCode;
        int16_t             subsysErrorCode;

        /* Error/Warning: Unable to stop the mmWave module */
        MMWave_decodeError (errCode, &errorLevel, &mmWaveErrorCode, &subsysErrorCode);
        System_printf ("Error: mmWave Start failed [mmWave Error: %d Subsys: %d]\n", mmWaveErrorCode, subsysErrorCode);
		/* datapath has already been moved to start state; so either we initiate a cleanup of start sequence or
           assert here and re-start from the beginning. For now, choosing the latter path */ 
        MmwDemo_debugAssert(0);																							   
        return -1;
    }

    /*****************************************************************************
     * The sensor has been started successfully. Switch on the LED 
     *****************************************************************************/
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 1U);

    gMmwMssMCB.sensorStartCount++;
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Epilog processing after sensor has stopped
 *
 *  @retval None
 */
static void MmwDemo_sensorStopEpilog(void)
{
    Task_Stat stat;
    Hwi_StackInfo stackInfo;
    Bool stackOverflow;

    /* Print task statistics, note data path has completely stopped due to
     * end of frame, so we can do non-real time processing like prints on
     * console */
    System_printf("Data Path Stopped (last frame processing done)\n");

    System_printf("============================================\n");
    System_printf("MSS Task Stack Usage (Note: Task Stack Usage) ==========\n");
    System_printf("%20s %12s %12s %12s\n", "Task Name", "Size", "Used", "Free");


    Task_stat(gMmwMssMCB.taskHandles.initTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "Init",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.mmwaveCtrl, &stat);
    System_printf("%20s %12d %12d %12d\n", "Mmwave Control",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    Task_stat(gMmwMssMCB.taskHandles.objDetDpmTask, &stat);
    System_printf("%20s %12d %12d %12d\n", "ObjDet DPM",
                  stat.stackSize, stat.used, stat.stackSize - stat.used);

    System_printf("HWI Stack (same as System Stack) Usage ============\n");
    stackOverflow = Hwi_getStackInfo(&stackInfo, TRUE);
    if (stackOverflow == TRUE)
    {
        System_printf("HWI Stack overflowed\n");
        MmwDemo_debugAssert(0);
    }
    else
    {
        System_printf("%20s %12s %12s %12s\n", " ", "Size", "Used", "Free");
        System_printf("%20s %12d %12d %12d\n", " ",
                      stackInfo.hwiStackSize, stackInfo.hwiStackPeak,
                      stackInfo.hwiStackSize - stackInfo.hwiStackPeak);
    }
}


/**
 *  @b Description
 *  @n
 *      Stops the RF and datapath for the sensor. Blocks until both operation are completed.
 *      Prints epilog at the end.
 *
 *  @retval  None
 */
void MmwDemo_stopSensor(void)
{
    int32_t errCode;

    /* Stop sensor RF , data path will be stopped after RF stop is completed */
    MmwDemo_mmWaveCtrlStop();

    /* Wait until DPM_stop is completed */
    Semaphore_pend(gMmwMssMCB.DPMstopSemHandle, BIOS_WAIT_FOREVER);

    /* Delete any active streaming session */
    if(gMmwMssMCB.lvdsStream.hwSessionHandle != NULL)
    {
        /* Evaluate need to deactivate h/w session:
         * One sub-frame case:
         *   if h/w only enabled, deactivation never happened, hence need to deactivate
         *   if h/w and s/w both enabled, then s/w would leave h/w activated when it is done
         *   so need to deactivate
         *   (only s/w enabled cannot be the case here because we are checking for non-null h/w session)
         * Multi sub-frame case:
         *   Given stop, we must have re-configured the next sub-frame by now which is next of the
         *   last sub-frame i.e we must have re-configured sub-frame 0. So if sub-frame 0 had
         *   h/w enabled, then it is left in active state and need to deactivate. For all
         *   other cases, h/w was already deactivated when done.
         */
        if ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames == 1) ||
            ((gMmwMssMCB.objDetCommonCfg.preStartCommonCfg.numSubFrames > 1) &&
             (gMmwMssMCB.subFrameCfg[0].lvdsStreamCfg.dataFmt != MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED))
           )
        {
            if (CBUFF_deactivateSession(gMmwMssMCB.lvdsStream.hwSessionHandle, &errCode) < 0)
            {
                System_printf("CBUFF_deactivateSession failed with errorCode = %d\n", errCode);
                MmwDemo_debugAssert(0);
            }
        }
        MmwDemo_LVDSStreamDeleteHwSession();
    }

    /* Delete s/w session if it exists. S/w session never needs to be deactivated in stop because
     * it always (unconditionally) deactivates itself upon completion.
     */
    if(gMmwMssMCB.lvdsStream.swSessionHandle != NULL)
    {
        MmwDemo_LVDSStreamDeleteSwSession();
    }

    /* Print epilog */
    MmwDemo_sensorStopEpilog();

    /* The sensor has been stopped successfully. Switch off the LED */
    GPIO_write (gMmwMssMCB.cfg.platformCfg.SensorStatusGPIO, 0U);

    gMmwMssMCB.sensorStopCount++;

    /* print for user */
    System_printf("Sensor has been stopped: startCount: %d stopCount %d\n",
                  gMmwMssMCB.sensorStartCount,gMmwMssMCB.sensorStopCount);
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of frame/sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during inter-frame
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwMssMCB.subFrameStats[subFrameIndx].outputStats.interFrameCPULoad = Load_getCPULoad();
}

/**
 *  @b Description
 *  @n
 *      Call back function that was registered during config time and is going
 *      to be called in DPC processing at the beginning of inter-frame/inter-sub-frame processing,
 *      we use this to issue BIOS calls for computing CPU load during active frame (chirping)
 *
 *  @param[in] subFrameIndx     Sub-frame index of the sub-frame during which processing
 *                              this function was called.
 *
 *  @retval None
 */
static void MmwDemo_DPC_ObjectDetection_processInterFrameCallBackFxn(uint8_t subFrameIndx)
{
    Load_update();
    gMmwMssMCB.subFrameStats[subFrameIndx].outputStats.activeFrameCPULoad = Load_getCPULoad();
}

/**************************************************************************
 ******************** Millimeter Wave Demo init Functions ************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      Platform specific hardware initialization.
 *
 *  @param[in]  config     Platform initialization configuraiton
 *
 *  @retval
 *      Not Applicable.
 */
static void MmwDemo_platformInit(MmwDemo_platformCfg *config)
{

    /* Setup the PINMUX to bring out the UART-1 */
    Pinmux_Set_OverrideCtrl(SOC_XWRXXXX_PINN5_PADBE, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWRXXXX_PINN5_PADBE, SOC_XWRXXXX_PINN5_PADBE_MSS_UARTA_TX);
    Pinmux_Set_OverrideCtrl(SOC_XWRXXXX_PINN4_PADBD, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWRXXXX_PINN4_PADBD, SOC_XWRXXXX_PINN4_PADBD_MSS_UARTA_RX);

    /* Setup the PINMUX to bring out the UART-3 */
    Pinmux_Set_OverrideCtrl(SOC_XWRXXXX_PINF14_PADAJ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWRXXXX_PINF14_PADAJ, SOC_XWRXXXX_PINF14_PADAJ_MSS_UARTB_TX);

    /**********************************************************************
     * Setup the PINMUX:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    Pinmux_Set_OverrideCtrl(SOC_XWRXXXX_PINK13_PADAZ, PINMUX_OUTEN_RETAIN_HW_CTRL, PINMUX_INPEN_RETAIN_HW_CTRL);
    Pinmux_Set_FuncSel(SOC_XWRXXXX_PINK13_PADAZ, SOC_XWRXXXX_PINK13_PADAZ_GPIO_2);

    /**********************************************************************
     * Setup the GPIO:
     * - GPIO Output: Configure pin K13 as GPIO_2 output
     **********************************************************************/
    config->SensorStatusGPIO    = SOC_XWRXXXX_GPIO_2;

    /* Initialize the DEMO configuration: */
    config->sysClockFrequency   = MSS_SYS_VCLK;
    config->loggingBaudRate     = 921600;
    config->commandBaudRate     = 115200;

    /**********************************************************************
     * Setup the DS3 LED on the EVM connected to GPIO_2
     **********************************************************************/
    GPIO_setConfig (config->SensorStatusGPIO, GPIO_CFG_OUTPUT);
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
static void MmwDemo_initTask(UArg arg0, UArg arg1)
{
    int32_t             errCode;
    MMWave_InitCfg      initCfg;
    UART_Params         uartParams;
    Task_Params         taskParams;
    Semaphore_Params    semParams;
    DPM_InitCfg         dpmInitCfg;
    DPC_ObjectDetectionRangeHWA_InitParams objDetInitParams;
    int32_t             i;

    /* Debug Message: */
    System_printf("Debug: Launched the Initialization Task\n");

    /*****************************************************************************
     * Initialize the mmWave SDK components:
     *****************************************************************************/
    /* Initialize the UART */
    UART_init();

    /* Initialize the GPIO */
    GPIO_init();

    /* Initialize the Mailbox */
    Mailbox_init(MAILBOX_TYPE_MSS);

    /* Initialize LVDS streaming components */
    if ((errCode = MmwDemo_LVDSStreamInit()) < 0 )
    {
        System_printf ("Error: MMWDemoDSS LVDS stream init failed with Error[%d]\n",errCode);
        return;
    }

    /* initialize cq configs to invalid profile index to be able to detect
     * unconfigured state of these when monitors for them are enabled.
     */
    for(i = 0; i < RL_MAX_PROFILES_CNT; i++)
    {
        gMmwMssMCB.cqSatMonCfg[i].profileIndx    = (RL_MAX_PROFILES_CNT + 1);
        gMmwMssMCB.cqSigImgMonCfg[i].profileIndx = (RL_MAX_PROFILES_CNT + 1);
    }

    /* Platform specific configuration */
    MmwDemo_platformInit(&gMmwMssMCB.cfg.platformCfg);

    /*****************************************************************************
     * Open the mmWave SDK components:
     *****************************************************************************/
    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.commandBaudRate;
    uartParams.isPinMuxDone   = 1;

    /* Open the UART Instance */
    gMmwMssMCB.commandUartHandle = UART_open(0, &uartParams);
    if (gMmwMssMCB.commandUartHandle == NULL)
    {
        MmwDemo_debugAssert (0);
        return;
    }

    /* Setup the default UART Parameters */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.clockFrequency = gMmwMssMCB.cfg.platformCfg.sysClockFrequency;
    uartParams.baudRate       = gMmwMssMCB.cfg.platformCfg.loggingBaudRate;
    uartParams.isPinMuxDone   = 1U;

    /* Open the Logging UART Instance: */
    gMmwMssMCB.loggingUartHandle = UART_open(1, &uartParams);
    if (gMmwMssMCB.loggingUartHandle == NULL)
    {
        System_printf("Error: Unable to open the Logging UART Instance\n");
        MmwDemo_debugAssert (0);
        return;
    }

    /* Create binary semaphores which is used to signal DPM_start/DPM_stop/DPM_ioctl is done
     * to the sensor management task. The signalling (Semaphore_post) will be done
     * from DPM registered report function (which will execute in the DPM execute task context). */
    Semaphore_Params_init(&semParams);
    semParams.mode              = Semaphore_Mode_BINARY;
    gMmwMssMCB.DPMstartSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMstopSemHandle   = Semaphore_create(0, &semParams, NULL);
    gMmwMssMCB.DPMioctlSemHandle   = Semaphore_create(0, &semParams, NULL);

    /* Open EDMA driver */
    MmwDemo_edmaInit(&gMmwMssMCB.dataPathObj, DPC_OBJDET_R4F_EDMA_INSTANCE);

    /* Use EDMA instance 0 on MSS */
    MmwDemo_edmaOpen(&gMmwMssMCB.dataPathObj, DPC_OBJDET_R4F_EDMA_INSTANCE);

    MmwDemo_hwaInit(&gMmwMssMCB.dataPathObj);
    MmwDemo_hwaOpen(&gMmwMssMCB.dataPathObj, gMmwMssMCB.socHandle);

    /*****************************************************************************
     * mmWave: Initialization of the high level module
     *****************************************************************************/

    /* Initialize the mmWave control init configuration */
    memset ((void*)&initCfg, 0 , sizeof(MMWave_InitCfg));

    /* Populate the init configuration: */
    initCfg.domain                  = MMWave_Domain_MSS;
    initCfg.socHandle               = gMmwMssMCB.socHandle;
    initCfg.eventFxn                = MmwDemo_eventCallbackFxn;
    initCfg.linkCRCCfg.useCRCDriver = 1U;
    initCfg.linkCRCCfg.crcChannel   = CRC_Channel_CH1;
    initCfg.cfgMode                 = MMWave_ConfigurationMode_FULL;
    initCfg.executionMode           = MMWave_ExecutionMode_ISOLATION;

    /* Initialize and setup the mmWave Control module */
    gMmwMssMCB.ctrlHandle = MMWave_init (&initCfg, &errCode);
    if (gMmwMssMCB.ctrlHandle == NULL)
    {
        /* Error: Unable to initialize the mmWave control module */
        System_printf ("Error: mmWave Control Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Initialization was successful\n");

    /* Synchronization: This will synchronize the execution of the control module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    if (MMWave_sync (gMmwMssMCB.ctrlHandle, &errCode) < 0)
    {
        /* Error: Unable to synchronize the mmWave control module */
        System_printf ("Error: mmWave Control Synchronization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }
    System_printf ("Debug: mmWave Control Synchronization was successful\n");

    /*****************************************************************************
     * Launch the mmWave control execution task
     * - This should have a higher priroity than any other task which uses the
     *   mmWave control API
     *****************************************************************************/
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_MMWAVE_CTRL_TASK_PRIORITY;
    taskParams.stackSize = 3*1024;
    gMmwMssMCB.taskHandles.mmwaveCtrl = Task_create(MmwDemo_mmWaveCtrlTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialization of the DPM Module:
     *****************************************************************************/
    memset ((void *)&objDetInitParams, 0, sizeof(DPC_ObjectDetectionRangeHWA_InitParams));

    /* Note this must be after MmwDemo_dataPathOpen() above which opens the hwa */
    objDetInitParams.L3ramCfg.addr = (void *)&gMmwL3[0];
    objDetInitParams.L3ramCfg.size = sizeof(gMmwL3);
    objDetInitParams.CoreLocalRamCfg.addr = &gDPCTCM[0];
    objDetInitParams.CoreLocalRamCfg.size = sizeof(gDPCTCM);
    objDetInitParams.edmaHandle = gMmwMssMCB.dataPathObj.edmaHandle;
    objDetInitParams.hwaHandle = gMmwMssMCB.dataPathObj.hwaHandle;

    /* DPC Call-back config */
    objDetInitParams.processCallBackFxn.processInterFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processInterFrameCallBackFxn;

    objDetInitParams.processCallBackFxn.processFrameBeginCallBackFxn =
        MmwDemo_DPC_ObjectDetection_processFrameBeginCallBackFxn;

    /* Setup the configuration: */
    memset ((void *)&dpmInitCfg, 0, sizeof(DPM_InitCfg));
    dpmInitCfg.socHandle        = gMmwMssMCB.socHandle;
    dpmInitCfg.ptrProcChainCfg  = &gDPC_ObjDetRangeHWACfg;;
    dpmInitCfg.instanceId       = DPC_OBJDET_R4F_INSTANCEID;
    dpmInitCfg.domain           = DPM_Domain_DISTRIBUTED;
    dpmInitCfg.reportFxn        = MmwDemo_DPC_ObjectDetection_reportFxn;
    dpmInitCfg.arg              = &objDetInitParams;
    dpmInitCfg.argSize          = sizeof(DPC_ObjectDetectionRangeHWA_InitParams);

    /* Initialize the DPM Module: */
    gMmwMssMCB.objDetDpmHandle = DPM_init (&dpmInitCfg, &errCode);
    if (gMmwMssMCB.objDetDpmHandle == NULL)
    {
        System_printf ("Error: Unable to initialize the DPM Module [Error: %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return;
    }

    /* Synchronization: This will synchronize the execution of the datapath module
     * between the domains. This is a prerequiste and always needs to be invoked. */
    while (1)
    {
        int32_t syncStatus;

        /* Get the synchronization status: */
        syncStatus = DPM_synch (gMmwMssMCB.objDetDpmHandle, &errCode);
        if (syncStatus < 0)
        {
            /* Error: Unable to synchronize the framework */
            System_printf ("Error: DPM Synchronization failed [Error code %d]\n", errCode);
            MmwDemo_debugAssert (0);
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

    /* Launch the DPM Task */
    Task_Params_init(&taskParams);
    taskParams.priority  = MMWDEMO_DPC_OBJDET_DPM_TASK_PRIORITY;
    taskParams.stackSize = 8*1024;
    gMmwMssMCB.taskHandles.objDetDpmTask = Task_create(mmwDemo_mssDPMTask, &taskParams, NULL);

    /*****************************************************************************
     * Initialize the Profiler
     *****************************************************************************/
    Cycleprofiler_init();

    /*****************************************************************************
     * Initialize the CLI Module:
     *****************************************************************************/
    MmwDemo_CLIInit(MMWDEMO_CLI_TASK_PRIORITY);

    return;
}

/**
 *  @b Description
 *  @n
 *     Function to sleep the R4F using WFI (Wait For Interrupt) instruction.
 *     When R4F has no work left to do,
 *     the BIOS will be in Idle thread and will call this function. The R4F will
 *     wake-up on any interrupt (e.g chirp interrupt).
 *
 *  @retval
 *      Not Applicable.
 */
void MmwDemo_sleep(void)
{
    /* issue WFI (Wait For Interrupt) instruction */
    asm(" WFI ");
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
    SOC_Handle      socHandle;
    SOC_Cfg         socCfg;

    /* Initialize the ESM: Dont clear errors as TI RTOS does it */
    ESM_init(0U);

    /* Initialize the SOC confiugration: */
    memset ((void *)&socCfg, 0, sizeof(SOC_Cfg));

    /* Populate the SOC configuration: */
    socCfg.clockCfg = SOC_SysClock_INIT;
    socCfg.mpuCfg = SOC_MPUCfg_CONFIG;
    socCfg.dssCfg = SOC_DSSCfg_UNHALT;

    /* Initialize the SOC Module: This is done as soon as the application is started
     * to ensure that the MPU is correctly configured. */
    socHandle = SOC_init (&socCfg, &errCode);
    if (socHandle == NULL)
    {
        System_printf ("Error: SOC Module Initialization failed [Error code %d]\n", errCode);
        MmwDemo_debugAssert (0);
        return -1;
    }

    /* Initialize and populate the demo MCB */
    memset ((void*)&gMmwMssMCB, 0, sizeof(MmwDemo_MSS_MCB));

    gMmwMssMCB.socHandle = socHandle;

    /* Debug Message: */
    System_printf ("**********************************************\n");
    System_printf ("Debug: Launching the MMW Demo on MSS\n");
    System_printf ("**********************************************\n");

    /* Initialize the Task Parameters. */
    Task_Params_init(&taskParams);
    gMmwMssMCB.taskHandles.initTask = Task_create(MmwDemo_initTask, &taskParams, NULL);

    /* Start BIOS */
    BIOS_start();
    return 0;
}

