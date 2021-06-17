/**
 *   @file  aoaproc_common.h
 *
 *   @brief
 *      Implements Common definition across cfarcaProc DPU.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018-2019 Texas Instruments, Inc.
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

 /** @mainpage AOA DPU
 *
 *  @section intro_section Introduction
 *
 *  Description
 *  ----------------
 *
 * This DPU implements angle of arrival (AoA) estimation. 
 * It generates information about detected objects into two lists: 1) list with coordinates
 * defined by @ref DPIF_PointCloudCartesian_t and 2) side information list defined by @ref DPIF_PointCloudSideInfo_t.
 * The DPU receives 1D-FFT Radar Cube matrix calculated by range processing DPU
 * and the list of detected objects (@ref DPIF_CFARDetList_t) which is assumed to
 * be populated by previous CFAR algorithm processing.\n\n
 *
 * The AoA DPU is available in two distinct implementations:
 * 
 *  DPU Implementation   |  Runs on cores  | Uses HWA   
 * :---------------------|:----------------|:---------
 * AoAProcHWA            |  R4F or DSP     |  Yes
 * AoAProcDSP            |     DSP         |  No
 *\n
 *
 * This document is organized as follows:\n
 *
 *  Starting Section          |  Description  
 * :--------------------------|:----------------
 * @ref HWA-AOA-start         |  Start of the sections describing the AoAProcHWA DPU. It covers DPU implementation details.  
 * @ref DSP-AOA-start         |  Start of the sections describing the AoAProcDSP DPU. It covers DPU implementation details.  
 * @ref HWA-DSP-Common-start  |  Start of the sections that are common for AoAProcDSP and AoAProcHWA. It covers Doppler compensation, XYZ estimation/geometry and FoV configuration.    
 * \n\n 
 *
 * @anchor HWA-AOA-start
 * @section section_HWA-AOA HWA-AOA DPU (AoAProcHWA)
 *
 * @subsection section_AOA_Description Description
 * This DPU implements angle of arrival (AoA) estimation using HWA and CPU.
 * A high level data flow is illustrated in figure below.
 *
 * @image html hwa_aoa_top_level.png "HWA-AOA high level data flow"
 *
 * The following figure depicts detailed DPU implementation. DPU processes data
 * from the input CFAR detection list in a loop, two objects from the input list per loop.
 * The processing is done in ping/pong manner so that the HWA processing, the EDMA transfers and
 * the local core processing is done in parallel. Data processing is split in two paths,
 * ping and pong path, one detected object per path.
 * In each path processing is split between HWA and local core and it is divided in four stages:
 * 2D-FFT calculation by HWA, Doppler compensation by local core, 3D-FFT calculation by HWA and
 * XYZ estimation by local core. These four stages include:
 * 1. Calculation of the array of 2D-FFT Rx virtual antenna symbols as input to Azimuth FFT computation.
 *    1. Input EDMA pages from radarCube memory to HWA memory the 1D-FFT data corresponding to the range index
 *       of the detected object. Note the input size limiting condition:
 *       \f$N_{TxAnt}*N_{RxAnt}*N_{DopplerChirps}*sizeof(cmplx16ImRe\_t) \leq 16\;KB\f$
 *    2. HWA calculates 2D FFT. Note the output size limiting condition:
 *       \f$N_{TxAnt}*N_{RxAnt}*N_{DopplerBins}*sizeof(cmplx16ImRe\_t) \leq 32\;KB\f$
 *    3. Output EDMA picks the data of the Doppler bin corresponding to the detected object's
 *       Doppler index from the above FFT output and copies into core local memory.
 * 2. CPU performs
 *    1. Rx channel phase/gain compensation
 *    2. Doppler compensation
 *    3. Triggers EDMA to copy data back to HWA memory
 * 3. Azimuth FFT calculation:
 *    1. EDMA copies 2D-FFT antenna symbols from local memory to HWA memory,
 *    2. HWA calculates azimuth FFT (complex output) on azimuth Tx antennas,
 *    3. HWA calculates azimuth FFT (complex output) on elevation Tx antennas,
 *    4. HWA calculates Magnitude square of the FFT output of azimuth Tx antennas,
 *    5. EDMA copies magnitude squares from HWA memory to the local core memory
 * 4. Final azimuth and elevation estimation and x/y/z calculation performed by local core.
 *    The calculated data of detected object is appended to the output lists.
 *
 * One HWA loop iteration processes 2 detected objects. (If the number of detected objects is odd, a
 * dummy one is appended at the end of the list of detected points.)
 *
 * @image html hwa_aoa.png "HWA-AOA implementation (for 3 Tx antennas)"
 *
 * The following figure shows the local core processing flow including two objects per loop.
 * The HWA runs in parallel, processing data within its loop in the following order: ping path 2D-FFT, pong path 2D-FFT,
 * ping path 3D-FFTs, pong path 3D-FFTs. This is illustrated in the timing diagram shown in figure below.
 *
 * @image html hwa_aoa_flow_chart.png "Local core processing flow"
 *
 * @image html hwa_aoa_timing_diagram.png "Timing diagram example: processing 5 detected objects"
 *
 * The scheme uses four EDMA physical channels, two EDMA channels to input data from the
 * local core memory to HWA, one channel per path, and two EDMA channels to output data from
 * HWA memory to the local core memory, one EDMA channel per path. The scheme also uses 12 EDMA param
 * sets programmed for data transfer and HWA triggering. HWA programming is done by up to 12 HWA Param sets.
 * The usage of EDMA and HWA param set is described in figure below.
 * Note that EDMA Input to HWA in the first stage is AB-Synchronized type transfer,
 * while all other EDMAs in the scheme are A-Synchronized type transfers. Since the
 * radar cube is aligned to 4 byte boundary, none of these transfers fall under the EDMA
 * 4K boundary problem.
 * @image html hwa_aoa_param_sets.png "EDMA and HWA param set usage"
 *
 * @subsection Azimuth_heat_map Azimuth heatmap preparation
 *    In addition to AoA estimation, this DPU also generates data for computing
 *    azimuth heat-map (e.g for visualization purposes), this data is 2D FFT of the
 *    phase compensated Doppler bin 0 (static objects) output of each of the range bins.
 *    The following figure depicts data generation for azimuth heat-map visualization.
 *    Here, as well none of the trunfers fall under the EDMA 4K boundary problem.
 *
 * @image html hwa_aoa_azimuth_heatmap.png "Data processing for azimuth heat-map visualization"
 *
 *
 * @section api_section AoAProcHWA APIs
 *
 * - @ref DPU_AoAProcHWA_init - DPUs initialization function.
 * - @ref DPU_AoAProcHWA_config - DPUs configuration function. The configuration can only be done after
 *   the DPU has been initialized using @ref DPU_AoAProcHWA_init. If the parameters used by this DPU do not
 *   change from one frame to the next, this function can be called only once, for the first
 *   frame, otherwise it has to be called before each @ref DPU_AoAProcHWA_process.
 * - @ref DPU_AoAProcHWA_process - DPUs processing function performs AoA estimation of
 *   detected objects in the given frame. This processing can only be done after the
 *   DPU has been configured.
 * - @ref DPU_AoAProcHWA_control - DPUs control function. It processes following messages from DPC:
 *   - @ref DPU_AoAProcHWA_Cmd_FovAoACfg,
 *   - @ref DPU_AoAProcHWA_Cmd_MultiObjBeamFormingCfg,
 *   - @ref DPU_AoAProcHWA_Cmd_CompRxChannelBiasCfg,
 *   - @ref DPU_AoAProcHWA_Cmd_PrepareRangeAzimuthHeatMap,
 *   - @ref DPU_AoAProcHWA_Cmd_ExtMaxVelocityCfg
 *
 *  Full API details can be seen at @ref DPU_AOAPROC_EXTERNAL
 *
 * 
 * 
 *
 *\n\n\n 
 * @anchor DSP-AOA-start
 * @section section_DSP-AOA DSP-AOA DPU (AoAProcDSP)
 *
 * @subsection section_AOADSP_Description Description
 * This DPU implements angle of arrival (AoA) estimation using the DSP only(HWA is not used).
 * A high level data flow is illustrated in figure below.
 *
 * @image html dsp_aoa_top_level.png "DSP-AOA high level data flow"
 *
 * This DPU has the following restrictions regarding its input parameters:\n
 * The number of Doppler chirps (numDopplerChirps) must be a multiple of 4 and the number of Doppler bins (numDopplerBins)
 * must be a power of 2 greater or equal than numDopplerChirps. Also, due to restrictions on the FFT implementation by DSPLIB,
 * numDopplerBins must be at least 16.
 *
 *   Parameter         |  Restriction  
 *  :------------------|:--------------
 *  numDopplerChirps   |  Must be a multiple of 4.   
 *  numDopplerBins     |  Must be at least 16. Must be a power of two, greater or equal than numDopplerChirps.      
 * \n
 *
 * Below are details of the DPU implementation.\n\n
 * The AoA DSP DPU processes data from the input CFAR detection list in a loop.
 * For each object in the CFAR detection list the DPU performs the following operations:
 *
 * **Input data**\n
 * The CFAR detection list determines which range bin should be processed for the given object.
 * EDMA is used to move data from the radar cube into the DPU scratch buffers.\n 
 * A ping/pong buffer scheme is used where in each iteration (ping/pong), data pertinent to one virtual antenna 
 * (for the given range bin) is brought from the radar cube matrix for processing. 
 *
 * **Static Clutter Removal**\n
 * When Static Clutter Removal is enabled, the mean value of the input samples to the
 * Doppler FFT is subtracted from the samples.\n\n
 *
 * **Windowing**\n
 * Before the 2D FFT operation, input samples are multiplied by a real symmetric window function. Window size and coefficients are 
 * defined in @ref DPU_AoAProcDSP_HW_Resources. Window coefficients must be provided by application. The same window used for the
 * DSP Doppler DPU should be used here.\n
 * Note that the windowing function also executes IQ swap. Before windowing the samples are in the same format as in the radar cube,
 * which is cmplx16ImRe_t. After windowing the output is in cmplx32ReIm_t format.\n\n
 *
 * **2D FFT**\n
 * 2D FFT is computed, the output of the FFT has numDopplerBins samples of type cmplx32ReIm_t. 
 * The CFAR detection list determines which Doppler bin should be stored, the other bins are discarded.\n\n
 *
 * **Repeat for all virtual antennas**\n
 * The above steps are repeated for all virtual antennas for the given range bin before moving to the next step.\n\n
 *
 * **Doppler Compensation**\n
 * Doppler correction is done on the symbols belonging to TX1 and TX2 antennas.\n\n
 * 
 * **BPM decoding**\n
 * If BPM is enabled, BPM is decoded. \n\n
 *
 * **RX Channel Phase Compensation**\n
 * RX channel phase compensation is performed. \n\n
 *
 * **Angle of Arrival Estimation**\n
 * Azimuth FFT is computed, Elevation FFT is computed. These FFTs are input to @ref AoAProcDSP_angleEstimationAzimElev,
 * which calculates the x/y/z coordinates of the object. \n\n
 * 
 * **Detailed block diagram for 3 TX 4 RX TDM-MIMO (no BPM)**\n
 * 
 * The following figure depicts in detail the DPU implementation for the case of 3 TX and 4RX antennas (no BPM).
 *
 * @image html dsp_aoa_withMaxVelocity.png "DSP-AOA detailed implementation for 3 TX 4 RX TDM-MIMO (no BPM)"
 *
 * @subsection Azimuth_heat_mapDSP Azimuth heatmap preparation
 *    In addition to AoA estimation, this DPU also generates data for computing
 *    azimuth heat-map (e.g for visualization purposes), this data is 2D FFT of the
 *    phase compensated Doppler bin 0 (static objects) output of each of the range bins.
 *
 * Below are details of the Azimuth heatmap preparation.\n\n
 *
 * **Input data**\n
 * EDMA is used to move data from the radar cube into the DPU scratch buffers.\n 
 * A ping/pong buffer scheme is used where in each iteration (ping/pong), data pertinent to one virtual antenna 
 * (for a given range bin) is brought from the radar cube matrix for processing. 
 *
 * **Static Clutter Removal**\n
 * When Static Clutter Removal is enabled, the mean value of the input samples to the
 * Doppler FFT is subtracted from the samples.\n\n
 *
 * **BPM decoding**\n
 * If BPM is enabled, BPM is decoded. \n\n
 *
 * **Doppler bin zero**\n
 * Doppler bin zero is computed. Note that only bin zero is required.\n\n
 *
 * **RX Channel Phase Compensation**\n
 * RX channel phase compensation is performed. \n\n
 *
 * **Repeat for all Azimuth virtual antennas for all range bins**\n
 * The above steps are repeated for all Azimuth virtual antennas for all range bin. Note that data pertinent to the 
 * elevation virtual antennas are not processed.\n\n
 *
 * The following figure depicts the heatmap computation for the case of 3TX and 4RX antennas.
 *
 * @image html dsp_aoa_azimuth.png "DSP-AOA Azimuth heatmap computation"
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
 *    @image html coordinate_geometry.png "6843 EVM antenna arrangement"
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
 *  @subsection bpmNotes AoA DSP DPU changes when BPM is enabled
 *    When BPM is enabled the following changes are done in the AoA DSP DPU processing:\n
 *    For AoA estimation, BPM decoding is done after Doppler compensation. The following
 *    figure depicts the DPU implementation when BPM is enabled. 
 *
 *      @image html dsp_aoa_BPM.png "BPM Scheme Antenna configuration"
 *
 *
 * @section api_sectionDSP AoAProcDSP APIs
 *
 * - @ref DPU_AoAProcDSP_init - DPUs initialization function.
 * - @ref DPU_AoAProcDSP_config - DPUs configuration function. The configuration can only be done after
 *   the DPU has been initialized using @ref DPU_AoAProcDSP_init. 
 * - @ref DPU_AoAProcDSP_process - DPUs processing function performs AoA estimation of
 *   detected objects in the given frame. This processing can only be done after the
 *   DPU has been configured.
 * - @ref DPU_AoAProcDSP_control - DPUs control function. It processes following messages from DPC:
 *   - @ref DPU_AoAProcDSP_Cmd_FovAoACfg,
 *   - @ref DPU_AoAProcDSP_Cmd_MultiObjBeamFormingCfg,
 *   - @ref DPU_AoAProcDSP_Cmd_CompRxChannelBiasCfg,
 *   - @ref DPU_AoAProcDSP_Cmd_PrepareRangeAzimuthHeatMap,
 *   - @ref DPU_AoAProcDSP_Cmd_staticClutterCfg
 *   - @ref DPU_AoAProcDSP_Cmd_ExtMaxVelocityCfg
 *
 *  Full API details can be seen at @ref DPU_AOAPROC_EXTERNAL \n\n\n
 *
 *
 *
 * @anchor HWA-DSP-Common-start
 * @section common_HWA_DSP_DPUs Common Sections for AoAProcDSP and AoAProcHW
 * The following sections are common for AoAProcDSP and AoAProcHWA\n
 *
 * @subsection Sect_dopplerComp Doppler Compensation
 *    Doppler compensation function  (@ref aoaHwa_dopplerCompensation or @ref AoAProcDSP_dopplerCompensation) is done by CPU after
 *    Rx channel phase/gain compensation. It performs compensation for the Doppler phase shift on
 *    the symbols corresponding to the virtual Rx antennas. In case of 2Tx MIMO scheme, the second
 *    set of Rx symbols is rotated by half of the estimated Doppler phase shift between subsequent
 *    chirps corresponding to the same Tx antenna. In case of 3Tx MIMO elevation scheme, the second
 *    set of Rx symbols is rotated by 1/3 of the estimated Doppler phase shift, while the third set
 *    of Rx symbols is rotated by 2/3 of the estimated Doppler phase shift. Refer to the pictures below.
 *    @anchor Figure_doppler
 *    @image html angle_doppler_compensation.png "Figure_doppler: Doppler Compensation"
 *
 * @subsection Extended_max_velocity Extended maximum velocity feature
 *    In TDM-MIMO scheme the angle resolution increases with the number of Tx antennas, as the length of
 *    array of Rx virtual antennas increases, however the maximum unambiguous radial velocity Vmax decreases
 *    proportionally to the number of Tx antennas, as illustrated in Figure below.
 *
 *    @image html max_velocity_comparison.png "Maximum unambiguous velocities in TDM-MIMO for different number of Tx antennas, assuming the same profile configuration in each scheme"
 *
 *    For the target velocities beyond Vmax, the Doppler measurements are aliased. This is illustrated in Figure below
 *    for 3TxTDM-MIMO where several actual velocities can fold into the same aliased velocity.
 *
 *    @image html max_velocity_aliasing.png "Examples of different actual velocities beyond Vmax folding into same aliased velocity"
 *
 *    Maximum velocity extension method is based on the maximum peak in angular FFTs. It consists of the following steps:
 *
 *    1. Calculate phase change \f$\Delta\varphi_{dop}=2\pi\frac{l_d}{N_{DopFFT}}\f$ where \f$l_d\f$ is the Doppler detection index.
 *    2. Test \f$N_{Tx}\f$ hypotheses \f$H_n\f$, \f$ n=0,...N_{Tx}-1\f$. For each hypothesis calculate phase change \f$\Delta\varphi_n\f$.
 *       - for even \f$N_{Tx}\f$
 *         \f[
 *                \Delta\varphi_n= \left\{  \begin{array}{lr}     \Delta\varphi_{dop}+2\pi(n-\frac{N_{Tx}}{2}) & : \Delta\varphi_{dop} \ge 0\\     \Delta\varphi_{dop}+2\pi(n-\frac{N_{Tx}}{2}+1) & : \Delta\varphi_{dop} < 0   \end{array}\right.
 *         \f]
 *       - for odd \f$N_{Tx}\f$
 *         \f[
 *                \Delta\varphi_n= \Delta\varphi_{dop}+2\pi(n- \lfloor\frac{N_{Tx}}{2}\rfloor)
 *         \f]
 *       Figure below illustrates in 3Tx TDM-MIMO 3 hypothetical phase changes \f$\Delta\varphi_n\f$ based on measured \f$\Delta\varphi_{dop}\f$.
 *    @image html max_velocity_unwrpped_phases.png "Doppler phase offsets under different Hypothesis based on measured phase offset"
 *    3. Apply Doppler correction on the \f$i^{th}\f$ set of virtual Rx antennas:
 *          \f[
 *                \Delta\varphi_i= i \cdot \frac{\Delta\varphi_n}{N_{Tx}},  \;\;  i=0,1,...,N_{Tx} - 1
 *          \f]
 *    4. Compute azimuth spectrum \f$S_n\f$ for each \f$H_n\f$ and find the peaks \f$P_n = max(Sn)\f$.
 *    5. Find m = argmax \f$P_n\f$,
 *    6. Complete the azimuth estimation using the data from \f$S_m\f$
 *
 *    Extended maximum velocity feature can be enabled/disabled by using the cli command: extendedMaxVelocity.
 *    When the feature is enabled, local core performs Doppler compensation for all possible hypotheses,
 *    2 hypotheses for 2Tx, and 3 hypotheses for 3Tx antenna TDM MIMO scheme. HWA calculates FFTs for all hypotheses, and
 *    in the local core one hypothesis is picked based on the maximum peak in the magnitude square outputs.
 *
 *    @subsubsection LimitationExtMaxVel Limitation of extended maximum velocity algorithm
 *    The algorithm works well when the single target occurs in range/Doppler bin.
 *    However it can fail with multiple targets in the same range/Doppler bin.
 *    It also fails with multiple targets at the same range and different velocities
 *    wrapped to the same Doppler bin.
 *
 *  @subsection dataXYZ Data Path - Direction of Arrival Estimation (x,y,z)
 *    This processing is done on CPU in the function @ref AoAProcHWA_angleEstimationAzimElev or @ref AoAProcDSP_angleEstimationAzimElev.
 *    The actual computation per detected object is performed by @ref AoAProcHWA_XYZestimation or @ref AoAProcDSP_XYZestimation.
 *    \anchor Figure_geometry
 *    @image html coordinate_geometry.png "Figure A: Coordinate Geometry"
 *    \n
 *    \anchor Figure_wz
 *    @image html coordinate_geometry_wz.png "Figure wz"
 *    \n
 *    \anchor Figures_wx
 *    @image html coordinate_geometry_wx.png "Figures wx"
 *    \ref Figure_geometry shows orientation of x,y,z axes with respect to the sensor/antenna positions. The objective is to estimate
 *    the (x,y,z) coordinates of each detected object.
 *    \f$w_x\f$ is the phase difference between consecutive receive azimuth antennas of the 2D FFT and \f$w_z\f$ is the phase difference between
 *    azimuth and corresponding elevation antenna above the azimuth antenna. The phases for each antenna are shown in the \ref Figure_doppler.
 *    \ref Figure_wz shows that the distance AB which represents the relative distance between wavefronts intersecting consecutive elevation
 *    antennas is \f$AB = \frac{\lambda}{2} \sin (\phi)\f$. Therefore \f$W_z = \frac{2\pi}{\lambda} \cdot AB\f$, therefore \f$W_z = \pi \sin (\phi)\f$.
 *    Note that the phase of the lower antenna is advanced compared to the upper
 *    antenna which is why picture X shows -Wz term in the upper antenna.
 *    \ref Figures_wx show that distance CD which represents the relative distance between wavefronts intersecting consecutive azimuth
 *    antennas is \f$CD = \frac{\lambda}{2} \sin (\theta) \cos (\phi)\f$ Therefore \f$w_x =  \frac{2\pi}{\lambda} \cdot CD\f$,
 *    therefore \f$w_x = \pi \sin (\theta) \cos (\phi)\f$.
 *    For a single obstacle, the signal at the 8 azimuth antennas will be (\f$A_1\f$ and \f$\psi\f$ are the arbitrary starting
 *    amplitude/phase at the first antenna):
 *    \f[
 *    A_1 e^{j\psi} [ 1 \; e^{jw_x} \; e^{j2w_x} \; e^{j3w_x} \; e^{j4w_x} \; e^{j5w_x} \; e^{j6w_x} \; e^{j7w_x} ]
 *    \f]
 *
 *    An FFT of the above signal will yield a peak \f$P_1\f$ at \f$w_x\f$, with the phase of this peak being \f$\psi\f$ i.e
 *    \f[
 *       P_1 = A_1 e^{j\psi}
 *    \f]
 *    If \f$k_{MAX}\f$ is the index of the peak in log magnitude FFT represented as
 *    signed index in range \f$[-\frac{N}{2}, \frac{N}{2}-1]\f$, then \f$ w_x \f$ will be
 *       \f[
 *            w_x = \frac{2\pi}{N}k_{MAX}
 *       \f]
 *
 *    The signal at the 4 elevation antennas will be:
 *     \f[
 *      A_2 e^{j(\psi + 2 w_x - w_z)} [ 1 \; e^{jw_x} \; e^{j2w_x} \; e^{j3w_x}]
 *     \f]
 *
 *    An FFT of the above signal will yield a peak \f$P_2\f$ at \f$w_x\f$, with the phase of this peak being \f$\psi + 2w_x - w_z\f$.
 *     \f[
 *        P_2 = A_2 e^{j(\psi+ 2 w_x - w_z)}
 *     \f]
 *
 *    From above,
 *     \f[
 *        P_1 \cdot P_2^* = A_1 \cdot A_2 e^{j(\psi - (\psi+ 2 w_x - w_z))}
 *     \f]
 *
 *    Therefore,
 *     \f[
 *       w_z=\angle (P_1 \cdot P_2^* \cdot e^{j2w_x})
 *     \f]
 *
 *
 *    Calculate range (in meters) as:
 *       \f[
 *           R=k_r\frac{c \cdot F_{SAMP}}{2 \cdot S \cdot N_{FFT}}
 *       \f]
 *       where, \f$c\f$ is the speed of light (m/sec), \f$k_r\f$ is range index, \f$F_{SAMP}\f$ is the sampling frequency (Hz),
 *       \f$S\f$ is chirp slope (Hz/sec), \f$N_{FFT}\f$ is 1D FFT size.
 *    Based on above calculations of \f$R\f$, \f$w_x\f$ and \f$w_z\f$, the \f$(x,y,z)\f$ position of the object
 *    can be calculated as seen in the \ref Figure_geometry,
 *       \f[
 *           x = R\cos(\phi)\sin(\theta) = R\frac{w_x}{\pi}, \;\;\;\;\; z = R\sin(\phi) = R\frac{w_z}{\pi},\;\;\;\;\; y = \sqrt{R^2-x^2-z^2}
 *       \f]
 *    The computed \f$(x,y,z)\f$ along with the Doppler value for each detected object are populated in the output
 *    list, structured as @ref DPIF_PointCloudCartesian_t.
 *
 *    If the multi object beam forming feature is enabled, (it can be dynamically controlled from CLI), the algorithm
 *    searches for the second peak in the azimuth FFT output and compares its height relative to the first peak height,
 *    and if detected, it creates new object in the output list with the same Doppler
 *    value and calculated (x,y,z) coordinates.
 *
 * @subsection fovSubSection Elimination of detected objects based on FoV limits
 *
 * The AoA DPU filters out detected objects whose azimuth, \f$\theta\f$, and elevation, \f$\phi\f$, angles
 * are not within configured FoV limits. The DPU receives the following FoV parameters:
 *
 * \f$\phi_{min},\;\phi_{max},\f$ - elevation angle FoV limits
 *
 * \f$\theta_{min},\;\theta_{max}\f$ - azimuth angle FoV limits
 *
 * The DPU computes \f$W_z\f$  and \f$W_x\f$ directly from azimuth FFT output data, where
 *
 * \f$W_z = w_z/\pi\f$
 *
 * \f$W_x = w_x/\pi\f$
 *
 * From equations in previous section the following holds
 *
 * \f$W_z = \sin(\phi)\f$
 *
 * \f$W_x = \cos(\phi)  \sin(\theta)\f$
 *
 * Because sine is a monotonic increasing function in the maximum possible range
 * of -90 deg, 90 deg, the elevation angle \f$\phi\f$ FoV limits can be checked on \f$W_z\f$
 * as follows
 *
 * \f[
 * \sin(\phi_{min}) \leq W_z \leq \sin(\phi_{max})
 * \f]
 *
 * Since \f$W_x\f$ is not monotonic in \f$\phi\f$ but monotonic on \f$\theta\f$,
 * the azimuth angle \f$\theta\f$ limits can be checked by checking \f$W_x\f$ after computing
 * the \f$\cos(\phi)\f$ on the measured \f$\phi\f$.
 *
 * \f[
 * \cos(\phi)\sin(\theta_{min}) \leq W_x \leq \cos(\phi) \sin(\theta_{max})
 * \f]
 *
 * where \f$\cos(\phi) = \sqrt{1 - \sin(\phi)^2} = \sqrt{1 - W_z^2}\f$.
 * Note that the terms  \f$\sin(\phi_{min})\f$, \f$\sin(\phi_{max})\f$,
 * \f$\sin(\theta_{min})\f$ and \f$\sin(\theta_{max})\f$ are precomputed during configuration time
 * and stored in the dpu's instance (@ref DPU_AoAProc_fovAoaLocalCfg_t) to be used
 * in the real-time limit checks above.
 * The brute-force method of checking limits on \f$\phi\f$ and \f$\theta\f$ directly
 * requires computing these, which involves two sin inverses, one cosine and one
 * division which are computationally more expensive than the above method.
 * 
 * 
 */

 
 
/** @defgroup DPU_AOAPROC_INTERNAL       aoaProc DPU Internal
 *
 */
/**
@defgroup DPU_AOAPROC_INTERNAL_FUNCTION            aoaProc DPU Internal Functions
@ingroup DPU_AOAPROC_INTERNAL
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup DPU_AOAPROC_INTERNAL_DATA_STRUCTURE      aoaProc DPU Internal Data Structures
@ingroup DPU_AOAPROC_INTERNAL
@brief
*   The section has a list of all internal data structures which are used internally
*   by the aoaProc DPU module.
*/
/**
@defgroup DPU_AOAPROC_INTERNAL_DEFINITION          aoaProc DPU Internal Definitions
@ingroup DPU_AOAPROC_INTERNAL
@brief
*   The section has a list of all internal definitions which are used internally
*   by the aoaProc DPU.
*/
 
/** @defgroup DPU_AOAPROC_EXTERNAL       aoaProc DPU External
 */
/**
@defgroup DPU_AOAPROC_EXTERNAL_FUNCTION            aoaProc DPU External Functions
@ingroup DPU_AOAPROC_EXTERNAL
@brief
*   The section has a list of all the exported API which the user need to
*   invoke in order to use the aoaProc DPU
*/
/**
@defgroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE      aoaProc DPU External Data Structures
@ingroup DPU_AOAPROC_EXTERNAL
@brief
*   The section has a list of all the data structures which are exposed to the user
*/
/**
@defgroup DPU_AOAPROC_EXTERNAL_DEFINITIONS      aoaProc DPU External Definitions
@ingroup DPU_AOAPROC_EXTERNAL
@brief
*   The section has a list of all the definitions which are exposed to the user
*/
/**
@defgroup DPU_AOAPROC_ERROR_CODE                   aoaProc DPU Error Codes
@ingroup DPU_AOAPROC_EXTERNAL
@brief
*   The section has a list of all the error codes which are generated by the sampleProc DPU
*/
 
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef AOAPROC_COMMON_H
#define AOAPROC_COMMON_H

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

/*! @brief Converts Doppler index to signed variable. Value greater than or equal
 *         half the Doppler FFT size will become negative.
 */
#define AOA_DOPPLER_IDX_TO_SIGNED(_idx, _fftSize) ((_idx) < (_fftSize)/2 ? \
        ((int16_t) (_idx)) : ((int16_t) (_idx) - (int16_t) (_fftSize)))

/**
 * @brief
 *  Field of view - AoA Configuration
 *
 * @details
 *  The structure contains the field of view - AoA configuration used in data path
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_FovAoaCfg_t
{
    /*! @brief    minimum azimuth angle (in degrees) exported to Host*/
    float        minAzimuthDeg;

    /*! @brief    maximum azimuth angle (in degrees) exported to Host*/
    float        maxAzimuthDeg;

    /*! @brief    minimum elevation angle (in degrees) exported to Host*/
    float        minElevationDeg;

    /*! @brief    maximum elevation angle (in degrees) exported to Host*/
    float        maxElevationDeg;
} DPU_AoAProc_FovAoaCfg;

/**
 * @brief
 *  Multi object beam forming Configuration
 *
 * @details
 *  The structure contains the Peak grouping configuration used in data path
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_MultiObjBeamFormingCfg_t
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint8_t     enabled;

    /*! @brief    second peak detection threshold */
    float        multiPeakThrsScal;

} DPU_AoAProc_MultiObjBeamFormingCfg;

/**
 * @brief Range Bias and rx channel gain/phase compensation configuration.
 *
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_compRxChannelBiasCfg_t
{

    /*! @brief  Compensation for range estimation bias in meters */
    float rangeBias;

    /*! @brief  Compensation for Rx channel phase bias in Q15 format.
     *          The order here is like x[tx][rx] where rx order is 0,1,....SYS_COMMON_NUM_RX_CHANNEL-1
     *          and tx order is 0, 1,...,SYS_COMMON_NUM_TX_ANTENNAS-1 */
    cmplx16ImRe_t rxChPhaseComp[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];

} DPU_AoAProc_compRxChannelBiasCfg;

/**
 * @brief
 *  Extended maximum velocity configuration
 *
 * @details
 *  Extended maximum velocity configuration
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_ExtendedMaxVelocityCfg_t
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint8_t     enabled;

} DPU_AoAProc_ExtendedMaxVelocityCfg;


/**
 * @brief
 *  Static clutter removal configuration
 *
 * @details
 *  The structure contains the configuration for static clutter removal.
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProc_StaticClutterRemovalCfg_t
{
    /*! @brief Flag that indicates if static clutter removal is enabled.*/
    bool  isEnabled;
}DPU_AoAProc_StaticClutterRemovalCfg;

/**
 * @brief
 *  AoAProc dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration used for AoA
 *
 *  \ingroup    DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProc_DynamicConfig_t
{

    /** @brief     Multiobject beam forming configuration */
    DPU_AoAProc_MultiObjBeamFormingCfg *multiObjBeamFormingCfg;

    /** @brief     Flag indicates to prepare data for azimuth heat-map */
    bool  prepareRangeAzimuthHeatMap;

    /** @brief     Pointer to Rx channel compensation configuration */
    DPU_AoAProc_compRxChannelBiasCfg *compRxChanCfg;

    /** @brief      Field of view configuration for angle of arrival */
    DPU_AoAProc_FovAoaCfg     *fovAoaCfg;

    /** @brief      Extended maximum velocity configuration */
    DPU_AoAProc_ExtendedMaxVelocityCfg *extMaxVelCfg;

    /*! @brief Static clutter removal configuration. Valid only for DSP version of the AoA DPU.*/
    DPU_AoAProc_StaticClutterRemovalCfg  staticClutterCfg;
} DPU_AoAProc_DynamicConfig;



/**
 * @brief
 *  Data processing Unit statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU 
 *
 *  \ingroup DPU_AOAPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProc_Stats_t
{
    /** @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t            processingTime;

    /** @brief total wait time for EDMA data transfer during all chirps in a frame*/
    uint32_t            waitTime;
} DPU_AoAProc_Stats;

#ifdef __cplusplus
}
#endif

#endif 
