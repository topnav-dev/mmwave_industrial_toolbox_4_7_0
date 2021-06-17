// PROFILE CONFIG from profile_2d 

/*

% Carrier frequency     GHz                          60
% Ramp Slope    MHz/us                               166
% Num ADC Samples                                    256
% ADC Sampling Rate Msps                             12.5
% ADC Collection Time   us                           20.48
% Extra ramp time required (start time) us           3
% Chirp time (end time - start time)    us           21
% Chirp duration (end time) us                       24
% Sweep BW (useful) MHz                              3399.68
% Total BW  MHz                                      3984
% Max beat freq (80% of ADC sampling rate)  MHz      10
% Max distance (80%)    m                            9.04
% Range resolution  m                                0.044
% Range resolution (meter per 1D-FFT bin)   m/bin    0.044
%                                                    
% Inter-chirp duration  us                           7
% Number of chirp intervals in frame    -            64
% Number of TX (TDM MIMO)                            2
% Number of Tx elevation antennas                    0
% Number of RX channels -                            4
% Max umambiguous relative velocity kmph             72.58
%   mileph                                           45.36
% Max extended relative velocity    kmph             145.16
%   mileph                                           90.73
% Frame time (total)    ms                           1.984
% Frame time (active)   ms                           1.536
% Range FFT size    -                                256
% Doppler FFT size  -                                32
% Radar data memory required    KB                   272
% Velocity resolution   m/s                          1.26
% Velocity resolution (m/s per 2D-FFT bin)  m/s/bin  1.26
% Velocity Maximum  m/s                              20.16
% Extended Maximum Velocity m/s                      40.32
% Maximum sweep accorss range bins  range bin        0.91
% 
sensorStop
flushCfg
dfeDataOutputMode 1
channelCfg 15 5 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
lowPower 0 0
profileCfg 0 60 7 3 24 0 0 166 1 256 12500 0 0 30
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 4
frameCfg 0 1 32 0 100 1 0
guiMonitor -1 1 1 1 0 0 1
cfarCfg -1 0 2 8 4 3 0 15.0 0
cfarCfg -1 1 0 4 2 3 1 15.0 0
multiObjBeamForming -1 1 0.5
calibDcRangeSig -1 0 -5 8 256
clutterRemoval -1 0
compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
measureRangeBiasAndRxChanPhase 0 1. 0.2
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg -1 0 0.25 9.0
cfarFovCfg -1 1 -20.16 20.16
extendedMaxVelocity -1 0
CQRxSatMonitor 0 3 4 63 0
CQSigImgMonitor 0 127 4
analogMonitor 0 0
lvdsStreamCfg -1 0 0 0
bpmCfg -1 0 0 0
sensorStart

*/

#define PROFILE_HCC_0_PROFILE_ID            (0U)
#define PROFILE_HCC_0_START_FREQ_GHZ        (60U)
#define PROFILE_HCC_0_IDLE_TIME_VAL         (7U)
#define PROFILE_HCC_0_ADC_START_TIME_VAL    (3U)
#define PROFILE_HCC_0_RAMP_END_TIME_VAL     (24U)
#define PROFILE_HCC_0_TXOUT_POWER_BACKOFF   (0U)
#define PROFILE_HCC_0_TXPHASESHIFTER_VAL    (0U)
#define PROFILE_HCC_0_FREQ_SLOPE_MHZ_PER_US (166U)
#define PROFILE_HCC_0_TX_START_TIME_VAL     (1U)
#define PROFILE_HCC_0_ADC_SAMPLE_VAL        (256U)
#define PROFILE_HCC_0_DIGOUT_SAMPLERATE_VAL (12500U)
#define PROFILE_HCC_0_HPFCORNER_FREQ1_VAL   (0U)
#define PROFILE_HCC_0_HPFCORNER_FREQ2_VAL   (0U)
#define PROFILE_HCC_0_RX_GAIN_VAL           (30U)

#define CHIRP_HCC_0_START_INDEX             (0U)
#define CHIRP_HCC_0_END_INDEX               (0U)
#define CHIRP_HCC_0_PROFILE_ID              (0U)
#define CHIRP_HCC_0_START_FREQ_VAL          (0U)
#define CHIRP_HCC_0_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_0_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_0_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_0_TX_CHANNEL              (1U)

#define CHIRP_HCC_1_START_INDEX             (1U)
#define CHIRP_HCC_1_END_INDEX               (1U)
#define CHIRP_HCC_1_PROFILE_ID              (0U)
#define CHIRP_HCC_1_START_FREQ_VAL          (0U)
#define CHIRP_HCC_1_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_1_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_1_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_1_TX_CHANNEL              (4U)

#define DFE_DATA_OUTPUT_MODE_HCC            (1U)

#define CHANNEL_HCC_RX_CHANNEL_EN           (15U)
#define CHANNEL_HCC_TX_CHANNEL_EN           (5U)

#define FRAME_HCC_CHIRP_START_INDEX         (0U)
#define FRAME_HCC_CHIRP_END_INDEX           (1U)
#define FRAME_HCC_NUM_LOOPS                 (32U)
#define FRAME_HCC_NUM_FRAMES                (0U)
#define FRAME_HCC_FRAME_PERIODICITY         (100U)
#define FRAME_HCC_TRIGGER_SELECT            (1U)
#define FRAME_HCC_FRAME_TRIG_DELAY          (0U)

#define GUI_HCC_0_SUBFRAME_IDX              (-1)
#define GUI_HCC_0_DETECTED_OBJECTS          (1U)
#define GUI_HCC_0_LOG_MAGNITUDE_RANGE       (1U)
#define GUI_HCC_0_NOISE_PROFILE             (1U)
#define GUI_HCC_0_RANGE_AZIMUTH_MAP         (0U)
#define GUI_HCC_0_RANGE_DOPPLER_MAP         (0U)
#define GUI_HCC_0_STATS_INFO                (1U)

#define CFAR_HCC_0_SUBFRAME_IDX             (-1)
#define CFAR_HCC_0_PROC_DIRECTION           (0U)
#define CFAR_HCC_0_MODE                     (2U)
#define CFAR_HCC_0_NOISE_WIN                (8U)
#define CFAR_HCC_0_GUARD_LEN                (4U)
#define CFAR_HCC_0_DIV_SHIFT                (3U)
#define CFAR_HCC_0_CYCLIC_MODE              (0U)
#define CFAR_HCC_0_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_0_PEAK_GROUPING            (0U)

#define CFAR_HCC_1_SUBFRAME_IDX             (-1)
#define CFAR_HCC_1_PROC_DIRECTION           (1U)
#define CFAR_HCC_1_MODE                     (0U)
#define CFAR_HCC_1_NOISE_WIN                (4U)
#define CFAR_HCC_1_GUARD_LEN                (2U)
#define CFAR_HCC_1_DIV_SHIFT                (3U)
#define CFAR_HCC_1_CYCLIC_MODE              (1U)
#define CFAR_HCC_1_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_1_PEAK_GROUPING            (0U)

#define MULTI_OBJ_BEAM_HCC_0_SUBFRAME_IDX   (-1)
#define MULTI_OBJ_BEAM_HCC_0_ENABLED        (1U)
#define MULTI_OBJ_BEAM_HCC_0_THRESHOLD      (0.5f)

#define CALIB_DC_RANGE_HCC_0_SUBFRAME_IDX   (-1)
#define CALIB_DC_RANGE_HCC_0_ENABLED        (0U)
#define CALIB_DC_RANGE_HCC_0_NEG_BIN_IDX    (-5)
#define CALIB_DC_RANGE_HCC_0_POS_BIN_IDX    (8U)
#define CALIB_DC_RANGE_HCC_0_NUM_AVG        (256U)

#define CFAR_FOV_HCC_0_SUBFRAME_IDX         (-1)
#define CFAR_FOV_HCC_0_PROC_DIRECTION       (0U)
#define CFAR_FOV_HCC_0_MINIMUM              (0.25f)
#define CFAR_FOV_HCC_0_MAXIMUM              (9.0f)

#define CFAR_FOV_HCC_1_SUBFRAME_IDX         (-1)
#define CFAR_FOV_HCC_1_PROC_DIRECTION       (1U)
#define CFAR_FOV_HCC_1_MINIMUM              (-20.16f)
#define CFAR_FOV_HCC_1_MAXIMUM              (20.16f)

#define MEASURERANGEBIAS_HCC_ENABLED        (0U)
#define MEASURERANGEBIAS_HCC_TARGET_DIST    (1.0f)
#define MEASURERANGEBIAS_HCC_SEARCH_WIN     (0.2f)

#define CQRXSAT_HCC_0_PROFILE_ID            (0U)
#define CQRXSAT_HCC_0_SAT_MON_SEL           (3U)
#define CQRXSAT_HCC_0_PRI_SLICE_DURATION    (4U)
#define CQRXSAT_HCC_0_NUM_SLICES            (63U)
#define CQRXSAT_HCC_0_RX_CHAN_MASK          (0U)

#define CQSIGIMG_HCC_0_PROFILE_ID           (0U)
#define CQSIGIMG_HCC_0_NUM_SLICES           (127U)
#define CQSIGIMG_HCC_0_NUM_SAMPLE_PER_SLICE (4U)

#define BPM_HCC_SUBFRAME_IDX                (-1)
#define BPM_HCC_ENABLED                     (0U)
#define BPM_HCC_CHIRP_0_IDX                 (0U)
#define BPM_HCC_CHIRP_1_IDX                 (0U)
