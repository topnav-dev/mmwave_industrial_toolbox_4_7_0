// PROFILE CONFIG from profile_advanced_subframe

/*

% Four subframes:                                           
% Profile                                                    0       1       2       3
% Carrier frequency     GHz                                  60     60      60      60
% Ramp Slope    MHz/us                                       250    150     55      30
% Num ADC Samples                                            128    256     256     512
% ADC Sampling Rate Msps                                     9.2    12      12.5    12.5
% ADC Collection Time   us                                   13.91  21.33   20.48   40.96
% Extra ramp time required (start time) us                   2      3       3       3
% Chirp time (end time - start time)    us                   14     22      21      41
% Chirp duration (end time) us                               16     25      24      44
% Sweep BW (useful) MHz                                      3478.26 3200.00 1126.40 1228.80
% Total BW  MHz                                              4000   3750    1320    1320
% Max beat freq (80% of ADC sampling rate)  MHz              7.36   9.6     10      10
% Max distance (80%)    m                                    4.42   9.60    27.27   50.00
% Range resolution  m                                        0.043  0.047   0.133   0.122
% Range resolution (meter per 1D-FFT bin)   m/bin            0.043  0.047   0.133   0.122
%                                                                       
% Inter-chirp duration  us                                   7      7       7       7
% Number of chirp intervals in frame    -                    64     64      64      32
% Number of TX (TDM MIMO)                                    2      2       2       2
% Number of Tx elevation antennas                            0      0       0       0
% Number of RX channels -                                    4      4       4       4
% Max umambiguous relative velocity kmph                     97.83  70.31   72.58   44.12
%   mileph                                                   61.14  43.95   45.36   27.57
% Max extended relative velocity    kmph                     195.65 140.63  145.16  88.24
%   mileph                                                   122.28 87.89   90.73   55.15
% Frame time (total)    ms                                   1.472  2.048   1.984   1.632
% Frame time (active)   ms                                   1.024  1.6     1.536   1.408
% Range FFT size    -                                        128    256     256     512
% Doppler FFT size  -                                        32     32      32      16
% Radar data memory required    KB                           136    272     272     272
% Velocity resolution   m/s                                  1.70   1.22    1.26    1.53
% Velocity resolution (m/s per 2D-FFT bin)  m/s/bin          1.70   1.22    1.26    1.53
% Velocity Maximum  m/s                                      27.17  19.53   20.16   12.25
% Extended Maximum Velocity m/s                              54.35  39.06   40.32   24.51
% Maximum sweep accorss range bins  range bin                0.93   0.85    0.30    0.16
% 
sensorStop                                                  
flushCfg
dfeDataOutputMode 3
channelCfg 15 5 0
adcCfg 2 1
adcbufCfg -1 0 1 1 1
lowPower 0 0
%
profileCfg 0 60 7 2 16 0 0 250 1 128 9200 0 0 30
profileCfg 1 60 7 3 25 0 0 150 1 256 12000 0 0 30
profileCfg 2 60 7 3 24 0 0 55 1 256 12500 0 0 30
profileCfg 3 60 7 3 44 0 0 30 1 512 12500 0 0 30
%
chirpCfg 0 0 0 0 0 0 0 1
chirpCfg 1 1 0 0 0 0 0 4
%
chirpCfg 2 2 1 0 0 0 0 1
chirpCfg 3 3 1 0 0 0 0 4
%
chirpCfg 4 4 2 0 0 0 0 1
chirpCfg 5 5 2 0 0 0 0 4
%
chirpCfg 6 6 3 0 0 0 0 1
chirpCfg 7 7 3 0 0 0 0 4
%
advFrameCfg 4 0 0 1 0
subFrameCfg 0 0 0 2 32 100 0 1 1 100
subFrameCfg 1 0 2 2 32 100 0 1 1 100
subFrameCfg 2 0 4 2 32 100 0 1 1 100
subFrameCfg 3 0 6 2 16 100 0 1 1 100
%
guiMonitor 0 1 1 1 0 0 1
guiMonitor 1 1 0 0 0 0 1
guiMonitor 2 1 1 1 0 0 1
guiMonitor 3 1 0 0 0 0 1
%
cfarCfg 0 0 2 8 4 3 0 15.0 0
cfarCfg 0 1 0 4 2 3 1 15.0 0
cfarCfg 1 0 2 8 4 3 0 15.0 1
cfarCfg 1 1 0 4 2 3 1 15.0 1
cfarCfg 2 0 2 8 4 3 0 15.0 0
cfarCfg 2 1 0 4 2 3 1 15.0 0
cfarCfg 3 0 2 8 4 3 0 15.0 1
cfarCfg 3 1 0 4 2 3 1 15.0 1
%
multiObjBeamForming 0 1 0.5
multiObjBeamForming 1 1 0.6
multiObjBeamForming 2 0 0.4
multiObjBeamForming 3 1 0.7
%
calibDcRangeSig 0 0 -3 3 256
calibDcRangeSig 1 0 -2 2 256
calibDcRangeSig 2 0 -1 1 256
calibDcRangeSig 3 0 -1 1 256
%
aoaFovCfg -1 -90 90 -90 90
cfarFovCfg 0 1 -27.17 27.17
cfarFovCfg 1 1 -19.53 19.53
cfarFovCfg 2 1 -20.16 20.16
cfarFovCfg 3 1 -12.25 12.25

cfarFovCfg  0 0 0.25 4.4
cfarFovCfg  1 0 4.4 9.6
cfarFovCfg  2 0 9.6 27.27
cfarFovCfg  3 0 27.27 50.0


%
clutterRemoval -1 0
% 
compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0
measureRangeBiasAndRxChanPhase 0 1. 0.2
%
bpmCfg -1 0 0 0
extendedMaxVelocity -1 0
%
CQRxSatMonitor 0 3 4 43 0
CQSigImgMonitor 0 63 4
CQRxSatMonitor 1 3 4 67 0
CQSigImgMonitor 1 127 4
CQRxSatMonitor 2 3 4 63 0
CQSigImgMonitor 2 127 4
CQRxSatMonitor 3 3 4 127 0
CQSigImgMonitor 3 127 8

analogMonitor 0 0

lvdsStreamCfg -1 0 0 0
%
sensorStart


*/

#define PROFILE_HCC_0_PROFILE_ID            (0U)
#define PROFILE_HCC_0_START_FREQ_GHZ        (60U)
#define PROFILE_HCC_0_IDLE_TIME_VAL         (7U)
#define PROFILE_HCC_0_ADC_START_TIME_VAL    (2U)
#define PROFILE_HCC_0_RAMP_END_TIME_VAL     (16U)
#define PROFILE_HCC_0_TXOUT_POWER_BACKOFF   (0U)
#define PROFILE_HCC_0_TXPHASESHIFTER_VAL    (0U)
#define PROFILE_HCC_0_FREQ_SLOPE_MHZ_PER_US (250U)
#define PROFILE_HCC_0_TX_START_TIME_VAL     (1U)
#define PROFILE_HCC_0_ADC_SAMPLE_VAL        (128U)
#define PROFILE_HCC_0_DIGOUT_SAMPLERATE_VAL (9200U)
#define PROFILE_HCC_0_HPFCORNER_FREQ1_VAL   (0U)
#define PROFILE_HCC_0_HPFCORNER_FREQ2_VAL   (0U)
#define PROFILE_HCC_0_RX_GAIN_VAL           (30U)

#define PROFILE_HCC_1_PROFILE_ID            (1U)
#define PROFILE_HCC_1_START_FREQ_GHZ        (60U)
#define PROFILE_HCC_1_IDLE_TIME_VAL         (7U)
#define PROFILE_HCC_1_ADC_START_TIME_VAL    (3U)
#define PROFILE_HCC_1_RAMP_END_TIME_VAL     (25U)
#define PROFILE_HCC_1_TXOUT_POWER_BACKOFF   (0U)
#define PROFILE_HCC_1_TXPHASESHIFTER_VAL    (0U)
#define PROFILE_HCC_1_FREQ_SLOPE_MHZ_PER_US (150U)
#define PROFILE_HCC_1_TX_START_TIME_VAL     (1U)
#define PROFILE_HCC_1_ADC_SAMPLE_VAL        (256U)
#define PROFILE_HCC_1_DIGOUT_SAMPLERATE_VAL (12000U)
#define PROFILE_HCC_1_HPFCORNER_FREQ1_VAL   (0U)
#define PROFILE_HCC_1_HPFCORNER_FREQ2_VAL   (0U)
#define PROFILE_HCC_1_RX_GAIN_VAL           (30U)

#define PROFILE_HCC_2_PROFILE_ID            (2U)
#define PROFILE_HCC_2_START_FREQ_GHZ        (60U)
#define PROFILE_HCC_2_IDLE_TIME_VAL         (7U)
#define PROFILE_HCC_2_ADC_START_TIME_VAL    (3U)
#define PROFILE_HCC_2_RAMP_END_TIME_VAL     (24U)
#define PROFILE_HCC_2_TXOUT_POWER_BACKOFF   (0U)
#define PROFILE_HCC_2_TXPHASESHIFTER_VAL    (0U)
#define PROFILE_HCC_2_FREQ_SLOPE_MHZ_PER_US (55U)
#define PROFILE_HCC_2_TX_START_TIME_VAL     (1U)
#define PROFILE_HCC_2_ADC_SAMPLE_VAL        (256U)
#define PROFILE_HCC_2_DIGOUT_SAMPLERATE_VAL (12500U)
#define PROFILE_HCC_2_HPFCORNER_FREQ1_VAL   (0U)
#define PROFILE_HCC_2_HPFCORNER_FREQ2_VAL   (0U)
#define PROFILE_HCC_2_RX_GAIN_VAL           (30U)

#define PROFILE_HCC_3_PROFILE_ID            (3U)
#define PROFILE_HCC_3_START_FREQ_GHZ        (60U)
#define PROFILE_HCC_3_IDLE_TIME_VAL         (7U)
#define PROFILE_HCC_3_ADC_START_TIME_VAL    (3U)
#define PROFILE_HCC_3_RAMP_END_TIME_VAL     (44U)
#define PROFILE_HCC_3_TXOUT_POWER_BACKOFF   (0U)
#define PROFILE_HCC_3_TXPHASESHIFTER_VAL    (0U)
#define PROFILE_HCC_3_FREQ_SLOPE_MHZ_PER_US (30U)
#define PROFILE_HCC_3_TX_START_TIME_VAL     (1U)
#define PROFILE_HCC_3_ADC_SAMPLE_VAL        (512U)
#define PROFILE_HCC_3_DIGOUT_SAMPLERATE_VAL (12500U)
#define PROFILE_HCC_3_HPFCORNER_FREQ1_VAL   (0U)
#define PROFILE_HCC_3_HPFCORNER_FREQ2_VAL   (0U)
#define PROFILE_HCC_3_RX_GAIN_VAL           (30U)

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

#define CHIRP_HCC_2_START_INDEX             (2U)
#define CHIRP_HCC_2_END_INDEX               (2U)
#define CHIRP_HCC_2_PROFILE_ID              (1U)
#define CHIRP_HCC_2_START_FREQ_VAL          (0U)
#define CHIRP_HCC_2_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_2_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_2_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_2_TX_CHANNEL              (1U)

#define CHIRP_HCC_3_START_INDEX             (3U)
#define CHIRP_HCC_3_END_INDEX               (3U)
#define CHIRP_HCC_3_PROFILE_ID              (1U)
#define CHIRP_HCC_3_START_FREQ_VAL          (0U)
#define CHIRP_HCC_3_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_3_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_3_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_3_TX_CHANNEL              (4U)

#define CHIRP_HCC_4_START_INDEX             (4U)
#define CHIRP_HCC_4_END_INDEX               (4U)
#define CHIRP_HCC_4_PROFILE_ID              (2U)
#define CHIRP_HCC_4_START_FREQ_VAL          (0U)
#define CHIRP_HCC_4_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_4_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_4_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_4_TX_CHANNEL              (1U)

#define CHIRP_HCC_5_START_INDEX             (5U)
#define CHIRP_HCC_5_END_INDEX               (5U)
#define CHIRP_HCC_5_PROFILE_ID              (2U)
#define CHIRP_HCC_5_START_FREQ_VAL          (0U)
#define CHIRP_HCC_5_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_5_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_5_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_5_TX_CHANNEL              (4U)

#define CHIRP_HCC_6_START_INDEX             (6U)
#define CHIRP_HCC_6_END_INDEX               (6U)
#define CHIRP_HCC_6_PROFILE_ID              (3U)
#define CHIRP_HCC_6_START_FREQ_VAL          (0U)
#define CHIRP_HCC_6_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_6_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_6_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_6_TX_CHANNEL              (1U)

#define CHIRP_HCC_7_START_INDEX             (7U)
#define CHIRP_HCC_7_END_INDEX               (7U)
#define CHIRP_HCC_7_PROFILE_ID              (3U)
#define CHIRP_HCC_7_START_FREQ_VAL          (0U)
#define CHIRP_HCC_7_FREQ_SLOPE_VAL          (0U)
#define CHIRP_HCC_7_IDLE_TIME_VAL           (0U)
#define CHIRP_HCC_7_ADC_START_TIME_VAL      (0U)
#define CHIRP_HCC_7_TX_CHANNEL              (4U)

#define ADVFRAME_HCC_NUM_SUBFRAMES          (4U)
#define ADVFRAME_HCC_FORCE_PROFILE          (0U)
#define ADVFRAME_HCC_NUM_FRAMES             (0U)
#define ADVFRAME_HCC_TRIGGER_SELECT         (1U)
#define ADVFRAME_HCC_FRAME_TRIG_DELAY       (0U)

#define SUBFRAME_HCC_0_NUMBER               (0U)
#define SUBFRAME_HCC_0_FORCE_PROFILE_INDEX  (0U)
#define SUBFRAME_HCC_0_CHIRP_START_INDEX    (0U)
#define SUBFRAME_HCC_0_NUM_CHIRPS           (2U)
#define SUBFRAME_HCC_0_NUM_LOOPS            (32U)
#define SUBFRAME_HCC_0_BURST_PERIODICITY    (100U)
#define SUBFRAME_HCC_0_CHIRP_START_OFFSET   (0U)
#define SUBFRAME_HCC_0_NUM_BURST            (1U)
#define SUBFRAME_HCC_0_NUM_BURST_LOOPS      (1U)
#define SUBFRAME_HCC_0_SUBFRAME_PERIODICITY (100U)

#define SUBFRAME_HCC_1_NUMBER               (1U)
#define SUBFRAME_HCC_1_FORCE_PROFILE_INDEX  (0U)
#define SUBFRAME_HCC_1_CHIRP_START_INDEX    (2U)
#define SUBFRAME_HCC_1_NUM_CHIRPS           (2U)
#define SUBFRAME_HCC_1_NUM_LOOPS            (32U)
#define SUBFRAME_HCC_1_BURST_PERIODICITY    (100U)
#define SUBFRAME_HCC_1_CHIRP_START_OFFSET   (0U)
#define SUBFRAME_HCC_1_NUM_BURST            (1U)
#define SUBFRAME_HCC_1_NUM_BURST_LOOPS      (1U)
#define SUBFRAME_HCC_1_SUBFRAME_PERIODICITY (100U)

#define SUBFRAME_HCC_2_NUMBER               (2U)
#define SUBFRAME_HCC_2_FORCE_PROFILE_INDEX  (0U)
#define SUBFRAME_HCC_2_CHIRP_START_INDEX    (4U)
#define SUBFRAME_HCC_2_NUM_CHIRPS           (2U)
#define SUBFRAME_HCC_2_NUM_LOOPS            (32U)
#define SUBFRAME_HCC_2_BURST_PERIODICITY    (100U)
#define SUBFRAME_HCC_2_CHIRP_START_OFFSET   (0U)
#define SUBFRAME_HCC_2_NUM_BURST            (1U)
#define SUBFRAME_HCC_2_NUM_BURST_LOOPS      (1U)
#define SUBFRAME_HCC_2_SUBFRAME_PERIODICITY (100U)

#define SUBFRAME_HCC_3_NUMBER               (3U)
#define SUBFRAME_HCC_3_FORCE_PROFILE_INDEX  (0U)
#define SUBFRAME_HCC_3_CHIRP_START_INDEX    (6U)
#define SUBFRAME_HCC_3_NUM_CHIRPS           (2U)
#define SUBFRAME_HCC_3_NUM_LOOPS            (16U)
#define SUBFRAME_HCC_3_BURST_PERIODICITY    (100U)
#define SUBFRAME_HCC_3_CHIRP_START_OFFSET   (0U)
#define SUBFRAME_HCC_3_NUM_BURST            (1U)
#define SUBFRAME_HCC_3_NUM_BURST_LOOPS      (1U)
#define SUBFRAME_HCC_3_SUBFRAME_PERIODICITY (100U)

#define DFE_DATA_OUTPUT_MODE_HCC            (3U)

#define CHANNEL_HCC_RX_CHANNEL_EN           (15U)
#define CHANNEL_HCC_TX_CHANNEL_EN           (5U)

#define FRAME_HCC_CHIRP_START_INDEX         (0U)
#define FRAME_HCC_CHIRP_END_INDEX           (2U)
#define FRAME_HCC_NUM_LOOPS                 (32U)
#define FRAME_HCC_NUM_FRAMES                (0U)
#define FRAME_HCC_FRAME_PERIODICITY         (100U)
#define FRAME_HCC_TRIGGER_SELECT            (1U)
#define FRAME_HCC_FRAME_TRIG_DELAY          (0U)

#define GUI_HCC_0_SUBFRAME_IDX              (0U)
#define GUI_HCC_0_DETECTED_OBJECTS          (1U)
#define GUI_HCC_0_LOG_MAGNITUDE_RANGE       (1U)
#define GUI_HCC_0_NOISE_PROFILE             (1U)
#define GUI_HCC_0_RANGE_AZIMUTH_MAP         (0U)
#define GUI_HCC_0_RANGE_DOPPLER_MAP         (0U)
#define GUI_HCC_0_STATS_INFO                (1U)

#define GUI_HCC_1_SUBFRAME_IDX              (1U)
#define GUI_HCC_1_DETECTED_OBJECTS          (1U)
#define GUI_HCC_1_LOG_MAGNITUDE_RANGE       (0U)
#define GUI_HCC_1_NOISE_PROFILE             (0U)
#define GUI_HCC_1_RANGE_AZIMUTH_MAP         (0U)
#define GUI_HCC_1_RANGE_DOPPLER_MAP         (0U)
#define GUI_HCC_1_STATS_INFO                (1U)

#define GUI_HCC_2_SUBFRAME_IDX              (2U)
#define GUI_HCC_2_DETECTED_OBJECTS          (1U)
#define GUI_HCC_2_LOG_MAGNITUDE_RANGE       (1U)
#define GUI_HCC_2_NOISE_PROFILE             (1U)
#define GUI_HCC_2_RANGE_AZIMUTH_MAP         (0U)
#define GUI_HCC_2_RANGE_DOPPLER_MAP         (0U)
#define GUI_HCC_2_STATS_INFO                (1U)

#define GUI_HCC_3_SUBFRAME_IDX              (3U)
#define GUI_HCC_3_DETECTED_OBJECTS          (1U)
#define GUI_HCC_3_LOG_MAGNITUDE_RANGE       (0U)
#define GUI_HCC_3_NOISE_PROFILE             (0U)
#define GUI_HCC_3_RANGE_AZIMUTH_MAP         (0U)
#define GUI_HCC_3_RANGE_DOPPLER_MAP         (0U)
#define GUI_HCC_3_STATS_INFO                (1U)

#define CFAR_HCC_0_SUBFRAME_IDX             (0U)
#define CFAR_HCC_0_PROC_DIRECTION           (0U)
#define CFAR_HCC_0_MODE                     (2U)
#define CFAR_HCC_0_NOISE_WIN                (8U)
#define CFAR_HCC_0_GUARD_LEN                (4U)
#define CFAR_HCC_0_DIV_SHIFT                (3U)
#define CFAR_HCC_0_CYCLIC_MODE              (0U)
#define CFAR_HCC_0_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_0_PEAK_GROUPING            (0U)

#define CFAR_HCC_1_SUBFRAME_IDX             (0U)
#define CFAR_HCC_1_PROC_DIRECTION           (1U)
#define CFAR_HCC_1_MODE                     (0U)
#define CFAR_HCC_1_NOISE_WIN                (4U)
#define CFAR_HCC_1_GUARD_LEN                (2U)
#define CFAR_HCC_1_DIV_SHIFT                (3U)
#define CFAR_HCC_1_CYCLIC_MODE              (1U)
#define CFAR_HCC_1_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_1_PEAK_GROUPING            (0U)

#define CFAR_HCC_2_SUBFRAME_IDX             (1U)
#define CFAR_HCC_2_PROC_DIRECTION           (0U)
#define CFAR_HCC_2_MODE                     (2U)
#define CFAR_HCC_2_NOISE_WIN                (8U)
#define CFAR_HCC_2_GUARD_LEN                (4U)
#define CFAR_HCC_2_DIV_SHIFT                (3U)
#define CFAR_HCC_2_CYCLIC_MODE              (0U)
#define CFAR_HCC_2_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_2_PEAK_GROUPING            (1U)

#define CFAR_HCC_3_SUBFRAME_IDX             (1U)
#define CFAR_HCC_3_PROC_DIRECTION           (1U)
#define CFAR_HCC_3_MODE                     (0U)
#define CFAR_HCC_3_NOISE_WIN                (4U)
#define CFAR_HCC_3_GUARD_LEN                (2U)
#define CFAR_HCC_3_DIV_SHIFT                (3U)
#define CFAR_HCC_3_CYCLIC_MODE              (1U)
#define CFAR_HCC_3_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_3_PEAK_GROUPING            (1U)

#define CFAR_HCC_4_SUBFRAME_IDX             (2U)
#define CFAR_HCC_4_PROC_DIRECTION           (0U)
#define CFAR_HCC_4_MODE                     (2U)
#define CFAR_HCC_4_NOISE_WIN                (8U)
#define CFAR_HCC_4_GUARD_LEN                (4U)
#define CFAR_HCC_4_DIV_SHIFT                (3U)
#define CFAR_HCC_4_CYCLIC_MODE              (0U)
#define CFAR_HCC_4_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_4_PEAK_GROUPING            (0U)

#define CFAR_HCC_5_SUBFRAME_IDX             (2U)
#define CFAR_HCC_5_PROC_DIRECTION           (1U)
#define CFAR_HCC_5_MODE                     (0U)
#define CFAR_HCC_5_NOISE_WIN                (4U)
#define CFAR_HCC_5_GUARD_LEN                (2U)
#define CFAR_HCC_5_DIV_SHIFT                (3U)
#define CFAR_HCC_5_CYCLIC_MODE              (1U)
#define CFAR_HCC_5_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_5_PEAK_GROUPING            (0U)

#define CFAR_HCC_6_SUBFRAME_IDX             (3U)
#define CFAR_HCC_6_PROC_DIRECTION           (0U)
#define CFAR_HCC_6_MODE                     (2U)
#define CFAR_HCC_6_NOISE_WIN                (8U)
#define CFAR_HCC_6_GUARD_LEN                (4U)
#define CFAR_HCC_6_DIV_SHIFT                (3U)
#define CFAR_HCC_6_CYCLIC_MODE              (0U)
#define CFAR_HCC_6_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_6_PEAK_GROUPING            (1U)

#define CFAR_HCC_7_SUBFRAME_IDX             (3U)
#define CFAR_HCC_7_PROC_DIRECTION           (1U)
#define CFAR_HCC_7_MODE                     (0U)
#define CFAR_HCC_7_NOISE_WIN                (4U)
#define CFAR_HCC_7_GUARD_LEN                (2U)
#define CFAR_HCC_7_DIV_SHIFT                (3U)
#define CFAR_HCC_7_CYCLIC_MODE              (1U)
#define CFAR_HCC_7_THRESHOLD_SCALE          (15.0f)
#define CFAR_HCC_7_PEAK_GROUPING            (1U)

#define MULTI_OBJ_BEAM_HCC_0_SUBFRAME_IDX   (0U)
#define MULTI_OBJ_BEAM_HCC_0_ENABLED        (1U)
#define MULTI_OBJ_BEAM_HCC_0_THRESHOLD      (0.5f)

#define MULTI_OBJ_BEAM_HCC_1_SUBFRAME_IDX   (1U)
#define MULTI_OBJ_BEAM_HCC_1_ENABLED        (1U)
#define MULTI_OBJ_BEAM_HCC_1_THRESHOLD      (0.6f)

#define MULTI_OBJ_BEAM_HCC_2_SUBFRAME_IDX   (2U)
#define MULTI_OBJ_BEAM_HCC_2_ENABLED        (0U)
#define MULTI_OBJ_BEAM_HCC_2_THRESHOLD      (0.4f)

#define MULTI_OBJ_BEAM_HCC_3_SUBFRAME_IDX   (3U)
#define MULTI_OBJ_BEAM_HCC_3_ENABLED        (1U)
#define MULTI_OBJ_BEAM_HCC_3_THRESHOLD      (0.7f)

#define CALIB_DC_RANGE_HCC_0_SUBFRAME_IDX   (0U)
#define CALIB_DC_RANGE_HCC_0_ENABLED        (0U)
#define CALIB_DC_RANGE_HCC_0_NEG_BIN_IDX    (-3)
#define CALIB_DC_RANGE_HCC_0_POS_BIN_IDX    (3U)
#define CALIB_DC_RANGE_HCC_0_NUM_AVG        (256U)

#define CALIB_DC_RANGE_HCC_1_SUBFRAME_IDX   (1U)
#define CALIB_DC_RANGE_HCC_1_ENABLED        (0U)
#define CALIB_DC_RANGE_HCC_1_NEG_BIN_IDX    (-2)
#define CALIB_DC_RANGE_HCC_1_POS_BIN_IDX    (2U)
#define CALIB_DC_RANGE_HCC_1_NUM_AVG        (256U)

#define CALIB_DC_RANGE_HCC_2_SUBFRAME_IDX   (2U)
#define CALIB_DC_RANGE_HCC_2_ENABLED        (0U)
#define CALIB_DC_RANGE_HCC_2_NEG_BIN_IDX    (-1)
#define CALIB_DC_RANGE_HCC_2_POS_BIN_IDX    (1U)
#define CALIB_DC_RANGE_HCC_2_NUM_AVG        (256U)

#define CALIB_DC_RANGE_HCC_3_SUBFRAME_IDX   (3U)
#define CALIB_DC_RANGE_HCC_3_ENABLED        (0U)
#define CALIB_DC_RANGE_HCC_3_NEG_BIN_IDX    (-1)
#define CALIB_DC_RANGE_HCC_3_POS_BIN_IDX    (1U)
#define CALIB_DC_RANGE_HCC_3_NUM_AVG        (256U)

#define CFAR_FOV_HCC_0_SUBFRAME_IDX         (0U)
#define CFAR_FOV_HCC_0_PROC_DIRECTION       (1U)
#define CFAR_FOV_HCC_0_MINIMUM              (-27.17f)
#define CFAR_FOV_HCC_0_MAXIMUM              (27.17f)

#define CFAR_FOV_HCC_1_SUBFRAME_IDX         (0U)
#define CFAR_FOV_HCC_1_PROC_DIRECTION       (0U)
#define CFAR_FOV_HCC_1_MINIMUM              (0.25f)
#define CFAR_FOV_HCC_1_MAXIMUM              (4.4f)

#define CFAR_FOV_HCC_2_SUBFRAME_IDX         (1U)
#define CFAR_FOV_HCC_2_PROC_DIRECTION       (1U)
#define CFAR_FOV_HCC_2_MINIMUM              (-19.53f)
#define CFAR_FOV_HCC_2_MAXIMUM              (19.53f)

#define CFAR_FOV_HCC_3_SUBFRAME_IDX         (1U)
#define CFAR_FOV_HCC_3_PROC_DIRECTION       (0U)
#define CFAR_FOV_HCC_3_MINIMUM              (4.4f)
#define CFAR_FOV_HCC_3_MAXIMUM              (9.6f)

#define CFAR_FOV_HCC_4_SUBFRAME_IDX         (2U)
#define CFAR_FOV_HCC_4_PROC_DIRECTION       (1U)
#define CFAR_FOV_HCC_4_MINIMUM              (-20.16f)
#define CFAR_FOV_HCC_4_MAXIMUM              (20.16f)

#define CFAR_FOV_HCC_5_SUBFRAME_IDX         (2U)
#define CFAR_FOV_HCC_5_PROC_DIRECTION       (0U)
#define CFAR_FOV_HCC_5_MINIMUM              (9.6f)
#define CFAR_FOV_HCC_5_MAXIMUM              (27.27f)

#define CFAR_FOV_HCC_6_SUBFRAME_IDX         (3U)
#define CFAR_FOV_HCC_6_PROC_DIRECTION       (1U)
#define CFAR_FOV_HCC_6_MINIMUM              (-12.25f)
#define CFAR_FOV_HCC_6_MAXIMUM              (12.25f)

#define CFAR_FOV_HCC_7_SUBFRAME_IDX         (3U)
#define CFAR_FOV_HCC_7_PROC_DIRECTION       (0U)
#define CFAR_FOV_HCC_7_MINIMUM              (27.27f)
#define CFAR_FOV_HCC_7_MAXIMUM              (50.0f)

#define MEASURERANGEBIAS_HCC_ENABLED        (0U)
#define MEASURERANGEBIAS_HCC_TARGET_DIST    (1.0f)
#define MEASURERANGEBIAS_HCC_SEARCH_WIN     (0.2f)

#define CQRXSAT_HCC_0_PROFILE_ID            (0U)
#define CQRXSAT_HCC_0_SAT_MON_SEL           (3U)
#define CQRXSAT_HCC_0_PRI_SLICE_DURATION    (4U)
#define CQRXSAT_HCC_0_NUM_SLICES            (43U)
#define CQRXSAT_HCC_0_RX_CHAN_MASK          (0U)

#define CQSIGIMG_HCC_0_PROFILE_ID           (0U)
#define CQSIGIMG_HCC_0_NUM_SLICES           (63U)
#define CQSIGIMG_HCC_0_NUM_SAMPLE_PER_SLICE (4U)

#define CQRXSAT_HCC_1_PROFILE_ID            (1U)
#define CQRXSAT_HCC_1_SAT_MON_SEL           (3U)
#define CQRXSAT_HCC_1_PRI_SLICE_DURATION    (4U)
#define CQRXSAT_HCC_1_NUM_SLICES            (67U)
#define CQRXSAT_HCC_1_RX_CHAN_MASK          (0U)

#define CQSIGIMG_HCC_1_PROFILE_ID           (1U)
#define CQSIGIMG_HCC_1_NUM_SLICES           (127U)
#define CQSIGIMG_HCC_1_NUM_SAMPLE_PER_SLICE (4U)

#define CQRXSAT_HCC_2_PROFILE_ID            (2U)
#define CQRXSAT_HCC_2_SAT_MON_SEL           (3U)
#define CQRXSAT_HCC_2_PRI_SLICE_DURATION    (4U)
#define CQRXSAT_HCC_2_NUM_SLICES            (63U)
#define CQRXSAT_HCC_2_RX_CHAN_MASK          (0U)

#define CQSIGIMG_HCC_2_PROFILE_ID           (2U)
#define CQSIGIMG_HCC_2_NUM_SLICES           (127U)
#define CQSIGIMG_HCC_2_NUM_SAMPLE_PER_SLICE (4U)

#define CQRXSAT_HCC_3_PROFILE_ID            (3U)
#define CQRXSAT_HCC_3_SAT_MON_SEL           (3U)
#define CQRXSAT_HCC_3_PRI_SLICE_DURATION    (4U)
#define CQRXSAT_HCC_3_NUM_SLICES            (127U)
#define CQRXSAT_HCC_3_RX_CHAN_MASK          (0U)

#define CQSIGIMG_HCC_3_PROFILE_ID           (3U)
#define CQSIGIMG_HCC_3_NUM_SLICES           (127U)
#define CQSIGIMG_HCC_3_NUM_SAMPLE_PER_SLICE (8U)

#define BPM_HCC_SUBFRAME_IDX                (-1)
#define BPM_HCC_ENABLED                     (0U)
#define BPM_HCC_CHIRP_0_IDX                 (0U)
#define BPM_HCC_CHIRP_1_IDX                 (0U)
