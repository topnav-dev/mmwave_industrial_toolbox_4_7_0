#ifndef HC_CONFIG_DEFS_H
#define HC_CONFIG_DEFS_H
#include <ti/common/sys_common.h>

// #ifndef PROFILE_CALIBRATION
// #define PROFILE_CALIBRATION
// #endif

// #ifndef PROFILE_ADVANCED_SUBFRAME
// #define PROFILE_ADVANCED_SUBFRAME
// #endif

// #ifndef PROFILE_2d_BPM
// #define PROFILE_2d_BPM
// #endif

// #ifndef PROFILE_2d
// #define PROFILE_2d
// #endif


#define CHANNEL_HCC_CASCADING               (0U)

#define ADC_HCC_NUM_ADC_BITS                (2U)
#define ADC_HCC_OUTPUT_FMT                  (1U)

#define ADCBUF_HCC_SUBFRAME_IDX             (-1)
#define ADCBUF_HCC_OUTPUT_FMT               (0U)
#define ADCBUF_HCC_SAMPLE_SWAP              (1U)
#define ADCBUF_HCC_CHAN_INTERLEAVE          (1U)
#define ADCBUF_HCC_CHIRP_THRESHOLD          (1U)

#define LP_HCC_DONT_CARE                    (0U)
#define LP_HCC_LOW_POWER_MODE               (0U)

#define CLUTTER_HCC_SUBFRAME_IDX            (-1)
#define CLUTTER_HCC_ENABLED                 (0)

#define AOAFOV_HCC_SUBFRAME_IDX             (-1)
#define AOAFOV_HCC_MIN_AZIMUTH_DEG          (-90)
#define AOAFOV_HCC_MAX_AZIMUTH_DEG          (90U)
#define AOAFOV_HCC_MIN_ELEVATION_DEG        (-90)
#define AOAFOV_HCC_MAX_ELEVATION_DEG        (90U)

#define COMPRANGEBIASANDRXCHANPHASE         (0.0f)

#define EXTENDMAXVELOCITY_HCC_SUBFRAME_IDX  (-1)
#define EXTENDMAXVELOCITY_HCC_ENABLED       (0U)

#define ANALOGMONITOR_HCC_RX_SATURATION     (0U)
#define ANALOGMONITOR_HCC_SIG_IMG_BAND      (0U)

#define LVDSSTREAM_HCC_SUBFRAME_IDX         (-1)
#define LVDSSTREAM_HCC_ENABLE_HEADER        (0U)
#define LVDSSTREAM_HCC_DATA_FMT             (0U)
#define LVDSSTREAM_HCC_ENABLE_SW            (0U)

#if defined(PROFILE_CALIBRATION)
    #include "../mss/hc_config_profile_calibration.h"
#elif defined(PROFILE_ADVANCED_SUBFRAME)
    #include "../mss/hc_config_profile_advanced_subframe.h"
#elif defined(PROFILE_2d_BPM)
    #include "../mss/hc_config_profile_2d_bpm.h"
#elif defined(PROFILE_2d)
    #include "../mss/hc_config_profile_2d.h"
#else
    #define PROFILE_3d
    #include "../mss/hc_config_profile_3d.h"
#endif

#endif
