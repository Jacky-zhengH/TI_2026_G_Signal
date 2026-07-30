#ifndef ALOG_SIGNAL_H
#define ALOG_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

#define ALOG_SAMPLE_RATE_HZ  2000000.0f
#define ALOG_FFT_SIZE        4096U
#define ALOG_SPECTRUM_SIZE   (ALOG_FFT_SIZE / 2U)
#define ALOG_BIN_HZ          (ALOG_SAMPLE_RATE_HZ / ALOG_FFT_SIZE)

#define ALOG_MAX_COMP        3U
#define ALOG_MAX_CANDIDATE   8U
#define ALOG_CAL_MAX         16U
#define ALOG_ADC_MASK        0x0FFFU

#define ALOG_FREQ_MIN_HZ     10000.0f
#define ALOG_FREQ_MAX_HZ     500000.0f

typedef struct
{
    float freq_hz;

    float amp_code;
    float rms_code;

    float amp_mv;
    float rms_mv;

    float phase_rad;
    float phase_in_rad;

    uint16_t bin;
    uint8_t harmonic;
} signal_comp_t;

typedef struct
{
    bool valid;
    bool calibrated;

    float dc_code;
    float min_code;
    float max_code;
    float raw_pp_code;
    float time_rms_code;

    float fundamental_hz;

    float urms_code;
    float upp_code;

    float urms_mv;
    float upp_mv;

    float noise_code;
    float fit_error_code;

    uint8_t comp_count;
    signal_comp_t comp[ALOG_MAX_COMP];
} signal_result_t;

typedef struct
{
    uint8_t count;
    bool phase_valid;

    float freq_hz[ALOG_CAL_MAX];
    float mv_per_code[ALOG_CAL_MAX];
    float phase_corr_rad[ALOG_CAL_MAX];
} alog_cal_t;

bool alog_init(void);
bool alog_set_cal(const alog_cal_t *cal);
void alog_clear_cal(void);

bool alog_analyze(const uint32_t *raw,
                  uint32_t count,
                  signal_result_t *result);

const float *alog_get_spectrum(uint32_t *count);

bool alog_make_wave(const signal_result_t *result,
                    uint8_t cycles,
                    float *out,
                    uint32_t count);

#endif /* ALOG_SIGNAL_H */
