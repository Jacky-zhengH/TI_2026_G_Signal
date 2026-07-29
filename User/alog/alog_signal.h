#ifndef ALOG_SIGNAL_H
#define ALOG_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

#define ALOG_SAMPLE_RATE_HZ  2000000.0f
#define ALOG_FFT_SIZE        4096U
#define ALOG_SPECTRUM_SIZE   (ALOG_FFT_SIZE / 2U)
#define ALOG_MAX_COMP        3U
#define ALOG_ADC_MASK        0x0FFFU

#define ALOG_FREQ_MIN_HZ     10000.0f
#define ALOG_FREQ_MAX_HZ     500000.0f

#define ALOG_BIN_HZ \
    (ALOG_SAMPLE_RATE_HZ / (float)ALOG_FFT_SIZE)

typedef struct
{
    float freq_hz;       /* 插值后的频率，单位Hz */
    float amp_code;      /* 正弦峰值，单位ADC码 */
    float rms_code;      /* 正弦分量RMS，单位ADC码 */
    uint16_t bin;        /* 原始FFT整数峰值位置 */
    uint8_t harmonic;    /* 1为基波，0为未确认 */
} signal_comp_t;

typedef struct
{
    bool valid;

    float dc_code;
    float min_code;
    float max_code;
    float raw_pp_code;
    float time_rms_code;

    float fundamental_hz;
    float spec_rms_code;

    uint8_t comp_count;
    signal_comp_t comp[ALOG_MAX_COMP];
} signal_result_t;

bool alog_init(void);

bool alog_analyze(const uint32_t *raw,
                  uint32_t count,
                  signal_result_t *result);

const float *alog_get_spectrum(uint32_t *count);

#endif /* ALOG_SIGNAL_H */
