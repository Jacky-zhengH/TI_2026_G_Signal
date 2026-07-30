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
#define ALOG_ADC_MASK        0x0FFFU

#define ALOG_FREQ_MIN_HZ     10000.0f
#define ALOG_FREQ_MAX_HZ     500000.0f

typedef struct
{
    float freq_hz;

    /*
     * TODO_CAL: amp_code/rms_code是AD9220侧ADC码值。
     * 输入端mV必须等整条模拟链实测后再换算，当前不执行补偿。
     */
    float amp_code;
    float rms_code;

    /*
     * TODO_CAL: phase_rad仅用于ADC采样波形重构。
     * 最终输入端相位需要实测十二阶低通和模拟链相位后修正。
     */
    float phase_rad;

    uint16_t bin;
    uint8_t harmonic;
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

    /*
     * TODO_CAL: urms_code和upp_code均为ADC侧code域结果。
     * 当前禁止直接按理论18倍或AD9220理论满量程换算成mV。
     */
    float urms_code;
    float upp_code;

    float noise_code;

    uint8_t comp_count;
    signal_comp_t comp[ALOG_MAX_COMP];
} signal_result_t;

bool alog_init(void);

bool alog_analyze(const uint32_t *raw,
                  uint32_t count,
                  signal_result_t *result);

const float *alog_get_spectrum(uint32_t *count);

bool alog_make_wave(const signal_result_t *result,
                    uint8_t cycles,
                    float *out,
                    uint32_t count);

#endif /* ALOG_SIGNAL_H */
