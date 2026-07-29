#ifndef ALOG_SIGNAL_H
#define ALOG_SIGNAL_H

#include <stdbool.h>
#include <stdint.h>

#define ALOG_SAMPLE_RATE_HZ  2000000U
#define ALOG_FFT_SIZE        4096U
#define ALOG_HALF_SIZE       (ALOG_FFT_SIZE / 2U)
#define ALOG_SPECTRUM_SIZE   (ALOG_HALF_SIZE + 1U)
#define ALOG_ADC_MASK        0x0FFFU
#define ALOG_MAX_COMPONENTS  3U

#define ALOG_FREQ_MIN_HZ     10000.0f
#define ALOG_FREQ_MAX_HZ     500000.0f

typedef struct
{
    float freq_hz;      /* 插值频率，单位Hz */
    float amp_code;     /* ADC侧正弦峰值，单位code */
    float bin_index;    /* 插值后的浮点频点 */
    uint8_t harmonic;   /* 基波为1，无法确认时为0 */
} alog_component_t;

typedef struct
{
    bool valid;

    float dc_code;      /* 原始ADC码平均值 */
    float min_code;     /* 原始ADC码最小值 */
    float max_code;     /* 原始ADC码最大值 */

    /*
     * 当前p2p_code直接由采样点max-min得到。500kHz、2MSPS时每周期
     * 约4个点，可能错过真实峰值；这里只作为第一版ADC侧时域结果。
     * 后续可通过频率、幅值和相位重构后再计算，当前不实现波形重构。
     */
    float p2p_code;
    float rms_code;     /* 去直流后的ADC侧RMS */

    float fundamental_hz;

    uint8_t component_count;
    alog_component_t component[ALOG_MAX_COMPONENTS];
} alog_result_t;

bool alog_signal_init(void);

bool alog_signal_analyze(const uint32_t *raw,
                         uint32_t count,
                         alog_result_t *result);

bool alog_signal_analyze_f32(const float *samples,
                             uint32_t count,
                             alog_result_t *result);

uint32_t alog_signal_get_sample_rate(void);
float alog_signal_get_bin_hz(void);

#endif /* ALOG_SIGNAL_H */
