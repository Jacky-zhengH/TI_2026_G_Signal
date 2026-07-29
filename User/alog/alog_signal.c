#include "alog_signal.h"

#include "arm_math.h"

#include <math.h>
#include <string.h>

#define ALOG_TWO_PI          6.28318530717958647692f
#define ALOG_PEAK_REL_MIN     0.02f
#define ALOG_PEAK_GUARD_BINS  3U
#define ALOG_INTERP_EPSILON   1.0e-12f
#define ALOG_INV_SQRT_TWO     0.70710678f

static float hann_window[ALOG_FFT_SIZE];
static float fft_input[ALOG_FFT_SIZE];
static float fft_output[ALOG_FFT_SIZE];
static float spectrum[ALOG_SPECTRUM_SIZE];

static arm_rfft_fast_instance_f32 fft_instance;
static float window_sum;
static bool alog_ready;

static void clear_result(signal_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

static void calc_time(const uint32_t *raw, signal_result_t *result)
{
    uint32_t i;
    float code;
    float diff;
    float sum = 0.0f;
    float sum_sq = 0.0f;
    float min_code = (float)(raw[0] & ALOG_ADC_MASK);
    float max_code = min_code;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        code = (float)(raw[i] & ALOG_ADC_MASK);
        sum += code;

        if (code < min_code)
        {
            min_code = code;
        }
        if (code > max_code)
        {
            max_code = code;
        }
    }

    result->dc_code = sum / (float)ALOG_FFT_SIZE;
    result->min_code = min_code;
    result->max_code = max_code;
    result->raw_pp_code = max_code - min_code;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        code = (float)(raw[i] & ALOG_ADC_MASK);
        diff = code - result->dc_code;
        sum_sq += diff * diff;
    }

    result->time_rms_code =
        sqrtf(sum_sq / (float)ALOG_FFT_SIZE);
}

static void make_fft_input(const uint32_t *raw, float dc_code)
{
    uint32_t i;
    float code;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        code = (float)(raw[i] & ALOG_ADC_MASK);
        fft_input[i] = (code - dc_code) * hann_window[i];
    }
}

static void calc_spectrum(void)
{
    uint32_t k;
    float real;
    float imag;

    arm_rfft_fast_f32(&fft_instance,
                      fft_input,
                      fft_output,
                      0U);

    /*
     * 快速RFFT打包格式：
     * output[0]为DC，output[1]为Nyquist；
     * k=1..N/2-1的实部和虚部分别位于2k和2k+1。
     */
    spectrum[0] = fabsf(fft_output[0]);
    for (k = 1U; k < ALOG_SPECTRUM_SIZE; k++)
    {
        real = fft_output[2U * k];
        imag = fft_output[2U * k + 1U];
        spectrum[k] = sqrtf(real * real + imag * imag);
    }
}

static float interp_peak(uint16_t bin, float *peak_mag)
{
    float left = spectrum[bin - 1U];
    float center = spectrum[bin];
    float right = spectrum[bin + 1U];
    float denom = left - 2.0f * center + right;
    float delta = 0.0f;
    float interp_mag;

    if (fabsf(denom) > ALOG_INTERP_EPSILON)
    {
        delta = 0.5f * (left - right) / denom;
        if (delta > 0.5f)
        {
            delta = 0.5f;
        }
        else if (delta < -0.5f)
        {
            delta = -0.5f;
        }
    }

    interp_mag = center - 0.25f * (left - right) * delta;
    if (interp_mag < 0.0f)
    {
        interp_mag = center;
    }

    *peak_mag = interp_mag;
    return ((float)bin + delta) * ALOG_BIN_HZ;
}

static uint8_t find_peaks(signal_comp_t *comp)
{
    uint16_t selected_bin[ALOG_MAX_COMP];
    uint32_t start_bin;
    uint32_t end_bin;
    uint32_t i;
    uint32_t j;
    uint32_t best_bin;
    uint32_t distance;
    uint8_t count = 0U;
    bool guarded;
    float strongest = 0.0f;
    float threshold;
    float center;
    float best_mag;
    float peak_mag;

    start_bin = (uint32_t)ceilf(ALOG_FREQ_MIN_HZ / ALOG_BIN_HZ);
    end_bin = (uint32_t)floorf(ALOG_FREQ_MAX_HZ / ALOG_BIN_HZ);

    if (start_bin < 1U)
    {
        start_bin = 1U;
    }
    if (end_bin >= (ALOG_SPECTRUM_SIZE - 1U))
    {
        end_bin = ALOG_SPECTRUM_SIZE - 2U;
    }
    if (start_bin > end_bin)
    {
        return 0U;
    }

    for (i = start_bin; i <= end_bin; i++)
    {
        center = spectrum[i];
        if ((center > spectrum[i - 1U]) &&
            (center >= spectrum[i + 1U]) &&
            (center > strongest))
        {
            strongest = center;
        }
    }

    if (strongest <= 0.0f)
    {
        return 0U;
    }

    /*
     * 当前2%阈值用于无硬件模拟测试，
     * AD9220到货后需根据实际底噪调整。
     */
    threshold = strongest * ALOG_PEAK_REL_MIN;

    while (count < ALOG_MAX_COMP)
    {
        best_bin = 0U;
        best_mag = 0.0f;

        for (i = start_bin; i <= end_bin; i++)
        {
            center = spectrum[i];
            if ((center < threshold) ||
                (center <= spectrum[i - 1U]) ||
                (center < spectrum[i + 1U]))
            {
                continue;
            }

            guarded = false;
            for (j = 0U; j < count; j++)
            {
                if (i > selected_bin[j])
                {
                    distance = i - selected_bin[j];
                }
                else
                {
                    distance = selected_bin[j] - i;
                }

                if (distance <= ALOG_PEAK_GUARD_BINS)
                {
                    guarded = true;
                    break;
                }
            }

            if ((!guarded) && (center > best_mag))
            {
                best_mag = center;
                best_bin = i;
            }
        }

        if (best_bin == 0U)
        {
            break;
        }

        selected_bin[count] = (uint16_t)best_bin;
        comp[count].freq_hz =
            interp_peak((uint16_t)best_bin, &peak_mag);
        comp[count].amp_code = 2.0f * peak_mag / window_sum;
        comp[count].rms_code =
            comp[count].amp_code * ALOG_INV_SQRT_TWO;
        comp[count].bin = (uint16_t)best_bin;
        comp[count].harmonic = 0U;
        count++;
    }

    return count;
}

static void sort_comp(signal_comp_t *comp, uint8_t count)
{
    uint32_t i;
    uint32_t j;
    signal_comp_t temp;

    for (i = 0U; i < count; i++)
    {
        for (j = i + 1U; j < count; j++)
        {
            if (comp[j].freq_hz < comp[i].freq_hz)
            {
                temp = comp[i];
                comp[i] = comp[j];
                comp[j] = temp;
            }
        }
    }
}

static void find_harmonics(signal_result_t *result)
{
    uint32_t i;
    uint32_t order;
    float ratio;
    float error_hz;
    float sum_rms_sq = 0.0f;

    if (result->comp_count == 0U)
    {
        return;
    }

    result->fundamental_hz = result->comp[0].freq_hz;
    result->comp[0].harmonic = 1U;

    for (i = 0U; i < result->comp_count; i++)
    {
        sum_rms_sq +=
            result->comp[i].rms_code * result->comp[i].rms_code;

        if (i == 0U)
        {
            continue;
        }

        ratio = result->comp[i].freq_hz / result->fundamental_hz;
        order = (uint32_t)roundf(ratio);
        error_hz = fabsf(result->comp[i].freq_hz -
                         (float)order * result->fundamental_hz);

        if ((order >= 2U) &&
            (order <= UINT8_MAX) &&
            (error_hz <= 2.0f * ALOG_BIN_HZ))
        {
            result->comp[i].harmonic = (uint8_t)order;
        }
    }

    result->spec_rms_code = sqrtf(sum_rms_sq);
    result->valid = true;
}

bool alog_init(void)
{
    uint32_t i;
    float phase;

    if (alog_ready)
    {
        return true;
    }

    if (arm_rfft_fast_init_f32(&fft_instance,
                               (uint16_t)ALOG_FFT_SIZE) !=
        ARM_MATH_SUCCESS)
    {
        return false;
    }

    window_sum = 0.0f;
    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        phase = ALOG_TWO_PI * (float)i /
                (float)(ALOG_FFT_SIZE - 1U);
        hann_window[i] = 0.5f - 0.5f * arm_cos_f32(phase);
        window_sum += hann_window[i];
    }

    memset(fft_input, 0, sizeof(fft_input));
    memset(fft_output, 0, sizeof(fft_output));
    memset(spectrum, 0, sizeof(spectrum));

    alog_ready = true;
    return true;
}

bool alog_analyze(const uint32_t *raw,
                  uint32_t count,
                  signal_result_t *result)
{
    if (result == NULL)
    {
        return false;
    }

    clear_result(result);
    if ((!alog_ready) ||
        (raw == NULL) ||
        (count != ALOG_FFT_SIZE))
    {
        return false;
    }

    calc_time(raw, result);
    make_fft_input(raw, result->dc_code);
    calc_spectrum();

    result->comp_count = find_peaks(result->comp);
    if (result->comp_count == 0U)
    {
        return true;
    }

    sort_comp(result->comp, result->comp_count);
    find_harmonics(result);
    return true;
}

const float *alog_get_spectrum(uint32_t *count)
{
    if (count != NULL)
    {
        *count = ALOG_SPECTRUM_SIZE;
    }

    return spectrum;
}
