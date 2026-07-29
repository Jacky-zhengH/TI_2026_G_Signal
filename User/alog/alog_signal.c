#include "alog_signal.h"

#include "arm_math.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define ALOG_TWO_PI             6.28318530717958647692f
#define ALOG_PEAK_RELATIVE_MIN  0.01f
#define ALOG_NOISE_MULTIPLE     6.0f
#define ALOG_PEAK_GUARD_BINS    3U
#define ALOG_INTERP_DEN_MIN     1.0e-12f

static arm_rfft_fast_instance_f32 fft_instance;

static float sample_buf[ALOG_FFT_SIZE];
static float fft_buf[ALOG_FFT_SIZE];
static float mag_buf[ALOG_SPECTRUM_SIZE];
static float hann_window[ALOG_FFT_SIZE];

static float hann_gain;
static bool alog_initialized;

static void alog_clear_result(alog_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

static void alog_prepare_raw(const uint32_t *raw, alog_result_t *result)
{
    uint32_t i;
    float code;
    float sum = 0.0f;
    float square_sum = 0.0f;
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
    result->p2p_code = max_code - min_code;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        code = (float)(raw[i] & ALOG_ADC_MASK) - result->dc_code;
        sample_buf[i] = code;
        square_sum += code * code;
    }

    (void)arm_sqrt_f32(square_sum / (float)ALOG_FFT_SIZE,
                       &result->rms_code);
}

static void alog_prepare_f32(const float *samples, alog_result_t *result)
{
    uint32_t i;
    float value;
    float sum = 0.0f;
    float square_sum = 0.0f;
    float min_code = samples[0];
    float max_code = samples[0];

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        value = samples[i];
        sum += value;

        if (value < min_code)
        {
            min_code = value;
        }
        if (value > max_code)
        {
            max_code = value;
        }
    }

    result->dc_code = sum / (float)ALOG_FFT_SIZE;
    result->min_code = min_code;
    result->max_code = max_code;
    result->p2p_code = max_code - min_code;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        value = samples[i] - result->dc_code;
        sample_buf[i] = value;
        square_sum += value * value;
    }

    (void)arm_sqrt_f32(square_sum / (float)ALOG_FFT_SIZE,
                       &result->rms_code);
}

static void alog_make_spectrum(void)
{
    uint32_t i;
    float real;
    float imag;
    float magnitude;
    float scale;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        sample_buf[i] *= hann_window[i];
    }

    arm_rfft_fast_f32(&fft_instance, sample_buf, fft_buf, 0U);

    scale = 1.0f / ((float)ALOG_FFT_SIZE * hann_gain);
    /* 快速RFFT中fft_buf[0]为DC，fft_buf[1]为Nyquist。 */
    mag_buf[0] = fabsf(fft_buf[0]) * scale;
    mag_buf[ALOG_HALF_SIZE] = fabsf(fft_buf[1]) * scale;

    for (i = 1U; i < ALOG_HALF_SIZE; i++)
    {
        real = fft_buf[2U * i];
        imag = fft_buf[2U * i + 1U];
        (void)arm_sqrt_f32(real * real + imag * imag, &magnitude);
        mag_buf[i] = 2.0f * magnitude * scale;
    }
}

static uint8_t alog_find_peaks(alog_result_t *result)
{
    uint16_t selected_bin[ALOG_MAX_COMPONENTS];
    uint32_t start_bin;
    uint32_t end_bin;
    uint32_t search_count;
    uint32_t i;
    uint32_t j;
    uint32_t best_bin;
    uint8_t selected_count = 0U;
    bool guarded;
    float bin_hz = alog_signal_get_bin_hz();
    float strongest_peak = 0.0f;
    float mean_mag = 0.0f;
    float threshold;
    float best_amp;
    float left;
    float center;
    float right;
    float den;
    float delta;
    float amp_interp;

    start_bin = (uint32_t)ceilf(ALOG_FREQ_MIN_HZ / bin_hz);
    end_bin = (uint32_t)floorf(ALOG_FREQ_MAX_HZ / bin_hz);

    if (start_bin < 1U)
    {
        start_bin = 1U;
    }
    if (end_bin >= ALOG_HALF_SIZE)
    {
        end_bin = ALOG_HALF_SIZE - 1U;
    }
    if (start_bin > end_bin)
    {
        return 0U;
    }

    search_count = end_bin - start_bin + 1U;
    for (i = start_bin; i <= end_bin; i++)
    {
        mean_mag += mag_buf[i];
        if (mag_buf[i] > strongest_peak)
        {
            strongest_peak = mag_buf[i];
        }
    }
    mean_mag /= (float)search_count;

    threshold = strongest_peak * ALOG_PEAK_RELATIVE_MIN;
    if (threshold < mean_mag * ALOG_NOISE_MULTIPLE)
    {
        threshold = mean_mag * ALOG_NOISE_MULTIPLE;
    }

    while (selected_count < ALOG_MAX_COMPONENTS)
    {
        best_bin = 0U;
        best_amp = 0.0f;

        for (i = start_bin; i <= end_bin; i++)
        {
            center = mag_buf[i];
            if ((center < threshold) ||
                (center <= mag_buf[i - 1U]) ||
                (center < mag_buf[i + 1U]))
            {
                continue;
            }

            guarded = false;
            for (j = 0U; j < selected_count; j++)
            {
                if ((i > selected_bin[j] ?
                     i - selected_bin[j] : selected_bin[j] - i) <
                    ALOG_PEAK_GUARD_BINS)
                {
                    guarded = true;
                    break;
                }
            }
            if (!guarded && (center > best_amp))
            {
                best_amp = center;
                best_bin = i;
            }
        }

        if (best_bin == 0U)
        {
            break;
        }

        selected_bin[selected_count] = (uint16_t)best_bin;
        left = mag_buf[best_bin - 1U];
        center = mag_buf[best_bin];
        right = mag_buf[best_bin + 1U];
        den = left - 2.0f * center + right;
        delta = 0.0f;

        if (fabsf(den) > ALOG_INTERP_DEN_MIN)
        {
            delta = 0.5f * (left - right) / den;
            if (delta > 0.5f)
            {
                delta = 0.5f;
            }
            else if (delta < -0.5f)
            {
                delta = -0.5f;
            }
        }

        amp_interp = center - 0.25f * (left - right) * delta;
        if ((amp_interp < 0.0f) ||
            (amp_interp != amp_interp) ||
            (amp_interp > FLT_MAX))
        {
            amp_interp = center;
        }

        result->component[selected_count].bin_index =
            (float)best_bin + delta;
        result->component[selected_count].freq_hz =
            result->component[selected_count].bin_index * bin_hz;
        result->component[selected_count].amp_code = amp_interp;
        result->component[selected_count].harmonic = 0U;
        selected_count++;
    }

    return selected_count;
}

static void alog_sort_components(alog_result_t *result)
{
    uint32_t i;
    uint32_t j;
    alog_component_t temp;

    for (i = 0U; i < result->component_count; i++)
    {
        for (j = i + 1U; j < result->component_count; j++)
        {
            if (result->component[j].freq_hz <
                result->component[i].freq_hz)
            {
                temp = result->component[i];
                result->component[i] = result->component[j];
                result->component[j] = temp;
            }
        }
    }
}

static void alog_mark_harmonics(alog_result_t *result)
{
    uint32_t i;
    uint32_t order;
    float f0;
    float ratio;
    float error_hz;
    float tolerance_hz = 2.0f * alog_signal_get_bin_hz();

    if (result->component_count == 0U)
    {
        return;
    }

    f0 = result->component[0].freq_hz;
    result->fundamental_hz = f0;
    result->component[0].harmonic = 1U;

    for (i = 1U; i < result->component_count; i++)
    {
        ratio = result->component[i].freq_hz / f0;
        order = (uint32_t)(ratio + 0.5f);
        error_hz = fabsf(result->component[i].freq_hz -
                         (float)order * f0);

        if ((order >= 2U) &&
            (order <= UINT8_MAX) &&
            (error_hz <= tolerance_hz))
        {
            result->component[i].harmonic = (uint8_t)order;
        }
    }
}

static bool alog_finish_analysis(alog_result_t *result)
{
    alog_make_spectrum();
    result->component_count = alog_find_peaks(result);

    if (result->component_count == 0U)
    {
        return true;
    }

    alog_sort_components(result);
    alog_mark_harmonics(result);
    result->valid = true;
    return true;
}

bool alog_signal_init(void)
{
    uint32_t i;
    float phase;
    float window_sum = 0.0f;

    if (alog_initialized)
    {
        return true;
    }

    if (arm_rfft_fast_init_f32(&fft_instance,
                               (uint16_t)ALOG_FFT_SIZE) != ARM_MATH_SUCCESS)
    {
        return false;
    }

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        phase = ALOG_TWO_PI * (float)i / (float)(ALOG_FFT_SIZE - 1U);
        hann_window[i] = 0.5f - 0.5f * arm_cos_f32(phase);
        window_sum += hann_window[i];
    }

    hann_gain = window_sum / (float)ALOG_FFT_SIZE;
    memset(sample_buf, 0, sizeof(sample_buf));
    memset(fft_buf, 0, sizeof(fft_buf));
    memset(mag_buf, 0, sizeof(mag_buf));

    alog_initialized = true;
    return true;
}

bool alog_signal_analyze(const uint32_t *raw,
                         uint32_t count,
                         alog_result_t *result)
{
    if (result == NULL)
    {
        return false;
    }

    alog_clear_result(result);
    if ((!alog_initialized) || (raw == NULL) ||
        (count != ALOG_FFT_SIZE))
    {
        return false;
    }

    alog_prepare_raw(raw, result);
    return alog_finish_analysis(result);
}

bool alog_signal_analyze_f32(const float *samples,
                             uint32_t count,
                             alog_result_t *result)
{
    if (result == NULL)
    {
        return false;
    }

    alog_clear_result(result);
    if ((!alog_initialized) || (samples == NULL) ||
        (count != ALOG_FFT_SIZE))
    {
        return false;
    }

    alog_prepare_f32(samples, result);
    return alog_finish_analysis(result);
}

uint32_t alog_signal_get_sample_rate(void)
{
    return ALOG_SAMPLE_RATE_HZ;
}

float alog_signal_get_bin_hz(void)
{
    return (float)ALOG_SAMPLE_RATE_HZ / (float)ALOG_FFT_SIZE;
}
