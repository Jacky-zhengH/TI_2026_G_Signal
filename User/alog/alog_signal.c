#include "alog_signal.h"

#include "arm_math.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define ALOG_TWO_PI             6.28318530717958647692f
#define ALOG_INV_SQRT_TWO       0.70710678f
#define ALOG_NOISE_MULTIPLE     6.0f
#define ALOG_PEAK_REL_MIN       0.01f
#define ALOG_PEAK_GUARD_BINS    3U
#define ALOG_FREQ_TOL_BINS      2.0f
#define ALOG_HARMONIC_MAX       50U
#define ALOG_LOG_EPSILON        1.0e-12f
#define ALOG_OSC_RENORM_MASK    255U
#define ALOG_UPP_POINTS         512U
#define ALOG_CAL_COUNT          13U

typedef struct
{
    float freq_hz;
    float mag_code;
    uint16_t bin;
} peak_t;

/*
 * TEMP_CAL:
 * 当前系数来自100mVpp单正弦初步实测。
 * 已包含完整模拟链和AD9220的幅频影响。
 * 后续将使用多次平均和示波器实测输入更新。
 */
static const float cal_freq_hz[ALOG_CAL_COUNT] =
{
    10000.0f,
    25000.0f,
    50000.0f,
    100000.0f,
    200000.0f,
    250000.0f,
    300000.0f,
    350000.0f,
    400000.0f,
    425000.0f,
    450000.0f,
    475000.0f,
    500000.0f
};

static const float cal_mv_per_code[ALOG_CAL_COUNT] =
{
    0.136287f,
    0.136584f,
    0.136742f,
    0.137792f,
    0.139736f,
    0.139539f,
    0.138205f,
    0.134540f,
    0.128183f,
    0.125340f,
    0.123693f,
    0.125813f,
    0.135255f
};

static float hann_window[ALOG_FFT_SIZE];
static float fft_input[ALOG_FFT_SIZE];
static float fft_output[ALOG_FFT_SIZE];
static float spectrum[ALOG_SPECTRUM_SIZE];

static arm_rfft_fast_instance_f32 fft_instance;
static float window_sum;
static bool alog_ready;

static bool valid_float(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

static void clear_result(signal_result_t *result)
{
    memset(result, 0, sizeof(*result));
}

static float cal_gain(float freq_hz)
{
    uint32_t i;

    if (freq_hz <= cal_freq_hz[0])
    {
        return cal_mv_per_code[0];
    }
    if (freq_hz >= cal_freq_hz[ALOG_CAL_COUNT - 1U])
    {
        return cal_mv_per_code[ALOG_CAL_COUNT - 1U];
    }

    for (i = 0U; i < (ALOG_CAL_COUNT - 1U); i++)
    {
        if (freq_hz <= cal_freq_hz[i + 1U])
        {
            return cal_mv_per_code[i] +
                   (freq_hz - cal_freq_hz[i]) *
                   (cal_mv_per_code[i + 1U] -
                    cal_mv_per_code[i]) /
                   (cal_freq_hz[i + 1U] -
                    cal_freq_hz[i]);
        }
    }

    return cal_mv_per_code[ALOG_CAL_COUNT - 1U];
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
        diff = (float)(raw[i] & ALOG_ADC_MASK) -
               result->dc_code;
        sum_sq += diff * diff;
    }

    result->time_rms_code =
        sqrtf(sum_sq / (float)ALOG_FFT_SIZE);
}

static void calc_spectrum(const uint32_t *raw, float dc_code)
{
    uint32_t i;
    uint32_t k;
    float real;
    float imag;

    for (i = 0U; i < ALOG_FFT_SIZE; i++)
    {
        fft_input[i] =
            ((float)(raw[i] & ALOG_ADC_MASK) - dc_code) *
            hann_window[i];
    }

    arm_rfft_fast_f32(&fft_instance,
                      fft_input,
                      fft_output,
                      0U);

    spectrum[0] = 0.0f;
    for (k = 1U; k < ALOG_SPECTRUM_SIZE; k++)
    {
        real = fft_output[2U * k];
        imag = fft_output[2U * k + 1U];
        spectrum[k] =
            2.0f * sqrtf(real * real + imag * imag) /
            window_sum;
    }
}

static float interp_freq(uint16_t bin)
{
    float left;
    float center;
    float right;
    float den;
    float delta = 0.0f;

    left = logf(spectrum[bin - 1U] + ALOG_LOG_EPSILON);
    center = logf(spectrum[bin] + ALOG_LOG_EPSILON);
    right = logf(spectrum[bin + 1U] + ALOG_LOG_EPSILON);
    den = left - 2.0f * center + right;

    if (fabsf(den) > ALOG_LOG_EPSILON)
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

    return ((float)bin + delta) * ALOG_BIN_HZ;
}

static uint8_t find_peaks(peak_t *peak, float *noise_code)
{
    uint16_t selected_bin[ALOG_MAX_CANDIDATE];
    uint32_t start_bin;
    uint32_t end_bin;
    uint32_t span;
    uint32_t i;
    uint32_t j;
    uint32_t best_bin;
    uint32_t distance;
    uint8_t count = 0U;
    bool guarded;
    float sum = 0.0f;
    float strongest = 0.0f;
    float threshold;
    float best_mag;

    start_bin = (uint32_t)floorf(ALOG_FREQ_MIN_HZ /
                                 ALOG_BIN_HZ);
    end_bin = (uint32_t)floorf(ALOG_FREQ_MAX_HZ /
                               ALOG_BIN_HZ);
    if (end_bin >= (ALOG_SPECTRUM_SIZE - 1U))
    {
        end_bin = ALOG_SPECTRUM_SIZE - 2U;
    }

    span = end_bin - start_bin + 1U;
    for (i = start_bin; i <= end_bin; i++)
    {
        sum += spectrum[i];
        if (spectrum[i] > strongest)
        {
            strongest = spectrum[i];
        }
    }

    *noise_code = sum / (float)span;
    threshold = ALOG_NOISE_MULTIPLE * (*noise_code);
    if (threshold < ALOG_PEAK_REL_MIN * strongest)
    {
        threshold = ALOG_PEAK_REL_MIN * strongest;
    }

    while (count < ALOG_MAX_CANDIDATE)
    {
        best_bin = 0U;
        best_mag = 0.0f;

        for (i = start_bin; i <= end_bin; i++)
        {
            if ((spectrum[i] < threshold) ||
                (spectrum[i] <= spectrum[i - 1U]) ||
                (spectrum[i] < spectrum[i + 1U]))
            {
                continue;
            }

            guarded = false;
            for (j = 0U; j < count; j++)
            {
                distance = (i > selected_bin[j]) ?
                           (i - selected_bin[j]) :
                           (selected_bin[j] - i);
                if (distance <= ALOG_PEAK_GUARD_BINS)
                {
                    guarded = true;
                    break;
                }
            }

            if ((!guarded) && (spectrum[i] > best_mag))
            {
                best_mag = spectrum[i];
                best_bin = i;
            }
        }

        if (best_bin == 0U)
        {
            break;
        }

        selected_bin[count] = (uint16_t)best_bin;
        peak[count].bin = (uint16_t)best_bin;
        peak[count].mag_code = spectrum[best_bin];
        peak[count].freq_hz =
            interp_freq((uint16_t)best_bin);
        count++;
    }

    return count;
}

static uint8_t select_model(const peak_t *peak,
                            uint8_t peak_count,
                            peak_t *selected,
                            uint8_t *harmonic)
{
    uint32_t base;
    uint32_t i;
    uint32_t j;
    uint32_t order;
    uint32_t top_count;
    uint32_t insert;
    uint32_t match_count;
    uint32_t best_count = 0U;
    uint32_t best_base = 0U;
    uint8_t selected_count = 0U;
    float ratio;
    float error_hz;
    float base_energy;
    float candidate_energy;
    float energy;
    float best_energy = 0.0f;
    float best_base_energy = 0.0f;
    float top_energy[ALOG_MAX_COMP - 1U];
    peak_t temp_peak;
    uint8_t temp_harmonic;

    for (base = 0U; base < peak_count; base++)
    {
        top_count = 0U;
        base_energy =
            peak[base].mag_code * peak[base].mag_code;

        for (i = 0U; i < peak_count; i++)
        {
            if (i == base)
            {
                continue;
            }

            ratio = peak[i].freq_hz / peak[base].freq_hz;
            order = (uint32_t)roundf(ratio);
            error_hz = fabsf(peak[i].freq_hz -
                             (float)order *
                             peak[base].freq_hz);
            if ((order < 2U) ||
                (order > ALOG_HARMONIC_MAX) ||
                (error_hz > ALOG_FREQ_TOL_BINS *
                            ALOG_BIN_HZ))
            {
                continue;
            }

            candidate_energy =
                peak[i].mag_code * peak[i].mag_code;
            if (top_count < (ALOG_MAX_COMP - 1U))
            {
                insert = top_count;
                top_count++;
            }
            else if (candidate_energy <
                     top_energy[ALOG_MAX_COMP - 2U])
            {
                continue;
            }
            else
            {
                insert = ALOG_MAX_COMP - 2U;
            }

            while ((insert > 0U) &&
                   (candidate_energy >
                    top_energy[insert - 1U]))
            {
                top_energy[insert] =
                    top_energy[insert - 1U];
                insert--;
            }
            top_energy[insert] = candidate_energy;
        }

        match_count = 1U + top_count;
        energy = base_energy;
        for (j = 0U; j < top_count; j++)
        {
            energy += top_energy[j];
        }

        if ((match_count > best_count) ||
            ((match_count == best_count) &&
             ((energy > best_energy) ||
              ((energy == best_energy) &&
               ((base_energy > best_base_energy) ||
                ((base_energy == best_base_energy) &&
                 (peak[base].freq_hz <
                  peak[best_base].freq_hz)))))))
        {
            best_count = match_count;
            best_energy = energy;
            best_base_energy = base_energy;
            best_base = base;
        }
    }

    selected[0] = peak[best_base];
    harmonic[0] = 1U;
    selected_count = 1U;

    for (i = 0U;
         (i < peak_count) &&
         (selected_count < ALOG_MAX_COMP);
         i++)
    {
        if (i == best_base)
        {
            continue;
        }

        ratio = peak[i].freq_hz /
                peak[best_base].freq_hz;
        order = (uint32_t)roundf(ratio);
        error_hz = fabsf(peak[i].freq_hz -
                         (float)order *
                         peak[best_base].freq_hz);
        if ((order >= 2U) &&
            (order <= ALOG_HARMONIC_MAX) &&
            (error_hz <= ALOG_FREQ_TOL_BINS *
                         ALOG_BIN_HZ))
        {
            selected[selected_count] = peak[i];
            harmonic[selected_count] = (uint8_t)order;
            selected_count++;
        }
    }

    for (i = 0U; i < selected_count; i++)
    {
        for (j = i + 1U; j < selected_count; j++)
        {
            if (harmonic[j] < harmonic[i])
            {
                temp_peak = selected[i];
                selected[i] = selected[j];
                selected[j] = temp_peak;

                temp_harmonic = harmonic[i];
                harmonic[i] = harmonic[j];
                harmonic[j] = temp_harmonic;
            }
        }
    }

    return selected_count;
}

static bool fit_components(const uint32_t *raw,
                           const peak_t *selected,
                           const uint8_t *harmonic,
                           uint8_t count,
                           signal_result_t *result)
{
    uint32_t i;
    uint32_t n;
    float angle;
    float cos_step;
    float sin_step;
    float cos_now;
    float sin_now;
    float next_cos;
    float norm;
    float code;
    float real;
    float imag;
    float amplitude;
    float weight;
    float weighted_f0 = 0.0f;
    float weight_sum = 0.0f;

    for (i = 0U; i < count; i++)
    {
        weight = selected[i].mag_code *
                 selected[i].mag_code;
        weighted_f0 += weight *
                       selected[i].freq_hz /
                       (float)harmonic[i];
        weight_sum += weight;
    }
    if (weight_sum <= ALOG_LOG_EPSILON)
    {
        return false;
    }

    result->fundamental_hz = weighted_f0 / weight_sum;
    result->comp_count = count;

    for (i = 0U; i < count; i++)
    {
        angle = ALOG_TWO_PI * selected[i].freq_hz /
                ALOG_SAMPLE_RATE_HZ;
        cos_step = arm_cos_f32(angle);
        sin_step = arm_sin_f32(angle);
        cos_now = 1.0f;
        sin_now = 0.0f;
        real = 0.0f;
        imag = 0.0f;

        for (n = 0U; n < ALOG_FFT_SIZE; n++)
        {
            code = ((float)(raw[n] & ALOG_ADC_MASK) -
                    result->dc_code) * hann_window[n];
            real += code * cos_now;
            imag += code * sin_now;

            next_cos = cos_now * cos_step -
                       sin_now * sin_step;
            sin_now = sin_now * cos_step +
                      cos_now * sin_step;
            cos_now = next_cos;

            if ((n & ALOG_OSC_RENORM_MASK) ==
                ALOG_OSC_RENORM_MASK)
            {
                norm = sqrtf(cos_now * cos_now +
                             sin_now * sin_now);
                if (norm <= ALOG_LOG_EPSILON)
                {
                    return false;
                }
                cos_now /= norm;
                sin_now /= norm;
            }
        }

        amplitude = 2.0f *
                    sqrtf(real * real + imag * imag) /
                    window_sum;
        if (!valid_float(amplitude))
        {
            return false;
        }

        result->comp[i].freq_hz = selected[i].freq_hz;
        result->comp[i].amp_code = amplitude;
        result->comp[i].rms_code =
            amplitude * ALOG_INV_SQRT_TWO;
        result->comp[i].phase_rad = atan2f(-imag, real);
        result->comp[i].bin = selected[i].bin;
        result->comp[i].harmonic = harmonic[i];
        result->comp[i].amp_mv =
            result->comp[i].amp_code *
            cal_gain(result->comp[i].freq_hz);
        result->comp[i].rms_mv =
            result->comp[i].rms_code *
            cal_gain(result->comp[i].freq_hz);
    }

    return true;
}

static bool calc_result(signal_result_t *result)
{
    uint32_t i;
    uint32_t point;
    float rms_sum = 0.0f;
    float rms_mv_sum = 0.0f;
    float base_phase;
    float wave;
    float wave_mv;
    float min_wave = FLT_MAX;
    float max_wave = -FLT_MAX;
    float min_wave_mv = FLT_MAX;
    float max_wave_mv = -FLT_MAX;

    for (i = 0U; i < result->comp_count; i++)
    {
        rms_sum += result->comp[i].rms_code *
                   result->comp[i].rms_code;
        rms_mv_sum += result->comp[i].rms_mv *
                      result->comp[i].rms_mv;
    }
    result->urms_code = sqrtf(rms_sum);
    result->urms_mv = sqrtf(rms_mv_sum);

    /*
     * TODO_PHASE:
     * 当前只完成幅频补偿。
     * 多谐波输入端Upp仍受模拟链相位响应影响。
     * 单正弦Upp应接近2*amp_mv。
     */
    for (point = 0U; point < ALOG_UPP_POINTS; point++)
    {
        base_phase = ALOG_TWO_PI * (float)point /
                     (float)ALOG_UPP_POINTS;
        wave = 0.0f;
        wave_mv = 0.0f;
        for (i = 0U; i < result->comp_count; i++)
        {
            wave += result->comp[i].amp_code *
                    arm_cos_f32(
                        (float)result->comp[i].harmonic *
                        base_phase +
                        result->comp[i].phase_rad);
            wave_mv += result->comp[i].amp_mv *
                       arm_cos_f32(
                           (float)result->comp[i].harmonic *
                           base_phase +
                           result->comp[i].phase_rad);
        }

        if (wave < min_wave)
        {
            min_wave = wave;
        }
        if (wave > max_wave)
        {
            max_wave = wave;
        }
        if (wave_mv < min_wave_mv)
        {
            min_wave_mv = wave_mv;
        }
        if (wave_mv > max_wave_mv)
        {
            max_wave_mv = wave_mv;
        }
    }

    result->upp_code = max_wave - min_wave;
    result->upp_mv = max_wave_mv - min_wave_mv;
    return valid_float(result->urms_code) &&
           valid_float(result->upp_code) &&
           valid_float(result->urms_mv) &&
           valid_float(result->upp_mv);
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
        hann_window[i] =
            0.5f - 0.5f * arm_cos_f32(phase);
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
    peak_t peak[ALOG_MAX_CANDIDATE];
    peak_t selected[ALOG_MAX_COMP];
    uint8_t harmonic[ALOG_MAX_COMP];
    uint8_t peak_count;
    uint8_t selected_count;

    if ((!alog_ready) ||
        (raw == NULL) ||
        (result == NULL) ||
        (count != ALOG_FFT_SIZE))
    {
        return false;
    }

    clear_result(result);
    calc_time(raw, result);
    calc_spectrum(raw, result->dc_code);
    peak_count = find_peaks(peak, &result->noise_code);
    if (peak_count == 0U)
    {
        return true;
    }

    selected_count = select_model(peak,
                                  peak_count,
                                  selected,
                                  harmonic);
    if ((!fit_components(raw,
                         selected,
                         harmonic,
                         selected_count,
                         result)) ||
        (!calc_result(result)))
    {
        clear_result(result);
        return false;
    }

    result->valid = true;
    return true;
}

const float *alog_get_spectrum(uint32_t *count)
{
    if (count != NULL)
    {
        *count = ALOG_SPECTRUM_SIZE;
    }
    return alog_ready ? spectrum : NULL;
}

bool alog_make_wave(const signal_result_t *result,
                    uint8_t cycles,
                    float *out,
                    uint32_t count)
{
    uint32_t point;
    uint32_t i;
    float base_phase;
    float wave;
    float scale = 0.0f;

    if ((result == NULL) ||
        (out == NULL) ||
        (!result->valid) ||
        (result->comp_count == 0U) ||
        (cycles == 0U) ||
        (count < 2U))
    {
        return false;
    }

    for (i = 0U; i < result->comp_count; i++)
    {
        scale += result->comp[i].amp_code;
    }
    if (scale <= ALOG_LOG_EPSILON)
    {
        return false;
    }

    for (point = 0U; point < count; point++)
    {
        base_phase =
            ALOG_TWO_PI * (float)cycles * (float)point /
            (float)(count - 1U);
        wave = 0.0f;
        for (i = 0U; i < result->comp_count; i++)
        {
            wave += result->comp[i].amp_code *
                    arm_cos_f32(
                        (float)result->comp[i].harmonic *
                        base_phase +
                        result->comp[i].phase_rad);
        }
        out[point] = wave / scale;
    }

    return true;
}
