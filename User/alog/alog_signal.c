#include "alog_signal.h"

#include "arm_math.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define ALOG_TWO_PI             6.28318530717958647692f
#define ALOG_INV_SQRT_TWO       0.70710678f
#define ALOG_NOISE_BLOCKS       16U
#define ALOG_NOISE_MULTIPLE     6.0f
#define ALOG_PEAK_REL_MIN       0.01f
#define ALOG_PEAK_GUARD_BINS    3U
#define ALOG_FREQ_TOL_BINS      2.0f
#define ALOG_HARMONIC_MAX       50U
#define ALOG_LOG_EPSILON        1.0e-12f
#define ALOG_PIVOT_EPSILON      1.0e-6f
#define ALOG_LS_MAX_PARAM       7U
#define ALOG_LS_COLS            (ALOG_LS_MAX_PARAM + 1U)
#define ALOG_OSC_RENORM_MASK     255U
#define ALOG_UPP_POINTS          1024U

typedef struct
{
    float freq_hz;
    float mag_code;
    uint16_t bin;
} peak_t;

static float hann_window[ALOG_FFT_SIZE];
static float fft_input[ALOG_FFT_SIZE];
static float fft_output[ALOG_FFT_SIZE];
static float spectrum[ALOG_SPECTRUM_SIZE];

static arm_rfft_fast_instance_f32 fft_instance;

static float window_sum;
static bool alog_ready;

static alog_cal_t current_cal;
static bool cal_ready;

static bool valid_float(float value)
{
    return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

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
    float magnitude;

    arm_rfft_fast_f32(&fft_instance,
                      fft_input,
                      fft_output,
                      0U);

    spectrum[0] = fabsf(fft_output[0]) / window_sum;
    for (k = 1U; k < ALOG_SPECTRUM_SIZE; k++)
    {
        real = fft_output[2U * k];
        imag = fft_output[2U * k + 1U];
        magnitude = sqrtf(real * real + imag * imag);
        spectrum[k] = 2.0f * magnitude / window_sum;
    }
}

static float calc_noise(uint32_t start_bin, uint32_t end_bin)
{
    float block_mean[ALOG_NOISE_BLOCKS];
    uint32_t span = end_bin - start_bin + 1U;
    uint32_t block;
    uint32_t first;
    uint32_t last;
    uint32_t i;
    uint32_t j;
    float sum;
    float temp;

    for (block = 0U; block < ALOG_NOISE_BLOCKS; block++)
    {
        first = start_bin +
                (span * block) / ALOG_NOISE_BLOCKS;
        last = start_bin +
               (span * (block + 1U)) / ALOG_NOISE_BLOCKS;

        sum = 0.0f;
        for (i = first; i < last; i++)
        {
            sum += spectrum[i];
        }
        block_mean[block] = sum / (float)(last - first);
    }

    for (i = 0U; i < ALOG_NOISE_BLOCKS; i++)
    {
        for (j = i + 1U; j < ALOG_NOISE_BLOCKS; j++)
        {
            if (block_mean[j] < block_mean[i])
            {
                temp = block_mean[i];
                block_mean[i] = block_mean[j];
                block_mean[j] = temp;
            }
        }
    }

    return 0.5f *
           (block_mean[ALOG_NOISE_BLOCKS / 2U - 1U] +
            block_mean[ALOG_NOISE_BLOCKS / 2U]);
}

static float interp_freq(uint16_t bin)
{
    float left = logf(spectrum[bin - 1U] + ALOG_LOG_EPSILON);
    float center = logf(spectrum[bin] + ALOG_LOG_EPSILON);
    float right = logf(spectrum[bin + 1U] + ALOG_LOG_EPSILON);
    float denom = left - 2.0f * center + right;
    float delta = 0.0f;

    if (fabsf(denom) > ALOG_LOG_EPSILON)
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

    return ((float)bin + delta) * ALOG_BIN_HZ;
}

static uint8_t find_peaks(peak_t *peak, float *noise_code)
{
    uint16_t selected_bin[ALOG_MAX_CANDIDATE];
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
        *noise_code = 0.0f;
        return 0U;
    }

    *noise_code = calc_noise(start_bin, end_bin);
    for (i = start_bin; i <= end_bin; i++)
    {
        if (spectrum[i] > strongest)
        {
            strongest = spectrum[i];
        }
    }

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
        peak[count].bin = (uint16_t)best_bin;
        peak[count].mag_code = spectrum[best_bin];
        peak[count].freq_hz = interp_freq((uint16_t)best_bin);
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
    uint32_t order;
    uint32_t match_count;
    uint32_t best_match_count = 0U;
    uint32_t best_base = 0U;
    uint8_t selected_count = 0U;
    float ratio;
    float error_hz;
    float energy;
    float best_energy = 0.0f;
    peak_t temp_peak;
    uint8_t temp_harmonic;

    for (base = 0U; base < peak_count; base++)
    {
        match_count = 0U;
        energy = 0.0f;

        for (i = 0U; i < peak_count; i++)
        {
            ratio = peak[i].freq_hz / peak[base].freq_hz;
            order = (uint32_t)roundf(ratio);
            error_hz = fabsf(peak[i].freq_hz -
                             (float)order * peak[base].freq_hz);

            if ((order >= 1U) &&
                (order <= ALOG_HARMONIC_MAX) &&
                (error_hz <= ALOG_FREQ_TOL_BINS * ALOG_BIN_HZ))
            {
                match_count++;
                energy += peak[i].mag_code * peak[i].mag_code;
            }
        }

        if ((match_count > best_match_count) ||
            ((match_count == best_match_count) &&
             ((energy > best_energy) ||
              ((energy == best_energy) &&
               (peak[base].freq_hz < peak[best_base].freq_hz)))))
        {
            best_match_count = match_count;
            best_energy = energy;
            best_base = base;
        }
    }

    if (best_match_count == 0U)
    {
        return 0U;
    }

    selected[selected_count] = peak[best_base];
    harmonic[selected_count] = 1U;
    selected_count++;

    for (i = 0U;
         (i < peak_count) && (selected_count < ALOG_MAX_COMP);
         i++)
    {
        if (i == best_base)
        {
            continue;
        }

        ratio = peak[i].freq_hz / peak[best_base].freq_hz;
        order = (uint32_t)roundf(ratio);
        error_hz = fabsf(peak[i].freq_hz -
                         (float)order * peak[best_base].freq_hz);

        if ((order >= 2U) &&
            (order <= ALOG_HARMONIC_MAX) &&
            (error_hz <= ALOG_FREQ_TOL_BINS * ALOG_BIN_HZ))
        {
            selected[selected_count] = peak[i];
            harmonic[selected_count] = (uint8_t)order;
            selected_count++;
        }
    }

    for (i = 0U; i < selected_count; i++)
    {
        for (base = i + 1U; base < selected_count; base++)
        {
            if (harmonic[base] < harmonic[i])
            {
                temp_peak = selected[i];
                selected[i] = selected[base];
                selected[base] = temp_peak;

                temp_harmonic = harmonic[i];
                harmonic[i] = harmonic[base];
                harmonic[base] = temp_harmonic;
            }
        }
    }

    return selected_count;
}

static bool refine_f0(const peak_t *selected,
                      const uint8_t *harmonic,
                      uint8_t count,
                      signal_result_t *result)
{
    uint32_t i;
    float weight;
    float weight_sum = 0.0f;
    float frequency_sum = 0.0f;

    for (i = 0U; i < count; i++)
    {
        weight = selected[i].mag_code * selected[i].mag_code;
        weight_sum += weight;
        frequency_sum +=
            weight * selected[i].freq_hz / (float)harmonic[i];
    }

    if ((!valid_float(weight_sum)) ||
        (weight_sum <= ALOG_LOG_EPSILON))
    {
        return false;
    }

    result->fundamental_hz = frequency_sum / weight_sum;
    if ((!valid_float(result->fundamental_hz)) ||
        (result->fundamental_hz <= 0.0f))
    {
        return false;
    }

    result->comp_count = count;
    for (i = 0U; i < count; i++)
    {
        result->comp[i].freq_hz =
            (float)harmonic[i] * result->fundamental_hz;
        result->comp[i].bin = selected[i].bin;
        result->comp[i].harmonic = harmonic[i];
    }

    return true;
}

static bool solve_linear(float matrix[][ALOG_LS_COLS],
                         uint8_t size,
                         float *solution)
{
    uint32_t col;
    uint32_t row;
    uint32_t i;
    uint32_t pivot;
    float max_value;
    float value;
    float factor;
    float temp;
    float sum;

    for (col = 0U; col < size; col++)
    {
        pivot = col;
        max_value = fabsf(matrix[col][col]);

        for (row = col + 1U; row < size; row++)
        {
            value = fabsf(matrix[row][col]);
            if (value > max_value)
            {
                max_value = value;
                pivot = row;
            }
        }

        if ((!valid_float(max_value)) ||
            (max_value < ALOG_PIVOT_EPSILON))
        {
            return false;
        }

        if (pivot != col)
        {
            for (i = 0U; i <= size; i++)
            {
                temp = matrix[col][i];
                matrix[col][i] = matrix[pivot][i];
                matrix[pivot][i] = temp;
            }
        }

        for (row = col + 1U; row < size; row++)
        {
            factor = matrix[row][col] / matrix[col][col];
            matrix[row][col] = 0.0f;

            for (i = col + 1U; i <= size; i++)
            {
                matrix[row][i] -= factor * matrix[col][i];
            }
        }
    }

    for (i = size; i > 0U; i--)
    {
        row = i - 1U;
        sum = matrix[row][size];

        for (col = row + 1U; col < size; col++)
        {
            sum -= matrix[row][col] * solution[col];
        }

        solution[row] = sum / matrix[row][row];
        if (!valid_float(solution[row]))
        {
            return false;
        }
    }

    return true;
}

static bool fit_signal(const uint32_t *raw,
                       signal_result_t *result)
{
    float matrix[ALOG_LS_MAX_PARAM][ALOG_LS_COLS] = {{0.0f}};
    float solution[ALOG_LS_MAX_PARAM] = {0.0f};
    float basis[ALOG_LS_MAX_PARAM];
    float cos_step[ALOG_MAX_COMP];
    float sin_step[ALOG_MAX_COMP];
    float cos_now[ALOG_MAX_COMP];
    float sin_now[ALOG_MAX_COMP];
    uint32_t parameter_count =
        1U + 2U * (uint32_t)result->comp_count;
    uint32_t n;
    uint32_t i;
    uint32_t row;
    uint32_t col;
    float angle;
    float code;
    float next_cos;
    float next_sin;
    float norm;
    float a;
    float b;
    float amplitude;
    float fit;
    float error;
    float error_sum = 0.0f;

    for (i = 0U; i < result->comp_count; i++)
    {
        angle = ALOG_TWO_PI * result->comp[i].freq_hz /
                ALOG_SAMPLE_RATE_HZ;
        cos_step[i] = arm_cos_f32(angle);
        sin_step[i] = arm_sin_f32(angle);
        cos_now[i] = 1.0f;
        sin_now[i] = 0.0f;
    }

    for (n = 0U; n < ALOG_FFT_SIZE; n++)
    {
        code = (float)(raw[n] & ALOG_ADC_MASK);
        basis[0] = 1.0f;

        for (i = 0U; i < result->comp_count; i++)
        {
            basis[1U + 2U * i] = cos_now[i];
            basis[2U + 2U * i] = sin_now[i];
        }

        for (row = 0U; row < parameter_count; row++)
        {
            for (col = 0U; col < parameter_count; col++)
            {
                matrix[row][col] += basis[row] * basis[col];
            }
            matrix[row][parameter_count] += basis[row] * code;
        }

        for (i = 0U; i < result->comp_count; i++)
        {
            next_cos = cos_now[i] * cos_step[i] -
                       sin_now[i] * sin_step[i];
            next_sin = sin_now[i] * cos_step[i] +
                       cos_now[i] * sin_step[i];
            cos_now[i] = next_cos;
            sin_now[i] = next_sin;
        }

        if ((n & ALOG_OSC_RENORM_MASK) == ALOG_OSC_RENORM_MASK)
        {
            for (i = 0U; i < result->comp_count; i++)
            {
                norm = sqrtf(cos_now[i] * cos_now[i] +
                             sin_now[i] * sin_now[i]);
                if ((!valid_float(norm)) ||
                    (norm <= ALOG_LOG_EPSILON))
                {
                    return false;
                }
                cos_now[i] /= norm;
                sin_now[i] /= norm;
            }
        }
    }

    if (!solve_linear(matrix,
                      (uint8_t)parameter_count,
                      solution))
    {
        return false;
    }

    if (!valid_float(solution[0]))
    {
        return false;
    }

    for (i = 0U; i < result->comp_count; i++)
    {
        a = solution[1U + 2U * i];
        b = solution[2U + 2U * i];
        amplitude = sqrtf(a * a + b * b);

        if ((!valid_float(amplitude)) || (amplitude < 0.0f))
        {
            return false;
        }

        result->comp[i].amp_code = amplitude;
        result->comp[i].rms_code = amplitude * ALOG_INV_SQRT_TWO;
        result->comp[i].phase_rad = atan2f(-b, a);
        result->comp[i].phase_in_rad =
            result->comp[i].phase_rad;

        if (!valid_float(result->comp[i].phase_rad))
        {
            return false;
        }
    }

    for (i = 0U; i < result->comp_count; i++)
    {
        cos_now[i] = 1.0f;
        sin_now[i] = 0.0f;
    }

    for (n = 0U; n < ALOG_FFT_SIZE; n++)
    {
        fit = solution[0];
        for (i = 0U; i < result->comp_count; i++)
        {
            fit += solution[1U + 2U * i] * cos_now[i] +
                   solution[2U + 2U * i] * sin_now[i];
        }

        code = (float)(raw[n] & ALOG_ADC_MASK);
        error = code - fit;
        error_sum += error * error;

        for (i = 0U; i < result->comp_count; i++)
        {
            next_cos = cos_now[i] * cos_step[i] -
                       sin_now[i] * sin_step[i];
            next_sin = sin_now[i] * cos_step[i] +
                       cos_now[i] * sin_step[i];
            cos_now[i] = next_cos;
            sin_now[i] = next_sin;
        }

        if ((n & ALOG_OSC_RENORM_MASK) == ALOG_OSC_RENORM_MASK)
        {
            for (i = 0U; i < result->comp_count; i++)
            {
                norm = sqrtf(cos_now[i] * cos_now[i] +
                             sin_now[i] * sin_now[i]);
                if ((!valid_float(norm)) ||
                    (norm <= ALOG_LOG_EPSILON))
                {
                    return false;
                }
                cos_now[i] /= norm;
                sin_now[i] /= norm;
            }
        }
    }

    result->fit_error_code =
        sqrtf(error_sum / (float)ALOG_FFT_SIZE);
    return valid_float(result->fit_error_code);
}

static float cal_interp(float freq_hz, const float *value)
{
    uint32_t i;
    float ratio;

    if (freq_hz <= current_cal.freq_hz[0])
    {
        return value[0];
    }

    if (freq_hz >= current_cal.freq_hz[current_cal.count - 1U])
    {
        return value[current_cal.count - 1U];
    }

    for (i = 0U; i < current_cal.count - 1U; i++)
    {
        if (freq_hz <= current_cal.freq_hz[i + 1U])
        {
            ratio = (freq_hz - current_cal.freq_hz[i]) /
                    (current_cal.freq_hz[i + 1U] -
                     current_cal.freq_hz[i]);
            return value[i] + ratio * (value[i + 1U] - value[i]);
        }
    }

    return value[current_cal.count - 1U];
}

static bool apply_cal(signal_result_t *result)
{
    uint32_t i;
    float gain;
    float correction;

    result->calibrated = false;
    for (i = 0U; i < result->comp_count; i++)
    {
        result->comp[i].amp_mv = 0.0f;
        result->comp[i].rms_mv = 0.0f;
        result->comp[i].phase_in_rad =
            result->comp[i].phase_rad;
    }

    if (!cal_ready)
    {
        return true;
    }

    for (i = 0U; i < result->comp_count; i++)
    {
        gain = cal_interp(result->comp[i].freq_hz,
                          current_cal.mv_per_code);
        result->comp[i].amp_mv =
            result->comp[i].amp_code * gain;
        result->comp[i].rms_mv =
            result->comp[i].amp_mv * ALOG_INV_SQRT_TWO;

        if (current_cal.phase_valid)
        {
            correction =
                cal_interp(result->comp[i].freq_hz,
                           current_cal.phase_corr_rad);
            result->comp[i].phase_in_rad =
                result->comp[i].phase_rad + correction;
        }

        if ((!valid_float(result->comp[i].amp_mv)) ||
            (!valid_float(result->comp[i].rms_mv)) ||
            (!valid_float(result->comp[i].phase_in_rad)))
        {
            return false;
        }
    }

    result->calibrated = true;
    return true;
}

static bool calc_rms(signal_result_t *result)
{
    uint32_t i;
    float code_sum = 0.0f;
    float mv_sum = 0.0f;

    for (i = 0U; i < result->comp_count; i++)
    {
        code_sum += result->comp[i].rms_code *
                    result->comp[i].rms_code;
        if (result->calibrated)
        {
            mv_sum += result->comp[i].rms_mv *
                      result->comp[i].rms_mv;
        }
    }

    result->urms_code = sqrtf(code_sum);
    result->urms_mv =
        result->calibrated ? sqrtf(mv_sum) : 0.0f;

    return valid_float(result->urms_code) &&
           valid_float(result->urms_mv);
}

static bool calc_upp(signal_result_t *result)
{
    uint32_t point;
    uint32_t i;
    float base_phase;
    float wave_code;
    float wave_mv;
    float min_code = FLT_MAX;
    float max_code = -FLT_MAX;
    float min_mv = FLT_MAX;
    float max_mv = -FLT_MAX;

    for (point = 0U; point < ALOG_UPP_POINTS; point++)
    {
        base_phase = ALOG_TWO_PI * (float)point /
                     (float)ALOG_UPP_POINTS;
        wave_code = 0.0f;
        wave_mv = 0.0f;

        for (i = 0U; i < result->comp_count; i++)
        {
            wave_code +=
                result->comp[i].amp_code *
                arm_cos_f32((float)result->comp[i].harmonic *
                            base_phase +
                            result->comp[i].phase_rad);

            if (result->calibrated)
            {
                wave_mv +=
                    result->comp[i].amp_mv *
                    arm_cos_f32((float)result->comp[i].harmonic *
                                base_phase +
                                result->comp[i].phase_in_rad);
            }
        }

        if ((!valid_float(wave_code)) ||
            (!valid_float(wave_mv)))
        {
            return false;
        }

        if (wave_code < min_code)
        {
            min_code = wave_code;
        }
        if (wave_code > max_code)
        {
            max_code = wave_code;
        }

        if (result->calibrated)
        {
            if (wave_mv < min_mv)
            {
                min_mv = wave_mv;
            }
            if (wave_mv > max_mv)
            {
                max_mv = wave_mv;
            }
        }
    }

    result->upp_code = max_code - min_code;
    result->upp_mv =
        result->calibrated ? max_mv - min_mv : 0.0f;

    return valid_float(result->upp_code) &&
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
        hann_window[i] = 0.5f - 0.5f * arm_cos_f32(phase);
        window_sum += hann_window[i];
    }

    memset(fft_input, 0, sizeof(fft_input));
    memset(fft_output, 0, sizeof(fft_output));
    memset(spectrum, 0, sizeof(spectrum));
    alog_clear_cal();

    alog_ready = true;
    return true;
}

bool alog_set_cal(const alog_cal_t *cal)
{
    uint32_t i;

    if ((cal == NULL) ||
        (cal->count < 2U) ||
        (cal->count > ALOG_CAL_MAX))
    {
        return false;
    }

    for (i = 0U; i < cal->count; i++)
    {
        if ((!valid_float(cal->freq_hz[i])) ||
            (!valid_float(cal->mv_per_code[i])) ||
            (cal->freq_hz[i] <= 0.0f) ||
            (cal->mv_per_code[i] <= 0.0f))
        {
            return false;
        }

        if ((i > 0U) &&
            (cal->freq_hz[i] <= cal->freq_hz[i - 1U]))
        {
            return false;
        }

        if (cal->phase_valid &&
            (!valid_float(cal->phase_corr_rad[i])))
        {
            return false;
        }
    }

    current_cal = *cal;
    cal_ready = true;
    return true;
}

void alog_clear_cal(void)
{
    memset(&current_cal, 0, sizeof(current_cal));
    cal_ready = false;
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

    peak_count = find_peaks(peak, &result->noise_code);
    if (peak_count == 0U)
    {
        return true;
    }

    selected_count =
        select_model(peak, peak_count, selected, harmonic);
    if (selected_count == 0U)
    {
        return true;
    }

    if ((!refine_f0(selected,
                    harmonic,
                    selected_count,
                    result)) ||
        (!fit_signal(raw, result)) ||
        (!apply_cal(result)) ||
        (!calc_rms(result)) ||
        (!calc_upp(result)))
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

    return spectrum;
}

bool alog_make_wave(const signal_result_t *result,
                    uint8_t cycles,
                    float *out,
                    uint32_t count)
{
    uint32_t point;
    uint32_t i;
    float base_phase;
    float amplitude;
    float phase;
    float value;
    float max_abs = 0.0f;
    float abs_value;

    if ((result == NULL) ||
        (out == NULL) ||
        (!result->valid) ||
        (result->comp_count == 0U) ||
        (result->comp_count > ALOG_MAX_COMP) ||
        ((cycles != 1U) && (cycles != 3U)) ||
        (count < 2U))
    {
        return false;
    }

    for (point = 0U; point < count; point++)
    {
        base_phase = ALOG_TWO_PI * (float)cycles *
                     (float)point / (float)(count - 1U);
        value = 0.0f;

        for (i = 0U; i < result->comp_count; i++)
        {
            if (result->comp[i].harmonic == 0U)
            {
                continue;
            }

            if (result->calibrated)
            {
                amplitude = result->comp[i].amp_mv;
                phase = result->comp[i].phase_in_rad;
            }
            else
            {
                amplitude = result->comp[i].amp_code;
                phase = result->comp[i].phase_rad;
            }

            if ((!valid_float(amplitude)) ||
                (!valid_float(phase)))
            {
                return false;
            }

            value += amplitude *
                     arm_cos_f32(
                         (float)result->comp[i].harmonic *
                         base_phase + phase);
        }

        if (!valid_float(value))
        {
            return false;
        }

        out[point] = value;
        abs_value = fabsf(value);
        if (abs_value > max_abs)
        {
            max_abs = abs_value;
        }
    }

    if (max_abs <= ALOG_LOG_EPSILON)
    {
        return false;
    }

    for (point = 0U; point < count; point++)
    {
        out[point] /= max_abs;
    }

    return true;
}
