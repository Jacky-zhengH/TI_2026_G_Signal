#include "app_process.h"
#include "alog_signal.h"
#include "bsp_ad9220.h"
#include "main.h"
#include "arm_math.h"

#include <float.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

//*********************************************************************************************************
extern UART_HandleTypeDef huart1; // HMI 控制串口
extern UART_HandleTypeDef huart3; // PC 调试串口
//*********************************************************************************************************

static uint8_t hmi_rx_buffer[1];          // HMI 单字节命令接收缓冲区
static char debug_buffer[200];            // PC串口调试发送缓冲区
static volatile uint8_t hmi_cmd_flag = 0; // 新命令标志位
static volatile uint8_t hmi_cmd_data = 0; // 最新命令字节
static volatile uint8_t hmi_reply_code = 0;
//*********************************************************************************************************
/*测试用*/
// #define AD9220_TEST_INTERVAL_MS 1000U   //1000ms --> 1s采集
#define APP_CAPTURE_TIMEOUT_MS 10U
#define APP_PLOT_COUNT 509U
#define APP_SPEC_RIGHT_X 500U
#define APP_CAL_COUNT 13U
#define APP_REBUILD_POINTS 2048U
#define APP_ZERO_PHASE_ENABLE 1U

#ifndef ARM_MATH_PI
#define ARM_MATH_PI 3.14159265358979323846f
#endif

typedef enum
{
    app_view_wave1 = 0,
    app_view_wave3,
    app_view_spec
} app_view_t;

typedef struct
{
    float amp_mv[ALOG_MAX_COMP];
    float rms_mv[ALOG_MAX_COMP];
    float rel_phase_rad[ALOG_MAX_COMP];
    float urms_mv;
    float upp_mv;
    float rebuild_min_mv;
    float rebuild_max_mv;
} app_mv_result_t;

/*
 * TEMP_CAL:
 * 当前系数来自100mVpp单正弦初步实测。
 * 系数单位为mV/code，对应正弦峰值。
 * 后续使用示波器实测输入和多次平均数据替换。
 */
static const float cal_freq_hz[APP_CAL_COUNT] =
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

static const float cal_mv_per_code[APP_CAL_COUNT] =
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

static uint32_t adc_raw[AD9220_SAMPLE_COUNT];
static signal_result_t signal_result;
static app_mv_result_t mv_result;
static float plot_float[APP_PLOT_COUNT];
static uint8_t plot_byte[APP_PLOT_COUNT];
static float zero_wave_buf[APP_REBUILD_POINTS];

//=========================================================================================================
// 1. 基础功能函数
//=========================================================================================================
/**
 * @name HMI_Process_Init
 * @brief 启动应用层接收与测量任务框架
 */
void HMI_Process_Init(void)
{
    hmi_cmd_flag = 0U;
    hmi_reply_code = 0U;
    HAL_UART_Receive_IT(&huart1, hmi_rx_buffer, 1U);

    HMI_Send_Cmd("cle 2,0");
    HMI_Send_Cmd("t_tim.txt=\"0ms\"");
    HMI_Send_Cmd("n_upp.txt=\"Upp: ---mV\"");
    HMI_Send_Cmd("n_urms.txt=\"Urms: ---mV\"");
    HMI_Send_Cmd("n_f0.txt=\"f0: ---kHz\"");
    HMI_Send_Cmd("t_comp.txt=\"---\"");
}

/**
 * @name    HMI_Send_Cmd
 * @brief   向HMI串口屏发送原始指令
 */
void HMI_Send_Cmd(const char *cmd_string)
{
    char cmd_buffer[200];
    int len;

    if (cmd_string == NULL)
    {
        return;
    }

    len = snprintf(cmd_buffer,
                   sizeof(cmd_buffer),
                   "%s\xff\xff\xff",
                   cmd_string);
    if ((len <= 0) || (len >= (int)sizeof(cmd_buffer)))
    {
        return;
    }

    HAL_UART_Transmit(&huart1,
                      (uint8_t *)cmd_buffer,
                      (uint16_t)len,
                      200U);
}

/**
 * @name    Debug_printf
 * @brief   PC串口调试打印
 */
void Debug_printf(const char *text, ...)
{
    va_list args;
    int len;

    if (text == NULL)
    {
        return;
    }

    va_start(args, text);
    len = vsnprintf(debug_buffer,
                    sizeof(debug_buffer),
                    text,
                    args);
    va_end(args);

    if (len <= 0)
    {
        return;
    }
    if (len >= (int)sizeof(debug_buffer))
    {
        len = (int)sizeof(debug_buffer) - 1;
    }

    HAL_UART_Transmit(&huart3,
                      (uint8_t *)debug_buffer,
                      (uint16_t)len,
                      100U);
}
//=========================================================================================================
// 2. 任务辅助函数 static
//=========================================================================================================

static float cal_gain(float freq_hz)
{
    uint32_t i;

    if (freq_hz <= cal_freq_hz[0])
    {
        return cal_mv_per_code[0];
    }
    if (freq_hz >= cal_freq_hz[APP_CAL_COUNT - 1U])
    {
        return cal_mv_per_code[APP_CAL_COUNT - 1U];
    }

    for (i = 0U; i < (APP_CAL_COUNT - 1U); i++)
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

    return cal_mv_per_code[APP_CAL_COUNT - 1U];
}

static float wrap_pi(float phase)
{
    while (phase > ARM_MATH_PI)
    {
        phase -= 2.0f * ARM_MATH_PI;
    }
    while (phase < -ARM_MATH_PI)
    {
        phase += 2.0f * ARM_MATH_PI;
    }
    return phase;
}

#if !APP_ZERO_PHASE_ENABLE
static float rebuild_value(const signal_result_t *result,
                           const app_mv_result_t *mv,
                           float base_phase)
{
    uint32_t i;
    float wave = 0.0f;

    for (i = 0U; i < result->comp_count; i++)
    {
        wave += mv->amp_mv[i] *
                arm_cos_f32(
                    (float)result->comp[i].harmonic *
                    base_phase +
                    mv->rel_phase_rad[i]);
    }
    return wave;
}
#endif

static bool signal_rebuild_zero_phase(
    const uint8_t *harmonic,
    const float *amp_mv,
    uint8_t comp_count,
    float *wave_buffer,
    uint32_t count)
{
    uint32_t n;
    uint32_t i;
    float phase;
    float wave;

    if ((harmonic == NULL) ||
        (amp_mv == NULL) ||
        (wave_buffer == NULL) ||
        (comp_count == 0U) ||
        (comp_count > ALOG_MAX_COMP) ||
        (count == 0U))
    {
        return false;
    }

    for (n = 0U; n < count; n++)
    {
        phase = 2.0f * ARM_MATH_PI * (float)n /
                (float)count;
        wave = 0.0f;
        for (i = 0U; i < comp_count; i++)
        {
            wave += amp_mv[i] *
                    arm_sin_f32(
                        (float)harmonic[i] * phase);
        }
        wave_buffer[n] = wave;
    }
    return true;
}

static bool calc_mv(const signal_result_t *result,
                    app_mv_result_t *mv)
{
    uint32_t i;
    uint32_t point;
    uint8_t harmonic[ALOG_MAX_COMP];
    float gain;
    float sum_sq = 0.0f;
#if !APP_ZERO_PHASE_ENABLE
    float base_phase;
#endif
    float wave;
    float min_wave = FLT_MAX;
    float max_wave = -FLT_MAX;

    if ((result == NULL) ||
        (mv == NULL) ||
        (!result->valid) ||
        (result->comp_count == 0U) ||
        (result->comp_count > ALOG_MAX_COMP))
    {
        return false;
    }

    memset(mv, 0, sizeof(*mv));
    for (i = 0U; i < result->comp_count; i++)
    {
        gain = cal_gain(result->comp[i].freq_hz);
        mv->amp_mv[i] =
            result->comp[i].amp_code * gain;
        mv->rms_mv[i] =
            result->comp[i].rms_code * gain;
        harmonic[i] = result->comp[i].harmonic;
        sum_sq += mv->rms_mv[i] * mv->rms_mv[i];
    }
    mv->urms_mv = sqrtf(sum_sq);

    /*
     * 去除随机采集起点造成的公共时间平移。
     * TODO_PHASE_CAL:
     * 当前仍是ADC侧相对相位。
     * 实测模拟链相频响应后，应减去：
     * phase_chain(fi) - harmonic * phase_chain(f0)。
     */
    for (i = 0U; i < result->comp_count; i++)
    {
        mv->rel_phase_rad[i] =
            wrap_pi(
                result->comp[i].phase_rad -
                (float)result->comp[i].harmonic *
                result->comp[0].phase_rad);
    }

#if APP_ZERO_PHASE_ENABLE
    if (!signal_rebuild_zero_phase(harmonic,
                                   mv->amp_mv,
                                   result->comp_count,
                                   zero_wave_buf,
                                   APP_REBUILD_POINTS))
    {
        return false;
    }

    for (point = 0U; point < APP_REBUILD_POINTS; point++)
    {
        wave = zero_wave_buf[point];
        if (wave < min_wave)
        {
            min_wave = wave;
        }
        if (wave > max_wave)
        {
            max_wave = wave;
        }
    }
#else
    for (point = 0U; point < APP_REBUILD_POINTS; point++)
    {
        base_phase =
            2.0f * ARM_MATH_PI * (float)point /
            (float)APP_REBUILD_POINTS;
        wave = rebuild_value(result, mv, base_phase);

        if (wave < min_wave)
        {
            min_wave = wave;
        }
        if (wave > max_wave)
        {
            max_wave = wave;
        }
    }
#endif

    mv->rebuild_min_mv = min_wave;
    mv->rebuild_max_mv = max_wave;
    mv->upp_mv = max_wave - min_wave;
    return true;
}

static bool hmi_wait(uint8_t code, uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((uint32_t)(HAL_GetTick() - start_tick) <
           timeout_ms)
    {
        if (hmi_reply_code == code)
        {
            hmi_reply_code = 0U;
            return true;
        }
    }
    return false;
}

static bool hmi_wave_send(void)
{
    char cmd[24];
    int len;

    hmi_reply_code = 0U;
    HMI_Send_Cmd("cle 2,0");
    len = snprintf(cmd,
                   sizeof(cmd),
                   "addt 2,0,%u",
                   (unsigned int)APP_PLOT_COUNT);
    if ((len <= 0) || (len >= (int)sizeof(cmd)))
    {
        return false;
    }
    HMI_Send_Cmd(cmd);
    if (!hmi_wait(0xFEU, 200U))
    {
        return false;
    }

    hmi_reply_code = 0U;
    if (HAL_UART_Transmit(&huart1,
                          plot_byte,
                          APP_PLOT_COUNT,
                          1000U) != HAL_OK)
    {
        return false;
    }
    return hmi_wait(0xFDU, 1000U);
}

static bool make_wave(uint8_t cycles)
{
    uint32_t i;
    uint32_t source;
    int32_t value;
#if !APP_ZERO_PHASE_ENABLE
    float base_phase;
#endif
    float scale = 0.0f;
    float wave;

    if ((!signal_result.valid) ||
        (signal_result.comp_count == 0U) ||
        (cycles == 0U))
    {
        return false;
    }

    for (i = 0U; i < signal_result.comp_count; i++)
    {
        scale += mv_result.amp_mv[i];
    }
    if (scale <= 0.0f)
    {
        return false;
    }

    for (i = 0U; i < APP_PLOT_COUNT; i++)
    {
#if APP_ZERO_PHASE_ENABLE
        source = (uint32_t)cycles * i *
                 APP_REBUILD_POINTS /
                 (APP_PLOT_COUNT - 1U);
        source %= APP_REBUILD_POINTS;
        wave = zero_wave_buf[source];
#else
        base_phase =
            2.0f * ARM_MATH_PI * (float)cycles *
            (float)i /
            (float)(APP_PLOT_COUNT - 1U);
        wave = rebuild_value(&signal_result,
                             &mv_result,
                             base_phase);
#endif
        plot_float[i] = wave / scale;

        value = (int32_t)(128.5f +
                          plot_float[i] * 110.0f);
        if (value < 8)
        {
            value = 8;
        }
        else if (value > 247)
        {
            value = 247;
        }
        plot_byte[i] = (uint8_t)value;
    }
    return true;
}

static bool make_spec(void)
{
    uint32_t i;
    uint32_t point;
    int32_t value;
    float max_amp = 0.0f;
    float freq_hz;

    for (i = 0U; i < signal_result.comp_count; i++)
    {
        if ((signal_result.comp[i].harmonic != 0U) &&
            (mv_result.amp_mv[i] > max_amp))
        {
            max_amp = mv_result.amp_mv[i];
        }
    }
    if (max_amp <= 1.0e-6f)
    {
        return false;
    }

    memset(plot_byte, 0, sizeof(plot_byte));

    for (i = 0U; i < signal_result.comp_count; i++)
    {
        if (signal_result.comp[i].harmonic == 0U)
        {
            continue;
        }

        freq_hz = signal_result.comp[i].freq_hz;
        if (freq_hz < 0.0f)
        {
            freq_hz = 0.0f;
        }
        else if (freq_hz > ALOG_FREQ_MAX_HZ)
        {
            freq_hz = ALOG_FREQ_MAX_HZ;
        }

        /*
         * TJC曲线批量数据的屏幕位置与发送顺序相反。
         * 反向写入后，屏幕频率才会从左到右增大。
         */
        point = APP_PLOT_COUNT - 1U -
                (uint32_t)(
                    freq_hz *
                    (float)APP_SPEC_RIGHT_X /
                    ALOG_FREQ_MAX_HZ + 0.5f);

        value = (int32_t)(
            mv_result.amp_mv[i] / max_amp * 247.0f +
            0.5f);
        if (value > 247)
        {
            value = 247;
        }
        if (value > (int32_t)plot_byte[point])
        {
            plot_byte[point] = (uint8_t)value;
        }
    }
    return true;
}

static void hmi_show_result(void)
{
    char cmd[200];
    uint32_t i;
    uint32_t used;
    int len;

    len = snprintf(cmd,
                   sizeof(cmd),
                   "n_upp.txt=\"Upp: %.1fmV\"",
                   mv_result.upp_mv);
    if ((len > 0) && (len < (int)sizeof(cmd)))
    {
        HMI_Send_Cmd(cmd);
    }

    len = snprintf(cmd,
                   sizeof(cmd),
                   "n_urms.txt=\"Urms: %.1fmV\"",
                   mv_result.urms_mv);
    if ((len > 0) && (len < (int)sizeof(cmd)))
    {
        HMI_Send_Cmd(cmd);
    }

    len = snprintf(cmd,
                   sizeof(cmd),
                   "n_f0.txt=\"f0: %.3fkHz\"",
                   signal_result.fundamental_hz / 1000.0f);
    if ((len > 0) && (len < (int)sizeof(cmd)))
    {
        HMI_Send_Cmd(cmd);
    }

    used = (uint32_t)snprintf(cmd,
                              sizeof(cmd),
                              "t_comp.txt=\"");
    for (i = 0U; i < signal_result.comp_count; i++)
    {
        len = snprintf(&cmd[used],
                       sizeof(cmd) - used,
                       "%s%u: %.2fkHz %.1fmV",
                       (i == 0U) ? "" : "\\r\\r",
                       (unsigned int)
                       signal_result.comp[i].harmonic,
                       signal_result.comp[i].freq_hz /
                       1000.0f,
                       mv_result.amp_mv[i]);
        if ((len <= 0) ||
            (len >= (int)(sizeof(cmd) - used)))
        {
            return;
        }
        used += (uint32_t)len;
    }

    cmd[used] = '"';
    cmd[used + 1U] = '\0';
    HMI_Send_Cmd(cmd);
}

static void debug_measure(uint32_t time_ms)
{
    uint32_t i;

    Debug_printf("\r\n========== MEASURE ==========\r\n");
    Debug_printf("TIME: %lu ms\r\n",
                 (unsigned long)time_ms);
    Debug_printf("dc_code: %.3f\r\n",
                 signal_result.dc_code);
    Debug_printf("min_code: %.3f\r\n",
                 signal_result.min_code);
    Debug_printf("max_code: %.3f\r\n",
                 signal_result.max_code);
    Debug_printf("raw_pp_code: %.3f\r\n",
                 signal_result.raw_pp_code);
    Debug_printf("time_rms_code: %.3f\r\n",
                 signal_result.time_rms_code);
    Debug_printf("noise_code: %.3f\r\n",
                 signal_result.noise_code);

    Debug_printf("\r\nRESULT:\r\n");
    Debug_printf("valid: %u\r\n",
                 signal_result.valid ? 1U : 0U);
    Debug_printf("fundamental: %.3f Hz\r\n",
                 signal_result.fundamental_hz);
    Debug_printf("comp_count: %u\r\n",
                 (unsigned int)signal_result.comp_count);
    Debug_printf("upp_code: %.3f\r\n",
                 signal_result.upp_code);
    Debug_printf("urms_code: %.3f\r\n",
                 signal_result.urms_code);
    Debug_printf("upp_mv: %.3f mV\r\n",
                 mv_result.upp_mv);
    Debug_printf("urms_mv: %.3f mV\r\n",
                 mv_result.urms_mv);

    for (i = 0U; i < signal_result.comp_count; i++)
    {
        Debug_printf("\r\nCOMP%u:\r\n",
                     (unsigned int)(i + 1U));
        Debug_printf("harmonic: %u\r\n",
                     (unsigned int)
                     signal_result.comp[i].harmonic);
        Debug_printf("freq: %.3f Hz\r\n",
                     signal_result.comp[i].freq_hz);
        Debug_printf("amp_code: %.3f\r\n",
                     signal_result.comp[i].amp_code);
        Debug_printf("rms_code: %.3f\r\n",
                     signal_result.comp[i].rms_code);
        Debug_printf("amp_mv: %.3f mV\r\n",
                     mv_result.amp_mv[i]);
        Debug_printf("rms_mv: %.3f mV\r\n",
                     mv_result.rms_mv[i]);
        Debug_printf("phase: %.6f rad\r\n",
                     signal_result.comp[i].phase_rad);
        Debug_printf("rel_phase: %.6f rad\r\n",
                     mv_result.rel_phase_rad[i]);
    }

    Debug_printf("\r\n===== ZERO PHASE =====\r\n");
    Debug_printf("comp_count: %u\r\n",
                 (unsigned int)signal_result.comp_count);
    for (i = 0U; i < signal_result.comp_count; i++)
    {
        Debug_printf("\r\nH%u:\r\n",
                     (unsigned int)
                     signal_result.comp[i].harmonic);
        Debug_printf("amp: %.3f mV\r\n",
                     mv_result.amp_mv[i]);
    }
    Debug_printf("\r\nrebuild_max: %.3f mV\r\n",
                 mv_result.rebuild_max_mv);
    Debug_printf("rebuild_min: %.3f mV\r\n",
                 mv_result.rebuild_min_mv);
    Debug_printf("rebuild_upp: %.3f mV\r\n",
                 mv_result.upp_mv);

    Debug_printf("\r\n=============================\r\n");
}

//=========================================================================================================
// 3. 应用任务函数
//=========================================================================================================
/**
 * @brief AD9220一次采集和算法分析
 * 每次按键采集4096点，不打印原始采样数组。
 */
static void task_measure(app_view_t view)
{
    char cmd[40];
    uint32_t start_tick;
    uint32_t measure_ms;
    bool plot_ok;
    bool print_result = false;
    bool show_time = false;
    const char *error_text = NULL;

    start_tick = HAL_GetTick();
    if (!bsp_ad9220_capture(adc_raw,
                            AD9220_SAMPLE_COUNT,
                            APP_CAPTURE_TIMEOUT_MS))
    {
        error_text = "[MEAS] capture failed\r\n";
        goto measure_end;
    }

    if (!alog_analyze(adc_raw,
                      AD9220_SAMPLE_COUNT,
                      &signal_result))
    {
        error_text = "[MEAS] analyze failed\r\n";
        goto measure_end;
    }

    if (!signal_result.valid)
    {
        memset(&mv_result, 0, sizeof(mv_result));
        print_result = true;
        goto measure_end;
    }

    if (!calc_mv(&signal_result, &mv_result))
    {
        error_text = "[MEAS] mv failed\r\n";
        goto measure_end;
    }

    if (view == app_view_wave1)
    {
        plot_ok = make_wave(1U);
    }
    else if (view == app_view_wave3)
    {
        plot_ok = make_wave(3U);
    }
    else
    {
        plot_ok = make_spec();
    }

    hmi_show_result();
    if (!plot_ok)
    {
        error_text = "[MEAS] plot failed\r\n";
    }
    else if (!hmi_wave_send())
    {
        error_text = "[HMI] addt failed\r\n";
    }
    else
    {
        show_time = true;
    }
    print_result = true;

measure_end:
    measure_ms = HAL_GetTick() - start_tick;
    if (show_time)
    {
        (void)snprintf(cmd,
                       sizeof(cmd),
                       "t_tim.txt=\"%lums\"",
                       (unsigned long)measure_ms);
        HMI_Send_Cmd(cmd);
    }
    if (error_text != NULL)
    {
        Debug_printf("%s", error_text);
    }
    if (print_result)
    {
        debug_measure(measure_ms);
    }
}

/**
 * @brief   按键响应任务：
 */
static void task_button(void)
{
    uint8_t cmd;
    app_view_t view;

    if (hmi_cmd_flag == 0U)
    {
        return;
    }

    __disable_irq();
    cmd = hmi_cmd_data;
    hmi_cmd_flag = 0U;
    __enable_irq();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    view = (cmd == 0xA1U) ? app_view_wave1 :
           ((cmd == 0xA3U) ? app_view_wave3 :
                             app_view_spec);
    task_measure(view);
}

//=========================================================================================================
// 4. 主轮询整合
//=========================================================================================================

/**
 * @name   App_Main_Process_Poll
 * @brief  放在main.c的while(1)中，统筹调度所有应用任务
 */
void App_Main_Process_Poll(void)
{
    // HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    // HAL_Delay(500);
    task_button();
}

//=========================================================================================================
// 5. 中断回调
//=========================================================================================================

/**
 * @brief USART接收完成回调，只保存命令并重启接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t data;

    if (huart->Instance == USART1)
    {
        data = hmi_rx_buffer[0];
        if ((data == 0xA1U) ||
            (data == 0xA3U) ||
            (data == 0xAFU))
        {
            hmi_cmd_data = data;
            hmi_cmd_flag = 1U;
        }
        else if ((data == 0xFEU) ||
                 (data == 0xFDU))
        {
            hmi_reply_code = data;
        }

        HAL_UART_Receive_IT(huart,
                            hmi_rx_buffer,
                            1U);
    }
}
