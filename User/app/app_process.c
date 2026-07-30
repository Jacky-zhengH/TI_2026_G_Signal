#include "app_process.h"
#include "alog_signal.h"
#include "bsp_ad9220.h"
#include "main.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>

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
#define APP_PLOT_COUNT 538U

typedef enum
{
    app_view_wave1 = 0,
    app_view_wave3,
    app_view_spec
} app_view_t;

static uint32_t adc_raw[AD9220_SAMPLE_COUNT];
static signal_result_t signal_result;
static float plot_float[APP_PLOT_COUNT];
static uint8_t plot_byte[APP_PLOT_COUNT];

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
    HMI_Send_Cmd("t_tim.txt=\"0 ms\"");
    HMI_Send_Cmd("n_upp.txt=\"Upp: --- mV\"");
    HMI_Send_Cmd("n_urms.txt=\"Urms: --- mV\"");
    HMI_Send_Cmd("n_f0.txt=\"f0: --- kHz\"");
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
    hmi_reply_code = 0U;
    HMI_Send_Cmd("cle 2,0");
    HMI_Send_Cmd("addt 2,0,538");
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
    int32_t value;

    if (!alog_make_wave(&signal_result,
                        cycles,
                        plot_float,
                        APP_PLOT_COUNT))
    {
        return false;
    }

    for (i = 0U; i < APP_PLOT_COUNT; i++)
    {
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
    const float *data;
    uint32_t data_count;
    uint32_t start_bin;
    uint32_t end_bin;
    uint32_t span;
    uint32_t point;
    uint32_t first;
    uint32_t limit;
    uint32_t bin;
    int32_t value;
    float max_mag = 0.0f;
    float mag;
    float db;

    data = alog_get_spectrum(&data_count);
    if (data == NULL)
    {
        return false;
    }

    start_bin = (uint32_t)ceilf(ALOG_FREQ_MIN_HZ /
                                ALOG_BIN_HZ);
    end_bin = (uint32_t)floorf(ALOG_FREQ_MAX_HZ /
                               ALOG_BIN_HZ);
    if (end_bin >= data_count)
    {
        return false;
    }

    for (bin = start_bin; bin <= end_bin; bin++)
    {
        if (data[bin] > max_mag)
        {
            max_mag = data[bin];
        }
    }
    if (max_mag <= 1.0e-6f)
    {
        return false;
    }

    span = end_bin - start_bin + 1U;
    for (point = 0U; point < APP_PLOT_COUNT; point++)
    {
        first = start_bin +
                span * point / APP_PLOT_COUNT;
        limit = start_bin +
                span * (point + 1U) / APP_PLOT_COUNT;
        mag = 0.0f;

        for (bin = first; bin < limit; bin++)
        {
            if (data[bin] > mag)
            {
                mag = data[bin];
            }
        }

        db = (mag > 0.0f) ?
             20.0f * log10f(mag / max_mag) :
             -60.0f;
        if (db < -60.0f)
        {
            db = -60.0f;
        }

        plot_float[point] = db;
        value = (int32_t)(((db + 60.0f) / 60.0f) *
                          240.0f + 8.5f);
        plot_byte[point] = (uint8_t)value;
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
                   "n_upp.txt=\"Upp: %.1f mV\"",
                   signal_result.upp_mv);
    if ((len > 0) && (len < (int)sizeof(cmd)))
    {
        HMI_Send_Cmd(cmd);
    }

    len = snprintf(cmd,
                   sizeof(cmd),
                   "n_urms.txt=\"Urms: %.1f mV\"",
                   signal_result.urms_mv);
    if ((len > 0) && (len < (int)sizeof(cmd)))
    {
        HMI_Send_Cmd(cmd);
    }

    len = snprintf(cmd,
                   sizeof(cmd),
                   "n_f0.txt=\"f0: %.3f kHz\"",
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
                       "%s%u: %.3f kHz %.1f mV",
                       (i == 0U) ? "" : "\\r",
                       (unsigned int)(i + 1U),
                       signal_result.comp[i].freq_hz /
                       1000.0f,
                       signal_result.comp[i].amp_mv);
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
                 signal_result.upp_mv);
    Debug_printf("urms_mv: %.3f mV\r\n",
                 signal_result.urms_mv);

    for (i = 0U; i < ALOG_MAX_COMP; i++)
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
                     signal_result.comp[i].amp_mv);
        Debug_printf("rms_mv: %.3f mV\r\n",
                     signal_result.comp[i].rms_mv);
        Debug_printf("phase: %.6f rad\r\n",
                     signal_result.comp[i].phase_rad);
    }

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
        print_result = true;
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
    if (plot_ok && (!hmi_wave_send()))
    {
        error_text = "[HMI] addt failed\r\n";
    }
    print_result = true;

measure_end:
    measure_ms = HAL_GetTick() - start_tick;
    (void)snprintf(cmd,
                   sizeof(cmd),
                   "t_tim.txt=\"%lu ms\"",
                   (unsigned long)measure_ms);
    HMI_Send_Cmd(cmd);
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
