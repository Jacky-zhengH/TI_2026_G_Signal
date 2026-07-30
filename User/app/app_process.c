#include "app_process.h"
#include "bsp_ad9220.h"
//*********************************************************************************************************
extern UART_HandleTypeDef huart1; // HMI 控制串口
extern UART_HandleTypeDef huart3; // PC 调试串口
//*********************************************************************************************************

static uint8_t hmi_rx_buffer[1];          // HMI 单字节命令接收缓冲区
static char debug_buffer[160];            // PC串口调试发送缓冲区
static volatile uint8_t hmi_cmd_flag = 0; // 新命令标志位
static volatile uint8_t hmi_cmd_data = 0; // 最新命令字节
//*********************************************************************************************************
/*测试用*/
// #define AD9220_TEST_INTERVAL_MS 1000U   //1000ms --> 1s采集
#define AD9220_TEST_TIMEOUT_MS 10U
#define AD9220_TEST_PRINT_COUNT 32U

static uint32_t ad9220_samples[AD9220_SAMPLE_COUNT];

//=========================================================================================================
// 1. 基础功能函数
//=========================================================================================================
/**
 * @name HMI_Process_Init
 * @brief 启动应用层接收与测量任务框架
 */
void HMI_Process_Init(void)
{
    HAL_UART_Receive_IT(&huart1, hmi_rx_buffer, 1);
}

/**
 * @name    HMI_Send_Cmd
 * @brief   向HMI串口屏发送原始指令
 */
void HMI_Send_Cmd(const char *cmd_string)
{
    char cmd_buffer[200];
    int len = snprintf(cmd_buffer, sizeof(cmd_buffer), "%s\xff\xff\xff", cmd_string);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart1, (uint8_t *)cmd_buffer, len, HAL_MAX_DELAY);
    }
}

/**
 * @name    Debug_printf
 * @brief   PC串口调试打印
 */
void Debug_printf(const char *text, ...)
{
    va_list args;
    va_start(args, text);
    int len = vsnprintf(debug_buffer, sizeof(debug_buffer), text, args);
    va_end(args);

    if (len > 0)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)debug_buffer, len, 100);
    }
}
//=========================================================================================================
// 2. 任务辅助函数 static
//=========================================================================================================

//=========================================================================================================
// 3. 应用任务函数
//=========================================================================================================
/**
 * @brief AD9220简易采集测试
 * 每隔1秒采集4096点，随后通过USART3电脑串口输出前32点。
 */
static void Task_AD9220_Test(uint8_t cmd)
{
    // static uint32_t last_tick = 0U;

    // uint32_t now;  // 1s计时
    uint32_t step;
    uint32_t index;
    uint32_t raw;  // 原始值
    uint16_t code; // 12位ADC值
    uint32_t i;

    uint16_t min_code;
    uint16_t max_code;
    uint16_t sample_code;

    uint32_t sum;
    uint32_t avg_int;
    uint32_t avg_frac;
    // now = HAL_GetTick();

    // /* 使用无符号减法，兼容HAL_GetTick回绕 */
    // if ((uint32_t)(now - last_tick) < AD9220_TEST_INTERVAL_MS)
    // {
    //     return;
    // }

    // last_tick = now;
    if (cmd == 0xA1U)
    {
        step = 1U;
        Debug_printf(
            "[Task] AD9220 single-cycle test, step = 1\r\n");
    }
    else if (cmd == 0xA3U)
    {
        step = 3U;
        Debug_printf(
            "[Task] AD9220 three-cycle test, step = 3\r\n");
    }
    else
    {
        return;
    }
    /*
     * 采集一帧
     * 2MHz采4096点理论耗时2.048ms。设置10ms超时。
     */
    if (!bsp_ad9220_capture(ad9220_samples,
                            AD9220_SAMPLE_COUNT,
                            AD9220_TEST_TIMEOUT_MS))
    {
        Debug_printf("[Task] AD9220 capture failed\r\n");
        return;
    }
    Debug_printf(
        "[Task] AD9220 capture success, count = %u, otr = %u\r\n",
        (unsigned int)AD9220_SAMPLE_COUNT,
        bsp_ad9220_is_overrange() ? 1U : 0U);

    min_code = AD9220_DATA_MASK;
    max_code = 0U;
    sum = 0U;

    for (i = 0U; i < AD9220_SAMPLE_COUNT; i++)
    {
        sample_code = bsp_ad9220_get_code(ad9220_samples[i]);

        if (sample_code < min_code)
        {
            min_code = sample_code;
        }

        if (sample_code > max_code)
        {
            max_code = sample_code;
        }

        sum += sample_code;
    }
    avg_int = sum / AD9220_SAMPLE_COUNT;

    avg_frac =
        ((sum % AD9220_SAMPLE_COUNT) * 100U) /
        AD9220_SAMPLE_COUNT;

    Debug_printf(
        "[Task] Stats: min = %u, max = %u, avg = %lu.%02lu\r\n",
        (unsigned int)min_code,
        (unsigned int)max_code,
        (unsigned long)avg_int,
        (unsigned long)avg_frac);

    for (i = 0U; i < AD9220_TEST_PRINT_COUNT; i++)
    {
        index = i * step; // 根据步长设置
        raw = ad9220_samples[index];
        // code = (uint16_t)(raw & AD9220_DATA_MASK);
        code = bsp_ad9220_get_code(raw);
        Debug_printf(
            "[Task] Sample: raw = %lu, raw & AD9220_DATA_MASK = %u\r\n",
            (unsigned long)raw,
            (unsigned int)code);
    }
}
/**
 * @brief   按键响应任务：
 */
static void Task_Button_Response(void)
{
    uint8_t cmd;

    if (hmi_cmd_flag == 0U)
    {
        return;
    }

    __disable_irq();
    cmd = hmi_cmd_data;
    hmi_cmd_flag = 0;
    __enable_irq();

    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

    if (cmd == 0xA1)
    {
        Debug_printf("[KEY]KEY=b_wave1 cmd=\"single-cycle waveform;\" \r\n");
        Task_AD9220_Test(cmd);
    }
    else if (cmd == 0xA3)
    {
        Debug_printf("[KEY]KEY=b_wave3 cmd=\"three-cycle waveform;\" \r\n");
        Task_AD9220_Test(cmd);
    }
    else if (cmd == 0xAF)
    {
        Debug_printf("[KEY]KEY=b_spec cmd=\"turn to frequency-domain analysis;\" \r\n");
    }
    else
    {
        Debug_printf("[KEY] error:Unknown cmd;\r\n");
    }
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
    Task_Button_Response();
}

//=========================================================================================================
// 5. 中断回调
//=========================================================================================================

/**
 * @brief USART接收完成回调，只保存命令并重启接收
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        hmi_cmd_data = hmi_rx_buffer[0];
        hmi_cmd_flag = 1;
        HAL_UART_Receive_IT(huart, hmi_rx_buffer, 1);
    }
}
