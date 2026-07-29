#include "app_process.h"

//*********************************************************************************************************
extern UART_HandleTypeDef huart1; // HMI 控制串口
extern UART_HandleTypeDef huart3; // PC 调试串口
//*********************************************************************************************************

static uint8_t hmi_rx_buffer[1];          // HMI 单字节命令接收缓冲区
static char debug_buffer[160];            // PC串口调试发送缓冲区
static volatile uint8_t hmi_cmd_flag = 0; // 新命令标志位
static volatile uint8_t hmi_cmd_data = 0; // 最新命令字节

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
    char cmd_buffer[100];
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

    if (cmd == 'A')
    {
        Debug_printf("[KEY]KEY=B cmd=\"process B!\" \r\n");
    }
    else if (cmd == 'B')
    {
        Debug_printf("[KEY]KEY=B cmd=\"process B!\" \r\n");
    }
    else
    {
        Debug_printf("[KEY] Unknown cmd\r\n");
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
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    HAL_Delay(500);
    // Task_Button_Response();
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
