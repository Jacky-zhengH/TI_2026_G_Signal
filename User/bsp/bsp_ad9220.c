#include "bsp_ad9220.h"

#include "tim.h"

#define AD9220_DMA_MAX_COUNT 0xFFFFU

extern TIM_HandleTypeDef htim1;
extern DMA_HandleTypeDef hdma_tim1_ch3;

static bool ad9220_stop_dma(void)
{
    HAL_DMA_StateTypeDef state;

    state = HAL_DMA_GetState(&hdma_tim1_ch3);
    if (state == HAL_DMA_STATE_BUSY)
    {
        return HAL_DMA_Abort(&hdma_tim1_ch3) == HAL_OK;
    }

    /*
     * HAL_DMA_PollForTransfer超时会先把状态改为READY，
     * 此时DMA流可能仍开启，恢复BUSY后用HAL安全终止。
     */
    if ((hdma_tim1_ch3.Instance->CR & DMA_SxCR_EN) != 0U)
    {
        hdma_tim1_ch3.State = HAL_DMA_STATE_BUSY;
        return HAL_DMA_Abort(&hdma_tim1_ch3) == HAL_OK;
    }

    return state == HAL_DMA_STATE_READY;
}

bool bsp_ad9220_init(void)
{
    if ((htim1.Instance != TIM1) ||
        (hdma_tim1_ch3.Instance != DMA2_Stream6) ||
        (htim1.hdma[TIM_DMA_ID_CC3] != &hdma_tim1_ch3))
    {
        return false;
    }

    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        return false;
    }

    HAL_Delay(1U);
    return true;
}

bool bsp_ad9220_capture(uint32_t *buffer, uint32_t count, uint32_t timeout_ms)
{
    HAL_StatusTypeDef status;

    if ((buffer == NULL) || (count == 0U) || (count > AD9220_DMA_MAX_COUNT))
    {
        return false;
    }
    /*关闭定时器DMA事件*/
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);

    if (!ad9220_stop_dma())
    {
        return false;
    }
    /*清除上一次传输留下的DMA标志*/
    __HAL_DMA_CLEAR_FLAG(&hdma_tim1_ch3,
                         __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_tim1_ch3) |
                             __HAL_DMA_GET_HT_FLAG_INDEX(&hdma_tim1_ch3) |
                             __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_tim1_ch3) |
                             __HAL_DMA_GET_DME_FLAG_INDEX(&hdma_tim1_ch3) |
                             __HAL_DMA_GET_FE_FLAG_INDEX(&hdma_tim1_ch3));
    /**
     * DMA传输：
     * 源地址为GPIOE->IDR，即PE端口引脚（后面取低12位即可）
     * 目标地址为 para ： buffer【通常为app层的采样数组】
     */
    status = HAL_DMA_Start(&hdma_tim1_ch3,
                           (uint32_t)&GPIOE->IDR,
                           (uint32_t)buffer,
                           count);

    if (status != HAL_OK)
    {
        return false;
    }
    /*使能定时器DMA事件*/
    __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_CC3);
    status = HAL_DMA_PollForTransfer(&hdma_tim1_ch3,
                                     HAL_DMA_FULL_TRANSFER,
                                     timeout_ms); // 超时判断
    __HAL_TIM_DISABLE_DMA(&htim1, TIM_DMA_CC3);

    if (status != HAL_OK)
    {
        (void)ad9220_stop_dma();
        return false;
    }

    return true;
}

bool bsp_ad9220_is_overrange(void)
{
    return HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == GPIO_PIN_SET;
}

uint32_t bsp_ad9220_get_sample_rate(void)
{
    return AD9220_SAMPLE_RATE_HZ;
}

/*掩码*/
uint16_t bsp_ad9220_get_code(uint32_t raw)
{
    return (uint16_t)(raw & AD9220_DATA_MASK);
}
