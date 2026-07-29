#ifndef __BSP_AD9220_H
#define __BSP_AD9220_H
/**
 * 模块功能：12bit并行ADC采集，最高支持40MHz采样率
 * 驱动方式：12并行GPIO输入 TIM1-CH1输出采样时钟 CH3 指定事件触发DMA
 */
/*----标准库----*/
#include "stdbool.h"
#include "stdint.h"
/*----stm32 f4 hal库----*/
#include "stm32f4xx_hal.h"
/*---- 引脚宏定义 ----*/
// D0~D11  --> PC0~PC11
// OTR过量程提示 --> PC12

//=======================================
// 函数声明
//=======================================

void bsp_adc9920_init(void);
bool bsp_ad9220_capture(uint32_t *buffer, uint32_t count, uint32_t timeout_ms);
bool bsp_ad9220_IsOverrange(void);
uint32_t bsp_ad9220_GetRawSampleRate(void);

#endif /* bsp_ad9220.h*/
