#ifndef BSP_AD9220_H
#define BSP_AD9220_H

#include <stdbool.h>
#include <stdint.h>

/*
 * AD9220 D0~D11连接PE0~PE11，OTR连接PE12。
 * PA8/TIM1_CH1固定输出2MHz采样时钟，TIM1_CH3事件触发DMA读取GPIOE->IDR。
 */
#define AD9220_SAMPLE_RATE_HZ  2000000U
#define AD9220_SAMPLE_COUNT    4096U
#define AD9220_DATA_MASK       0x0FFFU

bool bsp_ad9220_init(void);
bool bsp_ad9220_capture(uint32_t *buffer, uint32_t count, uint32_t timeout_ms);
bool bsp_ad9220_is_overrange(void);
uint32_t bsp_ad9220_get_sample_rate(void);
uint16_t bsp_ad9220_get_code(uint32_t raw);

#endif /* BSP_AD9220_H */
