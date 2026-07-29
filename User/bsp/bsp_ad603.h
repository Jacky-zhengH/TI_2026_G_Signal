#ifndef BSP_AD603_H
#define BSP_AD603_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    ad603_gain_low = 0,
    ad603_gain_mid,
    ad603_gain_high
} ad603_gain_t;

#define AD603_DAC_LOW   1226U
#define AD603_DAC_MID   1470U
#define AD603_DAC_HIGH  1713U

bool bsp_ad603_init(void);
bool bsp_ad603_set_gain(ad603_gain_t gain);
ad603_gain_t bsp_ad603_get_gain(void);
uint8_t bsp_ad603_get_gain_db(void);
uint16_t bsp_ad603_get_dac_code(void);

#endif /* BSP_AD603_H */
