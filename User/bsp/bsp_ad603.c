#include "bsp_ad603.h"

#include "dac.h"

extern DAC_HandleTypeDef hdac;

static ad603_gain_t current_gain = ad603_gain_low;
static uint16_t current_dac_code = AD603_DAC_LOW;

/*
 * 手册公式给出的25/40/55dB仅用于计算初始DAC码。
 * 不同模块的控制电压与实际增益会有偏差，后续需整链实测标定。
 * BSP只输出三档控制电压，算法不能把三档理论值作为最终精确增益。
 */
static bool ad603_get_dac_code(ad603_gain_t gain, uint16_t *dac_code)
{
    switch (gain)
    {
        case ad603_gain_low:
            *dac_code = AD603_DAC_LOW;
            break;

        case ad603_gain_mid:
            *dac_code = AD603_DAC_MID;
            break;

        case ad603_gain_high:
            *dac_code = AD603_DAC_HIGH;
            break;

        default:
            return false;
    }

    return true;
}

bool bsp_ad603_init(void)
{
    if (hdac.Instance != DAC)
    {
        return false;
    }

    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
    {
        return false;
    }

    return bsp_ad603_set_gain(ad603_gain_low);
}

bool bsp_ad603_set_gain(ad603_gain_t gain)
{
    uint16_t dac_code;

    if (!ad603_get_dac_code(gain, &dac_code))
    {
        return false;
    }

    if (HAL_DAC_SetValue(&hdac,
                         DAC_CHANNEL_1,
                         DAC_ALIGN_12B_R,
                         dac_code) != HAL_OK)
    {
        return false;
    }

    HAL_Delay(1U);
    current_gain = gain;
    current_dac_code = dac_code;
    return true;
}

ad603_gain_t bsp_ad603_get_gain(void)
{
    return current_gain;
}

uint8_t bsp_ad603_get_gain_db(void)
{
    static const uint8_t gain_db[] = {25U, 40U, 55U};

    return gain_db[(uint32_t)current_gain];
}

uint16_t bsp_ad603_get_dac_code(void)
{
    return current_dac_code;
}
