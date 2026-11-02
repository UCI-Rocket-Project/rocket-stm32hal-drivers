#pragma once

#include "main.h"  // includes "stm32xxxx_hal.h"

class RGBLED {
public:

    RGBLED(TIM_HandleTypeDef *htimRed, uint32_t channelRed, TIM_HandleTypeDef *htimGreen, uint32_t channelGreen,
        TIM_HandleTypeDef *htimBlue, uint32_t channelBlue);

    void Init();

    void setRedBrightness(float duty_cycle);
    void setGreenBrightness(float duty_cycle);
    void setBlueBrightness(float duty_cycle);

private:
    TIM_HandleTypeDef *htimRed, *htimGreen, *htimBlue;
    uint32_t channelRed, channelGreen, channelBlue;
};
