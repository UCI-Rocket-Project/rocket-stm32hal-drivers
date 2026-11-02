#include "led.h"

RGBLED::RGBLED(TIM_HandleTypeDef *htimRed, uint32_t channelRed, TIM_HandleTypeDef *htimGreen, uint32_t channelGreen,
    TIM_HandleTypeDef *htimBlue, uint32_t channelBlue): htimRed(htimRed), htimGreen(htimGreen),
    htimBlue(htimBlue), channelRed(channelRed), channelGreen(channelGreen), channelBlue(channelBlue) {

}

void RGBLED::Init() {
    HAL_TIM_PWM_Start(htimRed, channelRed);
    HAL_TIM_PWM_Start(htimGreen, channelGreen);
    HAL_TIM_PWM_Start(htimBlue, channelBlue);
}

void RGBLED::setRedBrightness(float duty_cycle) {
    if (duty_cycle > 1) duty_cycle = 1;
    if (duty_cycle < 0) duty_cycle = 0;
    uint32_t pulse = 36000U * (1.0f - duty_cycle) / 1.0f;
    __HAL_TIM_SET_COMPARE(htimRed, channelRed, pulse);
}

void RGBLED::setGreenBrightness(float duty_cycle) {
    if (duty_cycle > 1) duty_cycle = 1;
    if (duty_cycle < 0) duty_cycle = 0;
    uint32_t pulse = 36000U * (1.0f - duty_cycle) / 1.0f;
    __HAL_TIM_SET_COMPARE(htimGreen, channelGreen, pulse);
}

void RGBLED::setBlueBrightness(float duty_cycle) {
    if (duty_cycle > 100) duty_cycle = 100;
    uint32_t pulse = 36000U * (100U - duty_cycle) / 100U;
    __HAL_TIM_SET_COMPARE(htimBlue, channelBlue, pulse);
}