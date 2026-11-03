#include "led.h"
#include <math.h>

RGBLED::RGBLED(TIM_HandleTypeDef *htimRed, uint32_t channelRed, TIM_HandleTypeDef *htimGreen, uint32_t channelGreen,
    TIM_HandleTypeDef *htimBlue, uint32_t channelBlue): htimRed(htimRed), htimGreen(htimGreen),
    htimBlue(htimBlue), channelRed(channelRed), channelGreen(channelGreen), channelBlue(channelBlue) {

}

void RGBLED::Init() {
    SetColor({0.0f, 0.0f, 0.0f});
    HAL_TIM_PWM_Start(htimRed, channelRed);
    HAL_TIM_PWM_Start(htimGreen, channelGreen);
    HAL_TIM_PWM_Start(htimBlue, channelBlue);
}

bool RGBLED::QueueBlink(Color color, uint32_t durationTicks, uint32_t periodTicks) {
    return Enqueue(OperationType::Blink, color, durationTicks, periodTicks);
}

bool RGBLED::QueuePulse(Color color, uint32_t durationTicks, uint32_t periodTicks) {
    return Enqueue(OperationType::Pulse, color, durationTicks, periodTicks);
}

bool RGBLED::Enqueue(OperationType type, Color color, uint32_t durationTicks, uint32_t periodTicks) {
    if (durationTicks == 0 || periodTicks == 0 || (type == OperationType::Blink && periodTicks < 2)
        || queueSize == QueueCapacity) return false;

    queue[(queueHead + queueSize) % QueueCapacity] = {type, color, durationTicks, periodTicks};
    ++queueSize;
    return true;
}

bool RGBLED::SetDefaultPulse(Color color, uint32_t periodTicks) {
    if (periodTicks == 0) return false;
    defaultPulseColor = color;
    defaultPulsePeriodTicks = periodTicks;
    defaultStarted = false;
    return true;
}

void RGBLED::Process(uint32_t nowTicks) {
    while (queueSize != 0) {
        if (!activeStarted) {
            activeStartTicks = nowTicks;
            activeStarted = true;
        }

        const Operation &operation = queue[queueHead];
        const uint32_t elapsedTicks = nowTicks - activeStartTicks;
        if (elapsedTicks < operation.durationTicks) {
            defaultStarted = false;
            if (operation.type == OperationType::Blink) {
                const bool on = (elapsedTicks % operation.periodTicks) < (operation.periodTicks / 2U);
                SetColor(on ? operation.color : Color{0.0f, 0.0f, 0.0f});
            } else {
                DisplayPulse(operation.color, elapsedTicks, operation.periodTicks);
            }
            return;
        }

        queueHead = (queueHead + 1) % QueueCapacity;
        --queueSize;
        activeStarted = false;
    }

    if (defaultPulsePeriodTicks != 0) {
        if (!defaultStarted) {
            defaultStartTicks = nowTicks;
            defaultStarted = true;
        }
        DisplayPulse(defaultPulseColor, nowTicks - defaultStartTicks, defaultPulsePeriodTicks);
    } else {
        SetColor({0.0f, 0.0f, 0.0f});
    }
}

void RGBLED::DisplayPulse(Color color, uint32_t elapsedTicks, uint32_t periodTicks) {
    const float phase = 2.0f * 3.141592f * static_cast<float>(elapsedTicks % periodTicks)
        / static_cast<float>(periodTicks);
    float wave = 0.5f * sinf(phase + 2.0f) + 0.5f;
    wave = wave * wave * wave;
    SetColor({color.red * wave, color.green * wave, color.blue * wave});
}

void RGBLED::SetColor(Color color) {
    setRedBrightness(color.red);
    setGreenBrightness(color.green);
    setBlueBrightness(color.blue);
}

void RGBLED::SetChannelBrightness(TIM_HandleTypeDef *htim, uint32_t channel, float brightness) {
    if (!(brightness >= 0.0f)) brightness = 0.0f;
    if (brightness > 1.0f) brightness = 1.0f;
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    const uint32_t pulse = static_cast<uint32_t>(period * (1.0f - brightness) + 0.5f);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void RGBLED::setRedBrightness(float brightness) {
    SetChannelBrightness(htimRed, channelRed, brightness);
}

void RGBLED::setGreenBrightness(float brightness) {
    SetChannelBrightness(htimGreen, channelGreen, brightness);
}

void RGBLED::setBlueBrightness(float brightness) {
    SetChannelBrightness(htimBlue, channelBlue, brightness);
}
