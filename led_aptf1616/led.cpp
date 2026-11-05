#include "led.h"

namespace {
// Half of a cosine-shaped pulse, cubed to match the old fade curve.
// Values are 0..4096; the other half is its mirror image.
constexpr uint16_t PulseWave[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2,
    2, 3, 4, 5, 6, 7, 9, 11, 13, 15, 18, 21,
    25, 29, 34, 39, 45, 51, 59, 67, 76, 85, 96, 108,
    120, 134, 149, 166, 183, 202, 222, 244, 267, 292, 318, 346,
    376, 407, 440, 475, 512, 551, 591, 634, 678, 724, 772, 822,
    874, 928, 983, 1041, 1100, 1161, 1223, 1288, 1353, 1421, 1490, 1560,
    1631, 1704, 1777, 1852, 1927, 2003, 2080, 2158, 2235, 2313, 2391, 2469,
    2547, 2625, 2702, 2778, 2854, 2928, 3002, 3074, 3145, 3215, 3283, 3349,
    3413, 3474, 3534, 3591, 3646, 3698, 3747, 3794, 3837, 3877, 3915, 3949,
    3979, 4006, 4030, 4050, 4066, 4079, 4089, 4094, 4096,
};
static_assert(sizeof(PulseWave) / sizeof(PulseWave[0]) == 129);
}

RGBLED::RGBLED(TIM_HandleTypeDef *htimRed, uint32_t channelRed, TIM_HandleTypeDef *htimGreen, uint32_t channelGreen,
    TIM_HandleTypeDef *htimBlue, uint32_t channelBlue): htimRed(htimRed), htimGreen(htimGreen),
    htimBlue(htimBlue), channelRed(channelRed), channelGreen(channelGreen), channelBlue(channelBlue) {

}

void RGBLED::Init() {
    SetColor({0, 0, 0});
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
                SetColor(on ? operation.color : Color{0, 0, 0});
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
        SetColor({0, 0, 0});
    }
}

void RGBLED::DisplayPulse(Color color, uint32_t elapsedTicks, uint32_t periodTicks) {
    const uint32_t ticksInPeriod = elapsedTicks % periodTicks;
    // Normal LED periods fit in 24 bits, so both phase operations stay 32-bit.
    const uint32_t phase = periodTicks < (1U << 24)
        ? (ticksInPeriod << 8) / periodTicks
        : static_cast<uint32_t>((static_cast<uint64_t>(ticksInPeriod) << 8) / periodTicks);
    const uint16_t wave = PulseWave[phase <= 128U ? phase : 256U - phase];
    SetColor({
        static_cast<uint16_t>((static_cast<uint32_t>(color.red) * wave + 2048U) >> 12),
        static_cast<uint16_t>((static_cast<uint32_t>(color.green) * wave + 2048U) >> 12),
        static_cast<uint16_t>((static_cast<uint32_t>(color.blue) * wave + 2048U) >> 12),
    });
}

void RGBLED::SetColor(Color color) {
    setRedBrightness(color.red);
    setGreenBrightness(color.green);
    setBlueBrightness(color.blue);
}

void RGBLED::SetChannelBrightness(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t brightness) {
    if (brightness > MaxBrightness) brightness = MaxBrightness;
    const uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim) + 1U;
    const uint32_t pulse = static_cast<uint32_t>(
        (static_cast<uint64_t>(period) * (MaxBrightness - brightness) + 2048U) >> 12);
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

void RGBLED::setRedBrightness(uint16_t brightness) {
    SetChannelBrightness(htimRed, channelRed, brightness);
}

void RGBLED::setGreenBrightness(uint16_t brightness) {
    SetChannelBrightness(htimGreen, channelGreen, brightness);
}

void RGBLED::setBlueBrightness(uint16_t brightness) {
    SetChannelBrightness(htimBlue, channelBlue, brightness);
}
