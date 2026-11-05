#pragma once

#include "main.h"  // includes "stm32xxxx_hal.h"
#include <cstddef>
#include <cstdint>

class RGBLED {
public:
    static constexpr uint16_t MaxBrightness = 4096;

    struct Color {
        uint16_t red;
        uint16_t green;
        uint16_t blue;
    };

    static constexpr size_t QueueCapacity = 8;

    RGBLED(TIM_HandleTypeDef *htimRed, uint32_t channelRed, TIM_HandleTypeDef *htimGreen, uint32_t channelGreen,
        TIM_HandleTypeDef *htimBlue, uint32_t channelBlue);

    void Init();

    // All times are raw ticks from the clock passed to Process(). Durations
    // begin when Process first selects an operation. Queued operations run FIFO;
    // the default pulse runs only while the queue is empty.
    // Call these methods from one execution context (not concurrently with an ISR).
    // Returns false for a zero duration, an invalid period, or a full queue.
    bool QueueBlink(Color color, uint32_t durationTicks, uint32_t periodTicks);
    bool QueuePulse(Color color, uint32_t durationTicks, uint32_t periodTicks);
    bool SetDefaultPulse(Color color, uint32_t periodTicks);
    void Process(uint32_t nowTicks);

    // Brightness values are in the range 0 (off) to MaxBrightness (full).
    void setRedBrightness(uint16_t brightness);
    void setGreenBrightness(uint16_t brightness);
    void setBlueBrightness(uint16_t brightness);

private:
    enum class OperationType { Blink, Pulse };
    struct Operation {
        OperationType type;
        Color color;
        uint32_t durationTicks;
        uint32_t periodTicks;
    };

    bool Enqueue(OperationType type, Color color, uint32_t durationTicks, uint32_t periodTicks);
    void SetColor(Color color);
    void DisplayPulse(Color color, uint32_t elapsedTicks, uint32_t periodTicks);
    static void SetChannelBrightness(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t brightness);

    TIM_HandleTypeDef *htimRed, *htimGreen, *htimBlue;
    uint32_t channelRed, channelGreen, channelBlue;
    Operation queue[QueueCapacity] = {};
    size_t queueHead = 0;
    size_t queueSize = 0;
    uint32_t activeStartTicks = 0;
    bool activeStarted = false;
    Color defaultPulseColor = {0, 0, 0};
    uint32_t defaultPulsePeriodTicks = 0;
    uint32_t defaultStartTicks = 0;
    bool defaultStarted = false;
};
