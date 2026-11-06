#pragma once

#include "main.h"
#include "stm32f105xc.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include <optional>
#include "accel_adxl383_registers.h"

/* Three enabled axes give 318 FIFO entries, or 106 complete sets. */
#define BYTES_PER_AXIS 2
#define AXES 3
#define FIFO_MAX_SETS 106
#define READ_SIZE (BYTES_PER_AXIS * AXES * FIFO_MAX_SETS + 1)

class ADXL383 {

public:
    using DataCallback = void (*)(const int16_t *values, size_t count);
    using RecoveryCallback = void (*)();
    using Time10usCounter = uint32_t (*)();

    ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin,
            uint16_t dataReadyPin, DataCallback dataCallback, RecoveryCallback recoveryCallback,
            Time10usCounter getCurrentTime10us);
    bool Init();

    std::optional<uint8_t> ReadRegisterBlocking(uint8_t address);
    bool WriteRegisterBlocking(uint8_t address, uint8_t value);

    void ReadRegisterDMA(uint8_t address, size_t cnt);

    void FIFOWatermarkIntCallback(uint16_t pin);

    void TxCompleteCallback(SPI_HandleTypeDef *completeSPI);

    void ProcessTask();

    // Inspect via AMIController::instance->accel. The first read establishes
    // the interval baseline; interval and rate update from the second read.
    struct ReadRate {
        volatile uint32_t intervalMicroseconds = 0;
        volatile uint16_t fifoAxisEntries = 0; // Raw count from FIFO_STATUS.
        volatile uint16_t xyzSampleSets = 0; // Complete XYZ sets actually read.
        volatile uint32_t estimatedXyzSampleRateHz = 0; // Sets / interval.
        volatile uint32_t recoveryCount = 0; // Recovery sequences started.
    } readRate;

private:
    enum class State {
        ReadingDataCount,
        WaitingForDataCount,
        ReadingData,
        WaitingForReadData,
        ClearingWatermark,
        WaitingForClearedWatermark,
        Idle,

        EnteringStandby,
        WaitingForStandby,
        EnteringHPOPMode,
        WaitingToEnterHPOPMode
    };

    State state;

    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nCS_Port;
    uint16_t nCS_Pin;
    uint16_t dataReadyPin;
    DataCallback dataCallback;
    RecoveryCallback recoveryCallback;
    Time10usCounter getCurrentTime10us;

    void BeginRecovery();

    bool init;

    volatile bool recoveryRequested = false;
    volatile bool recovering = false;

    static constexpr uint32_t RecoveryWatchdogTimeout10us = 4U * 100U;
    volatile uint32_t lastWatermarkInterruptTime10us = 0;

    // DMA reads change only byte 0. The remaining zero-initialized bytes supply
    // zero dummy bytes for every transfer, including the 4-byte count read.
    uint8_t txBuffer[READ_SIZE]{};
    uint8_t rxBuffer[READ_SIZE]; // DMA overwrites every byte that is read.
    int16_t fifoValues[AXES * FIFO_MAX_SETS]{};

    volatile uint16_t fifoCount;
    volatile bool dataNeedsProcessing = false;
    uint32_t previousDataReadTime10us = 0;
    bool hasPreviousDataRead = false;
};
