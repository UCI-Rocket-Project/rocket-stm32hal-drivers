#pragma once

#include "main.h"
#include "stm32f105xc.h"
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
    ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, uint16_t dataReadyPin);
    bool Init();

    std::optional<uint8_t> ReadRegisterBlocking(uint8_t address);
    bool WriteRegisterBlocking(uint8_t address, uint8_t value);

    void ReadRegisterDMA(uint8_t address, size_t cnt);

    void FIFOWatermarkIntCallback(uint16_t pin);

    void TxCompleteCallback(SPI_HandleTypeDef *completeSPI);

    void ProcessTask();

private:
    void UpdateAveragesFromFIFO(uint16_t fullSets);

    static constexpr uint32_t AverageWindowMs = 1000;
    struct AverageBucket {
        int32_t xSum;
        int32_t ySum;
        int32_t zSum;
        uint16_t samples;
    };

    enum class State {
        ReadingDataCount,
        WaitingForDataCount,
        ReadingData,
        WaitingForReadData,
        ClearingWatermark,
        WaitingForClearedWatermark,
        Idle 
    };

    State state;

    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nCS_Port;
    uint16_t nCS_Pin;
    uint16_t dataReadyPin;

    bool init;

    uint8_t txBuffer[READ_SIZE]{};
    uint8_t rxBuffer[READ_SIZE]{};

    uint16_t fifoCount;
    AverageBucket averageBuckets[AverageWindowMs]{};
    int64_t xSum = 0;
    int64_t ySum = 0;
    int64_t zSum = 0;
    uint32_t averageSamples = 0;
    uint32_t lastAverageMs = 0;
    bool averageStarted = false;
};
