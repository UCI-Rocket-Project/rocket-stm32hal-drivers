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
    using DataCallback = void (*)(const int16_t *values, size_t count);

    ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, uint16_t dataReadyPin, DataCallback dataCallback);
    bool Init();

    std::optional<uint8_t> ReadRegisterBlocking(uint8_t address);
    bool WriteRegisterBlocking(uint8_t address, uint8_t value);

    void ReadRegisterDMA(uint8_t address, size_t cnt);

    void FIFOWatermarkIntCallback(uint16_t pin);

    void TxCompleteCallback(SPI_HandleTypeDef *completeSPI);

    void ProcessTask();

private:
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
    DataCallback dataCallback;

    bool init;

    uint8_t txBuffer[READ_SIZE]{};
    uint8_t rxBuffer[READ_SIZE]{};
    int16_t fifoValues[AXES * FIFO_MAX_SETS]{};

    uint16_t fifoCount;
    bool dataNeedsProcessing = false;
};
