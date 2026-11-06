#include "accel_adxl383.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

ADXL383::ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin,
                 uint16_t dataReadyPin, DataCallback dataCallback,
                 Time10usCounter getCurrentTime10us)
    : spi(spi), nCS_Port(nCS_Port), nCS_Pin(nCS_Pin), dataReadyPin(dataReadyPin),
      dataCallback(dataCallback), getCurrentTime10us(getCurrentTime10us) {

}

std::optional<uint8_t> ADXL383::ReadRegisterBlocking(uint8_t address) {
    uint8_t tx[2] = {static_cast<uint8_t>((address << 1) | 0x01), 0x00}; // As per page 55
    uint8_t rx[2] = {0x00, 0x00};

    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef stat = HAL_SPI_TransmitReceive(spi, tx, rx, 2, 100);
    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);

    if (stat == HAL_OK) {
        return rx[1];
    }
    return std::nullopt;
}

bool ADXL383::WriteRegisterBlocking(uint8_t address, uint8_t value) {
    uint8_t tx[2] = {static_cast<uint8_t>(address << 1), value}; // As per page 55

    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef stat = HAL_SPI_Transmit(spi, tx, 2, 100);
    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);

    return stat == HAL_OK;
}

void ADXL383::TxCompleteCallback(SPI_HandleTypeDef *completeSPI) {
    if (completeSPI != spi) return;
    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);

    // Deal with FIFO Fault
    if (recoveryRequested && !recovering) {
        recovering = true;
        ++readRate.recoveryCount;
        recoveryRequested = false;
        state = State::EnteringStandby;
        return;
    }

    switch (state) {
        case State::WaitingForDataCount:
            fifoCount = rxBuffer[1] | ((rxBuffer[2] & 0x01) << 8);
            state = State::ReadingData;
            break;
        case State::WaitingForReadData:
            dataNeedsProcessing = true;
            state = State::ClearingWatermark;
            break;
        case State::WaitingForClearedWatermark:
            if (recovering) {
                state = State::EnteringHPOPMode;
            } else {
                state = State::Idle;
            }
            break;
        case State::WaitingForStandby:
            state = State::ReadingDataCount;
            break;
        case State::WaitingToEnterHPOPMode:
            lastWatermarkInterruptTime10us = getCurrentTime10us();
            state = State::Idle;
            recovering = false;
            break;
        default:
            break;
    }
}

void ADXL383::FIFOWatermarkIntCallback(uint16_t pin) {
    if (pin != dataReadyPin) return; // Ensure the correct GPIO interrupt has been called

    if (recovering) return;

    lastWatermarkInterruptTime10us = getCurrentTime10us();

    if (state == State::Idle) {
        state = State::ReadingDataCount;
    } else {
        recoveryRequested = true;
    }
}

void ADXL383::ProcessTask() {
    const uint32_t lastInterruptTime10us = lastWatermarkInterruptTime10us;
    const uint32_t elapsed10us = getCurrentTime10us() - lastInterruptTime10us;
    if (init && !recovering && elapsed10us >= RecoveryWatchdogTimeout10us) {
        recoveryRequested = true;
    }

    const bool betweenTransfers =
        state == State::Idle ||
        state == State::ReadingDataCount ||
        state == State::ReadingData ||
        state == State::ClearingWatermark;

    if (recoveryRequested && !recovering && betweenTransfers) {
        recovering = true;
        ++readRate.recoveryCount;
        recoveryRequested = false;
        state = State::EnteringStandby;
    }

    switch (state) {
        case State::ReadingDataCount: {
            state = State::WaitingForDataCount;
            txBuffer[0] = static_cast<uint8_t>((FIFO_STATUS0 << 1) | 0x01);

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 4) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ReadingDataCount;
            }
            break;
        }
        case State::ReadingData: {
            state = State::WaitingForReadData;    
            txBuffer[0] = static_cast<uint8_t>((FIFO_DATA << 1) | 0x01);

            uint16_t fullSets = fifoCount / AXES; // Truncation is intentional
            if (fullSets > FIFO_MAX_SETS) fullSets = FIFO_MAX_SETS;

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 1 + fullSets * AXES * BYTES_PER_AXIS) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ReadingData;
            }
            break;
        }
        case State::ClearingWatermark: {
            if (dataNeedsProcessing) {
                // Measure when the task consumes each completed FIFO data read.
                const uint32_t readTime10us = getCurrentTime10us();
                dataNeedsProcessing = false;

                uint16_t fullSets = fifoCount / AXES;
                if (fullSets > FIFO_MAX_SETS) fullSets = FIFO_MAX_SETS;
                readRate.fifoAxisEntries = fifoCount;
                readRate.xyzSampleSets = fullSets;

                if (hasPreviousDataRead) {
                    const uint32_t elapsed10us = readTime10us - previousDataReadTime10us;
                    if (elapsed10us != 0) {
                        readRate.intervalMicroseconds = static_cast<uint32_t>(
                            static_cast<uint64_t>(elapsed10us) * 10U);
                        readRate.estimatedXyzSampleRateHz = static_cast<uint32_t>(
                            (static_cast<uint64_t>(fullSets) * 100000U) / elapsed10us);
                    }
                }
                previousDataReadTime10us = readTime10us;
                hasPreviousDataRead = true;

                // volatile uint16_t fullSets = fifoCount / AXES;
                // if (fullSets > FIFO_MAX_SETS) fullSets = FIFO_MAX_SETS;
                // const size_t valueCount = fullSets * AXES;
                // for (size_t i = 0; i < valueCount; ++i) {
                //     const size_t offset = 1 + i * BYTES_PER_AXIS;
                //     fifoValues[i] = static_cast<int16_t>((static_cast<uint16_t>(rxBuffer[offset]) << 8) | rxBuffer[offset + 1]);
                // }
                // if (dataCallback) dataCallback(fifoValues, valueCount);
            }
            state = State::WaitingForClearedWatermark;    
            txBuffer[0] = static_cast<uint8_t>((STATUS0 << 1) | 0x01);

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 2) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ClearingWatermark;
            }
            break;
        }
        case State::EnteringStandby: {
            state = State::WaitingForStandby;
            txBuffer[0] = static_cast<uint8_t>(OP_MODE << 1);
            txBuffer[1] = 0x00; // Standby

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 2) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::EnteringStandby;
            }
            break;
        }
        case State::EnteringHPOPMode: {
            state = State::WaitingToEnterHPOPMode;

            // Enable ±60 g HP mode
            constexpr uint8_t range30g  = 0b01000000;
            constexpr uint8_t range60g  = 0b10000000;
            constexpr uint8_t pdmMode   = 0b00100000;
            constexpr uint8_t audioMode = 0b00010000;
            constexpr uint8_t hpMode    = 0b00001100;

            txBuffer[0] = static_cast<uint8_t>(OP_MODE << 1);
            txBuffer[1] = range60g | hpMode;

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 2) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::EnteringHPOPMode;
            }
            break;
        }
        default:
            break;
    }
}

bool ADXL383::Init() {
    // Deselect IC & allow for startup
    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // Verify comms are good
    for (int attempts = 0; attempts < 3; attempts++) {
        auto devId = ReadRegisterBlocking(DEVID_AD);
        if (devId && *devId == EXPECTED_DEVID_AD) break;
        else if (attempts == 2) {
            init = false;
            return false;
        }
    }

    // Reset
    constexpr uint8_t resetCode = 0x52;

    // Enable XYZ & enable FIFO
    constexpr uint8_t tempEn     = 0b10000000;
    constexpr uint8_t zEn        = 0b01000000;
    constexpr uint8_t yEn        = 0b00100000;
    constexpr uint8_t xEn        = 0b00010000;
    constexpr uint8_t fifoEn     = 0b00001000;
    constexpr uint8_t int01Event = 0b00000010;
    constexpr uint8_t parityEn   = 0b00000001;

    // FIFO normal mode, no channel IDs, watermark at 96 axis entries
    constexpr uint8_t fifoNormalMode  = 0b00010000;
    constexpr uint8_t fifoSamples96   = 96;

    // Enable HP mode (32 ksps)
    constexpr uint8_t dec2xBypass = 0b10000000;
    constexpr uint8_t sincRate    = 0b01000000;
    constexpr uint8_t iir1En      = 0b00100000;
    constexpr uint8_t roundMode   = 0b00000100;
    constexpr uint8_t trigSrc     = 0b00000010;
    constexpr uint8_t trigMode    = 0b00000001;

    // Enable FIFO watermark interrupt on INT0
    constexpr uint8_t nvmBusyInt0     = 0b10000000;
    constexpr uint8_t inactInt0       = 0b01000000;
    constexpr uint8_t actInt0         = 0b00100000;
    constexpr uint8_t fifoWaterInt0   = 0b00001000;
    constexpr uint8_t fifoOverrunInt0 = 0b00000100;
    constexpr uint8_t fifoFullInt0    = 0b00000010;
    constexpr uint8_t dataReadyInt0   = 0b00000001;

    // Increase MISO drive strength
    constexpr uint8_t gainScalerBypass = 0b01000000;
    constexpr uint8_t misoPullDown     = 0b00000010;
    constexpr uint8_t misoDrive        = 0b00000001;

    // Enable ±60 g HP mode
    constexpr uint8_t range30g  = 0b01000000;
    constexpr uint8_t range60g  = 0b10000000;
    constexpr uint8_t pdmMode   = 0b00100000;
    constexpr uint8_t audioMode = 0b00010000;
    constexpr uint8_t hpMode    = 0b00001100;

    init = false;
    recoveryRequested = false;
    recovering = false;
    lastWatermarkInterruptTime10us = 0;

    state = State::Idle;
    fifoCount = 0;
    dataNeedsProcessing = false;
    previousDataReadTime10us = 0;
    hasPreviousDataRead = false;
    readRate.intervalMicroseconds = 0;
    readRate.fifoAxisEntries = 0;
    readRate.xyzSampleSets = 0;
    readRate.estimatedXyzSampleRateHz = 0;
    readRate.recoveryCount = 0;

    // Reset
    if (!WriteRegisterBlocking(REG_RESET, resetCode)) return false;
    HAL_Delay(3);

    // Write
    if (!WriteRegisterBlocking(OP_MODE, 0x00) || // Standby
        !WriteRegisterBlocking(DIG_EN, xEn | yEn | zEn | fifoEn) ||
        !WriteRegisterBlocking(FIFO_CFG0, fifoNormalMode) ||
        !WriteRegisterBlocking(FIFO_CFG1, fifoSamples96) ||
        !WriteRegisterBlocking(INT0_MAP0, fifoWaterInt0) ||
        !WriteRegisterBlocking(TRIG_CFG, dec2xBypass) ||
        !WriteRegisterBlocking(MISO, misoDrive) ||
        !WriteRegisterBlocking(OP_MODE, range60g | hpMode)) {
        return false;
    }

    lastWatermarkInterruptTime10us = getCurrentTime10us();

    init = true;

    return true;
}
