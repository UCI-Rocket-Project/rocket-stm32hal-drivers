#include "accel_adxl383.h"
#include <cstring>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

extern "C" {
volatile int16_t x_avg = 0;
volatile int16_t y_avg = 0;
volatile int16_t z_avg = 0;
}

ADXL383::ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin, uint16_t dataReadyPin): spi(spi), nCS_Port(nCS_Port), nCS_Pin(nCS_Pin), dataReadyPin(dataReadyPin) {

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

void ADXL383::UpdateAveragesFromFIFO(uint16_t fullSets) {
    size_t offset = 1;
    x_avg = static_cast<int16_t>((static_cast<uint16_t>(rxBuffer[offset]) << 8) | rxBuffer[offset + 1]);
    y_avg = static_cast<int16_t>((static_cast<uint16_t>(rxBuffer[offset + 2]) << 8) | rxBuffer[offset + 3]);
    z_avg = static_cast<int16_t>((static_cast<uint16_t>(rxBuffer[offset + 4]) << 8) | rxBuffer[offset + 5]);
}

size_t i = 0;
HAL_StatusTypeDef reads[100] = {HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT, HAL_TIMEOUT};
uint32_t readscnt = 0;

void ADXL383::ReadRegisterDMA(uint8_t address, size_t cnt) {
    // Multibyte read
    txBuffer[0] = static_cast<uint8_t>((address << 1) | 0x01);

    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
    reads[i++] = HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, cnt + 1);
    if (reads[i - 1] != HAL_OK) {
        HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
    }

    readscnt++;
}

void ADXL383::TxCompleteCallback(SPI_HandleTypeDef *completeSPI) {
    if (completeSPI != spi) return;

    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
    switch (state) {
        case State::WaitingForDataCount:
            fifoCount = rxBuffer[1] | ((rxBuffer[2] & 0x01) << 8);
            state = State::ReadingData;
            break;
        case State::WaitingForReadData:
            UpdateAveragesFromFIFO(fifoCount / AXES);
            state = State::ClearingWatermark;
            break;
        case State::WaitingForClearedWatermark:
            state = State::Idle;
            break;
        default:
            break;
    }

}

uint32_t datas = 0;

void ADXL383::FIFOWatermarkIntCallback(uint16_t pin) {
    if (pin != dataReadyPin) return; // Ensure the correct GPIO interrupt has been called

    if (state == State::Idle) {
        state = State::ReadingDataCount;
    } else {
        // Do something weird here we shouldn't get here, means we're running too slow
    }

    datas++;
}

void ADXL383::ProcessTask() {
    switch (state) {
        case State::ReadingDataCount:
            state = State::WaitingForDataCount;
            memset(txBuffer, 0, READ_SIZE);
            memset(rxBuffer, 0, READ_SIZE);
            txBuffer[0] = static_cast<uint8_t>((FIFO_STATUS0 << 1) | 0x01);

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 4) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ReadingDataCount;
            }
            break;
        case State::ReadingData: {
            state = State::WaitingForReadData;    
            memset(txBuffer, 0, READ_SIZE);
            memset(rxBuffer, 0, READ_SIZE);
            txBuffer[0] = static_cast<uint8_t>((FIFO_DATA << 1) | 0x01);

            uint16_t fullSets = fifoCount / AXES; // Truncation is intentional

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 1 + fullSets * AXES * BYTES_PER_AXIS) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ReadingData;
            }
            break;
        }
        case State::ClearingWatermark:
            state = State::WaitingForClearedWatermark;    
            memset(txBuffer, 0, READ_SIZE);
            memset(rxBuffer, 0, READ_SIZE);
            txBuffer[0] = static_cast<uint8_t>((STATUS0 << 1) | 0x01);

            HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_RESET);
            if (HAL_SPI_TransmitReceive_DMA(spi, txBuffer, rxBuffer, 2) != HAL_OK) {
                HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
                state = State::ClearingWatermark;
            }
            break;
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

    state = State::ReadingDataCount;
    fifoCount = 0;
    memset(averageBuckets, 0, sizeof(averageBuckets));
    xSum = ySum = zSum = 0;
    averageSamples = 0;
    lastAverageMs = 0;
    averageStarted = false;
    x_avg = y_avg = z_avg = 0.0f;

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

    HAL_Delay(2); // DS mandated 2 ms wait before reading can commence

    init = true;

    return true;
}
