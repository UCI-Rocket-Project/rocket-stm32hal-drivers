#include "accel_adxl383.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"

ADXL383::ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin): spi(spi), nCS_Port(nCS_Port), nCS_Pin(nCS_Pin) {

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

bool ADXL383::Init() {
    // Deselect IC & allow for startup
    HAL_GPIO_WritePin(nCS_Port, nCS_Pin, GPIO_PIN_SET);
    HAL_Delay(3);

    // Verify comms are good
    for (int attempts = 0; attempts < 3; attempts++) {
        auto devId = ReadRegisterBlocking(DEVID_AD);
        if (devId && *devId == EXPECTED_DEVID_AD) break;
        else if (attempts == 2) {
            init = false;
            return false;
        }
    }

    // Enable XYZ & disable FIFO
    constexpr uint8_t tempEn     = 0b10000000;
    constexpr uint8_t zEn        = 0b01000000;
    constexpr uint8_t yEn        = 0b00100000;
    constexpr uint8_t xEn        = 0b00010000;
    constexpr uint8_t fifoEn     = 0b00001000;
    constexpr uint8_t int01Event = 0b00000010;
    constexpr uint8_t parityEn   = 0b00000001;

    // Enable HP mode (32 ksps)
    constexpr uint8_t dec2xBypass = 0b10000000;
    constexpr uint8_t sincRate    = 0b01000000;
    constexpr uint8_t iir1En      = 0b00100000;
    constexpr uint8_t roundMode   = 0b00000100;
    constexpr uint8_t trigSrc     = 0b00000010;
    constexpr uint8_t trigMode    = 0b00000001;

    // Enable data-ready interrupt on INT0
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

    // Write
    if (!WriteRegisterBlocking(OP_MODE, 0x00) || // Standby
        !WriteRegisterBlocking(DIG_EN, xEn | yEn | zEn) ||
        !WriteRegisterBlocking(TRIG_CFG, dec2xBypass) ||
        !WriteRegisterBlocking(INT0_MAP0, dataReadyInt0) ||
        !WriteRegisterBlocking(MISO, misoDrive) ||
        !WriteRegisterBlocking(OP_MODE, range60g | hpMode)) {
        return false;
    }

    HAL_Delay(2); // DS mandated 2 ms wait before reading can commence

    init = true;

    return true;
}