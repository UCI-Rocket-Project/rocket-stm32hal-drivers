#pragma once

#include "main.h"
#include "stm32f105xc.h"
#include "stm32f1xx_hal_spi.h"
#include <optional>
#include "accel_adxl383_registers.h"

class ADXL383 {

public:
    ADXL383(SPI_HandleTypeDef *spi, GPIO_TypeDef *nCS_Port, uint16_t nCS_Pin);
    bool Init();

    std::optional<uint8_t> ReadRegisterBlocking(uint8_t address);
    bool WriteRegisterBlocking(uint8_t address, uint8_t value);

private:
    SPI_HandleTypeDef *spi;
    GPIO_TypeDef *nCS_Port;
    uint16_t nCS_Pin;

    bool init;

};