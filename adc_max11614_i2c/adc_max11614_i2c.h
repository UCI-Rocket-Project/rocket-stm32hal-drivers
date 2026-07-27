#pragma once

#include <cstdint>

#include "main.h"  // # includes "stm32xxxx_hal.h"

#define ADC_MAX_I2C_ADDRESS 0x33  // 0110011
#define ADC_MAX_I2C_TIMEOUT_MS 10

class AdcMax11614i2c {
  public:
    typedef struct Data {
        // adc channel output from 0-4096, with -1 meaning not read/not valid
        int16_t channelOutput0 = -1;
        int16_t channelOutput1 = -1;
        int16_t channelOutput2 = -1;
        int16_t channelOutput3 = -1;
        int16_t channelOutput4 = -1;
        int16_t channelOutput5 = -1;
        int16_t channelOutput6 = -1;
        int16_t channelOutput7 = -1;
    } Data;

    /**
     * @param hi2c I2C bus handler
     * @param sclPort serial clock GPIO port
     * @param sclPin serial clock GPIO pin
     * @param sdaPort serial data GPIO port
     * @param sdaPin serial data GPIO pin
     */
    AdcMax11614i2c(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* sclPort, uint16_t sclPin, GPIO_TypeDef* sdaPort, uint16_t sdaPin);

    /**
     * @brief write configs to Setup register
     * @retval write status, true for success
     */
    bool Init();

    /**
     * @brief read data from selected channels, reading extra data if there are gaps between selected channels
     * @param channelSelect 8-bit bitmask specifying the channels to read from, 0-7
     * @retval channel outputs from 0-4096; -1 means not read/not valid
     */
    Data Read(uint16_t channelSelect);

  private:
    I2C_HandleTypeDef* _hi2c;
    GPIO_TypeDef* _sclPort;
    uint16_t _sclPin;
    GPIO_TypeDef* _sdaPort;
    uint16_t _sdaPin;
};