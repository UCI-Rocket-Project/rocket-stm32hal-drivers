#pragma once

#include <cstdint>
#include "main.h"
#include "stm32f105xc.h"

/**
 * Datasheet:
 * https://www.analog.com/media/en/technical-documentation/data-sheets/max11612-max11617.pdf 
 */

class AdcMax11614i2c {
  public:
    // Placeholder struct based on the MAX11614's 8 channels
    struct Data {
      uint16_t channels[8];
    };

    static constexpr uint8_t kSlaveAddress = 0b0110011;

    /**
      * @param hi2c i2c bus handler
      */
    AdcMax11614i2c(I2C_HandleTypeDef* hi2c, GPIO_TypeDef *sclPort, uint16_t sclPin);

    /**
      * @brief Resets the ADC
      * @retval Operation status, 0 for success
      * @retval Operation failure, 1 for I2C transmit/receive failure
      */
    int Reset();

    /**
      * @brief Initializes the ADC and checks if device is ready on the bus
      * @retval Operation status, 0 for success
      * @retval Operation failure, 1 for I2C transmit/receive failure
      * @retval Operation failure, 2 for wrong device failure
      * @retval Operation failure, 3 for I2C is HAL_BUSY, should call reset
      */
    int Init();

    /**
      * @brief Reads ADC data
      * @retval Output is struct Data
      * @retval if any values = FFFF, then ERROR
      */
    Data Read();

    private:
      I2C_HandleTypeDef* hi2c_;
      GPIO_TypeDef* sclPort_;
      uint16_t sclPin_;
};