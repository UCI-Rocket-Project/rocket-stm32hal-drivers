#pragma once

#include <cstdint>
#include <optional>

#include "main.h"  // # includes "stm32xxxx_hal.h"

#define ADC_MAX_I2C_ADDRESS 0x33  // 0110011
#define ADC_MAX_I2C_TIMEOUT_MS 10
#define ADC_MAX_I2C_RETRY_DELAY_MS 25

class AdcMax11614i2c {
  public:
    struct Data {
        // adc channel output from 0-4096, with -1 meaning corrupted/invalid header or not read
        int16_t channelOutput0 = -1;
        int16_t channelOutput1 = -1;
        int16_t channelOutput2 = -1;
        int16_t channelOutput3 = -1;
        int16_t channelOutput4 = -1;
        int16_t channelOutput5 = -1;
        int16_t channelOutput6 = -1;
        int16_t channelOutput7 = -1;
    };

    // Strongly-typed enums for safe configuration
    enum class ClockMode : uint8_t {
        Internal = 0,
        External = 1
    };

    enum class ReferenceMode : uint8_t {
        VDD = 0b000,
        External = 0b010,
        Internal_AnalogIn = 0b100,
        Internal_AlwaysOn = 0b101,
        Internal_RefOut = 0b110,
        Internal_RefOut_AlwaysOn = 0b111
    };

    enum class Polarity : uint8_t {
        Unipolar = 0,
        Bipolar = 1
    };

    enum class InputMode : uint8_t {
        Differential = 0,
        SingleEnded = 1
    };

    struct Config {
        uint8_t maxAttempts = 3;  // total tries per read/write operation (1 initial + 2 retries)
        ClockMode clockMode = ClockMode::Internal;
        ReferenceMode refMode = ReferenceMode::VDD;
        Polarity polarity = Polarity::Unipolar;
        InputMode inputMode = InputMode::SingleEnded;
    };

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
     * @param config the persistent configuration settings for the ADC
     * @retval write status, true for success
     */
    bool Init(const Config& config);

    /**
     * @brief read data from selected channels, reading extra data if there are gaps between selected channels
     * @param channelSelect 8-bit bitmask specifying the channels to read from, 0-7
     * @retval std::optional containing channel outputs, or std::nullopt if the I2C read/write entirely fails
     */
    std::optional<Data> Read(uint16_t channelSelect);

    // externally visible max output for users of the driver to change units
    constexpr static uint16_t maxChannelOutput = 4095;

  private:
    I2C_HandleTypeDef* _hi2c;
    GPIO_TypeDef* _sclPort;
    uint16_t _sclPin;
    GPIO_TypeDef* _sdaPort;
    uint16_t _sdaPin;

    Config _config;
};