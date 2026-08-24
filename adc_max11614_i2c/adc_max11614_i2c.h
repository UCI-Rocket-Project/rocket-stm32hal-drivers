#pragma once

#include <cstdint>
#include <optional>

#include "main.h"  // # includes "stm32xxxx_hal.h"

#define ADC_MAX_I2C_ADDRESS 0x33  // 0110011
#define ADC_MAX_I2C_TIMEOUT_MS 10
#define ADC_MAX_I2C_RETRY_DELAY_MS 5

class AdcMax11614i2c {
  public:
    static constexpr uint16_t maxChannelOutput = 4095; // 12-bit ADC max value
    
    struct Data {
        // adc channel output from 0-4096, with -1 meaning not read/could not be read
        int16_t channelOutput0 = -1;
        int16_t channelOutput1 = -1;
        int16_t channelOutput2 = -1;
        int16_t channelOutput3 = -1;
        int16_t channelOutput4 = -1;
        int16_t channelOutput5 = -1;
        int16_t channelOutput6 = -1;
        int16_t channelOutput7 = -1;
    };

    // strongly-typed enums for safe configuration
    enum class ClockMode : uint8_t {
        Internal = 0,
        External = 1
    };

    enum class ReferenceMode : uint8_t {
        VDD                      = 0b000,
        External                 = 0b010,
        Internal_AnalogIn        = 0b100,
        Internal_AlwaysOn        = 0b101,
        Internal_RefOut          = 0b110,
        Internal_RefOut_AlwaysOn = 0b111
    };

    enum class Polarity : uint8_t {
        Unipolar = 0,
        Bipolar  = 1
    };

    enum class InputMode : uint8_t {
        Differential = 0,
        SingleEnded  = 1
    };

    struct Config {
        uint8_t maxAttempts = 3;  // total tries per operation (1 initial + 2 retries)
        ClockMode clockMode = ClockMode::Internal;
        ReferenceMode refMode = ReferenceMode::VDD;
        Polarity polarity = Polarity::Unipolar;
        InputMode inputMode = InputMode::SingleEnded;
    };

    // internal states for the non-blocking state machine
    enum class DriverState {
        IDLE,
        TRANSMITTING,
        RECEIVING,
        DATA_READY,
        ERROR
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
     * @brief initiate a non-blocking read from selected channels
     * @param channelSelect 8-bit bitmask specifying the channels to read from, 0-7
     * @retval true if the transaction successfully started on the bus
     */
    bool StartReadAsync(uint16_t channelSelect);

    /**
     * @brief check if the hardware has finished receiving data via interrupt
     * @retval true if data is ready to be fetched
     */
    bool IsDataReady() const;

    /**
     * @brief parse and fetch the data populated by the receive interrupt
     * @retval std::optional containing channel outputs, or std::nullopt if the transaction failed
     */
    std::optional<Data> FetchData();

    // global c callback routers
    static void HAL_TxCpltCallback(I2C_HandleTypeDef *hi2c);
    static void HAL_RxCpltCallback(I2C_HandleTypeDef *hi2c);
    static void HAL_ErrorCallback(I2C_HandleTypeDef *hi2c);

  private:
    I2C_HandleTypeDef* _hi2c;
    GPIO_TypeDef* _sclPort;
    uint16_t _sclPin;
    GPIO_TypeDef* _sdaPort;
    uint16_t _sdaPin;

    Config _config;
    
    // async state machine variables
    volatile DriverState _state = DriverState::IDLE;
    uint8_t _configByteTxBuffer; 
    uint8_t _rxBuffer[16];
    uint8_t _numChannelsToRead;
    uint8_t _startChannel;

    // internal interrupt handlers
    void HandleTxComplete();
    void HandleRxComplete();
    void HandleError();

    // static instance
    static AdcMax11614i2c* _instance;
};