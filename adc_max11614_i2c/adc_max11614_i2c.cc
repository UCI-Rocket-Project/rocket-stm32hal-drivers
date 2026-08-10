#include "adc_max11614_i2c.h"

#include <algorithm>
#include <cstdint>

AdcMax11614i2c::AdcMax11614i2c(I2C_HandleTypeDef* hi2c, GPIO_TypeDef* sclPort, uint16_t sclPin, GPIO_TypeDef* sdaPort, uint16_t sdaPin)
    : _hi2c(hi2c), _sclPort(sclPort), _sclPin(sclPin), _sdaPort(sdaPort), _sdaPin(sdaPin) {}

bool AdcMax11614i2c::Init(const Config& config) {
    _config = config;

    // build setup byte
    uint8_t setupByte = 0b10000000;  // REG bit = 1 for setup byte
    setupByte |= (static_cast<uint8_t>(_config.refMode) << 4);
    setupByte |= (static_cast<uint8_t>(_config.clockMode) << 3);
    setupByte |= (static_cast<uint8_t>(_config.polarity) << 2);
    // bit 1 (RST) = 0 to reset config register, Bit 0 (X) = 0

    // try to write setup byte up to maxAttempts times
    uint8_t attempts = std::max<uint8_t>(1, _config.maxAttempts);
    for (uint8_t i = 0; i < attempts; ++i) {
        if (HAL_I2C_Master_Transmit(_hi2c, ADC_MAX_I2C_ADDRESS << 1, &setupByte, 1, ADC_MAX_I2C_TIMEOUT_MS) == HAL_OK) {
            return true;
        }

        HAL_Delay(ADC_MAX_I2C_RETRY_DELAY_MS);
    }

    return false;
}

std::optional<AdcMax11614i2c::Data> AdcMax11614i2c::Read(uint16_t channelSelect) {
    Data data;

    // if no channels selected, return early
    if (channelSelect == 0) return data;

    uint8_t minChannel = 8;
    uint8_t maxChannel = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if (channelSelect & (1 << i)) {  // channel i is selected
            maxChannel = i;
            if (minChannel == 8) minChannel = i;
        }
    }

    uint8_t scanMode;
    uint8_t startChannel;

    // if only scanning one channel, use single channel scanning
    if (maxChannel == minChannel) {
        scanMode = 0b11;  // SCAN[1:0] = 11 (single channel selected)
        startChannel = minChannel;
    } else if (minChannel >= 6) {  // else if the lowest requested channel is 6 or 7, use Upper Quartile scanning
        scanMode = 0b10;           // SCAN[1:0] = 10 (AIN6 to maxChannel)
        startChannel = 6;
    } else {
        scanMode = 0b00;  // SCAN[1:0] =  00 (AIN0 to maxChannel)
        startChannel = 0;
    }

    uint8_t numChannels = (maxChannel - startChannel) + 1;

    // write configuration byte
    uint8_t configurationByte = 0b00000000;
    configurationByte |= (0 << 7);                                 // REG bit = 0 for configuration byte
    configurationByte |= (scanMode << 5);                          // SCAN[1:0] selects scanning config
    configurationByte |= (maxChannel << 1);                        // CS[3:0] selects upper channel
    configurationByte |= static_cast<uint8_t>(_config.inputMode);  // SGL/DIF based on persistent config

    bool transactionSuccess = false;
    uint8_t readBuffer[16] = {0};  // Initialize to 0x00 so corrupted headers can be caught

    uint8_t attempts = std::max<uint8_t>(1, _config.maxAttempts);
    for (uint8_t i = 0; i < attempts; ++i) {
        // transmit the configuration byte to start the conversion
        if (HAL_I2C_Master_Transmit(_hi2c, ADC_MAX_I2C_ADDRESS << 1, &configurationByte, 1, ADC_MAX_I2C_TIMEOUT_MS) != HAL_OK) {
            HAL_Delay(ADC_MAX_I2C_RETRY_DELAY_MS);
            continue;  // Transmit failed, retry
        }

        // ADC uses clock stretching to allow reading data as soon as config byte is written
        // receive the data
        if (HAL_I2C_Master_Receive(_hi2c, ADC_MAX_I2C_ADDRESS << 1, readBuffer, 2 * numChannels, numChannels * ADC_MAX_I2C_TIMEOUT_MS) == HAL_OK) {
            transactionSuccess = true;
            break;  // Both tx and rx succeeded
        } else {
            HAL_Delay(ADC_MAX_I2C_RETRY_DELAY_MS);
        }
    }

    if (!transactionSuccess) {
        return std::nullopt;  // total I2C failure after max attempts
    }

    // parse data into struct
    for (uint8_t i = 0; i < numChannels; ++i) {
        uint8_t currentChannel = startChannel + i;
        uint8_t byte1 = readBuffer[i * 2];
        uint8_t byte2 = readBuffer[(i * 2) + 1];
        int16_t rawValue;

        // check for the '1111' header in the upper 4 bits
        if ((byte1 & 0xF0) == 0xF0) {
            // the header is intact, mask it out and combine the remaining 12 bits
            rawValue = ((byte1 & 0x0F) << 8) | byte2;
        } else {
            // the data is corrupted, empty, or failed to transmit
            rawValue = -1;
        }

        switch (currentChannel) {
            case 0:
                data.channelOutput0 = rawValue;
                break;
            case 1:
                data.channelOutput1 = rawValue;
                break;
            case 2:
                data.channelOutput2 = rawValue;
                break;
            case 3:
                data.channelOutput3 = rawValue;
                break;
            case 4:
                data.channelOutput4 = rawValue;
                break;
            case 5:
                data.channelOutput5 = rawValue;
                break;
            case 6:
                data.channelOutput6 = rawValue;
                break;
            case 7:
                data.channelOutput7 = rawValue;
                break;
            default:
                break;
        }
    }

    return data;
}