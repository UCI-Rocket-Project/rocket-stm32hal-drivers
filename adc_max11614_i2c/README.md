# MAX11614 External ADC I2C Driver

## Overview

This is a C++ implementation using the STM32 HAL for the ADC MAX11614, primarily using the [datasheet](https://www.analog.com/media/en/technical-documentation/data-sheets/MAX11612-MAX11617.pdf)
from the 11612-11617 family of ADCs.

## Usage

### Configuration

The driver exposes a `Config` struct that can be passed into the constructor. This struct is used to write the setup pyte to the ADC during initialization.

This defaults to using the ADC's internal clock for I2C, VDD for the reference voltage when converting data, and unipolar, single-ended channel mode (each of the 8 channels independently reads from 0-4095). Use the provided defaults unless overriding is strictly necessary. The default values were chosen from the electrical setup indicated by the GSE2.1 hardware repo, and `max_attempts` for I2C reads/writes was arbitrarily chosen to be **3**.

### Initialization

Calling the `Init()` method with a Config object writes the configuration byte to the ADC and returns a boolean status, with `true` indicating success. This write is attemped up to the `max_attempts` limit specified in the config struct, with currently a fixed dealy between attempts.

### Reading Data

Calling the `Read()` method with a 8-bit bitmask `channel_select` argument will attempt to read data from the specified channels and return `std::optional<Data>`. Since the ADC can only read a single channel, from channel 0 up to a selected channel, or from channel 6 up to a selected channel, **this may read extraneous channels in order to capture the ones specified**.

This method will return `std::nullopt` if the entire read/write cycle fails after `max_attempts` tries, and -1 for any channels that were either not read or whose data was corrupted. This allows the user to handle I2C failures separately from per-channel failures, as well as ignore channels whose data was not requested.

## Current Limitations

### Extraneous Channel Reads

As noted in the [**Reading Data**](#reading-data) section, the driver currently reads any channels in between specified channels, and possibly channels before the first specified one. For example, if the user requested to read channels 1 and 3, the driver would read from 0-3 inclusive due to the ADCs interface limitations.

This could be improved by chaining multiple single-channel reads when a sparse `channelSelect` is detected, which would increase complexity but also reduce total read time for these cases.

### Configuration Limitations

**The driver does not currently support bipolar + differential mode**, where data is read as the difference between two input channels from `-VREF/2` to `VREF/2`, instead of from each channel as 0 to `VREF` as it does in the default mode (single-ended, unipolar).

## Basic Usage Example

```cpp
#include <optional>

// 1. Define your configuration
AdcMax11614i2c::Config adcConfig;
adcConfig.clockMode = AdcMax11614i2c::ClockMode::Internal;
adcConfig.refMode = AdcMax11614i2c::ReferenceMode::VDD;
adcConfig.maxAttempts = 3;

// 2. Initialize
if (!adcDriver.Init(adcConfig)) {
    // Handle initialization failure
}

// 3. Read channels (e.g., bitmask 0b00000111 to read Ch 0, 1, and 2)
std::optional<AdcMax11614i2c::Data> adcData = adcDriver.Read(0x07);

// 4. Process returned optional data
if (adcData.has_value()) {
    int16_t ch0 = adcData.channelOutput0;
    // Process valid data
} else {
    // Handle total I2C transaction failure
}
```