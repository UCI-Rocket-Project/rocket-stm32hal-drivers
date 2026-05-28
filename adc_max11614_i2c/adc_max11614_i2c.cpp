#include "adc_max11614_i2c.h"
#include <cstdint>
#include "stm32f1xx_hal_def.h"

AdcMax11614i2c::AdcMax11614i2c(I2C_HandleTypeDef *hi2c,GPIO_TypeDef *sclPort, uint16_t sclPin)
    : hi2c_(hi2c), sclPort_(sclPort), sclPin_(sclPin) {}

int AdcMax11614i2c::Init(){
    uint16_t shifted_address = kSlaveAddress << 1;

    // Probe the bus to see if the ADC acknowledges its address
    // Parameters: I2C handle, shifted address, number of trials, timeout in ms
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c_, shifted_address, 3, 100);

        /* reset i2c line if busy */
    if (status != HAL_OK) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        HAL_I2C_DeInit(hi2c_);

        // Set SCLK as GPIO
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pin = sclPin_;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

        // complete 10 cycles of SCLK to release module's last task
        for (int i = 0; i < 10; i++) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
            HAL_Delay(20);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
            HAL_Delay(20);
        }

        HAL_I2C_Init(hi2c_);
    }

    // if still busy, idk lol, its prob somehthing else
    HAL_StatusTypeDef status2 = HAL_I2C_IsDeviceReady(hi2c_, shifted_address, 3, 100);

    if (status == HAL_OK) {
    return 0;  // Success - Device ACKed
    } else if (status == HAL_BUSY) {
    return 3;  // I2C is HAL_BUSY
    } else {
    return 1;  // I2C transmit/receive failure (No ACK)
    }
}