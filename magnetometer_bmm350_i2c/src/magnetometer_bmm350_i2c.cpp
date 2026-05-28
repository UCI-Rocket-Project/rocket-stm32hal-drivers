/*
 * magnetometer_bmm350_i2c.cpp
 *
 *  Created on: Feb 17, 2026
 *      Author: maahividyarthi
 */

#include "magnetometer_bmm350_i2c.h"

MagBmm350i2c::MagBmm350i2c(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *intPort, uint16_t intPin, GPIO_TypeDef *drdyPort, uint16_t drdyPin)
    : _hi2c(hi2c), _intPort(intPort), _intPin(intPin), _drdyPort(drdyPort), _drdyPin(drdyPin) {}

// Helper: read one 16-bit OTP word at given word address
// OTP DIR_READ command = 0x20 | word_addr (cmd bits [7:5] = 001)
static HAL_StatusTypeDef ReadOtpWord(I2C_HandleTypeDef *hi2c, uint8_t devAddr,
                                      uint8_t wordAddr, uint16_t *out) {
    // Send DIR_READ command: otp_cmd=001, word_addr in lower 5 bits
    uint8_t READ_CMD[2] = {0x50, (uint8_t)(0x20 | (wordAddr & 0x1F))};
    if (HAL_I2C_Master_Transmit(hi2c, devAddr, READ_CMD, 2, SERIAL_TIMEOUT) != HAL_OK) return HAL_ERROR;
    HAL_Delay(1); // wait for otp_cmd_done

    // Read MSB then LSB from 0x52 and 0x53
    uint8_t buf[4]; // +2 dummy bytes!! important
    uint8_t OTP_DATA_REG[1] = {0x52};
    if (HAL_I2C_Master_Transmit(hi2c, devAddr, OTP_DATA_REG, 1, SERIAL_TIMEOUT) != HAL_OK) return HAL_ERROR;
    if (HAL_I2C_Master_Receive(hi2c, devAddr, buf, 4, SERIAL_TIMEOUT) != HAL_OK) return HAL_ERROR;

    // buf[0],buf[1] = dummy; buf[2]=MSB, buf[3]=LSB
    *out = ((uint16_t)buf[2] << 8) | buf[3];
    return HAL_OK;
}

int MagBmm350i2c::Reset() {
    /* check if I2C is busy */
    // BMM350 default I2C address is 0x14
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(_hi2c, 0x14 << 1, 1, 100);

    /* reset i2c line if busy */
    if (status != HAL_OK) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};

        HAL_I2C_DeInit(_hi2c);

        // Set SCLK as GPIO
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
        GPIO_InitStruct.Pin = GPIO_PIN_6;
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

        HAL_I2C_Init(_hi2c);
    }

    //still busy, something else is the issue
    if (HAL_I2C_IsDeviceReady(_hi2c, 0x14 << 1, 1, 100) != HAL_OK) return 1;

    /* Soft-reset */
    // BMM350 uses register 0x7E for commands.
    // Writing 0xB6 triggers a complete soft reset.
    uint8_t SOFT_RESET_CMD[2] = {0x7E, 0xB6};

    if (HAL_I2C_Master_Transmit(_hi2c, 0x14 << 1, SOFT_RESET_CMD, 2, SERIAL_TIMEOUT) != HAL_OK) return 1;

    // BMM350 requires a small delay after reset for internal initialization
    HAL_Delay(2);


    return 0;
}

int MagBmm350i2c::Init() {
    //1: Read OTP trim coefficients (must happen before OTP is closed)*/
    // OTP word addresses from Bosch SensorAPI bmm350.c: bmm350_get_compensation_data()
    uint16_t otp[32] = {0};
    for (uint8_t i = 0; i < 32; i++) {
        if (ReadOtpWord(_hi2c, _devAddr, i, &otp[i]) != HAL_OK) return 1;
    }

    // Parse trim coefficients — from Bosch SensorAPI bmm350_get_compensation_data()
    // https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/bmm350.c

    // Temperature offset ( 0x0D, lower byte signed)
    _dutT0 = (float)(int8_t)(otp[0x0D] & 0xFF) + 23.0f;

    // Sensitivity:  0x0E, 0x0F, 0x10
    _sensitivityX = ((float)(int16_t)otp[0x0E]) / 256.0f;
    _sensitivityY = ((float)(int16_t)otp[0x0F]) / 256.0f;
    _sensitivityZ = ((float)(int16_t)otp[0x10]) / 256.0f;

    // Offset: 0x0E upper nibble + 0x11, 0x12, 0x13
    _offsetX = ((float)(int16_t)otp[0x11]) / 2.0f;
    _offsetY = ((float)(int16_t)otp[0x12]) / 2.0f;
    _offsetZ = ((float)(int16_t)otp[0x13]) / 2.0f;

    // TCO (Temperature coefficient offset: 0x14, 0x15, 0x16
    _tcoX = ((float)(int8_t)(otp[0x14] & 0xFF)) / 1024.0f;
    _tcoY = ((float)(int8_t)(otp[0x15] & 0xFF)) / 1024.0f;
    _tcoZ = ((float)(int8_t)(otp[0x16] & 0xFF)) / 1024.0f;

    // TCS (Temperature coefficient sensitivity): words 0x17, 0x18, 0x19
    _tcsX = ((float)(int8_t)(otp[0x17] & 0xFF)) / 16384.0f;
    _tcsY = ((float)(int8_t)(otp[0x18] & 0xFF)) / 16384.0f;
    _tcsZ = ((float)(int8_t)(otp[0x19] & 0xFF)) / 16384.0f;

    // Cross-axis:  0x1A, 0x1B, 0x1C
    _crossAxisYX = ((float)(int8_t)(otp[0x1A] & 0xFF)) / 800.0f;
    _crossAxisZX = ((float)(int8_t)(otp[0x1B] & 0xFF)) / 800.0f;
    _crossAxisZY = ((float)(int8_t)(otp[0x1C] & 0xFF)) / 800.0f;

    /*2: Close OTP (was previously in reset, moved here)*/
    uint8_t OTP_CLOSE[2] = {0x50, 0x80};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, OTP_CLOSE, 2, SERIAL_TIMEOUT) != HAL_OK) return 1;
    HAL_Delay(10);

    /* 3: Chip ID verify*/
    uint8_t buf[3] = {0};
    uint8_t CHIP_ID_REG[1] = {0x00};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, CHIP_ID_REG, 1, SERIAL_TIMEOUT) != HAL_OK) return 1;
    if (HAL_I2C_Master_Receive(_hi2c, _devAddr, buf, 3, SERIAL_TIMEOUT) != HAL_OK) return 1;
    if (buf[2] != 0x33) return 2;

    /*4: ODR, averaging, normal mode (unchanged)*/
    uint8_t AGGR_SET[2] = {0x04, 0x14};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, AGGR_SET, 2, SERIAL_TIMEOUT) != HAL_OK) return 1;

    uint8_t UPDATE_CMD[2] = {0x06, 0x02};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, UPDATE_CMD, 2, SERIAL_TIMEOUT) != HAL_OK) return 1;
    HAL_Delay(I2C_DELAY);

    uint8_t NORMAL_MODE[2] = {0x06, 0x01};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, NORMAL_MODE, 2, SERIAL_TIMEOUT) != HAL_OK) return 1;
    HAL_Delay(70);

    return 0;
}

MagBmm350i2c::Data MagBmm350i2c::Read() {
    MagBmm350i2c::Data data;
    uint8_t buf[14] = {0};
    uint8_t MAG_DATA_REG[1] = {0x31};
    if (HAL_I2C_Master_Transmit(_hi2c, _devAddr, MAG_DATA_REG, 1, SERIAL_TIMEOUT) != HAL_OK) return data;
    if (HAL_I2C_Master_Receive(_hi2c, _devAddr, buf, 14, SERIAL_TIMEOUT) != HAL_OK) return data;
    uint8_t* raw = &buf[2]; // skip 2 dummy bytes

    /* Reconstruct 24-bit signed values —> only 21 bits used */
    auto fix_sign_24 = [](uint32_t val) -> int32_t {
        // Sign extend from bit 23
        if (val & 0x800000) val |= 0xFF000000;
        // Arithmetic right shift by 3: 24-bit -> 21-bit signed
        return (int32_t)val >> 3;
    };

    uint32_t ux = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
    uint32_t uy = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);
    uint32_t uz = (uint32_t)raw[6] | ((uint32_t)raw[7] << 8) | ((uint32_t)raw[8] << 16);
    uint32_t ut = (uint32_t)raw[9] | ((uint32_t)raw[10] << 8) | ((uint32_t)raw[11] << 16);

    float rx = (float)fix_sign_24(ux);
    float ry = (float)fix_sign_24(uy);
    float rz = (float)fix_sign_24(uz);
    float rt = (float)fix_sign_24(ut);

    // Compensated calculation for XYZ and temperature data values
    // Compensation code below found on BMM350 SensorAPI
    // https://github.com/boschsensortec/BMM350_SensorAPI/blob/main/bmm350.c
    // Scale raw temp to degrees C
    float temp = (rt / 512.0f) + _dutT0;
    data.temperature = temp;

    /* Sensitivity and offset correction */
    float cx = rx * (1.0f + _sensitivityX) + _offsetX;
    float cy = ry * (1.0f + _sensitivityY) + _offsetY;
    float cz = rz * (1.0f + _sensitivityZ) + _offsetZ;

    /* Temperature coefficient correction */
    float dT = temp - _dutT0;
    cx += _tcoX * dT;
    cy += _tcoY * dT;
    cz += _tcoZ * dT;

    cx *= (1.0f + _tcsX * dT);
    cy *= (1.0f + _tcsY * dT);
    cz *= (1.0f + _tcsZ * dT);

    /* Cross-axis correction */
    data.magneticFieldX = cx - _crossAxisYX * cy - _crossAxisZX * cz;
    data.magneticFieldY = cy - _crossAxisZY * cz;
    data.magneticFieldZ = cz;

    /* Scale to µT — 21-bit signed at +/-2000µT ful scale */
    data.magneticFieldX *= (2000.0f / 1048576.0f);
    data.magneticFieldY *= (2000.0f / 1048576.0f);
    data.magneticFieldZ *= (2000.0f / 1048576.0f);

    return data;
}

