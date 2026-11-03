/*
 * magnetometer_bmm350_i2c.h
 *
 *  Created on: Feb 17, 2026
 *      Author: maahividyarthi
 */

#pragma once

#if defined(STM32F1)
#include "stm32f1xx_hal.h"
#elif defined(STM32F4xx)
#include "stm32f4xx_hal.h"
#endif

#ifndef SERIAL_TIMEOUT
#define SERIAL_TIMEOUT 10
#endif

#define I2C_DELAY 5

/* Datasheet */
/* https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm350-ds001.pdf
*/

class MagBmm350i2c {
	public:
		struct Data {
			// Magnetic field along x-axis, launch vehicle frame
			// Units: microTesla (uT)
			float magneticFieldX = 0.0f;

			// Magnetic field along y-axis, launch vehicle frame
			// Units: microTesla (uT)
			float magneticFieldY = 0.0f;

			// Magnetic field along z-axis, launch vehicle frame
			// Units: microTesla (uT)
			float magneticFieldZ = 0.0f;

			//Temperature Transducer
			float temperature = 0.0f;
		};
	   /**
	    * @brief Constructs a BMM350 magnetometer driver instance
		* @param hi2c        i2c bus handler
		* @param intPort     Configurable interrupt port
		* @param intPin      Congifurable interrupt pin
		* @param drdyPort    Data ready interupt port
		* @param drdyPin     Data ready interupt pin
		*/
		MagBmm350i2c(I2C_HandleTypeDef *hi2c, GPIO_TypeDef *intPort, uint16_t intPin, GPIO_TypeDef *drdyPort, uint16_t drdyPin);

		/**
	     * @brief Resets the Magnetometer
	     * @retval Operation status, 0 for success
	     * @retval Operation failure, 1 for I2C transmit/recieve failure
	     */
	    int Reset();

	    /**
	     * @brief Initializes the  Magnetometer
	     * @retval Operation status, 0 for success
	     * @retval Operation failure, 1 for I2C transmit/receive failure
	     * @retval Operation failure, 2 for wrong device failure
	     * @retval Operation failure, 3 for I2C is HAL_BUSY, should call reset
	     */
	    int Init();

	    /**
	     * @brief Reads Magnetometer data
	     * @retval Output is struct Data
	     * @retval if any values = FFFF, then 0.0f = ERROR
	     */
	    Data Read();

	private:
	    // I2C bus handler
	    I2C_HandleTypeDef *_hi2c;
	    // Configureable interupt port
	    GPIO_TypeDef *_intPort;
	    // Configureable interupt pin
	    uint16_t _intPin;
	    // Data ready interupt port
	    GPIO_TypeDef *_drdyPort;
	    // Data ready interupt pin
	    uint16_t _drdyPin;
	    const uint8_t _devAddr = 0x14 << 1; // Default BMM350 I2C Address

	    float _offsetX, _offsetY, _offsetZ;
	    float _sensitivityX, _sensitivityY, _sensitivityZ;
	    float _tcoX, _tcoY, _tcoZ;
	    float _tcsX, _tcsY, _tcsZ;
	    float _crossAxisYX, _crossAxisZX, _crossAxisZY;
	    float _dutT0;
};








