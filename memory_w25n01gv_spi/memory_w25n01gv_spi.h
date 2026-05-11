/*
 * memory_w25q01gv_spi.h
 *
 *  Created on: Jan 20, 2026
 *      Author: maahividyarthi
 */

#pragma once

#ifndef INC_MEMORY_W25N01GV_SPI_H_
#define INC_MEMORY_W25N01GV_SPI_H_

#include "main.h"     // brings in stm32f1xx_hal.h
#include <cstdint>

class MemoryW25n01gvSpi {
  public:
	enum class State
	    {
	        WRITE_ENABLE,
			LOAD_DATA,
	        PAGE_EXECUTE,
	        COMPLETE,
			BACKUP,
	        ERROR,
	    };

    /**
     * @brief Constructor
     * @param hspi       SPI bus handler
     * @param csPort      select GPIO port
     * @param csPin      Chip select GPIO pin
     * @param holdPort   HOLD GPIO port
     * @param holdPin    HOLD GPIO pin
     * @param wpPort     Write Protect GPIO port
     * @param wpPin      Write Protect GPIO pin
     */
    MemoryW25n01gvSpi(SPI_HandleTypeDef *hspi,
                      GPIO_TypeDef *csPort, uint16_t csPin,
                      GPIO_TypeDef *holdPort, uint16_t holdPin,
                      GPIO_TypeDef *wpPort, uint16_t wpPin);

    /**
	* @brief Initialises the memory by finding the first free page (erased state 0xFF)
	* @retval Operation Success, 0 for success
	* @retval Operation Failure, -1 for wrong device (JEDEC ID mismatch)
	* @retval Operation Failure, -2 for hardware unlock/scan failure
	*/
    int Init();

    /**
     * @brief Reads the device ID of the memory
     * Should output 0xEF for manufacturer ID,
     * Should output 0xAA21 for Device ID
     * @retval Operation Success, 0xEFAA21 is return value
     * @retval Operation Failure, -1 for SPI protocol failure
     */
    int DeviceID();

    /**
     * @brief Reads a specific Status Register from memory
     * @param regAddress A0h (SR1), B0h (SR2), or C0h (SR3)
     * @retval Operation Success, Register value from 0-255 for success
     * @retval Operation Failure, -1 for SPI protocol failure
     */
    int ReadStatus(uint8_t regAddress);

    /**
     * @brief Sets the Write Enable Latch (WEL) bit
     * @retval Operation Success, 0 for success
     * @retval Operation Failure, -1 for SPI protocol or WEL bit failure
     */
    int WriteEnable();

    /**
     * @brief Unlocks the hardware block protection (Status Register 1)
     * @retval Operation Success, 0 for success
     * @retval Operation Failure, -1 for SPI protocol failure
     */
    int WriteStatusRegister();

    /**
     * @brief Writes a 144 byte array into flash memory,
     * Uses a multi-stage FSM (Load Data -> Program Execute -> Busy Check).
     * Auto increments from address found in Init
     * @param data Reference to the 144-byte data array to be stored.
     * @retval Operation FSM status
     */
    State PageWrite(uint8_t (&data)[144]);

    /**
     * @brief Reads a 144-byte  data packet from a specific page and slot.
     * chipData output value is updated in input chipData
     * @param pageAddress The 16-bit page address to read from (0 to 65535).
     * @param slot The packet index within that page (0, 1, 2, or 3).
     * @param chipData Reference to the 144-byte array where data will be stored.
     * @retval Operation Success, 0 for success
     * @retval Operation Failure, -1 for SPI or Hardware protocol failure
     * @retval Operation Failure, -2 for ECC uncorrectable error
     */
    int PageRead(uint16_t pageAddress, uint8_t slot, uint8_t (&chipData)[144]);

    /**
     * @brief Erases a 128KB block (64 pages)
     * @param blockAddress 0-1023 (1Gb capacity)
     * @retval Operation Success, 0 for success
     * @retval Operation Failure, -1 for SPI protocol or E-FAIL
     */
    int BlockErase(uint16_t blockAddress);

  private:

    //Internal Hardware Helpers

    /**
     * @brief Loads data into the NAND's internal SRAM buffer.
     * @param opcode 02h (Load) or 84h (Random Load)
     * @param columnAddr The byte offset in the 2112-byte buffer
     * @param data Pointer to the data array
     * @param len Length of the data to send
     */
    void LoadBuffer(uint8_t opcode, uint16_t columnAddr, uint8_t* data, uint16_t len);

    /**
     * @brief Transfers the internal SRAM buffer to the physical Flash array.
     * @param pageAddr The 16-bit destination page address
     */
    void ProgramExecute(uint16_t pageAddr);

    /**
     * @brief Transfers data from the physical Flash array to the internal SRAM buffer.
     * @param pageAddr The 16-bit source page address (PA[15:0])
     */
    void PageDataRead(uint16_t pageAddr);

    /**
     * @brief Reads data out of the NAND's internal SRAM buffer.
     * @param columnAddr The byte offset in the 2112-byte buffer (CA[11:0])
     * @param data Pointer to the data array where buffer content will be stored
     * @param len Length of the data to receive
     */
    void ReadBuffer(uint16_t columnAddr, uint8_t* data, uint16_t len);

    /**
     * @brief Checks Page 0 and Spare Area for bad block markers.
     * @param blockAddress 0-1023
     * @retval true if block is marked bad, false if block is good
     */
    bool BadBlockCheck(uint16_t blockAddress);

    /**
     * @brief Performs a write of 'currentPage' address into Block 1.
     * @retval Operation Success, 0 for success
     * @retval Operation Failure, -1 for write enable failure
     */
    int CommitBackup();

    //Private members

    SPI_HandleTypeDef *_hspi;

    GPIO_TypeDef *_csPort;
    uint16_t _csPin;

    GPIO_TypeDef *_holdPort;
    uint16_t _holdPin;

    GPIO_TypeDef *_wpPort;

    uint16_t _wpPin;

    State _state = State::WRITE_ENABLE;

    uint16_t currentPage = 0;      // Tracks Page Address PA[15:0]

    uint8_t  currentWriteCount = 0; // Tracks 0-3 (4 program operations per page limit)

    uint8_t backupCounter = 0; //Tracks every 100th write to data location backup

    static constexpr uint32_t SPI_Timeout = 100;
};

#endif /* MEMORY_W25Q01JV_SPI_H_ */

