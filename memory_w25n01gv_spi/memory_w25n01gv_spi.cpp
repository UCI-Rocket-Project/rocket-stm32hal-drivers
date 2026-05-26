/*
 * memory_w25q01jv_spi.cpp
 *
 *  Created on: Jan 20, 2026
 *      Author: maahividyarthi
 */
#include <memory_w25n01gv_spi.h>
#include <cstdint>

MemoryW25n01gvSpi::MemoryW25n01gvSpi(SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *csPort, uint16_t csPin,
                                     GPIO_TypeDef *holdPort, uint16_t holdPin,
                                     GPIO_TypeDef *wpPort, uint16_t wpPin)
    : _hspi(hspi), _csPort(csPort), _csPin(csPin),
      _holdPort(holdPort), _holdPin(holdPin),
      _wpPort(wpPort), _wpPin(wpPin) {}

/* Notes */
// Any UPPER_SNAKE_CASE variables are opcodes for instruction to sent to memory

int MemoryW25n01gvSpi::Init(){
	// Control pins his for no false memory actions
	HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
	//Hardware Identification
	if (DeviceID() != 0xEFAA21) return -1;

	//Unlock Chip: remove block protection
	if (WriteStatusRegister() != 0) return -2;

	//The Hex Counter Binary Search - Scan buffer to find where to resume
	uint8_t scanBuffer[144];
	uint16_t lastSavedAddress = 0;
	bool foundEntry = false;

	for (uint16_t p = 64; p < 128; p++) {
		if (PageRead(p, 0, scanBuffer) == 0) {
			if (scanBuffer[0] == 0xFF) {
				if (p > 64) {
					PageRead(p - 1, 0, scanBuffer);
					lastSavedAddress = (uint16_t)(scanBuffer[0] << 8 | scanBuffer[1]);
					foundEntry = true;
				}
				break;
			}
		}
	}
	//Set FSM Starting point after scan
	currentPage = foundEntry ? lastSavedAddress : 128;

	//Initial Bad Block Check
	while (BadBlockCheck(currentPage >> 6)) {
	    currentPage = (currentPage + 64) & 0xFFC0;
	    if (currentPage >= 65536) return -2;
	}

	//Initialize remaining counters
	currentWriteCount = 0;
	_state = State::WRITE_ENABLE;

	return 0;
}

int MemoryW25n01gvSpi::DeviceID() {
    HAL_GPIO_WritePin(_wpPort, _wpPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_holdPort, _holdPin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    uint8_t MEM_DEVICE_ID_CMD[2] = {0x9F, 0x00};
    uint8_t MEM_DEVICE_VALUE_CMD[3] = {0xFF,0xFF,0xFF};

    // SPI transaction
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit(_hspi, MEM_DEVICE_ID_CMD, 2, 100) != HAL_OK) return -1;
    if (HAL_SPI_Receive(_hspi, MEM_DEVICE_VALUE_CMD, 3, 100) != HAL_OK) return -1;

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    return (int)(MEM_DEVICE_VALUE_CMD[0] << 16 | MEM_DEVICE_VALUE_CMD[1] << 8 | MEM_DEVICE_VALUE_CMD[2]);
}

int MemoryW25n01gvSpi::ReadStatus(uint8_t regAddress) {
    uint8_t READ_CMD[2] = {0x05, regAddress};
    uint8_t regValue[1] = {0};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, READ_CMD, 2, SPI_Timeout) != HAL_OK) {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        return -1;
    }
    HAL_SPI_Receive(_hspi, regValue, 1, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    return (int)regValue[0];
}

int MemoryW25n01gvSpi::WriteEnable() {
    // Control pins high for safety
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    uint8_t WRITE_ENABLE[1] = {0x06};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, WRITE_ENABLE, 1, SPI_Timeout) != HAL_OK) {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    // Verify WEL bit is 1 (bit 2 of Status Register 3)
    if ((ReadStatus(0xC0) & 0x02) != 0x02) {
        return -1;
    }
    return 0;
}

int MemoryW25n01gvSpi::WriteStatusRegister() {
    if (WriteEnable() != 0) return -1;

    // Unlock chip - clearn block protection bits in Status Register 1
    uint8_t UNLOCK_CMD[3] = {0x01, 0xA0, 0x00};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, UNLOCK_CMD, 3, SPI_Timeout) != HAL_OK) {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    return 0;
}

MemoryW25n01gvSpi::State MemoryW25n01gvSpi::PageWrite(uint8_t (&data)[144]) {
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    switch (_state) {
        case State::WRITE_ENABLE: {
            if (WriteEnable() != 0) {
                _state = State::ERROR;
                break;
            }
            _state = State::LOAD_DATA;
        }

        case State::LOAD_DATA: {
            // Check WEL bit in SR3
            if ((ReadStatus(0xC0) & 0x02) != 0x02) break;

            // Use 0x02 (resets buffer) for the first write of a page,
            // Use 0x84 (Random Load) for subsequent partial writes.
            uint8_t opcode = (currentWriteCount == 0) ? 0x02 : 0x84;

            //Load data at specified Column Address
            uint16_t columnAddr = currentWriteCount * 144;
            LoadBuffer(opcode, columnAddr, data, 144);

            _state = State::PAGE_EXECUTE;
            break;
        }

        case State::PAGE_EXECUTE: {
            // Program Execute (10h) to move buffer to Flash Array: Page
        	ProgramExecute(currentPage);
            _state = State::COMPLETE;
            break;
        }

        case State::COMPLETE: {
        	if ((ReadStatus(0xC0) & 0x01) != 0) break; // Operation in progress

            // Increment counters
            currentWriteCount++;
            //Max 4 writes per page
            if (currentWriteCount >= 4) {
                currentWriteCount = 0;
                currentPage++; // Move to next hex page address

            	// Bad Block Check after increment
            	if ((currentPage & 0x3F) == 0) {            // skip bad block
            		while (BadBlockCheck(currentPage >> 6)) currentPage += 64;
            	}

            	//Update latest page address in block 1 when reached maximum page writes --> must move to next page
            	backupCounter++;
            	if (backupCounter >= 100) {
            		backupCounter = 0;
            		_state = State::BACKUP;
            		break;
            	}

            _state = State::WRITE_ENABLE;
            break;
        }

        case State::BACKUP: {
        	CommitBackup(); // Call the helper to write updated page address to Block 1
        	_state = State::WRITE_ENABLE;
        	break;
        }

        case State::ERROR:
        	// Handle error (e.g reset state or log failure)
        	break;
        }
    return _state;
    }
}

int MemoryW25n01gvSpi::PageRead(uint16_t pageAddress, uint8_t slot, uint8_t (&chipData)[144]) {
    if (slot > 3) return -1; // Protect against 4-packet-per-page limit

    //Read data off of array page onto buffer
    PageDataRead(pageAddress);
    while ((ReadStatus(0xC0) & 0x01) != 0);

    //Read ECC bits (bits 4 and 5 of status register 3)
    uint8_t status3 = (uint8_t)ReadStatus(0xC0);
    uint8_t eccStatus = (status3 >> 4) & 0x03;

    if (eccStatus >= 0x02) {
    	return -2;
    }

    //Read Data - read data off of buffer sequentially
    ReadBuffer(slot * 144, chipData, 144);

    return 0;
}

int MemoryW25n01gvSpi::BlockErase(uint16_t blockAddress) {
    if (blockAddress > 1023) return -1;

    if (WriteEnable() != 0) return -1;

    uint16_t pageAddress = blockAddress << 6;
    uint8_t BLOCK_ERASE[4] = {0xD8, 0x00, (uint8_t)(pageAddress >> 8), (uint8_t)(pageAddress & 0xFF)};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(_hspi, BLOCK_ERASE, 4, SPI_Timeout) != HAL_OK) {
        HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
        return -1;
    }
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    // Wait for BUSY bit 0 of Status Register 3
    while (ReadStatus(0xC0) & 0x01) {
    }

    // Check for E-FAIL (bit 2 of Status Register 3)
    if ((ReadStatus(0xC0) & 0x04) != 0) return -1;

    // Reset local tracking if we just erased the block we were currently in
    if ((currentPage >> 6) == blockAddress) {
        currentPage = blockAddress << 6;
        currentWriteCount = 0;
    }

    return 0;
}

void MemoryW25n01gvSpi::LoadBuffer(uint8_t opcode, uint16_t columnAddr, uint8_t* data, uint16_t len) {
    uint8_t LOAD_BUFFER_CMD[3] = {opcode, (uint8_t)(columnAddr >> 8), (uint8_t)(columnAddr & 0xFF)};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, LOAD_BUFFER_CMD, 3, SPI_Timeout);
    HAL_SPI_Transmit(_hspi, data, len, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

void MemoryW25n01gvSpi::ProgramExecute(uint16_t pageAddr) {
    uint8_t PROGRAM_EXECUTE_CMD[4] = {0x10, 0x00, (uint8_t)(pageAddr >> 8), (uint8_t)(pageAddr & 0xFF)};

    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, PROGRAM_EXECUTE_CMD, 4, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

void MemoryW25n01gvSpi::PageDataRead(uint16_t pageAddr) {
    uint8_t PAGE_DATA_READ_CMD[4] = { 0x13, 0x00, (uint8_t)(pageAddr >> 8), (uint8_t)(pageAddr & 0xFF) };
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, PAGE_DATA_READ_CMD, 4, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

void MemoryW25n01gvSpi::ReadBuffer(uint16_t columnAddr, uint8_t* data, uint16_t len) {
	// 03h is the standard Read Data opcode
	uint8_t READ_BUFFER_CMD[4] = { 0x03, (uint8_t)(columnAddr >> 8), (uint8_t)(columnAddr & 0xFF), 0x00 };
	HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(_hspi, READ_BUFFER_CMD, 4, SPI_Timeout);
	HAL_SPI_Receive(_hspi, data, len, SPI_Timeout);
	HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);
}

bool MemoryW25n01gvSpi::BadBlockCheck(uint16_t blockAddress) {
    uint8_t markerByte0[1];
    uint8_t markerSpare[1];
    uint16_t firstPageOfBlock = blockAddress << 6;

    // Load Page 0 of the block into internal data buffer
    uint8_t PAGE_READ_CMD[4] = {0x13, 0x00, (uint8_t)(firstPageOfBlock >> 8), (uint8_t)(firstPageOfBlock & 0xFF)};
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, PAGE_READ_CMD, 4, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    while ((ReadStatus(0xC0) & 0x01) != 0); // Wait for Busy bit to clear

    //Check Byte 0 of Page 0 in data
    uint8_t READ_BYTE0_CMD[4] = {0x03, 0x00, 0x00, 0x00}; // Column 0
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, READ_BYTE0_CMD, 4, SPI_Timeout);
    HAL_SPI_Receive(_hspi, markerByte0, 1, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    //Check Byte 0 of the Spare Area (Column 2048 / 0x0800)
    uint8_t READ_SPARE_CMD[4] = {0x03, 0x08, 0x00, 0x00}; // Column 2048
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(_hspi, READ_SPARE_CMD, 4, SPI_Timeout);
    HAL_SPI_Receive(_hspi, markerSpare, 1, SPI_Timeout);
    HAL_GPIO_WritePin(_csPort, _csPin, GPIO_PIN_SET);

    // If either marker isn't 0xFF, the block is bad
    if (markerByte0[0] != 0xFF || markerSpare[0] != 0xFF) {
        return true;
    }

    return false;
}

int MemoryW25n01gvSpi::CommitBackup() {
	uint8_t readAddress[2];//To read current written address
	uint16_t targetPage = 64;//initialize first page of Block 1

	// Find first empty page in Block 1 (64-127)
	// We start at 64 and increment until we hit 0xFF or the end of the block
	while (targetPage < 128) {
	    PageDataRead(targetPage);
	    while ((ReadStatus(0xC0) & 0x01) != 0); // Wait for transfer

	    //Read page address from readBuffer array
	    ReadBuffer(0, readAddress, 2);

	    if (readAddress[0] == 0xFF && readAddress[1] == 0xFF) break;
	    targetPage++;
	}

	// If we reached 128, block 1 is full. Erase and reset.
	if (targetPage == 128) {
	    BlockErase(1);
	    targetPage = 64;
	}

	//Read 2 byte current Page Address
	uint8_t storeAddress[2] = { (uint8_t)(currentPage >> 8), (uint8_t)(currentPage & 0xFF) };

	if (WriteEnable() != 0) return -1;

	LoadBuffer(0x02, 0, storeAddress, 2);
	ProgramExecute(targetPage);

	// Wait using the consolidated ReadStatus
	while ((ReadStatus(0xC0) & 0x01) != 0);
	return 0;
}
















