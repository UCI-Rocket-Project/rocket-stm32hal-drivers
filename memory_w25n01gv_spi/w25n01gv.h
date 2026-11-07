#pragma once

#include "stm32f1xx_hal.h"
#include <cstddef>
#include <cstdint>

// W25N01GV serial NAND log. All calls except ConnectBlocking() are foreground,
// nonblocking operations. Call ProcessTask() periodically (1 ms or faster).
// SPI2 must have both TX and RX DMA configured before calling an async method.
class W25N01GV {
public:
    static constexpr uint16_t BlockCount = 1024;
    static constexpr uint8_t PagesPerBlock = 64;
    static constexpr size_t PageBytes = 2048;
    static constexpr size_t PayloadBytes = 2043;
    static constexpr uint8_t FormatVersion = 1;

    enum class Result : uint8_t {
        Ok, Pending, EndOfLog, Full, NotConnected, NotMounted, Busy,
        BadArgument, SpiError, Timeout, WrongDevice, ConfigurationError,
        EraseFailed, ProgramFailed, CheckpointFailed
    };

    enum class Operation : uint8_t { None, Mount, Format, Append, ReadNext };
    using Completion = void (*)(Operation operation, Result result, uint16_t block,
                                uint8_t page, void *context);

    W25N01GV(SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin,
             GPIO_TypeDef *wpPort, uint16_t wpPin,
             GPIO_TypeDef *holdPort, uint16_t holdPin);

    // The sole blocking operation. Drives /HOLD high, resets the device,
    // verifies EF AA 21, and configures SR1=02, SR2=10. Allow 5 ms after
    // power-up before calling this (or let this function's delay provide it).
    Result ConnectBlocking();

    // One operation at a time. The completion callback runs in ProcessTask(),
    // never in an interrupt. Mount scans headers and the last block to recover
    // the append cursor. Format erases good blocks whose page 0 is programmed.
    Result MountAsync(Completion completion = nullptr, void *context = nullptr);
    Result FormatAsync(Completion completion = nullptr, void *context = nullptr);

    // Copies exactly PayloadBytes from payload before returning. The driver
    // creates a 2048-byte data page with marker and CRC, then appends it.
    Result AppendPageAsync(const uint8_t *payload, Completion completion = nullptr,
                           void *context = nullptr);

    // Reads the next valid record into output[PayloadBytes]. Keep output alive
    // until the callback. Occupied pages with bad CRC/ECC are skipped. A call
    // returning EndOfLog does not invoke the callback; Pending does.
    Result ReadNextPageAsync(uint8_t *output, Completion completion = nullptr,
                             void *context = nullptr);
    void RewindReadCursor();

    void ProcessTask();
    void TxRxCompleteCallback(SPI_HandleTypeDef *completedSpi);
    void SpiErrorCallback(SPI_HandleTypeDef *failedSpi);

    bool Connected() const { return connected_; }
    bool Mounted() const { return mounted_; }
    bool Busy() const { return operation_ != Operation::None; }
    Result LastResult() const { return lastResult_; }
    uint16_t NextBlock() const { return nextBlock_; }
    uint8_t NextPage() const { return nextPage_; }

private:
    enum class Io : uint8_t { Idle, Read, Program, Erase };
    enum class IoStage : uint8_t {
        Idle, ReadToCache, ReadWaitCache, ReadHeader, ReadData,
        ReadWaitRelease, WriteEnable, WriteCheckWel, WriteLoadHeader,
        WriteLoadData, WriteExecute, WriteWaitReady, EraseEnable,
        EraseCheckWel, EraseCommand, EraseWaitReady
    };
    enum class Step : uint8_t { Pending, Done, Failed };
    enum class Phase : uint8_t {
        Idle, MountHeaders, MountTail, FormatHeaders, FormatErasing,
        AppendFindBlock, AppendErase, AppendCheckpoint,
        AppendVerifyCheckpoint, AppendData, ReadHeader, ReadData
    };

    SPI_HandleTypeDef *spi_;
    GPIO_TypeDef *csPort_;
    uint16_t csPin_;
    GPIO_TypeDef *wpPort_;
    uint16_t wpPin_;
    GPIO_TypeDef *holdPort_;
    uint16_t holdPin_;

    bool connected_ = false;
    bool mounted_ = false;
    Operation operation_ = Operation::None;
    Phase phase_ = Phase::Idle;
    Result lastResult_ = Result::NotConnected;
    Completion completion_ = nullptr;
    void *completionContext_ = nullptr;

    // Persistent-position state is reconstructed by MountAsync().
    uint16_t firstBlock_ = BlockCount;
    uint16_t activeBlock_ = BlockCount;
    uint16_t nextBlock_ = 0;
    uint8_t nextPage_ = 1;
    uint16_t readBlock_ = 0;
    uint8_t readPage_ = 1;
    bool readHeaderNeeded_ = true;
    uint8_t *readOutput_ = nullptr;
    uint16_t scanBlock_ = 0;
    int8_t scanPage_ = 0;
    uint16_t targetBlock_ = 0;
    uint8_t targetPage_ = 0;

    // DMA uses stable member storage throughout each transaction. txPage_ is
    // the page being programmed or an all-FF read clock source; rxPage_ stores
    // the read result. Status/command transfers use small member buffers.
    alignas(4) uint8_t txPage_[PageBytes]{};
    alignas(4) uint8_t rxPage_[PageBytes]{};
    uint8_t payload_[PayloadBytes]{};
    alignas(4) uint8_t txSmall_[4]{};
    alignas(4) uint8_t rxSmall_[4]{};
    Io io_ = Io::Idle;
    IoStage ioStage_ = IoStage::Idle;
    Result ioResult_ = Result::Pending;
    uint16_t ioBlock_ = 0;
    uint8_t ioPage_ = 0;
    bool dmaActive_ = false;
    volatile bool dmaDone_ = false;
    volatile bool dmaError_ = false;
    bool csAsserted_ = false;
    bool releaseCsAfterDma_ = true;
    bool ioEccBad_ = false;
    uint32_t dmaStartMs_ = 0;
    uint32_t busyStartMs_ = 0;
    uint8_t status_ = 0;

    void Begin(Operation operation, Phase phase, Completion completion, void *context);
    void Finish(Result result);
    void StartRead(uint16_t block, uint8_t page);
    void StartProgram(uint16_t block, uint8_t page);
    void StartErase(uint16_t block);
    Step ProcessIo();
    bool TransferSmall(uint8_t count, bool holdCs = false);
    bool TransferPage(bool writing, bool holdCs = false);
    bool PollStatus();
    void EndTransaction();
    void FailIo(Result result);
    bool IoSucceeded() const { return ioResult_ == Result::Ok; }
    bool IsBadBlock() const { return rxPage_[0] != 0xFF; }
    bool PageOccupied() const;
    bool CheckpointValid(uint16_t block) const;
    bool DataPageValid() const;
    void MakeCheckpoint(uint16_t block);
    void MakeDataPage(const uint8_t *payload);
    void AdvanceReadCursor();
    static uint32_t Crc32(const uint8_t *data, size_t size);
    static void PutU32BE(uint8_t *dst, uint32_t value);
    static uint32_t GetU32BE(const uint8_t *src);

    bool BlockingTransfer(const uint8_t *tx, uint8_t *rx, uint16_t count);
    bool BlockingCommand(const uint8_t *bytes, uint16_t count);
    bool BlockingStatus(uint8_t address, uint8_t &value);
    bool BlockingWriteRegister(uint8_t address, uint8_t value);
};
