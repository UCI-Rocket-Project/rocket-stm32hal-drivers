#include "w25n01gv.h"
#include <cstring>

namespace {
constexpr uint8_t CmdReset = 0xFF;
constexpr uint8_t CmdJedec = 0x9F;
constexpr uint8_t CmdReadStatus = 0x0F;
constexpr uint8_t CmdWriteStatus = 0x1F;
constexpr uint8_t CmdWriteEnable = 0x06;
constexpr uint8_t CmdPageToCache = 0x13;
constexpr uint8_t CmdRead = 0x03;
constexpr uint8_t CmdLoad = 0x02;
constexpr uint8_t CmdProgram = 0x10;
constexpr uint8_t CmdErase = 0xD8;
constexpr uint8_t RegProtection = 0xA0;
constexpr uint8_t RegConfiguration = 0xB0;
constexpr uint8_t RegStatus = 0xC0;
constexpr uint8_t StatusBusy = 0x01;
constexpr uint8_t StatusWel = 0x02;
constexpr uint8_t StatusEraseFail = 0x04;
constexpr uint8_t StatusProgramFail = 0x08;
constexpr uint8_t StatusEccUncorrectable = 0x20;
constexpr uint32_t DmaTimeoutMs = 100;
constexpr uint32_t ReadTimeoutMs = 10;
constexpr uint32_t ProgramTimeoutMs = 10;
constexpr uint32_t EraseTimeoutMs = 100;
constexpr uint32_t ConnectTimeoutMs = 20;
constexpr uint8_t CheckpointMagic[3] = {'U', 'C', 'I'};
}

W25N01GV::W25N01GV(SPI_HandleTypeDef *spi, GPIO_TypeDef *csPort, uint16_t csPin,
                   GPIO_TypeDef *wpPort, uint16_t wpPin,
                   GPIO_TypeDef *holdPort, uint16_t holdPin)
    : spi_(spi), csPort_(csPort), csPin_(csPin), wpPort_(wpPort), wpPin_(wpPin),
      holdPort_(holdPort), holdPin_(holdPin) {}

bool W25N01GV::BlockingTransfer(const uint8_t *tx, uint8_t *rx, uint16_t count) {
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
    const HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(spi_,
        const_cast<uint8_t *>(tx), rx, count, ConnectTimeoutMs);
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
    return result == HAL_OK;
}

bool W25N01GV::BlockingCommand(const uint8_t *bytes, uint16_t count) {
    uint8_t ignored[4]{};
    return count <= sizeof ignored && BlockingTransfer(bytes, ignored, count);
}

bool W25N01GV::BlockingStatus(uint8_t address, uint8_t &value) {
    const uint8_t tx[3] = {CmdReadStatus, address, 0xFF};
    uint8_t rx[3]{};
    if (!BlockingTransfer(tx, rx, sizeof tx)) return false;
    value = rx[2];
    return true;
}

bool W25N01GV::BlockingWriteRegister(uint8_t address, uint8_t value) {
    const uint8_t wren = CmdWriteEnable;
    const uint8_t write[3] = {CmdWriteStatus, address, value};
    uint8_t status = 0;
    if (!BlockingCommand(&wren, 1) || !BlockingStatus(RegStatus, status) ||
        !(status & StatusWel) || !BlockingCommand(write, sizeof write)) return false;
    const uint32_t started = HAL_GetTick();
    do {
        if (!BlockingStatus(RegStatus, status)) return false;
        if (!(status & StatusBusy)) return true;
    } while (HAL_GetTick() - started < ConnectTimeoutMs);
    return false;
}

W25N01GV::Result W25N01GV::ConnectBlocking() {
    if (Busy()) return Result::Busy;
    connected_ = false;
    mounted_ = false;
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(holdPort_, holdPin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_SET);
    // Covers tVSL=1 ms and tPUW=5 ms even if called immediately after GPIO init.
    HAL_Delay(5);
    const uint8_t reset = CmdReset;
    if (!BlockingCommand(&reset, 1)) goto spi_error;
    {
        uint8_t status = 0;
        const uint32_t started = HAL_GetTick();
        do {
            if (!BlockingStatus(RegStatus, status)) goto spi_error;
            if (!(status & StatusBusy)) break;
        } while (HAL_GetTick() - started < ConnectTimeoutMs);
        if (status & StatusBusy) goto spi_error;
    }
    {
        const uint8_t tx[5] = {CmdJedec, 0xFF, 0xFF, 0xFF, 0xFF};
        uint8_t rx[5]{};
        if (!BlockingTransfer(tx, rx, sizeof tx)) goto spi_error;
        if (rx[2] != 0xEF || rx[3] != 0xAA || rx[4] != 0x21) {
            lastResult_ = Result::WrongDevice;
            HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
            return lastResult_;
        }
    }
    if (!BlockingWriteRegister(RegProtection, 0x02) ||
        !BlockingWriteRegister(RegConfiguration, 0x10)) goto spi_error;
    {
        uint8_t sr1 = 0, sr2 = 0;
        if (!BlockingStatus(RegProtection, sr1) ||
            !BlockingStatus(RegConfiguration, sr2)) goto spi_error;
        if (sr1 != 0x02 || sr2 != 0x10) {
            lastResult_ = Result::ConfigurationError;
            HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
            return lastResult_;
        }
    }
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
    connected_ = true;
    lastResult_ = Result::Ok;
    return lastResult_;
spi_error:
    HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
    lastResult_ = Result::SpiError;
    return lastResult_;
}

void W25N01GV::Begin(Operation operation, Phase phase, Completion completion, void *context) {
    operation_ = operation;
    phase_ = phase;
    completion_ = completion;
    completionContext_ = context;
    ioResult_ = Result::Pending;
    lastResult_ = Result::Pending;
}

void W25N01GV::Finish(Result result) {
    const Operation finished = operation_;
    const Completion callback = completion_;
    void *context = completionContext_;
    const uint16_t block = targetBlock_;
    const uint8_t page = targetPage_;
    EndTransaction();
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
    io_ = Io::Idle;
    ioStage_ = IoStage::Idle;
    operation_ = Operation::None;
    phase_ = Phase::Idle;
    completion_ = nullptr;
    completionContext_ = nullptr;
    readOutput_ = nullptr;
    appendPage_ = nullptr;
    programPage_ = nullptr;
    lastResult_ = result;
    if (callback) callback(finished, result, block, page, context);
}

W25N01GV::Result W25N01GV::MountAsync(Completion completion, void *context) {
    if (!connected_) return Result::NotConnected;
    if (!spi_->hdmarx || !spi_->hdmatx) return Result::ConfigurationError;
    if (Busy()) return Result::Busy;
    mounted_ = false;
    firstBlock_ = activeBlock_ = BlockCount;
    nextBlock_ = 0;
    nextPage_ = 1;
    scanBlock_ = BlockCount - 1;
    targetBlock_ = scanBlock_;
    targetPage_ = 0;
    Begin(Operation::Mount, Phase::MountHeaders, completion, context);
    return Result::Pending;
}

W25N01GV::Result W25N01GV::FormatAsync(Completion completion, void *context) {
    if (!connected_) return Result::NotConnected;
    if (!spi_->hdmarx || !spi_->hdmatx) return Result::ConfigurationError;
    if (Busy()) return Result::Busy;
    mounted_ = false;
    scanBlock_ = 0;
    targetBlock_ = 0;
    targetPage_ = 0;
    Begin(Operation::Format, Phase::FormatHeaders, completion, context);
    return Result::Pending;
}

W25N01GV::Result W25N01GV::AppendPageAsync(const uint8_t *page,
                                            Completion completion, void *context) {
    if (!connected_) return Result::NotConnected;
    if (!spi_->hdmarx || !spi_->hdmatx) return Result::ConfigurationError;
    if (!mounted_) return Result::NotMounted;
    if (Busy()) return Result::Busy;
    if (!page || page[0] != 0 || page[2044] != 0 || page[2045] != 0 ||
        page[2046] != 0 || page[2047] != 0) return Result::BadArgument;
    if (nextBlock_ >= BlockCount) return Result::Full;
    appendPage_ = page;
    targetBlock_ = nextBlock_;
    targetPage_ = nextPage_;
    if (activeBlock_ == nextBlock_ && nextPage_ < PagesPerBlock) {
        Begin(Operation::Append, Phase::AppendData, completion, context);
        StartProgram(nextBlock_, nextPage_);
    } else {
        scanBlock_ = nextBlock_;
        Begin(Operation::Append, Phase::AppendFindBlock, completion, context);
    }
    return Result::Pending;
}

W25N01GV::Result W25N01GV::ReadNextPageAsync(uint8_t *output,
                                               Completion completion, void *context) {
    if (!connected_) return Result::NotConnected;
    if (!spi_->hdmarx || !spi_->hdmatx) return Result::ConfigurationError;
    if (!mounted_) return Result::NotMounted;
    if (Busy()) return Result::Busy;
    if (!output) return Result::BadArgument;
    if (activeBlock_ == BlockCount || readBlock_ > activeBlock_ ||
        (readBlock_ == activeBlock_ && readPage_ >= nextPage_)) return Result::EndOfLog;
    readOutput_ = output;
    targetBlock_ = readBlock_;
    targetPage_ = readPage_;
    Begin(Operation::ReadNext, readHeaderNeeded_ ? Phase::ReadHeader : Phase::ReadData,
          completion, context);
    return Result::Pending;
}

void W25N01GV::RewindReadCursor() {
    if (Busy()) return;
    readBlock_ = firstBlock_;
    readPage_ = 1;
    readHeaderNeeded_ = true;
}

void W25N01GV::TxRxCompleteCallback(SPI_HandleTypeDef *completedSpi) {
    if (completedSpi == spi_ && dmaActive_) dmaDone_ = true;
}

void W25N01GV::SpiErrorCallback(SPI_HandleTypeDef *failedSpi) {
    if (failedSpi == spi_ && dmaActive_) dmaError_ = true;
}

void W25N01GV::EndTransaction() {
    if (csAsserted_) {
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_SET);
        csAsserted_ = false;
    }
}

bool W25N01GV::TransferSmall(uint8_t count, bool holdCs) {
    if (!csAsserted_) {
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
        csAsserted_ = true;
    }
    dmaDone_ = dmaError_ = false;
    releaseCsAfterDma_ = !holdCs;
    dmaStartMs_ = HAL_GetTick();
    dmaActive_ = true;
    if (HAL_SPI_TransmitReceive_DMA(spi_, txSmall_, rxSmall_, count) == HAL_OK) return true;
    dmaActive_ = false;
    EndTransaction();
    return false;
}

bool W25N01GV::TransferPage(bool writing, bool holdCs) {
    if (!csAsserted_) {
        HAL_GPIO_WritePin(csPort_, csPin_, GPIO_PIN_RESET);
        csAsserted_ = true;
    }
    if (!writing) std::memset(txPage_, 0xFF, PageBytes);
    dmaDone_ = dmaError_ = false;
    releaseCsAfterDma_ = !holdCs;
    dmaStartMs_ = HAL_GetTick();
    dmaActive_ = true;
    uint8_t *const source = writing ? const_cast<uint8_t *>(programPage_) : txPage_;
    if (HAL_SPI_TransmitReceive_DMA(spi_, source, rxPage_, PageBytes) == HAL_OK) return true;
    dmaActive_ = false;
    EndTransaction();
    return false;
}

bool W25N01GV::PollStatus() {
    txSmall_[0] = CmdReadStatus;
    txSmall_[1] = RegStatus;
    txSmall_[2] = 0xFF;
    return TransferSmall(3);
}

void W25N01GV::FailIo(Result result) {
    EndTransaction();
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
    ioResult_ = result;
    io_ = Io::Idle;
    ioStage_ = IoStage::Idle;
    // A HAL/DMA failure can leave the peripheral in an uncertain state. Do
    // not start another transaction until the board is reset and connected.
    if (result == Result::Timeout || result == Result::SpiError) connected_ = false;
}

void W25N01GV::StartRead(uint16_t block, uint8_t page) {
    ioBlock_ = block;
    ioPage_ = page;
    io_ = Io::Read;
    ioStage_ = IoStage::ReadToCache;
    ioResult_ = Result::Pending;
    ioEccBad_ = false;
}

void W25N01GV::StartProgram(uint16_t block, uint8_t page) {
    ioBlock_ = block;
    ioPage_ = page;
    io_ = Io::Program;
    ioStage_ = IoStage::WriteEnable;
    ioResult_ = Result::Pending;
    programPage_ = page == 0 ? txPage_ : appendPage_;
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_SET);
}

void W25N01GV::StartErase(uint16_t block) {
    ioBlock_ = block;
    ioPage_ = 0;
    io_ = Io::Erase;
    ioStage_ = IoStage::EraseEnable;
    ioResult_ = Result::Pending;
    HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_SET);
}

W25N01GV::Step W25N01GV::ProcessIo() {
    if (io_ == Io::Idle) return ioResult_ == Result::Ok ? Step::Done : Step::Failed;
    if (dmaActive_) {
        if (dmaError_) {
            dmaActive_ = false;
            FailIo(Result::SpiError);
            return Step::Failed;
        }
        if (!dmaDone_) {
            if (HAL_GetTick() - dmaStartMs_ >= DmaTimeoutMs) {
                // Do not start another transfer; the peripheral may still own DMA.
                dmaActive_ = false;
                FailIo(Result::Timeout);
                return Step::Failed;
            }
            return Step::Pending;
        }
        dmaActive_ = false;
        dmaDone_ = false;
        if (releaseCsAfterDma_) EndTransaction();
        switch (ioStage_) {
        case IoStage::ReadToCache:
            ioStage_ = IoStage::ReadWaitCache;
            busyStartMs_ = HAL_GetTick();
            break;
        case IoStage::ReadWaitCache:
            status_ = rxSmall_[2];
            if (status_ & StatusBusy) {
                if (HAL_GetTick() - busyStartMs_ >= ReadTimeoutMs) {
                    FailIo(Result::Timeout); return Step::Failed;
                }
                return Step::Pending;
            }
            ioStage_ = IoStage::ReadHeader;
            break;
        case IoStage::ReadHeader: ioStage_ = IoStage::ReadData; break;
        case IoStage::ReadData:
            ioStage_ = IoStage::ReadWaitRelease;
            busyStartMs_ = HAL_GetTick();
            break;
        case IoStage::ReadWaitRelease:
            status_ = rxSmall_[2];
            if (status_ & StatusBusy) {
                if (HAL_GetTick() - busyStartMs_ >= ReadTimeoutMs) {
                    FailIo(Result::Timeout); return Step::Failed;
                }
                return Step::Pending;
            }
            ioEccBad_ = (status_ & StatusEccUncorrectable) != 0;
            ioResult_ = Result::Ok;
            io_ = Io::Idle;
            return Step::Done;
        case IoStage::WriteEnable: ioStage_ = IoStage::WriteCheckWel; break;
        case IoStage::WriteCheckWel:
            if (!(rxSmall_[2] & StatusWel)) {
                FailIo(Result::ProgramFailed); return Step::Failed;
            }
            ioStage_ = IoStage::WriteLoadHeader;
            break;
        case IoStage::WriteLoadHeader: ioStage_ = IoStage::WriteLoadData; break;
        case IoStage::WriteLoadData: ioStage_ = IoStage::WriteExecute; break;
        case IoStage::WriteExecute:
            ioStage_ = IoStage::WriteWaitReady;
            busyStartMs_ = HAL_GetTick();
            break;
        case IoStage::WriteWaitReady:
            status_ = rxSmall_[2];
            if (status_ & StatusBusy) {
                if (HAL_GetTick() - busyStartMs_ >= ProgramTimeoutMs) {
                    FailIo(Result::Timeout); return Step::Failed;
                }
                return Step::Pending;
            }
            HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
            if (status_ & StatusProgramFail) {
                FailIo(Result::ProgramFailed); return Step::Failed;
            }
            ioResult_ = Result::Ok;
            io_ = Io::Idle;
            return Step::Done;
        case IoStage::EraseEnable: ioStage_ = IoStage::EraseCheckWel; break;
        case IoStage::EraseCheckWel:
            if (!(rxSmall_[2] & StatusWel)) {
                FailIo(Result::EraseFailed); return Step::Failed;
            }
            ioStage_ = IoStage::EraseCommand;
            break;
        case IoStage::EraseCommand:
            ioStage_ = IoStage::EraseWaitReady;
            busyStartMs_ = HAL_GetTick();
            break;
        case IoStage::EraseWaitReady:
            status_ = rxSmall_[2];
            if (status_ & StatusBusy) {
                if (HAL_GetTick() - busyStartMs_ >= EraseTimeoutMs) {
                    FailIo(Result::Timeout); return Step::Failed;
                }
                return Step::Pending;
            }
            HAL_GPIO_WritePin(wpPort_, wpPin_, GPIO_PIN_RESET);
            if (status_ & StatusEraseFail) {
                FailIo(Result::EraseFailed); return Step::Failed;
            }
            ioResult_ = Result::Ok;
            io_ = Io::Idle;
            return Step::Done;
        default: FailIo(Result::SpiError); return Step::Failed;
        }
    }
    // At most one new DMA transaction is launched per ProcessTask() call.
    const uint16_t pa = static_cast<uint16_t>((ioBlock_ << 6) | ioPage_);
    switch (ioStage_) {
    case IoStage::ReadToCache:
        txSmall_[0] = CmdPageToCache; txSmall_[1] = 0;
        txSmall_[2] = static_cast<uint8_t>(pa >> 8);
        txSmall_[3] = static_cast<uint8_t>(pa);
        if (!TransferSmall(4)) break;
        return Step::Pending;
    case IoStage::ReadWaitCache:
    case IoStage::ReadWaitRelease:
    case IoStage::WriteCheckWel:
    case IoStage::WriteWaitReady:
    case IoStage::EraseCheckWel:
    case IoStage::EraseWaitReady:
        if (!PollStatus()) break;
        return Step::Pending;
    case IoStage::ReadHeader:
        txSmall_[0] = CmdRead; txSmall_[1] = 0;
        txSmall_[2] = 0; txSmall_[3] = 0;
        if (!TransferSmall(4, true)) break;
        return Step::Pending;
    case IoStage::ReadData:
        if (!TransferPage(false)) break;
        return Step::Pending;
    case IoStage::WriteEnable:
    case IoStage::EraseEnable:
        txSmall_[0] = CmdWriteEnable;
        if (!TransferSmall(1)) break;
        return Step::Pending;
    case IoStage::WriteLoadHeader:
        txSmall_[0] = CmdLoad; txSmall_[1] = 0; txSmall_[2] = 0;
        if (!TransferSmall(3, true)) break;
        return Step::Pending;
    case IoStage::WriteLoadData:
        if (!TransferPage(true)) break;
        return Step::Pending;
    case IoStage::WriteExecute:
    case IoStage::EraseCommand:
        txSmall_[0] = (ioStage_ == IoStage::WriteExecute) ? CmdProgram : CmdErase;
        txSmall_[1] = 0;
        txSmall_[2] = static_cast<uint8_t>(pa >> 8);
        txSmall_[3] = static_cast<uint8_t>(pa);
        if (!TransferSmall(4)) break;
        return Step::Pending;
    default: break;
    }
    FailIo(Result::SpiError);
    return Step::Failed;
}

bool W25N01GV::PageOccupied() const {
    for (size_t i = 0; i < PageBytes; ++i) {
        if (rxPage_[i] != 0xFF) return true;
    }
    return false;
}

void W25N01GV::MakeCheckpoint(uint16_t block) {
    std::memset(txPage_, 0xFF, PageBytes);
    txPage_[1] = CheckpointMagic[0];
    txPage_[2] = CheckpointMagic[1];
    txPage_[3] = CheckpointMagic[2];
    txPage_[4] = FormatVersion;
    txPage_[5] = static_cast<uint8_t>(block >> 8);
    txPage_[6] = static_cast<uint8_t>(block);
    std::memset(txPage_ + 7, 0, 4);
}

bool W25N01GV::CheckpointValid(uint16_t block) const {
    return !ioEccBad_ && rxPage_[0] == 0xFF &&
        rxPage_[1] == CheckpointMagic[0] &&
        rxPage_[2] == CheckpointMagic[1] &&
        rxPage_[3] == CheckpointMagic[2] &&
        rxPage_[4] == FormatVersion &&
        rxPage_[5] == static_cast<uint8_t>(block >> 8) &&
        rxPage_[6] == static_cast<uint8_t>(block);
}

bool W25N01GV::DataPageValid() const {
    return !ioEccBad_ && rxPage_[0] == 0;
}

void W25N01GV::AdvanceReadCursor() {
    ++readPage_;
    if (readPage_ >= PagesPerBlock) {
        ++readBlock_;
        readPage_ = 1;
        readHeaderNeeded_ = true;
    }
}

void W25N01GV::ProcessTask() {
    if (operation_ == Operation::None) return;
    if (io_ != Io::Idle && ProcessIo() == Step::Pending) return;
    switch (phase_) {
    case Phase::MountHeaders:
        if (ioResult_ == Result::Pending) {
            StartRead(scanBlock_, 0);
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (!IsBadBlock() && CheckpointValid(scanBlock_)) {
            if (activeBlock_ == BlockCount) activeBlock_ = scanBlock_;
            firstBlock_ = scanBlock_;
        }
        if (scanBlock_ == 0) {
            if (activeBlock_ == BlockCount) {
                nextBlock_ = 0; nextPage_ = 1;
                readBlock_ = BlockCount;
                mounted_ = true;
                Finish(Result::Ok);
            } else {
                scanPage_ = PagesPerBlock - 1;
                phase_ = Phase::MountTail;
                ioResult_ = Result::Pending;
            }
            return;
        }
        --scanBlock_;
        ioResult_ = Result::Pending;
        return;
    case Phase::MountTail:
        if (ioResult_ == Result::Pending) {
            StartRead(activeBlock_, static_cast<uint8_t>(scanPage_));
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (PageOccupied()) {
            nextPage_ = static_cast<uint8_t>(scanPage_ + 1);
        } else if (scanPage_ > 1) {
            --scanPage_;
            ioResult_ = Result::Pending;
            return;
        } else {
            nextPage_ = 1;
        }
        nextBlock_ = nextPage_ < PagesPerBlock ? activeBlock_ : activeBlock_ + 1;
        readBlock_ = firstBlock_;
        readPage_ = 1;
        readHeaderNeeded_ = true;
        mounted_ = true;
        Finish(Result::Ok);
        return;
    case Phase::FormatHeaders:
        if (scanBlock_ >= BlockCount) {
            firstBlock_ = activeBlock_ = BlockCount;
            nextBlock_ = 0; nextPage_ = 1;
            readBlock_ = BlockCount; readPage_ = 1;
            readHeaderNeeded_ = true;
            mounted_ = true;
            Finish(Result::Ok);
            return;
        }
        if (ioResult_ == Result::Pending) {
            StartRead(scanBlock_, 0);
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (!IsBadBlock() && PageOccupied()) {
            phase_ = Phase::FormatErasing;
            StartErase(scanBlock_);
            return;
        }
        ++scanBlock_;
        ioResult_ = Result::Pending;
        return;
    case Phase::FormatErasing:
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        ++scanBlock_;
        phase_ = Phase::FormatHeaders;
        ioResult_ = Result::Pending;
        return;
    case Phase::AppendFindBlock:
        if (scanBlock_ >= BlockCount) { Finish(Result::Full); return; }
        if (ioResult_ == Result::Pending) {
            StartRead(scanBlock_, 0);
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (IsBadBlock()) {
            ++scanBlock_;
            ioResult_ = Result::Pending;
            return;
        }
        phase_ = Phase::AppendErase;
        StartErase(scanBlock_);
        return;
    case Phase::AppendErase:
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        MakeCheckpoint(scanBlock_);
        phase_ = Phase::AppendCheckpoint;
        StartProgram(scanBlock_, 0);
        return;
    case Phase::AppendCheckpoint:
        if (!IoSucceeded()) { Finish(Result::CheckpointFailed); return; }
        phase_ = Phase::AppendVerifyCheckpoint;
        StartRead(scanBlock_, 0);
        return;
    case Phase::AppendVerifyCheckpoint:
        if (!IoSucceeded() || !CheckpointValid(scanBlock_)) {
            Finish(Result::CheckpointFailed);
            return;
        }
        if (firstBlock_ == BlockCount) {
            firstBlock_ = scanBlock_;
            readBlock_ = scanBlock_;
            readPage_ = 1;
            readHeaderNeeded_ = true;
        }
        activeBlock_ = scanBlock_;
        nextBlock_ = scanBlock_;
        nextPage_ = 1;
        targetBlock_ = activeBlock_;
        targetPage_ = nextPage_;
        phase_ = Phase::AppendData;
        StartProgram(activeBlock_, nextPage_);
        return;
    case Phase::AppendData: {
        const Result result = ioResult_;
        // Once a program attempt starts, its destination is never reused in
        // this session even if power or SPI failed before completion.
        ++nextPage_;
        if (nextPage_ >= PagesPerBlock) nextBlock_ = activeBlock_ + 1;
        Finish(result);
        return;
    }
    case Phase::ReadHeader:
        if (readBlock_ > activeBlock_ ||
            (readBlock_ == activeBlock_ && readPage_ >= nextPage_)) {
            Finish(Result::EndOfLog); return;
        }
        if (ioResult_ == Result::Pending) {
            StartRead(readBlock_, 0);
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (!CheckpointValid(readBlock_)) {
            ++readBlock_;
            readPage_ = 1;
            ioResult_ = Result::Pending;
            return;
        }
        readHeaderNeeded_ = false;
        phase_ = Phase::ReadData;
        ioResult_ = Result::Pending;
        return;
    case Phase::ReadData:
        if (readBlock_ > activeBlock_ ||
            (readBlock_ == activeBlock_ && readPage_ >= nextPage_)) {
            Finish(Result::EndOfLog); return;
        }
        if (readPage_ >= PagesPerBlock) {
            AdvanceReadCursor();
            phase_ = Phase::ReadHeader;
            ioResult_ = Result::Pending;
            return;
        }
        if (readHeaderNeeded_) {
            phase_ = Phase::ReadHeader;
            ioResult_ = Result::Pending;
            return;
        }
        if (ioResult_ == Result::Pending) {
            targetBlock_ = readBlock_;
            targetPage_ = readPage_;
            StartRead(readBlock_, readPage_);
            return;
        }
        if (!IoSucceeded()) { Finish(ioResult_); return; }
        if (DataPageValid()) {
            std::memcpy(readOutput_, rxPage_ + 1, PayloadBytes);
            AdvanceReadCursor();
            Finish(Result::Ok);
            return;
        }
        AdvanceReadCursor();
        ioResult_ = Result::Pending;
        return;
    default:
        Finish(Result::SpiError);
        return;
    }
}
