#pragma once

#include <cstddef>
#include <cstdint>

// Cooperative USB CDC byte transport. The USB receive IRQ only copies bytes
// into a single-producer ring. Transmission advances from ProcessTask().
// The caller owns a transmit buffer until Busy() becomes false.
class UsbCdcSerial {
public:
    UsbCdcSerial();
    void OnReceive(const uint8_t *data, size_t length);
    bool ReadByte(uint8_t &byte);
    bool Write(const uint8_t *data, size_t length);
    void ProcessTask();
    bool Connected() const;
    bool Busy() const { return txData_ != nullptr; }
    uint32_t RxOverflows() const { return rxOverflows_; }

private:
    enum class VbusState : uint8_t { Startup, Detached, CheckingHardware, Attached, Failed };
    static constexpr uint16_t RxCapacity = 256;
    static constexpr size_t TxChunkBytes = 256;
    static constexpr uint32_t VbusDebounceMs = 10;
    void ProcessVbus();
    VbusState vbusState_ = VbusState::Startup;
    bool vbusSampleHigh_ = false;
    bool usbAttached_ = false;
    uint32_t vbusChangedMs_ = 0;
    uint8_t rx_[RxCapacity]{};
    volatile uint16_t rxHead_ = 0;
    volatile uint16_t rxTail_ = 0;
    volatile uint32_t rxOverflows_ = 0;
    const uint8_t *txData_ = nullptr;
    size_t txLength_ = 0;
    size_t txOffset_ = 0;
    uint16_t txChunk_ = 0;
    bool txInFlight_ = false;
};

extern "C" void AMI_CDC_Receive(const uint8_t *data, uint32_t length);
