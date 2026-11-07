#include "usb_cdc_serial.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "stm32f1xx_hal.h"

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

static UsbCdcSerial *activeSerial = nullptr;

UsbCdcSerial::UsbCdcSerial() { activeSerial = this; }

extern "C" void AMI_CDC_Receive(const uint8_t *data, uint32_t length) {
    if (activeSerial) activeSerial->OnReceive(data, length);
}

void UsbCdcSerial::OnReceive(const uint8_t *data, size_t length) {
    for (size_t i = 0; i < length; ++i) {
        const uint16_t next = static_cast<uint16_t>((rxHead_ + 1U) % RxCapacity);
        if (next == rxTail_) {
            ++rxOverflows_;
            break;
        }
        rx_[rxHead_] = data[i];
        __DMB();
        rxHead_ = next;
    }
}

bool UsbCdcSerial::ReadByte(uint8_t &byte) {
    const uint16_t tail = rxTail_;
    if (tail == rxHead_) return false;
    __DMB();
    byte = rx_[tail];
    rxTail_ = static_cast<uint16_t>((tail + 1U) % RxCapacity);
    return true;
}

bool UsbCdcSerial::Connected() const {
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
        hUsbDeviceFS.pClassData != nullptr;
}

bool UsbCdcSerial::Write(const uint8_t *data, size_t length) {
    if (!data || !length || Busy() || !Connected()) return false;
    txData_ = data;
    txLength_ = length;
    txOffset_ = 0;
    txChunk_ = 0;
    txInFlight_ = false;
    return true;
}

void UsbCdcSerial::ProcessTask() {
    if (!txData_) return;
    if (!Connected()) {
        txData_ = nullptr;
        txInFlight_ = false;
        return;
    }
    if (txInFlight_) {
        auto *cdc = static_cast<USBD_CDC_HandleTypeDef *>(hUsbDeviceFS.pClassData);
        if (cdc->TxState != 0U) return;
        txOffset_ += txChunk_;
        txInFlight_ = false;
    }
    if (txOffset_ == txLength_) {
        txData_ = nullptr;
        return;
    }
    const size_t remaining = txLength_ - txOffset_;
    txChunk_ = static_cast<uint16_t>(remaining < TxChunkBytes ? remaining : TxChunkBytes);
    if (CDC_Transmit_FS(const_cast<uint8_t *>(txData_ + txOffset_), txChunk_) == USBD_OK) {
        txInFlight_ = true;
    }
}
