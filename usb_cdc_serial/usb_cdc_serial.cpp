#include "usb_cdc_serial.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "usbd_core.h"
#include "stm32f1xx_hal.h"

extern "C" USBD_HandleTypeDef hUsbDeviceFS;
extern "C" PCD_HandleTypeDef hpcd_USB_OTG_FS;

static UsbCdcSerial *activeSerial = nullptr;
volatile uint8_t usbVbusHigh = 0;
volatile uint8_t usbVbusGpioHigh = 0;
volatile uint32_t usbVbusDetaches = 0;
volatile uint32_t usbVbusAttaches = 0;

UsbCdcSerial::UsbCdcSerial() {
    activeSerial = this;
}

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
    return usbAttached_ && hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED &&
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

void UsbCdcSerial::ProcessVbus() {
    if (!vbusInitialized_) {
        // Cube's F1 USB LL enables VBUSBSEN unconditionally. This board uses
        // a 33k/82k divider intended for GPIO sensing, so release PA9 from
        // the peripheral sensor before sampling it as an input.
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
        USBD_LL_DevDisconnected(&hUsbDeviceFS);
        USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBUSBSEN;
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        vbusInitialized_ = true;
        vbusSampleHigh_ = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET;
        vbusChangedMs_ = HAL_GetTick();
    }

    const bool high = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET;
    usbVbusHigh = (USB_OTG_FS->GOTGCTL & USB_OTG_GOTGCTL_BSVLD) != 0;
    usbVbusGpioHigh = high;
    const uint32_t now = HAL_GetTick();
    if (high != vbusSampleHigh_) {
        vbusSampleHigh_ = high;
        vbusChangedMs_ = now;
    }
    if (high == usbAttached_ || now - vbusChangedMs_ < VbusDebounceMs) return;

    // USB callbacks may also tear down CDC on cable removal. Keep the stack
    // transition out of IRQ context and exclude a concurrent USB callback.
    HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
    if (high) {
        HAL_PCD_DevConnect(&hpcd_USB_OTG_FS);
        ++usbVbusAttaches;
    } else {
        HAL_PCD_DevDisconnect(&hpcd_USB_OTG_FS);
        USBD_LL_DevDisconnected(&hUsbDeviceFS);
        txData_ = nullptr;
        txInFlight_ = false;
        rxTail_ = rxHead_;
        ++usbVbusDetaches;
    }
    usbAttached_ = high;
    HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
}

void UsbCdcSerial::ProcessTask() {
    ProcessVbus();
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
