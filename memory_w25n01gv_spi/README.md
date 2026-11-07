# W25N01GV SPI NAND log driver

`W25N01GV` matches the project's cooperative driver pattern: `ConnectBlocking()`
does the short boot-time reset, JEDEC ID check, and status-register setup; all
other flash work runs in `ProcessTask()`. The controller schedules it every
100 µs. Every post-connect SPI transaction, including a one-byte command or
status poll, uses SPI TX/RX DMA. DMA and SPI error callbacks set flags only;
chip select and the state machine change in `ProcessTask()`.

## CubeMX setup required before running

The driver does **not** edit `ami-firmware.ioc` or generated HAL files. In
CubeMX, configure SPI2 RX on DMA1 Channel 4 and SPI2 TX on DMA1 Channel 5,
both Normal mode, byte alignment, peripheral increment disabled, memory
increment enabled. Enable the DMA1 Channel 4/5 interrupts and SPI2 global
interrupt. Keep SPI2 as master, two-line, 8-bit, mode 0. Generate code and
check that `hspi2.hdmarx` and `hspi2.hdmatx` are linked and the generated IRQ
handlers call `HAL_DMA_IRQHandler`/`HAL_SPI_IRQHandler`. The driver rejects
async calls if either DMA handle is absent.

Set `MEM_nHOLD` (PC8) high in CubeMX's initial GPIO output level; the current
generated code sets it low, which holds the flash inactive until
`ConnectBlocking()` raises it. `MEM_nWP` (PC7) may start high; the driver raises
it for status setup, erase, and program, then returns it low. `MEM_nCS` (PB12)
must start high. Keep these GPIO settings in the IOC so regeneration preserves
them.

## Usage

```cpp
auto connected = flash.ConnectBlocking();
if (connected == W25N01GV::Result::Ok) flash.MountAsync(onComplete, context);
// Call flash.ProcessTask() periodically until flash.Mounted() is true.

uint8_t payload[W25N01GV::PayloadBytes];
flash.AppendPageAsync(payload, onComplete, context); // copies payload now

uint8_t output[W25N01GV::PayloadBytes];
flash.RewindReadCursor();
flash.ReadNextPageAsync(output, onComplete, context); // keep output alive
```

Only one operation may run at a time. An accepted call returns `Pending`; its
callback runs in `ProcessTask()` and reports `Ok`, `EndOfLog`, or an error.
Polling `Busy()` and `LastResult()` also works. `FormatAsync()` erases every
good block whose page 0 contains programmed main-area bits, then leaves an
empty mounted log. On startup, `MountAsync()` scans checkpoint pages backwards,
then scans the last block backwards to find the append position. A failed or
interrupted page program is never retried at the same page in the same running
session; after a power cycle, mount treats any page with a programmed main-area
bit as occupied. A failed checkpoint must be erased before reuse. SPI/DMA
errors or timeouts stop new transactions until reset/reconnection.

## On-flash format, version 1

- 1024 physical blocks; 64 pages per block; 2048 main-area bytes per page.
- A factory bad block has page 0 byte 0 different from `FF`; the driver skips
  it and never erases it. This version does not inspect the spare-area marker.
- A good block's page 0 has byte 0 = `FF`; bytes 1..3 = `UCI`, byte 4 = `01`,
  bytes 5..6 = physical block number (big endian), bytes 7..10 = CRC-32 of
  bytes 1..6 (big endian), and all other main-area bytes = `FF`.
- Each data page has byte 0 = `00`, bytes 1..2043 = exactly 2043 payload
  bytes, and bytes 2044..2047 = CRC-32 of bytes 0..2043 (big endian).
- CRC-32 uses the IEEE reflected polynomial `EDB88320`, initial value
  `FFFFFFFF`, and final XOR `FFFFFFFF`.

Page 0 is the checkpoint; data pages are programmed once, in ascending order
from 1 to 63. The driver uses the chip's ECC and skips data pages with an
uncorrectable ECC status or invalid CRC. It has no wear leveling or reclaim
scheme; when all usable blocks are consumed, append returns `Full`. A
power-interrupted format can leave old checkpoints; rerun `FormatAsync()` to
complete a requested erase. This test-board format considers only the 2048
main bytes when deciding whether a page is occupied.
