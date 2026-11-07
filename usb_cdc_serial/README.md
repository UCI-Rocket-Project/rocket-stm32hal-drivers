# USB CDC command interface

The `UsbCdcSerial` driver owns a 256-byte receive ring and one caller-owned
transmit buffer. `AMI_CDC_Receive()` is called from CubeMX's preserved
`CDC_Receive_FS` user-code section; it only copies received bytes. The
controller parses commands and advances USB transmission from its 1 ms task.
USB data is sent in 256-byte chunks, waiting for each CDC transfer to finish.

Connect to the board's virtual serial port (for example `/dev/ttyACM0` on
Linux). Commands are lowercase ASCII lines terminated by LF; CRLF also works.
CDC's configured baud rate does not control USB throughput. The board mounts
existing version 2 flash data at boot and does **not** erase or record until
commanded. ADXL initialization begins at least 10 seconds after boot; `record
start` returns `ERR not ready` until it succeeds.

| Command | Response and behavior |
| --- | --- |
| `help` | Lists commands and page indexing. |
| `status` | Reports mode, ADXL readiness, flash connection/mount/result, next physical block/page, pages written this boot, dropped samples this boot, and USB RX overflows. |
| `erase` | `OK erase started`, then `DONE erase` or `ERR`. Runs the async flash format; usable after `full`. |
| `record start` | `OK recording`. Appends 339-sample pages to the existing log. |
| `record stop` | `OK stopping at page boundary`, then `DONE record`. It fills and writes the current page before stopping; the ADXL must still be producing samples. |
| `read <n>` | One zero-based **logical** valid page as CSV; skipped bad/ECC-invalid pages do not get indexes. Ends with `DONE read` or `ERR page out of range`. |
| `dump csv` | All valid pages as CSV; ends with `DONE dump pages=N`. |
| `dump bin` | All valid pages as binary frames; ends with `UEND` and a page count. |
| `abort` | During a read/dump, stops after any in-flight flash/USB transfer. CSV ends with `DONE ... aborted`; binary ends with `UABT` and a count. |

Only `help`, `status`, `record stop`, and `abort` are accepted while their
relevant operation is active. During a read/dump, only `abort` is processed;
other input is discarded so text cannot corrupt a binary stream. Erase,
record, and read/dump are mutually exclusive. `record start` waits for both
flash mount and ADXL initialization. The initial flash connection and ADXL
initialization use the existing startup path; flash format, recording, reads,
and USB transmission are cooperative/asynchronous.

CSV begins with `page,sample,page_time10us,x,y,z`. Every sample in a page
shares the stored timestamp, which is captured when the first sample is
queued. Use `sample / 32000` seconds for nominal spacing within a page. The
32-bit 10 us clock wraps after 11 h 55 min 49.67 s and restarts on boot.

Binary `dump bin` begins with `OK dump bin\r\n`, then repeats:

| Bytes | Meaning |
| --- | --- |
| 0..3 | ASCII `UCP2` |
| 4..7 | Zero-based logical page index, little-endian uint32 |
| 8..9 | Payload length 2043, little-endian uint16 |
| 10..11 | Sample count 339, little-endian uint16 |
| 12..2054 | Flash payload: timestamp uint32 LE, 339 interleaved signed int16 LE XYZ triples, five `FF` pad bytes |

The final frame is eight bytes: `UEND` (or `UABT` if aborted) and the number
of completed pages as a little-endian uint32. The USB transport checks packet
integrity; the application frame has no separate CRC. Run
`python3 tools/dump_flash.py /dev/ttyACM0 capture.csv` from the repository
root for a binary dump decoded to CSV. That script requires `pyserial`.

CubeMX owns `ami-firmware.ioc` and the generated USB middleware. The receive
hook is inside `USER CODE BEGIN 6`, so regeneration should preserve it. For
capture timing, set `OTG_FS_IRQn` to a lower preemption priority than the
ADXL EXTI, SPI1, and DMA IRQs in CubeMX; the current IOC gives them all
priority 0. The driver does not edit the IOC or HAL-generated interrupt code.
