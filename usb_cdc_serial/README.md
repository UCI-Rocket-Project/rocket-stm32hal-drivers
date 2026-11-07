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
| `erase` | Requires three consecutive `erase` commands on separate lines. The first two report `ERASE 1/3` and `ERASE 2/3`; the third starts the async format and reports `ERASE 3/3`, then `DONE erase` or `ERR`. Any other command resets the count. Usable after `full`. |
| `arm [g]` | Waits for motion above the gravity-compensated threshold for 100 ms, then starts recording and reports `TRIGGER recording`. `arm` defaults to 2 g; for example, `arm 1.5` uses 1.5 g. Requires mounted flash and a ready accelerometer. |
| `disarm` | Cancels an armed trigger before recording starts. |
| `record start` | `OK recording`. Appends 339-sample pages to the existing log. |
| `record stop` | `OK stopping at page boundary`, then `DONE record`. It fills and writes the current page before stopping; the ADXL must still be producing samples. |
| `read <n>` | One zero-based **logical** valid page as CSV; skipped bad/ECC-invalid pages do not get indexes. Ends with `DONE read` or `ERR page out of range`. Also works when flash is full. |
| `dump csv` | All valid pages as CSV; ends with `DONE dump pages=N`. Also works when flash is full. |
| `dump bin` | All valid pages as binary frames; ends with `UEND` and a page count. Also works when flash is full. |
| `abort` | During a read/dump, stops after any in-flight flash/USB transfer. CSV ends with `DONE ... aborted`; binary ends with `UABT` and a count. |

While armed, `disarm` cancels the trigger and `record start` starts recording
immediately. During a read/dump, only `abort` is processed; other input is
discarded so text cannot corrupt a binary stream. Erase, record, and
read/dump are mutually exclusive. `record start` waits for both
flash mount and ADXL initialization. The initial flash connection and ADXL
initialization use the existing startup path; flash format, recording, reads,
and USB transmission are cooperative/asynchronous.

After `arm`, keep the board still for 100 ms while the controller averages a
gravity/bias baseline in its current orientation. It then checks one XYZ
sample every 32 accelerometer samples (about 1 kHz). Motion is the magnitude
of the current XYZ vector minus that stored baseline vector, so the initial
angle of the rocket does not matter. It must exceed the selected threshold
for 100 consecutive checks before recording starts. Accelerometer recovery
restarts calibration. A substantial orientation change after calibration can
also look like motion; `disarm` and `arm` again after repositioning. Recording
begins when the trigger fires, with no pre-trigger samples saved.
The LED breathes red while armed. On trigger it blinks green with a 100 ms
period for one second, then uses a blue default blink (500 ms period) during
recording. After `record stop` completes it returns to the blue pulse. A full flash
uses a purple pulse, including after reboot; a completed erase restores blue.
Queued LED indications run before the current default pulse; changing the
default color does not cancel them.
While erasing or running `dump csv`/`dump bin`, the LED shows a temporary
10 Hz pulse at R=100%, G=5%, B=30%. It does not change the saved default or
add an indication to the queue. Accelerometer recovery during those modes
does not queue the usual recovery pulse.

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
It queries `status` first and displays an estimated progress bar with pages,
XYZ samples, MiB, and transfer rate. The percentage uses the physical write
cursor, so bad or unreadable pages can make the estimate slightly high; the
final page count is exact.
Plot the CSV with `python3 tools/plot_accel.py capture.csv`, or save an image
with `python3 tools/plot_accel.py capture.csv --hz 100 --output accel.png`.
If Matplotlib has no interactive display, the first command automatically
saves `capture_plot.png` beside the CSV.
The plot uses every 320th XYZ sample by default (about 100 Hz), converts
the ±60 g readings at 500 counts/g, and puts X, Y, and Z on one set of axes.
Its time axis uses the nominal 32 kHz sample rate and does not show gaps
between separate recording sessions.

CubeMX owns `ami-firmware.ioc` and the generated USB middleware. The receive
hook is inside `USER CODE BEGIN 6`, so regeneration should preserve it. For
capture timing, set `OTG_FS_IRQn` to a lower preemption priority than the
ADXL EXTI, SPI1, and DMA IRQs in CubeMX; the current IOC gives them all
priority 0. The driver does not edit the IOC or HAL-generated interrupt code.

USB VBUS is wired through a 33 kΩ/82 kΩ divider to PA9. Configure PA9 as a
GPIO input without a pull in CubeMX, and disable the USB peripheral's hardware
VBUS sensing option if CubeMX exposes it. The F1 USB LL currently enables
`VBUSBSEN` regardless of that option, so the driver clears it once after USB
startup and uses PA9 GPIO to debounce cable attachment. This is a board-level
experiment: on the STM32F105, the internal D+ pull-up may still depend on
hardware B-session detection. If PA9 rises on replug but the host does not
enumerate, the VBUS circuit needs a hardware change.
