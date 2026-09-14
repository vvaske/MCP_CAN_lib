# MCP_CAN Library for Arduino

This directory contains the project's maintained copy of MCP_CAN v1.5.1 for MCP2515/MCP25625 controllers. It preserves the existing public function signatures and license, with fixes for message bounds, transmission results and recovery, configuration, and mode changes.

## Installation and initialization

For this PlatformIO project, use the files in this directory. For Arduino IDE projects, copy the library directory into the sketchbook's `libraries` directory and restart the IDE. Remove conflicting older MCP_CAN/CAN_BUS_Shield copies so the intended implementation is compiled.

`begin(idMode, bitrate, oscillator)` initializes the controller and returns `CAN_OK` or `CAN_FAILINIT`. A successful initialization leaves it in `MCP_LOOPBACK`; select `MCP_NORMAL` with `setMode()` when ready for bus operation, and check the result. Configuration failures now propagate to the caller.

The oscillator argument must match the controller's actual crystal. These are the discrete bitrate combinations accepted by this copy:

| Controller oscillator | Available nominal bitrates, kbit/s |
| --- | --- |
| `MCP_8MHZ` | 5, 10, 20, 31.25, 33.3, 40, 50, 80, 100, 125, 200, 250, 500 |
| `MCP_16MHZ` | 5, 10, 20, 33.3, 40, 50, 80, 100, 125, 200, 250, 500, 1000 |
| `MCP_20MHZ` | 40, 50, 80, 100, 125, 200, 250, 500, 1000 |

An 8 MHz oscillator with `CAN_1000KBPS` is rejected: the required four-TQ bit time cannot meet the MCP2515's segment constraints. The invalid `MCP_8MHz_1000kBPS_CFG*` definitions have been removed. The 8 MHz/80 kbit/s entry now uses an SJW of two TQ, within its PS2 setting. The previously missing CNF1 assignment for 16 MHz/50 kbit/s is also fixed.

The upstream README historically reported PCAN-USB reference testing for selected 16 MHz bitrates (5, 10, 20, 50, 100, 125, 250, 500 and 1000 kbit/s). That statement describes upstream testing, not hardware validation of this modified copy. The checks performed for these changes use software models and compilation; the table above is not a claim of new physical bus testing.

## Receiving messages

Supply valid output pointers and a receive array with space for eight bytes. Neither receive overload accepts a buffer-capacity argument. RX DLC values greater than eight are normalized to eight before the controller data is read or copied to the caller; DLC values zero through eight retain their length. After `CAN_OK`, only the returned `len` bytes belong to the current message; `CAN_NOMSG` leaves outputs unchanged. Application decoders must still check their own minimum payload length before accessing fields.

`readMsgBuf(&id, &len, data)` preserves the frame type in `id`:

- `id & 0x80000000UL`: extended identifier.
- `id & 0x40000000UL`: remote request (RTR).
- Mask with `0x1FFFFFFFUL` for the identifier after examining the flags. Standard IDs use eleven bits.

`readMsgBuf(&id, &ext, &len, data)` returns an unaltered identifier and a separate extended flag. **This overload does not expose RTR status**, as in the upstream API. Use the packed-ID overload when the application must distinguish data frames from remote requests. RTR frames request a data length; they do not carry that many valid application data bytes.

`checkReceive()` reports message availability. `getCanId()` is obsolete; use the identifier returned by `readMsgBuf()`.

## Sending messages and handling errors

Both `sendMsgBuf()` overloads accept lengths from zero through eight. The data pointer may be null only when the length is zero. Invalid input returns `CAN_FAILTX` before SPI access. The library copies exactly the specified number of bytes and zeros the unused internal bytes.

`sendMsgBuf(id, len, data)` uses the same `0x80000000UL` and `0x40000000UL` flags to select extended IDs and RTR. `sendMsgBuf(id, ext, len, data)` selects the identifier type through `ext` and sends a data frame.

| Result | Meaning and caller action |
| --- | --- |
| `CAN_OK` | The selected buffer's TXREQ cleared, a fresh TXnIF confirms transmission, and ABTF is clear. This does not acknowledge execution by the destination application. |
| `CAN_FAILTX` | Invalid input, or the attempt ended without confirmed successful transmission, including an aborted/failed one-shot attempt. Check arguments and controller state before deciding to retry. |
| `CAN_GETTXBFTIMEOUT` | No free TX buffer was obtained within the deadline; this call's message was not queued. Existing TX requests were drained and ABAT was cleared before returning. |
| `CAN_SENDMSGTIMEOUT` | The queued message did not finish within the deadline. All pending requests were drained and ABAT was cleared before returning; the message may have finished during recovery. |
| `CAN_CTRLERROR` | ABAT remains set or timeout recovery failed. Stop queuing commands and explicitly recover with `abortTX()` and/or controller reinitialization. |

A prior arbitration/error flag does not override a subsequent fresh TXnIF from a successful automatic retry. Conversely, clearing TXREQ alone is insufficient to report success.

`abortTX()` requests cancellation of **all three** transmit buffers, verifies that every TXREQ has cleared, then clears and verifies ABAT. It returns `CAN_OK` only after those checks. On a deadline or failed register check it returns `CAN_FAIL`; ABAT is retained/reasserted where register writes work. An already transmitting frame may finish during the abort. Retry only when duplicate execution is acceptable or handled by the application.

### Timeouts

These macros can be overridden through compiler definitions applied to the library:

| Macro | Default | Purpose |
| --- | --- | --- |
| `TIMEOUTVALUE` | 2500 microseconds | Wait for a free transmit buffer. |
| `TX_TIMEOUTVALUE` | 5000 microseconds | Wait for transmission completion. |
| `ABORT_TIMEOUTVALUE` | 5000 microseconds | Wait for pending requests to drain. |

A full frame at 50 kbit/s can exceed the old 2.5 ms transmission deadline with bit stuffing. Slower bitrates or prolonged arbitration can require longer bounds. If a buffer becomes free just before its deadline, the subsequent transmission and abort can each consume their own deadline: up to approximately 2.5 + 5 + 5 ms with the defaults, plus SPI/polling overhead. These calls remain synchronous and may delay receive processing. Serialize calls on an instance; its shared message state is not safe for concurrent use from interrupts and the main loop.

## Modes, filters and controller diagnostics

`setMode()` accepts `MCP_NORMAL`, `MCP_SLEEP`, `MCP_LOOPBACK`, `MCP_LISTENONLY` and `MODE_CONFIG`. It rejects other values before register access. The cached mode used after filter/mask configuration changes only when the mode transition succeeds. Mode requests use 32-bit elapsed-time arithmetic with a 200 ms deadline per request. Waking from sleep may involve an intermediate listen-only request before the final mode request. Cleanup restores the prior wake-interrupt enable setting and clears the software-requested wake flag on failure as well as success.

Use `setSleepWakeup(1)` to enable wake-up from bus activity and `setSleepWakeup(0)` to disable it. One-shot transmission is controlled by `enOneShotTX()` and `disOneShotTX()`.

Mask indices are zero through one; filter indices are zero through five. Invalid indices fail before a mode change. The existing standard-ID mask/filter representation places the SID in bits 26–16. Use a 32-bit expression on AVR, for example:

```cpp
CAN0.init_Mask(0, 0, 0x7FFUL << 16);
CAN0.init_Filt(0, 0, 0x130UL << 16);
```

Check the returned status for both calls. The `UL` suffix prevents an invalid 16-bit shift of an AVR `int`. This representation differs from the unshifted IDs accepted by `sendMsgBuf()`.

`checkError()` checks the existing `MCP_EFLG_ERRORMASK` (`0xF8`): receive overflow, bus-off and error-passive conditions. Warning-only bits are outside that mask; use `getError()` for the complete EFLG value and `errorCountRX()`/`errorCountTX()` for counters. `resetOverflowErrors()` clears only RX0OVR/RX1OVR, preserving other error flags; it cannot restore lost frames or clear bus-off.

## Validation and provenance

The regression suite compiles the actual library sources against an SPI/register model, with AddressSanitizer and UndefinedBehaviorSanitizer. It checks bounds, configuration, modes, completion and recovery.

Controller behavior and timing constraints are based on Microchip's [MCP2515 data sheet](https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/DataSheets/MCP2515-Family-Data-Sheet-DS20001801K.pdf), especially sections 3.3–3.6 and 5.0.

The original library credits Seeed Technology, Loovee and Cory J. Fowler. Existing copyright notices and LGPL-2.1-or-later license terms remain in the source files. Report issues with this maintained copy in the repository that distributes it.
