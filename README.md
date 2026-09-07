# SIYI A8 Mini ESP32 Driver

A from-scratch ESP32/Arduino (PlatformIO- and Arduino-IDE-compatible) C++17
driver for the SIYI A8 Mini gimbal camera, implementing the SIYI External
SDK UART protocol as documented in
`SIYI_Gimbal_Camera_External_SDK_Protocol_Update_Log_V0_1_1`.

## Folder structure

```
src/
    CRC16.h / CRC16.cpp          CRC16 (poly 0x1021, init 0), table transcribed from spec Ch.4
    Protocol.h                   Frame constants, CommandId enum, packed payload structs
    PacketParser.h                Reusable byte-at-a-time state machine parser
    SIYI.h / SIYI.cpp            Main driver class (SIYIDriver<N>, alias "SIYI")
examples/
    BasicUsage/BasicUsage.ino    Example sketch
```

## Scope

The protocol document also covers commands specific to other SIYI cameras
(ZR10 / ZR30 / ZT30 / quad-spectrum): thermal imaging, laser rangefinder,
video stitching, GPS/RC telemetry injection, etc. Those are **not**
implemented here — they're outside the A8 Mini's feature set and outside
the API surface that was asked for. Everything implemented is listed in
`Protocol.h`'s `CommandId` enum:

| CMD_ID | Command | ACK? |
|---|---|---|
| 0x00 | Heartbeat | No |
| 0x01 | Firmware Version | Yes |
| 0x02 | Hardware ID | Yes |
| 0x04 | Auto Focus | Yes |
| 0x05 | Manual Zoom w/ Autofocus | Yes |
| 0x06 | Manual Focus | Yes |
| 0x07 | Gimbal Rotation | Yes |
| 0x08 | One-Key Centering | Yes |
| 0x0A | Camera System Info ("gimbal status") | Yes |
| 0x0B | Function Feedback | Unsolicited push (no send) |
| 0x0C | Capture Photo / Record / Motion Mode / Video Output | **No** |
| 0x0D | Request Gimbal Attitude | Yes |
| 0x0E | Set Gimbal Attitude (absolute angle) | Yes |
| 0x0F | Absolute Zoom | Yes |

## Protocol interpretation notes (read before relying on this in the field)

The instructions were explicit that ambiguous fields should be explained
rather than guessed silently. Two things in this document are ambiguous:

**1. The CTRL byte.** The spec text says `CTRL`: `0 = need_ack`,
`1 = ack_pack` (a value of 1 marks the frame itself as an ACK reply). But
the only fully worked byte-level example in the whole 110-page document —
the heartbeat frame `55 66 01 01 00 00 00 00 00 59 8B` — is a
host-to-device command frame, and it uses `CTRL = 0x01`. That directly
contradicts the stated meaning (a command frame isn't an ACK). There's no
second worked example to resolve which is right. This driver follows the
one byte-accurate example it has: it always transmits `CTRL = 0x01`
(`kCtrlByte` in `Protocol.h`) and does **not** use the CTRL bit for
request/response correlation — that's done via `CMD_ID` + `SEQ` instead,
so this ambiguity doesn't affect correctness, only byte-for-byte fidelity
of an under-specified field.

**2. "Stop recording."** Command 0x0C only defines `FuncType` value `2`
as `"Start recording"`; the document never defines a separate
"stop recording" code. This driver implements `startRecording()`,
`stopRecording()`, and `toggleRecording()` as the same underlying send
(`FuncType::StartRecording`), on the assumption it's a toggle — consistent
with how this code point behaves on other SIYI camera firmwares. **Verify
the resulting state** with `getGimbalStatus().record_sta` rather than
trusting the call name, since this is an assumption, not a documented
fact.

## Design notes / how the "lessons learned" were applied

- **No assumed ACK payload format** — every struct in `Protocol.h` is
  transcribed field-by-field from the doc, with the doc's own field order
  and sizes (e.g. `GimbalAttitudeAck` is 6×`int16_t` in the documented
  order: yaw, pitch, roll, yaw_velocity, pitch_velocity, roll_velocity).
- **Sequence numbers** — `SIYIDriver` keeps an internal 16-bit counter
  (`seqCounter_`), assigns a fresh SEQ to every transmitted frame, and a
  blocking request (`requestAck()`) only completes when a reply with a
  **matching `CMD_ID` and `SEQ`** arrives — not just a matching CMD_ID.
- **No hardcoded packet lengths** — the parser reads `Data_len` from the
  frame itself; TX builds `Data_len` from `sizeof()` of the actual struct
  being sent.
- **CRC never ignored** — `PacketParser::finalizeFrame()` recomputes the
  CRC16 over the received bytes and silently discards the frame if it
  doesn't match. Verified against the doc's own worked example
  (see `CRC16.cpp` header comment).
- **Parser is a real state machine, one byte at a time**
  (`PacketParser::feed()`), so it works correctly regardless of how the
  UART driver chunks bytes — see fragmentation/back-to-back notes below.
- **Fragmented frames** — tested by feeding a frame's bytes to the parser
  one at a time across simulated "reads"; it reassembles correctly.
- **Back-to-back frames** — tested by feeding two full frames end-to-end
  in one stream; both are correctly extracted in order.
- **Corrupted frames discarded safely** — tested with a frame whose CRC
  was deliberately flipped: it's silently dropped and the parser recovers
  in time to correctly parse the next valid frame.
- **Timeouts, never blocks forever** — every request/response driver call
  takes an explicit `timeoutMs` and internally pumps `update()` in a
  bounded loop; it returns `false` on timeout rather than hanging.
- **Configurable buffer sizes, no magic numbers** — `PacketParser<N>` and
  `SIYIDriver<N>` are templated on max payload capacity
  (`RxPayloadCapacity`, default 64 bytes — comfortably larger than every
  implemented payload, the largest being 12 bytes).
- **`constexpr`, `enum class`, const-correctness** used throughout
  `Protocol.h` / `SIYI.h` in place of raw command IDs / magic numbers.
- **Commands with no ACK are fire-and-forget** — CMD 0x0C
  (`takePhoto()`, `startRecording()`, `setMotionMode()`, etc.) never call
  `requestAck()`; they send and return immediately, matching the spec's
  explicit "No ACK response" for that command.

## Known limitation (framing, inherent to any 2-byte-sync protocol)

If truly random noise on the wire happens to contain the exact byte pair
`0x55 0x66`, the parser will (correctly, per the protocol) interpret that
as the start of a new frame. If everything after it fails the CRC check,
the parser discards that bogus frame and resyncs — but any genuine frame
bytes that got "swallowed" as the bogus frame's declared-length payload in
the meantime are lost (there's no strict length field validation, since
you're not able to look at the CRC until you've already had length bytes
in hand). This is a structural property of length-prefixed framing with
only a 2-byte sync word (not unique to this driver) — mitigated in
practice by the CRC check catching and discarding the bad frame quickly,
and by the parser being ready to resync on the very next byte.

## Debug mode

```cpp
camera.setDebug(true, Serial); // logs TX/RX frames, CRC, seq, cmd, timeouts
```

This adds a couple of `if` checks per call in release builds (no
`#ifdef`/`#endif` divergence to maintain) — the cost is negligible and it
can be left compiled in.

## Usage

```cpp
#include <SIYI.h>
using namespace siyi;

HardwareSerial CameraSerial(2);
SIYI camera(CameraSerial);

void setup() {
    CameraSerial.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
    camera.begin();

    FirmwareVersionAck fw{};
    camera.getFirmwareVersion(fw, 500);

    camera.center();
    camera.setMotionMode(GimbalMotionMode::Follow);
}

void loop() {
    camera.update(); // call every iteration

    camera.setAngle(30.0f, -15.0f);
    camera.takePhoto();
}
```

See `examples/BasicUsage/BasicUsage.ino` for a fuller example including
the async `onFunctionFeedback()` callback and gimbal status polling.

## Testing performed

- **CRC16**: unit-verified byte-for-byte against the protocol document's
  own heartbeat example (`55 66 01 01 00 00 00 00 00 59 8B` → CRC
  `0x8B59`), independent of the driver code — see the transcription notes
  in `CRC16.h`.
- **Parser state machine**: exercised with (a) two valid frames
  back-to-back, (b) a frame with a deliberately corrupted CRC between
  them (correctly discarded, parser recovers), (c) a frame fed one byte
  at a time to simulate UART fragmentation, and (d) leading noise bytes
  before a valid frame (correct resync). All four passed.
- **API surface**: the full public API of `SIYI.h` was compiled against
  a mock `HardwareSerial`/`Arduino.h` to catch type/signature errors; it
  builds cleanly. This is a syntax/API-shape check, not a substitute for
  testing against real ESP32 + A8 Mini hardware — please verify timing
  (especially the CTRL-byte assumption above) against your unit before
  flight use.
