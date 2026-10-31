# T5 CAN Interpreter

Trionic 5 CAN bus interface using the [ASL CAN X2 Dev board](https://www.autosportlabs.com/product/esp32-can-x2-dual-can-bus-automotive-grade-development-board/).

## Purpose

Read live data from a Trionic 5 ECU over its native CAN protocol (non‑standard bitrate ~615 kbit/s), decode SRAM symbol responses, and publish a small JSON snapshot over **ESP-NOW** for other ESP32 clients (e.g. displays).

The ASL board exposes two CAN channels: ESP32 TWAI (**CAN1**) and an **MCP2515** (**CAN2**). This sketch uses the MCP2515 path so the bitrate can be set to match the T5 bus.

## Architecture

```mermaid
flowchart LR
  ECU[Trionic 5 ECU]
  MCP[MCP2515 CAN2]
  ESP[ESP32]
  Peer[ESP-NOW peer]

  ECU <-->|~615 kbit/s CAN| MCP
  MCP <-->|SPI| ESP
  ESP -->|WiFi STA + ESP-NOW| Peer
```

1. **Configure** the MCP2515 for T5‑like bit timing (register writes after `CAN.begin()`).
2. **Poll** each configured symbol on a timer: send an 8‑byte request frame, wait briefly, read a reply if present.
3. **Parse** ID `0x0C` responses, scale raw values, and collect them in a `StaticJsonDocument`.
4. **Send** one ESP-NOW packet per loop iteration **only if** at least one symbol was updated (`anyData`).

## Firmware layout

| Item | Role |
|------|------|
| `configureT5Baud()` | Reset MCP2515, `CAN.begin()`, enter config mode, set `CNF1/2/3`, return to normal mode |
| `txMsg()` | Build T5 SRAM read request payload (`0xC7` + address bytes) |
| `getValueFromCAN()` | Extract big‑endian value from response bytes (protocol‑specific layout) |
| `symbolsToPoll[]` | Labels, SRAM addresses, lengths, scalers, offsets, poll intervals |
| `receiverMAC` | Destination ESP32 MAC for ESP-NOW (must match your display/receiver) |

## Hardware setup

### Board and bus

- Use **CAN2 / MCP2515** to the Trionic 5 bus (bitrate handling is implemented there). Rev C vs Rev D differs only in connector layout; confirm against [ASL documentation](http://www.autosportlabs.net/ESP32-CAN-X2).

![Rev C ASL ESP32 CAN X2 (example)](https://github.com/user-attachments/assets/2cbf2ffe-808c-4d04-8eba-c1c3a0783b68)
- **Termination**: ensure 120 Ω at the bus ends per your network layout (ASL boards often have selectable termination).

### ECU CAN wiring

You can tap a wired OBD2 port with an OBD2 plug to the board’s CAN2 interface if the vehicle wiring follows the usual pinout:

- **CANH** → OBD2 pin 6  
- **CANL** → OBD2 pin 14  

Trionic 5 references (from your notes): CANH ECU **#62**, CANL **#63**.

Bench power (your notes): **12 V+** ECU pins **1, 48, 60**; **ground** **24, 25**. Use an appropriate automotive supply and fusing.

### SPI chip select

`T5_COM_ESP32.ino` defines `CAN_CS_PIN` as **10**. Confirm this against the schematic for your ASL board revision; if CAN does not initialize, wrong CS is a common cause.

### MCP2515 crystal vs bitrate

After `CAN.begin()`, the sketch writes `CNF1/2/3` with comments indicating **~615.4 kbit/s at an 8 MHz** MCP2515 crystal. The `CAN.begin(..., MCP_16MHZ)` third argument must match the **actual** crystal on your board. If the crystal frequency and the `MCP_*` constant disagree, the library’s expectations and your manual CNF tuning can be inconsistent—verify against the board schematic and adjust `CAN.begin()` **and** CNF values together if you change hardware.

## Software setup

### Arduino IDE

1. **Board**: ESP32 by Espressif (board package version should match what you use for other ESP32-S3 projects if applicable).
2. **Libraries** (Library Manager or vendor ZIPs):
   - **mcp_can** — Cory Fowler (`mcp_canbus` / MCP_CAN family)
   - **ArduinoJson** — Benoît Blanchon

### Configuration before flashing

1. **`receiverMAC`** — Set to the **peer ESP32’s Wi‑Fi STA MAC** (the device that receives ESP-NOW). The commented examples in the sketch are placeholders from your environment.
2. **`symbolsToPoll[]`** — Adjust SRAM addresses, `byteLength`, `scaler`, `adder`, and `pollInterval` for your ECU image and how fast you need each channel.
3. **Serial** — `115200` baud for debug prints.

### Optional: send callback

`esp_now_register_send_cb(onSent)` is commented out. Uncomment if you want per‑send success/fail logging.

## Runtime behavior and limits

- **Polling**: Each symbol is requested when `millis() - lastPolled >= pollInterval`. After a request, the code `delay(10)` before checking RX—tune if you miss replies under load.
- **JSON size**: `StaticJsonDocument<250>` and `jsonData[250]` align with the usual ESP-NOW payload cap (~250 bytes). If you add many symbols or long keys, confirm `serializeJson` output length (Serial print helps).
- **ESP-NOW init failure**: If `esp_now_init()` fails, `setup()` returns early but **`loop()` still runs**; `esp_now_send()` may then be invalid. Reflash after fixing Wi‑Fi/ESP‑NOW issues or gate sends on a “ready” flag.
- **LED**: Built-in LED is driven **low** when a JSON packet is sent (active-low pattern on many ESP32 boards).

## Troubleshooting

| Symptom | Things to check |
|--------|------------------|
| `MCP_CAN Initialization Failed` | CS pin, SPI wiring, power, MCP2515 clock vs `CAN.begin()` oscillator argument |
| No data / wrong values | Bitrate (CNF vs crystal), ECU online, CANH/CANL, termination, symbol map vs your binary |
| ESP-NOW receiver sees nothing | `receiverMAC`, peer on same channel, Wi‑Fi STA mode, receiver registered as ESP-NOW peer |
| Garbled or truncated JSON | Payload over ~250 bytes; reduce fields or shorten keys |

## Code review notes (maintenance)

- **Bitrate path**: Manual CNF values and `CAN.begin(..., MCP_16MHZ)` should be reconciled with the real MCP2515 crystal (see above).
- **Comments**: Request path uses ID `0x05` and checks RX ID `0x0C`; keep in sync with any ECU/protocol docs you rely on.
- **`String` in `SymbolMap`**: Fine for a fixed table; avoid growing these dynamically on ESP32 if you extend the code.

## License

See `LICENSE.txt`.
