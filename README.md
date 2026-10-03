[**English**](README.md) | [简体中文](README.zh-CN.md)

# AIR780EG 4G LTE-GNSS × HiSilicon Hi3863 (WS63) Integration Example

Full driver and example code that connects a Luat **Air780EG** (4G Cat.1 + GNSS) module to a HiSilicon **Hi3863 / WS63** SoC (OpenHarmony LiteOS-M) over UART + AT commands, providing:

**4G connectivity + GNSS positioning + MQTT telemetry upload + base-station time (NITZ) sync**

> Target board: `isoh/nl63pro`; SoC: HiSilicon WS63 (Hi3863); OS: OpenHarmony LiteOS-M.

---

## Table of Contents

- [Features](#features)
- [Hardware Connection](#hardware-connection)
- [Directory Structure](#directory-structure)
- [Architecture and Data Flow](#architecture-and-data-flow)
- [Getting Started](#getting-started)
- [Configuration](#configuration)
- [MQTT Topics and JSON Format](#mqtt-topics-and-json-format)
- [AT Command Reference](#at-command-reference)
- [Key Implementation Notes and Pitfalls](#key-implementation-notes-and-pitfalls)
- [API Reference](#api-reference)
- [Caveats](#caveats)
- [References](#references)

---

## Features

- **Interrupt-driven UART**: RX callback + 2048-byte ring buffer, non-blocking reads from the task, completely avoiding the interrupt-disabling spin of `uapi_uart_read` on WS63 (see [Pitfalls](#key-implementation-notes-and-pitfalls)).
- **Network registration and modem init**: echo off, GNSS on, wait for SIM/network attach, enable AGNSS aiding (EPO ephemeris requires network access).
- **GNSS parsing**: parses fix / latitude / longitude / altitude from `AT+CGNSINF` without relying on `%f` or `sscanf`.
- **MQTT over AT**: `MCONFIG → MIPSTART → MCONNECT` to establish the link, `MSUB` to subscribe to cloud commands, `MPUB` to publish telemetry; a failed publish automatically marks the connection as down for reconnection.
- **Telemetry data center**: any sensor module (GPS / IMU / NTC / battery…) writes through setters and gets unified JSON output.
- **Multi-channel publish registry**: 5 channel slots (0 = main, 1–4 reserved) to attach new topics and payload callbacks at any time.
- **Edge-computed sampling**: in average mode (AVG), automatically computes mean/peak kinetic energy and the active-frame ratio (see [JSON format](#mqtt-topics-and-json-format)).
- **4G base-station time (NITZ)**: `AT+CTZU=1` + `AT+CCLK?` fetches network time and injects it into the main board via `Set_Global_Time()`; zero overhead after the first success.
- **Link recovery**: UART-level self-check/rebuild example (`air780_test.c`, retired, troubleshooting reference only).

---

## Hardware Connection

| Air780EG Pin | WS63 / NL63Pro Pin | Direction | Notes |
| --- | --- | --- | --- |
| TXD | GPIO7 | Module → MCU | UART2 RX, `AIR780_RX_PIN` |
| RXD | GPIO8 | MCU → Module | UART2 TX, `AIR780_TX_PIN` |
| GND | GND | — | Common ground required |
| VBAT | Power supply | — | Design per Luat hardware manual; ensure current headroom and decoupling |
| RTS / CTS | Not connected | — | No hardware flow control |

- Serial parameters: **115200 8N1**, no hardware flow control.
- The Air780EG UART uses **1.8 V logic** while the WS63 uses 3.3 V; **level shifting is required** before connecting (already handled by this project's hardware).
- Connect the **4G antenna** and **GNSS antenna**, and insert a working SIM card (confirm activation for eSIM variants).
- This project only handles serial communication; it **does not control module power-up/down** (PWRKEY and power sequencing are handled by hardware). Make sure the module is powered on and able to respond to AT commands.

---

## Directory Structure

```text
mk1/
├── BUILD.gn               # GN build script: compiles into static library air780_app
├── air780_app.c           # Application main flow: init, main loop, MQTT config, NITZ time sync, command parsing
├── air780_uart.c/.h       # UART2 interrupt RX + ring buffer + AT TX/RX / URC waiting
├── air780_modem.c/.h      # Network attach, CSQ, GNSS parsing, AGNSS aiding
├── air780_mqtt.c/.h       # MQTT over AT: connect / disconnect / publish / state tracking
├── air780_telemetry.c/.h  # Telemetry data center, fixed-point JSON builder, 5-channel publish registry
└── air780_test.c          # Standalone diagnostic task (retired, not built, troubleshooting reference only)
```

---

## Architecture and Data Flow

```text
                    UART2 @ 115200 8N1
 ┌─────────────┐  ◄────── AT commands ────┐
 │  Air780EG   │                          │
 │  4G Cat.1   │  ───── responses / URC ─►│
 │  + GNSS     │                          │
 └─────────────┘                          ▼
                         ┌────────────────────────────────────┐
                         │       WS63 / Hi3863 (LiteOS-M)     │
                         │            Air780Task              │
                         │                                    │
                         │  air780_uart       IRQ → ring buf  │
                         │  air780_modem      attach/CSQ/GNSS │
                         │  air780_telemetry  data center/JSON│
                         │  air780_mqtt       MQTT over AT    │
                         └───────────────┬────────────────────┘
                                         │ 4G (module built-in TCP)
                                         ▼
                                   MQTT Broker
```

Main flow (`Air780Task`):

1. `Air780UartInit()`: initialize UART2 and register the RX callback; retries on failure.
2. `Air780ModemPrepare()`: `ATE0` → `AT+CGNSPWR=1` to power GNSS → wait for `CGATT` attach (~60 s timeout) → `AT+CGNSAID=31,1,1,1` to enable aiding.
3. `Air780MqttSetup()` + `Air780SetupChannels()`: store broker config, register the main and reserved channels.
4. Main loop:
   - `Air780RefreshSensors()` refreshes CSQ and GPS into the data center;
   - `Air780SyncTime()` syncs the main board clock from NITZ after the first successful connection;
   - `Air780EnsureMqtt()` ensures the connection (on reconnect it `MSUB`s `alpha/1/cmd` and publishes the online notice);
   - `Air780ChannelPublishAll()` publishes JSON on every enabled channel;
   - sleeps for the current upload interval, briefly polling the UART to handle cloud `+MSUB` commands and wheel/key events.

---

## Getting Started

### 1. Place into the SDK

Place this directory into your OpenHarmony SDK application source tree (the example project uses `//app/Air780/`; adjust to your SDK layout).

### 2. Register the build component

Add to the `features` list in your application-level `BUILD.gn`:

```gn
features = [
    # ... other components
    "Air780:air780_app",
]
```

The `BUILD.gn` in this directory builds the code into a static library named `air780_app` and declares the include paths and defines required by the WS63 SDK (`CHIP_WS63`, `CONFIG_UART_SUPPORT_TX/RX`, etc.). **Include paths differ between SDK versions — adjust as needed.**

### 3. Edit the configuration

Adjust `MQTT_HOST` / `MQTT_PORT` / `MQTT_CLIENTID` / topic macros at the top of `air780_app.c`, and the UART pins / baud rate in `air780_uart.h`.

### 4. Build and flash

Run the standard OpenHarmony build flow from the SDK root:

```bash
hb set          # select the target product (NL63Pro / WS63)
hb build        # incremental; use hb build -f for a full build
```

Flash the generated firmware (e.g. `ws63-liteos-app_all.fwpkg`) to the board and watch the serial log:

- `[AIR780]`: UART / AT traffic
- `[MODEM]`: network attach, GNSS aiding
- `[MQTT]`: connection, publish
- `[APP]`: main flow and command parsing

After a normal boot you should see logs such as `[MODEM] network attached` and `[MQTT] connected to ...`.

---

## Configuration

### UART Configuration (`air780_uart.h`)

| Macro | Default | Description |
| --- | --- | --- |
| `AIR780_UART_BUS` | `2` | UART controller index |
| `AIR780_RX_PIN` | `7` | GPIO7 ← module TXD |
| `AIR780_TX_PIN` | `8` | GPIO8 → module RXD |
| `AIR780_BAUDRATE` | `115200` | Baud rate |
| `AIR780_RX_BUFFER_SIZE` | `1024` | Driver RX buffer size |
| `AIR780_AT_BUFFER_SIZE` | `512` | AT buffer size |
| `AIR780_READ_TIMEOUT` | `100` | Read timeout (ticks, 1 tick = 10 ms) |
| `AIR780_AT_TIMEOUT` | `500` | AT response timeout (ticks) |

> Note: the `timeout_ms` argument of `Air780UartRead()` is actually handled as an **idle-tick count** (each idle tick is one `osDelay(1)` ≈ 10 ms), not strict milliseconds.

### Application Configuration (`air780_app.c`)

| Macro / Variable | Default | Description |
| --- | --- | --- |
| `MQTT_HOST` | `mqtt.example.com` | Placeholder; replace with your own broker address |
| `MQTT_PORT` | `1883` | Broker port |
| `MQTT_CLIENTID` | `MK-000100` (current default) | Device client ID; must be unique per broker. See below for how to change it |
| `MQTT_USER` / `MQTT_PASS` | `NULL` | Anonymous login (mosquitto `allow_anonymous`) |
| `MQTT_KEEPALIVE` | `120` | Keep-alive in seconds |
| `g_intervals[]` | `3/10/20/30/60/300` s | Upload intervals selectable with the wheel |
| `g_upload_ms` | `30000` | Current upload interval (default 30 s, `g_interval_idx = 3`) |
| `g_upload_mode` | `0` | Sampling mode: `0` = latest, `1` = average |

#### Changing the Client ID (`MQTT_CLIENTID`)

The client ID **defaults to `MK-000100`** as in the current code. To change it, edit the macro at the top of `air780_app.c` (around line 16) and replace the string with your own device ID:

```c
#define MQTT_CLIENTID      "MK-000100"   /* must be unique under the same broker */
```

- **No other code changes are needed**: this macro is used for the MQTT connection, the online notice and status reports, and is written into the telemetry JSON `device_id` field via `Air780TeleSetDeviceId(MQTT_CLIENTID)`.
- Every device under the same broker **must use a different client ID** (e.g. increment per device: `MK-000101`, `MK-000102`); otherwise a newly connected device will kick the previous one offline.
- The client ID is limited to **63 characters** (the buffer in `air780_mqtt.c` is 64 bytes).
- The ID is fixed at compile time; flashing multiple devices requires editing and rebuilding per device. If you need runtime switching, extend the code to read it from flash.

The main-board UI can change runtime parameters through the following functions (provided by `air780_app.c`):

- `Air780ChangeInterval(int direction)`: switch the upload interval with the wheel;
- `Air780ChangeMode(void)`: toggle the sampling mode with the K2 key.

### MQTT Topics (`air780_app.c`)

| Topic | Direction | Payload | Default |
| --- | --- | --- | --- |
| `omega/1` | Publish | Main-channel full telemetry JSON | Enabled |
| `omega/1/online` | Publish | Online notice `{"device_id":"..."}` | Sent on connect |
| `omega/1/status` | Publish | Wheel / mode change status | Sent on change |
| `alpha/1/cmd` | Subscribe | Cloud command `{"mode":x,"interval":x}` | `MSUB` |
| `omega/1/aux1` ~ `aux4` | Publish | Reserved channels for new data | Disabled |

---

## MQTT Topics and JSON Format

### Main Channel JSON (`omega/1`)

Generated by `Air780BuildMainJson()`, single-line, no newlines, built entirely with integer fixed-point math (no `%f`):

```json
{"device_id":"MK-000100","mode":0,"gps":{"lng":114.070000,"lat":22.528000,"alt":20.5},"imu":{"ax":10.50,"ay":2.10,"az":9.800,"gx":0.05,"gy":0.12,"gz":-0.03},"base":{"battery":59,"ntctemp":39.2,"temp":39.2,"humidity":45.0}}
```

- `mode = 0` (LATEST): `gps` / `imu` / `base` are the latest samples.
- `mode = 1` (AVG, averaged sampling): temperature/humidity, NTC and IMU are averaged over the period; the `imu` fields are reused for edge-computing statistics:

| Field | Meaning in AVG mode (mode=1) |
| --- | --- |
| `imu.ax` | Mean net kinetic energy (average after removing the 9.80665 g gravity) |
| `imu.ay` | Peak net kinetic energy |
| `imu.az` | Active-frame ratio (0–1) |
| `imu.gx` | Mean gyroscope jitter |
| `imu.gy` | Peak gyroscope jitter |
| `imu.gz` | Always 0 |

JSON values use 6 decimal places for latitude/longitude, 1 for altitude, 2 for acceleration and 3 for `az`, matching the structure of `json.txt`.

### Status JSON (reserved)

Generated by `Air780BuildStatusJson()`, not attached to any channel by default (can be wired to an `aux` channel when needed):

```json
{"device_id":"MK-000100","csq":20,"fix":1}
```

### Cloud Command (`alpha/1/cmd`)

```json
{"mode":1,"interval":30}
```

- `mode`: `0` = latest, `1` = average;
- `interval`: only `3 / 10 / 20 / 30 / 60 / 300` (seconds) are accepted; other values are ignored.

It takes effect immediately and a status report is published to `omega/1/status`:

```json
{"device_id":"MK-000100","current_interval":30000,"mode":1}
```

---

## AT Command Reference

| Command | Purpose |
| --- | --- |
| `AT` | Wake-up / alive check |
| `ATE0` | Disable echo |
| `AT+CGATT?` | Query PDP network attach state |
| `AT+CSQ` | Query signal quality (0–31, 99 = unknown) |
| `AT+CGNSPWR=1` | Power on GNSS |
| `AT+CGNSAID=31,1,1,1` | Enable AGNSS aiding (mode/time/epo/loc all on; requires network) |
| `AT+CGNSINF` | Read GNSS fix information |
| `AT+CTZU=1` | Enable automatic network time-zone/time update (NITZ) |
| `AT+CCLK?` | Read the module's local time |
| `AT+MCONFIG` | Configure MQTT client parameters |
| `AT+MIPSTART` | Open the TCP connection (wait for URC `CONNECT OK`) |
| `AT+MCONNECT=1,<keepalive>` | Start the MQTT connection (wait for URC `CONNACK OK`) |
| `AT+MSUB=<topic>,0` | Subscribe to a topic (QoS0) |
| `AT+MPUB=<topic>,0,0,"<payload>"` | Publish a message (QoS0, retain0) |
| `AT+MDISCONNECT` | Disconnect MQTT |

`+CGNSINF` field indices (parsed positions): `1 = fix`, `3 = lat`, `4 = lng`, `5 = alt`.

---

## Key Implementation Notes and Pitfalls

1. **Never poll AT responses with `uapi_uart_read` (the biggest pitfall on the Hi3863)**
   On WS63 this API: ① ignores the `timeout` argument; ② calls `uart_porting_lock()`, which disables interrupts; ③ only returns after the requested length is filled, spinning without yielding the CPU while the FIFO is empty.
   When used to poll variable-length AT responses, it spins at 100% CPU with interrupts disabled before the module even replies, starves the watchdog and ends in an NMI reset (seen as `Air780Task` at 100% CPU followed by a crash).
   The correct approach: register an interrupt RX callback with `uapi_uart_register_rx_callback()` (`UART_RX_CONDITION_FULL_OR_SUFFICIENT_DATA_OR_IDLE`, threshold 1 byte) and push bytes into a ring buffer; the task only reads from the buffer and yields with `osDelay(1)` every round — never call `uapi_uart_read`.

2. **Two ways to read AT responses**
   - `Air780UartRead()`: for command responses; returns early on `OK\r\n` / `ERROR\r\n`;
   - `Air780UartWaitToken()`: waits for asynchronous URCs such as `CONNECT OK` / `CONNACK OK`; when the buffer is about to overflow it keeps the last `strlen(token)-1` bytes so the keyword can still be matched across batches.

3. **Air780EG cold start struggles to get a fix**
   On a pure cold start few satellites are visible and fix stays 0 for a long time; the manual requires `AT+CGNSAID=<mode>,<time>,<epo>,<loc>` after enabling GNSS. The code enables all three aids (AGNSS/EPO/location) after network attach; the EPO ephemeris is downloaded over the network, and a fix usually arrives within seconds.

4. **Minimal libc does not support floating-point formatting**
   The platform does not guarantee `printf("%f")` / `sscanf` / `atof`; `air780_modem.c` implements integer parsing by hand and `air780_telemetry.c` builds JSON with fixed-point math, avoiding any `%f` dependency.
   (`air780_app.c` uses `sscanf` for NITZ to parse `+CCLK`, but only integer time fields — still no `%f`.)

5. **`AT+MPUB` payload escaping**
   Escaped automatically before publish: `"` → `\22`, `\r` → `\0D`, `\n` → `\0A`, `\` → `\5C`.

6. **MQTT connection state tracking**
   Any failed publish marks the internal state as disconnected; the main loop reconnects on the next cycle, re-subscribes to `alpha/1/cmd` and re-sends the online notice.

7. **NITZ base-station time sync**
   After network attach, `AT+CTZU=1` enables automatic time updates, then `AT+CCLK?` reads the time; only years between `24` and `99` are treated as valid network time. The time is injected into the expansion board through the main-board-provided `Set_Global_Time()` (cross-static-library link), once, after which the code stops trying.

8. **Single-task serialized AT access**
   All AT traffic runs serially inside `Air780Task`; only one task touches the UART at any time, so no locking is needed.

---

## API Reference

### `air780_uart.h`

| API | Description |
| --- | --- |
| `int Air780UartInit(void)` | Initialize UART2 and register the interrupt RX callback; 0 on success / -1 on failure |
| `void Air780UartDeinit(void)` | Unregister the callback and release the UART |
| `int Air780SendAT(cmd, resp, size, timeout)` | Send an AT command and read the response (clears stale data first) |
| `int Air780UartRead(buf, maxlen, timeout)` | Read a response from the ring buffer; returns early on `OK` / `ERROR` |
| `int Air780UartWaitToken(token, scratch, cap, timeout)` | Wait for a specific URC keyword; 1 = matched / 0 = timeout |
| `void Air780Wakeup(void)` | Send `AT` repeatedly to wake the module, then disable echo |

### `air780_modem.h`

| API | Description |
| --- | --- |
| `int Air780ModemPrepare(void)` | Echo off + GNSS on + wait for attach + enable AGNSS; 0 on success / -1 on timeout |
| `int Air780ModemCsq(void)` | Read signal quality (0–31, 99 unknown, -1 failure) |
| `int Air780ModemGps(air780_gps_t *g)` | Parse `+CGNSINF` into `g`; 1 = parsed / 0 = failed (check `g->fix` for a valid fix) |
| `int Air780ModemAttached(void)` | Query network attach state; 1 = attached / 0 = not attached |

### `air780_mqtt.h`

| API | Description |
| --- | --- |
| `void Air780MqttSetup(host, port, clientid, user, pass, keepalive)` | Store connection config (no connection is opened); pass `NULL` for `user`/`pass` to use anonymous login |
| `int Air780MqttConnect(void)` | Full link setup: `MCONFIG → MIPSTART → MCONNECT`; 0 on success / -1 on failure |
| `void Air780MqttDisconnect(void)` | Disconnect MQTT |
| `int Air780MqttConnected(void)` | Query the internal connection state |
| `int Air780MqttPublish(topic, payload)` | Publish a message (QoS0 / retain0); failure automatically marks the link as down |

### `air780_telemetry.h`

| API | Description |
| --- | --- |
| `Air780TeleSetGps / SetCsq / SetImu / SetBattery / SetNtcTemp / SetTemp / SetHumidity` | Write into the telemetry data center (`SetImu` also performs edge-computing accumulation) |
| `Air780TeleSetDeviceId / SetUploadInterval / GetUploadInterval` | Set the device ID and upload interval |
| `Air780BuildMainJson(out, cap)` | Build the main-channel telemetry JSON; returns the written length |
| `Air780BuildStatusJson(out, cap)` | Build the `{device_id, csq, fix}` status JSON |
| `Air780ChannelConfig(idx, topic, fn)` / `Air780ChannelEnable(idx, enable)` | Configure / enable channels (`AIR780_CH_MAIN` ~ `AIR780_CH_AUX4`) |
| `Air780ChannelPublishAll(void)` | Publish all enabled channels; returns the number published, or -1 on failure |

---

## Caveats

- **`air780_test.c` is retired**: it is an early standalone diagnostic task (periodic alive check + UART rebuild) that also registers a task via `APP_FEATURE_INIT`; building it together with `air780_app.c` causes a conflict. It is not included in `BUILD.gn` and is kept only as a reference for UART self-check/recovery.
- **Before publishing to a public repository**: check that `MQTT_HOST`, `MQTT_CLIENTID`, etc. in `air780_app.c` are safe to publish.
- **Time base**: on this board 1 tick = 10 ms; the `AIR780_DELAY_MS(ms)` macro converts milliseconds to ticks.
- **Dependencies**: the code depends on the WS63 SDK `uapi_uart_*` APIs and a specific directory layout (see `include_dirs` in `BUILD.gn`); paths may need adjustment for different SDK versions.
- **MQTT password**: anonymous by default; enable authentication on the broker for production. Setting a non-empty `MQTT_USER` / `MQTT_PASS` enables username/password login.

---

## References

- Luat Air780EG AT command and hardware design manuals: <https://doc.openluat.com/>
- OpenHarmony: <https://www.openharmony.cn/>
- OpenHarmony source (Gitee): <https://gitee.com/openharmony>

---

## License

This repository does not include a `LICENSE` file. If you plan to publish it as open source, consider adding one (e.g. MIT / Apache-2.0); also note the copyright of Luat's AT command documentation and related materials.
