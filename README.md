# energy_meter — ESP32-C3 smart energy meter bridge

Arduino sketch that polls an **ORNO OR-WE-526** single-phase RS485
Modbus-RTU multi-tariff energy meter and bridges it to MQTT with Home
Assistant discovery — `espMqttClient`, retained discovery configs, LWT
availability, WiFi RSSI + reset reason + fail counters as diagnostic
sensors, static IP, manufacturer `P@cho`. On top of relaying the meter's own
registers, it computes a day/night, winter/summer tariff split **itself**,
in firmware, from NTP time.

## Files

- `energy_meter.ino` — the sketch.
- `config.h.example` — copy to `config.h` and fill in: WiFi/MQTT/OTA
  credentials, static IP, device identity, and the RS485 GPIO pins. Keep
  `config.h` out of git (already covered by `.gitignore`).

## What this does

- Polls the OR-WE-526 over Modbus RTU for instantaneous electrical values
  (V, A, W, VA, var, Hz, PF), the meter's own cumulative energy registers
  (forward/reverse/combined active energy incl. the meter's T1-T4 slots,
  plus reactive energy totals, resettable counters, and demand values).
- Publishes everything to MQTT with Home Assistant MQTT discovery, same
  pattern as the other ESP32-C3 devices in this fleet.
- Computes a day/night, winter/summer tariff split itself and accumulates
  two persistent counters, `energy_day_kwh` / `energy_night_kwh`:
  - Winter (1 Nov – 31 Mar): night = 22:00–06:00
  - Summer (1 Apr – 31 Oct): night = 23:00–07:00

### Why the tariff split is done in firmware, not on the meter

The OR-WE-526 register map exposes native T1-T4 tariff registers and
programmable time-period/season tables, but the exact bit-packing of those
tables isn't documented in enough detail to program blind, and getting it
wrong risks putting the meter's own tariff logic in a bad state. Since the
meter's *total forward active energy* register (`0x010E`) is always
available and always correct, this firmware reads that single trusted
number every poll and buckets the delta into day/night itself, based on the
wall-clock rules above. The meter's own T1-T4 registers are still read and
published for reference/cross-check — if you never program the meter's own
TOU tables, they'll just mirror the totals (T1) with T2-T4 at 0, which is
normal.

## Wiring

```
ESP32-C3 Zero  --  MAX485
  RS485_TX_PIN  ->  DI
  RS485_RX_PIN  <-  RO
  RS485_DE_RE_PIN -> DE and RE (tied together)
  3V3           ->  VCC
  GND           ->  GND
MAX485 A/B  ->  meter terminals 23(A)/25(B)
```

Terminal 24 ("G" reference) is optional — only wire it if your converter has
a G pin. Meter defaults: Modbus ID 1, 9600 baud, 8N1 (per the OR-WE-526
manual) — adjust `MB_SLAVE_ID`/`MB_BAUD` in `config.h` if you've changed
those on the meter itself.

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems".
2. **Libraries** (Library Manager):
   - `ModbusMaster` (4-20ma/ModbusMaster)
   - `espMqttClient` by bertmelis
   - `Preferences` and `ArduinoOTA` (bundled with the ESP32 core)
3. Select board **"ESP32C3 Zero"** (or whichever ESP32-C3 board you're
   using).
4. Copy `config.h.example` to `config.h` and fill in your WiFi/MQTT/OTA
   values, static IP, device identity, and RS485 pins.

## OTA updates

`ArduinoOTA` is bundled with the ESP32 core, no extra library needed. Once
on WiFi, the device shows up in Arduino IDE's `Tools > Port` as a network
port (`energy-meter1 at 192.168.x.x`). Set `OTA_PASSWORD` in `config.h` to
something real before deploying. Mid-flash, the sketch pauses Modbus
polling and MQTT publishing and marks itself offline over MQTT (LWT) so
Home Assistant doesn't show stale data while updating.

## MQTT / Home Assistant

Base topic: `energy/<DEVICE_ID>/...`. Every sensor (instantaneous
electrical values, per-tariff energy totals for forward/reverse/combined,
reactive energy, demand, day/night tariff accumulators, WiFi RSSI, uptime,
Modbus/MQTT fail counts) plus text diagnostics (current tariff, current
season, reset reason, which Modbus function code is in use, meter
serial/firmware/hardware/address) gets a retained HA discovery config under
`homeassistant/sensor/<DEVICE_ID>/<key>/config` on every MQTT connect. Two
buttons are also exposed: a restart button and a "reset day/night energy"
button (`energy/<DEVICE_ID>/cmd/restart`, `.../cmd/reset_tariff_energy`).

Values publish on a 15s poll cycle (`POLL_INTERVAL_MS`); diagnostics
(WiFi RSSI, uptime, fail counts) every 30s. Tariff accumulators flush to NVS
every 5 minutes (only if dirty) and immediately before an OTA flash or a
restart command, so a power loss mid-cycle costs at most a few minutes of
counting, never the accumulated totals.

## Modbus function-code auto-detection

Some OR-WE-526 firmware variants answer the measurement/energy register zone
with Input Registers (FC04), others with Holding Registers (FC03), despite
the register map marking that zone read-only. The firmware probes both once
on the very first read after boot, locks onto whichever succeeds, and uses
only that one from then on — deliberately avoiding a double-timeout on every
read if the meter isn't responding at all (wrong wiring/baud/ID), since that
could otherwise block `loop()` long enough to trip the ESP32 task watchdog
and reboot the board.

## Config file

Credentials, static IP, device identity, RS485 pins, and the OTA password
all live in `config.h` (gitignored) — copy `config.h.example` to `config.h`
and fill in real values.

## Version History

`FIRMWARE_VER` lives in the gitignored `config.h`.

| Version | Date | Changes |
|---|---|---|
| v1.2.0 | 2026-09-03 | Initial release. |
