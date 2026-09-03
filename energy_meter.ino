/*
  ============================================================================
  ESP32-C3 Smart Energy Meter Bridge
  Meter:      ORNO OR-WE-526 (single-phase, RS485 Modbus-RTU, multi-tariff)
  Converter:  MAX485 (half-duplex, DE/RE tied to one GPIO)
  Board:      ESP32-C3 Zero
  ============================================================================

  WHAT THIS DOES
  --------------
  - Polls the OR-WE-526 over Modbus RTU for instantaneous electrical values
    (V, A, W, VA, var, Hz, PF), the meter's own cumulative energy registers
    (forward/reverse/combined active energy incl. the meter's T1-T4 slots,
    plus reactive energy totals, resettable counters, and demand values).
  - Publishes everything to MQTT with Home Assistant MQTT-discovery, following
    the same pattern as the other ESP32-C3 devices (retained discovery
    configs, LWT availability, WiFi RSSI, reset reason, static IP, fail
    counters as diagnostic sensors, manufacturer "P@cho").
  - Computes a day/night, winter/summer tariff split ITSELF, in firmware,
    from NTP time, and accumulates two persistent counters:
        energy_day_kwh, energy_night_kwh
    Winter (1 Nov - 31 Mar): night = 22:00-06:00
    Summer (1 Apr - 31 Oct): night = 23:00-07:00

  WHY THE TARIFF SPLIT IS DONE IN FIRMWARE, NOT ON THE METER
  ------------------------------------------------------------
  The OR-WE-526 register map (OR-WE-526_rejestry.pdf) DOES expose native
  T1-T4 tariff registers and 8 programmable "time period tables" plus a
  season table, written with Modbus function 0x10 as multi-register blocks.
  However the exact bit-packing of those tables ("hhmmNN*8", "MMDDNN*8") is
  not documented in enough detail to program blind, and getting it wrong
  risks putting the meter's own tariff logic in a bad state. Since the
  meter's *total forward active energy* register (0x010E) is always
  available and always correct, this firmware reads that single trusted
  number every poll and buckets the delta into day/night itself, based on
  the wall-clock rules you gave. This is transparent, fully debuggable over
  MQTT, and trivial to change if your tariff rules ever change.
  The meter's own T1-T4 registers are still read and published (they're
  informative / a cross-check), but if you never program the meter's TOU
  tables they'll just mirror the totals (T1) with T2-T4 at 0 - normal.

  WIRING
  ------
  ESP32-C3 Zero  --  MAX485
    RS485_TX_PIN  ->  DI
    RS485_RX_PIN  <-  RO
    RS485_DE_RE_PIN -> DE and RE (tied together)
    3V3           ->  VCC
    GND           ->  GND
  MAX485 A/B  ->  meter terminals 23(A)/25(B) (terminal 24 is the optional
  "G" reference pin - only wire it if your converter has a G pin).
  Meter defaults: Modbus ID 1, 9600 baud, 8N1 (confirmed in the OR-WE-526
  manual). Adjust MB_SLAVE_ID/MB_BAUD below if you've changed those.

  OTA UPDATES
  -----------
  ArduinoOTA is enabled (bundled with the ESP32 core, no extra library
  needed). Once the device is on WiFi, it shows up in the Arduino IDE under
  Tools > Port as a network port ("energy-meter1 at 192.168.x.x") and you can
  upload new firmware wirelessly. Set OTA_PASSWORD below to something real
  before deploying - the default is a placeholder. Mid-flash, the sketch
  pauses Modbus polling and MQTT publishing and marks itself "offline" over
  MQTT (LWT) so Home Assistant doesn't show stale data while updating.

  LIBRARIES NEEDED (Library Manager)
  -----------------------------------
  - ModbusMaster        (4-20ma/ModbusMaster)
  - espMqttClient        (bertmelis/espMqttClient)
  - Preferences (bundled with ESP32 core)
  - ArduinoOTA  (bundled with ESP32 core)

  ============================================================================
*/

#include <WiFi.h>
#include <espMqttClient.h>
#include <ModbusMaster.h>
#include <Preferences.h>
#include <time.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <ArduinoOTA.h>

// ============================================================================
// USER CONFIGURATION
// ============================================================================
// WiFi/MQTT credentials, static IP, device identity, RS485 pins, and the OTA
// password live in config.h (same folder - the Arduino IDE shows it as a
// second tab) so you can keep that one file per-device without touching the
// rest of this sketch.
#include "config.h"

// ---- Timing ----
const uint32_t POLL_INTERVAL_MS       = 15000;         // how often we read the meter
const uint32_t WIFI_RSSI_INTERVAL_MS  = 30000;
const uint32_t ACCUM_SAVE_INTERVAL_MS = 5UL * 60 * 1000; // NVS flush cadence for accumulators

// ---- Timezone (Bulgaria: EET/EEST, standard EU DST rule) ----
const char* TZ_STRING = "EET-2EEST,M3.5.0/3,M10.5.0/4";
const char* NTP_SERVER_1 = "pool.ntp.org";
const char* NTP_SERVER_2 = "time.google.com";

// ============================================================================
// MQTT TOPICS
// ============================================================================

String baseTopic         = String("energy/") + DEVICE_ID;
String availabilityTopic = baseTopic + "/status";       // "online" / "offline" (LWT)
String cmdRestartTopic   = baseTopic + "/cmd/restart";
String cmdResetAccumTopic= baseTopic + "/cmd/reset_tariff_energy";
String discoveryPrefix   = "homeassistant";

// ============================================================================
// GLOBALS
// ============================================================================

espMqttClient mqttClient;
ModbusMaster node;
Preferences prefs;

bool wifiWasConnected = false;
bool otaStarted = false;
bool otaInProgress = false;
bool mqttDiscoveryPublished = false;
uint32_t mqttReconnectDelayMs = 1000;
uint32_t lastMqttAttemptMs = 0;
uint32_t lastPollMs = 0;
uint32_t lastRssiMs = 0;
uint32_t lastAccumSaveMs = 0;
bool timeSynced = false;

uint32_t modbusFailCount = 0;
uint32_t mqttFailCount = 0;
bool modbusUseInputRegs = true; // auto-detected: try Input Registers (FC04) first, fall back to Holding (FC03)
bool modbusFnLocked = false;    // once a function code has worked, stop probing the other one every cycle
bool modbusDetectionAttempted = false; // only double-probe both FCs once - avoid doubling every timeout forever

// Tariff accumulators (persisted in NVS)
double energyDayKwh = 0.0;
double energyNightKwh = 0.0;
double lastTotalForwardKwh = -1.0; // -1 = not yet initialized
bool accumDirty = false;

// ============================================================================
// SENSOR TABLE (data-driven MQTT discovery + publish)
// ============================================================================

enum SensorIndex {
  IDX_VOLTAGE, IDX_CURRENT, IDX_ACTIVE_POWER, IDX_APPARENT_POWER,
  IDX_REACTIVE_POWER, IDX_FREQUENCY, IDX_POWER_FACTOR,

  IDX_E_FWD_TOTAL, IDX_E_FWD_T1, IDX_E_FWD_T2, IDX_E_FWD_T3, IDX_E_FWD_T4,
  IDX_E_REV_TOTAL, IDX_E_REV_T1, IDX_E_REV_T2, IDX_E_REV_T3, IDX_E_REV_T4,
  IDX_E_COMB_TOTAL, IDX_E_COMB_T1, IDX_E_COMB_T2, IDX_E_COMB_T3, IDX_E_COMB_T4,

  IDX_E_REACTIVE_FWD, IDX_E_REACTIVE_REV, IDX_E_REACTIVE_TOTAL,
  IDX_E_RESET_ACTIVE, IDX_E_RESET_REACTIVE,

  IDX_DEM_FWD_ACT, IDX_DEM_FWD_ACT_MAX, IDX_DEM_REV_ACT, IDX_DEM_REV_ACT_MAX,
  IDX_DEM_FWD_REACT, IDX_DEM_FWD_REACT_MAX, IDX_DEM_REV_REACT, IDX_DEM_REV_REACT_MAX,

  IDX_ENERGY_DAY, IDX_ENERGY_NIGHT,

  IDX_WIFI_RSSI, IDX_UPTIME, IDX_MODBUS_FAIL_COUNT, IDX_MQTT_FAIL_COUNT,

  SENSOR_COUNT
};

struct FloatSensorDef {
  const char* key;          // topic/unique_id suffix, no spaces
  const char* name;         // HA friendly name
  const char* deviceClass;  // nullptr if none
  const char* unit;         // nullptr if none
  const char* stateClass;   // nullptr if none ("measurement" | "total_increasing" | "total")
  bool diagnostic;          // entity_category: diagnostic
  uint8_t decimals;         // decimal places for the published state
};

FloatSensorDef floatDefs[SENSOR_COUNT] = {
  [IDX_VOLTAGE]            = {"voltage",              "Voltage",                    "voltage",       "V",    "measurement", false, 2},
  [IDX_CURRENT]             = {"current",               "Current",                    "current",       "A",    "measurement", false, 3},
  [IDX_ACTIVE_POWER]        = {"active_power",          "Active Power",                "power",         "W",    "measurement", false, 0},
  [IDX_APPARENT_POWER]      = {"apparent_power",        "Apparent Power",              "apparent_power","VA",   "measurement", false, 0},
  [IDX_REACTIVE_POWER]      = {"reactive_power",        "Reactive Power",              "reactive_power","var",  "measurement", false, 0},
  [IDX_FREQUENCY]           = {"frequency",             "Frequency",                   "frequency",     "Hz",   "measurement", false, 1},
  [IDX_POWER_FACTOR]        = {"power_factor",          "Power Factor",                "power_factor",  nullptr,"measurement", false, 3},

  [IDX_E_FWD_TOTAL]         = {"energy_forward_total",  "Forward Active Energy (Total)","energy",       "kWh",  "total_increasing", false, 2},
  [IDX_E_FWD_T1]            = {"energy_forward_t1",     "Forward Active Energy T1 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_FWD_T2]            = {"energy_forward_t2",     "Forward Active Energy T2 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_FWD_T3]            = {"energy_forward_t3",     "Forward Active Energy T3 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_FWD_T4]            = {"energy_forward_t4",     "Forward Active Energy T4 (meter)","energy",    "kWh",  "total_increasing", true, 2},

  [IDX_E_REV_TOTAL]         = {"energy_reverse_total",  "Reverse Active Energy (Total)","energy",       "kWh",  "total_increasing", false, 2},
  [IDX_E_REV_T1]            = {"energy_reverse_t1",     "Reverse Active Energy T1 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_REV_T2]            = {"energy_reverse_t2",     "Reverse Active Energy T2 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_REV_T3]            = {"energy_reverse_t3",     "Reverse Active Energy T3 (meter)","energy",    "kWh",  "total_increasing", true, 2},
  [IDX_E_REV_T4]            = {"energy_reverse_t4",     "Reverse Active Energy T4 (meter)","energy",    "kWh",  "total_increasing", true, 2},

  [IDX_E_COMB_TOTAL]        = {"energy_combined_total", "Combined Active Energy (Total)","energy",      "kWh",  "total_increasing", false, 2},
  [IDX_E_COMB_T1]           = {"energy_combined_t1",    "Combined Active Energy T1 (meter)","energy",   "kWh",  "total_increasing", true, 2},
  [IDX_E_COMB_T2]           = {"energy_combined_t2",    "Combined Active Energy T2 (meter)","energy",   "kWh",  "total_increasing", true, 2},
  [IDX_E_COMB_T3]           = {"energy_combined_t3",    "Combined Active Energy T3 (meter)","energy",   "kWh",  "total_increasing", true, 2},
  [IDX_E_COMB_T4]           = {"energy_combined_t4",    "Combined Active Energy T4 (meter)","energy",   "kWh",  "total_increasing", true, 2},

  [IDX_E_REACTIVE_FWD]      = {"energy_reactive_forward","Forward Reactive Energy",    nullptr,         "kVArh","total_increasing", true, 2},
  [IDX_E_REACTIVE_REV]      = {"energy_reactive_reverse","Reverse Reactive Energy",    nullptr,         "kVArh","total_increasing", true, 2},
  [IDX_E_REACTIVE_TOTAL]    = {"energy_reactive_total", "Total Reactive Energy",       nullptr,         "kVArh","total_increasing", true, 2},

  [IDX_E_RESET_ACTIVE]      = {"energy_resettable_active",  "Resettable Active Energy","energy",        "kWh",  "total", true, 2},
  [IDX_E_RESET_REACTIVE]    = {"energy_resettable_reactive","Resettable Reactive Energy",nullptr,       "kVArh","total", true, 2},

  [IDX_DEM_FWD_ACT]         = {"demand_forward_active",     "Forward Active Demand",       "power",          "W",   "measurement", true, 1},
  [IDX_DEM_FWD_ACT_MAX]     = {"demand_forward_active_max", "Forward Max Active Demand",   "power",          "W",   "measurement", true, 1},
  [IDX_DEM_REV_ACT]         = {"demand_reverse_active",     "Reverse Active Demand",       "power",          "W",   "measurement", true, 1},
  [IDX_DEM_REV_ACT_MAX]     = {"demand_reverse_active_max", "Reverse Max Active Demand",   "power",          "W",   "measurement", true, 1},
  [IDX_DEM_FWD_REACT]       = {"demand_forward_reactive",   "Forward Reactive Demand",     "reactive_power", "var", "measurement", true, 1},
  [IDX_DEM_FWD_REACT_MAX]   = {"demand_forward_reactive_max","Forward Max Reactive Demand","reactive_power", "var", "measurement", true, 1},
  [IDX_DEM_REV_REACT]       = {"demand_reverse_reactive",   "Reverse Reactive Demand",     "reactive_power", "var", "measurement", true, 1},
  [IDX_DEM_REV_REACT_MAX]   = {"demand_reverse_reactive_max","Reverse Max Reactive Demand","reactive_power", "var", "measurement", true, 1},

  [IDX_ENERGY_DAY]          = {"energy_day",  "Day Tariff Energy",   "energy", "kWh", "total_increasing", false, 3},
  [IDX_ENERGY_NIGHT]        = {"energy_night","Night Tariff Energy", "energy", "kWh", "total_increasing", false, 3},

  [IDX_WIFI_RSSI]           = {"wifi_rssi",   "WiFi RSSI",   "signal_strength", "dBm", "measurement", true, 0},
  [IDX_UPTIME]              = {"uptime",      "Uptime",      nullptr,           "s",   "measurement", true, 0},
  [IDX_MODBUS_FAIL_COUNT]   = {"modbus_fail_count", "Modbus Fail Count", nullptr, nullptr, nullptr, true, 0},
  [IDX_MQTT_FAIL_COUNT]     = {"mqtt_fail_count",   "MQTT Fail Count",   nullptr, nullptr, nullptr, true, 0},
};

float sensorValue[SENSOR_COUNT];
bool  sensorValid[SENSOR_COUNT];

// Text sensors (diagnostics + tariff state)
enum TextSensorIndex {
  TIDX_TARIFF, TIDX_SEASON, TIDX_RESET_REASON, TIDX_MODBUS_MODE,
  TIDX_METER_SERIAL, TIDX_METER_FW, TIDX_METER_HW, TIDX_METER_ADDR,
  TEXT_SENSOR_COUNT
};

struct TextSensorDef {
  const char* key;
  const char* name;
  const char* icon;      // mdi icon, nullptr for none
  bool diagnostic;
};

TextSensorDef textDefs[TEXT_SENSOR_COUNT] = {
  [TIDX_TARIFF]        = {"current_tariff", "Current Tariff", "mdi:clock-outline",  false},
  [TIDX_SEASON]         = {"current_season", "Current Season", "mdi:weather-sunny", false},
  [TIDX_RESET_REASON]   = {"reset_reason",   "Last Reset Reason", "mdi:restart",     true},
  [TIDX_MODBUS_MODE]    = {"modbus_fn",      "Modbus Function In Use", "mdi:swap-horizontal", true},
  [TIDX_METER_SERIAL]   = {"meter_serial",   "Meter Serial Number", "mdi:barcode",   true},
  [TIDX_METER_FW]       = {"meter_fw",       "Meter Firmware Version", "mdi:chip",   true},
  [TIDX_METER_HW]       = {"meter_hw",       "Meter Hardware Version", "mdi:chip",   true},
  [TIDX_METER_ADDR]     = {"meter_modbus_addr","Meter Modbus Address", "mdi:identifier",true},
};

String textValue[TEXT_SENSOR_COUNT];
bool textPublishedOnce[TEXT_SENSOR_COUNT] = {false};

// ============================================================================
// MODBUS HELPERS
// ============================================================================

void mbPreTransmission() {
  digitalWrite(RS485_DE_RE_PIN, HIGH);
  delayMicroseconds(50);
}

void mbPostTransmission() {
  // Make sure the last byte has actually left the UART before releasing the bus
  Serial1.flush();
  delayMicroseconds(50);
  digitalWrite(RS485_DE_RE_PIN, LOW);
}

int32_t combine32(uint16_t hi, uint16_t lo) {
  return (int32_t)(((uint32_t)hi << 16) | (uint32_t)lo);
}

// Reads `qty` registers starting at `addr`, auto-detecting whether this meter
// wants Modbus function 0x04 (Input Registers) or 0x03 (Holding Registers)
// for the measurement/energy zone (0x0100-0x0186). The register map marks
// this zone "R" (read-only), which usually means Input Registers on this
// family of meters, but some OEM firmware variants answer Holding Register
// reads instead.
//
// IMPORTANT: we only ever probe BOTH function codes back-to-back on the
// very first call (modbusDetectionAttempted == false). After that, every
// call makes exactly ONE blocking Modbus attempt. This matters because if
// the meter isn't responding at all (wrong wiring/baud/ID), doubling the
// timeout on every one of the ~9 reads per poll cycle can block loop() long
// enough to trip the ESP32 task watchdog and reboot the board - which looks
// like "everything is permanently unavailable" in Home Assistant even
// though nothing is actually wrong with WiFi/MQTT.
uint8_t mbReadBlock(uint16_t addr, uint16_t qty) {
  uint8_t result;

  if (!modbusDetectionAttempted) {
    modbusDetectionAttempted = true;
    result = node.readInputRegisters(addr, qty);
    if (result == node.ku8MBSuccess) {
      modbusUseInputRegs = true;
      modbusFnLocked = true;
      return result;
    }
    result = node.readHoldingRegisters(addr, qty);
    if (result == node.ku8MBSuccess) {
      modbusUseInputRegs = false;
      modbusFnLocked = true;
      return result;
    }
    Serial.printf("[MODBUS] Detection failed at 0x%04X (qty=%u), FC04 and FC03 both failed (last err=0x%02X). "
                  "Check wiring/power/baud/slave ID.\n", addr, qty, result);
    modbusFailCount++;
    return result;
  }

  result = modbusUseInputRegs ? node.readInputRegisters(addr, qty) : node.readHoldingRegisters(addr, qty);
  if (result == node.ku8MBSuccess) {
    modbusFnLocked = true;
    return result;
  }

  Serial.printf("[MODBUS] Read failed at 0x%04X (qty=%u), err=0x%02X\n", addr, qty, result);
  modbusFailCount++;
  return result;
}

// ============================================================================
// TARIFF LOGIC
// ============================================================================

bool isWinterSeason(int month /* 1-12 */) {
  return (month == 11 || month == 12 || month == 1 || month == 2 || month == 3);
}

bool isNightTariff(const struct tm& t, bool& outWinter) {
  outWinter = isWinterSeason(t.tm_mon + 1);
  int hour = t.tm_hour;
  if (outWinter) {
    return (hour >= 22 || hour < 6);   // winter night: 22:00-06:00
  } else {
    return (hour >= 23 || hour < 7);   // summer night: 23:00-07:00
  }
}

void accumulateTariffEnergy(double currentTotalKwh) {
  if (!timeSynced) return; // don't guess a tariff without real time

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  bool winter;
  bool night = isNightTariff(t, winter);

  textValue[TIDX_TARIFF] = night ? "night" : "day";
  textValue[TIDX_SEASON] = winter ? "winter" : "summer";

  if (lastTotalForwardKwh < 0) {
    lastTotalForwardKwh = currentTotalKwh; // establish baseline on first good reading
    accumDirty = true;
    return;
  }

  double delta = currentTotalKwh - lastTotalForwardKwh;
  if (delta < 0) {
    // Meter counter went backwards - likely a meter-side reset. Re-baseline
    // and skip this delta rather than under/over-counting.
    lastTotalForwardKwh = currentTotalKwh;
    accumDirty = true;
    return;
  }
  if (delta > 0) {
    if (night) energyNightKwh += delta; else energyDayKwh += delta;
    lastTotalForwardKwh = currentTotalKwh;
    accumDirty = true;
  }

  sensorValue[IDX_ENERGY_DAY] = (float)energyDayKwh;
  sensorValue[IDX_ENERGY_NIGHT] = (float)energyNightKwh;
  sensorValid[IDX_ENERGY_DAY] = true;
  sensorValid[IDX_ENERGY_NIGHT] = true;
}

void saveAccumulators() {
  prefs.begin("energymeter", false);
  prefs.putDouble("dayKwh", energyDayKwh);
  prefs.putDouble("nightKwh", energyNightKwh);
  prefs.putDouble("lastTotal", lastTotalForwardKwh);
  prefs.end();
  accumDirty = false;
  Serial.println("[NVS] Accumulators saved.");
}

void loadAccumulators() {
  prefs.begin("energymeter", true);
  energyDayKwh = prefs.getDouble("dayKwh", 0.0);
  energyNightKwh = prefs.getDouble("nightKwh", 0.0);
  lastTotalForwardKwh = prefs.getDouble("lastTotal", -1.0);
  prefs.end();
  Serial.printf("[NVS] Loaded: day=%.3f kWh, night=%.3f kWh, lastTotal=%.2f kWh\n",
                energyDayKwh, energyNightKwh, lastTotalForwardKwh);
}

void resetAccumulators() {
  energyDayKwh = 0.0;
  energyNightKwh = 0.0;
  lastTotalForwardKwh = -1.0; // re-baseline on next poll
  saveAccumulators();
  Serial.println("[TARIFF] Day/night accumulators reset.");
}

// ============================================================================
// METER POLLING
// ============================================================================

void pollInstantaneous() {
  // 0x0100 - 0x010B, 12 registers: V, A, W, VA, var, Hz, PF
  uint8_t res = mbReadBlock(0x0100, 12);
  if (res != node.ku8MBSuccess) return;

  sensorValue[IDX_VOLTAGE]  = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 1000.0f;
  sensorValue[IDX_CURRENT]  = combine32(node.getResponseBuffer(2), node.getResponseBuffer(3)) / 1000.0f;
  sensorValue[IDX_ACTIVE_POWER]   = (float)combine32(node.getResponseBuffer(4), node.getResponseBuffer(5));
  sensorValue[IDX_APPARENT_POWER] = (float)combine32(node.getResponseBuffer(6), node.getResponseBuffer(7));
  sensorValue[IDX_REACTIVE_POWER] = (float)combine32(node.getResponseBuffer(8), node.getResponseBuffer(9));
  sensorValue[IDX_FREQUENCY]      = (int16_t)node.getResponseBuffer(10) / 10.0f;
  sensorValue[IDX_POWER_FACTOR]   = (int16_t)node.getResponseBuffer(11) / 1000.0f;

  for (int i = IDX_VOLTAGE; i <= IDX_POWER_FACTOR; i++) sensorValid[i] = true;
}

void pollForwardActiveEnergy() {
  // 0x010E - 0x0117, 10 registers: total, T1, T2, T3, T4 (forward active)
  uint8_t res = mbReadBlock(0x010E, 10);
  if (res != node.ku8MBSuccess) return;

  double total = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0;
  sensorValue[IDX_E_FWD_TOTAL] = (float)total;
  sensorValue[IDX_E_FWD_T1]    = combine32(node.getResponseBuffer(2), node.getResponseBuffer(3)) / 100.0f;
  sensorValue[IDX_E_FWD_T2]    = combine32(node.getResponseBuffer(4), node.getResponseBuffer(5)) / 100.0f;
  sensorValue[IDX_E_FWD_T3]    = combine32(node.getResponseBuffer(6), node.getResponseBuffer(7)) / 100.0f;
  sensorValue[IDX_E_FWD_T4]    = combine32(node.getResponseBuffer(8), node.getResponseBuffer(9)) / 100.0f;
  for (int i = IDX_E_FWD_TOTAL; i <= IDX_E_FWD_T4; i++) sensorValid[i] = true;

  // This is the trusted running total we use for our own day/night split.
  accumulateTariffEnergy(total);
}

void pollReverseActiveEnergy() {
  // 0x0118 - 0x0121, 10 registers: total, T1, T2, T3, T4 (reverse active)
  uint8_t res = mbReadBlock(0x0118, 10);
  if (res != node.ku8MBSuccess) return;

  sensorValue[IDX_E_REV_TOTAL] = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
  sensorValue[IDX_E_REV_T1]    = combine32(node.getResponseBuffer(2), node.getResponseBuffer(3)) / 100.0f;
  sensorValue[IDX_E_REV_T2]    = combine32(node.getResponseBuffer(4), node.getResponseBuffer(5)) / 100.0f;
  sensorValue[IDX_E_REV_T3]    = combine32(node.getResponseBuffer(6), node.getResponseBuffer(7)) / 100.0f;
  sensorValue[IDX_E_REV_T4]    = combine32(node.getResponseBuffer(8), node.getResponseBuffer(9)) / 100.0f;
  for (int i = IDX_E_REV_TOTAL; i <= IDX_E_REV_T4; i++) sensorValid[i] = true;
}

void pollCombinedActiveEnergy() {
  // 0x0122 - 0x012B, 10 registers: total, T1, T2, T3, T4 (combined, per combination code)
  uint8_t res = mbReadBlock(0x0122, 10);
  if (res != node.ku8MBSuccess) return;

  sensorValue[IDX_E_COMB_TOTAL] = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
  sensorValue[IDX_E_COMB_T1]    = combine32(node.getResponseBuffer(2), node.getResponseBuffer(3)) / 100.0f;
  sensorValue[IDX_E_COMB_T2]    = combine32(node.getResponseBuffer(4), node.getResponseBuffer(5)) / 100.0f;
  sensorValue[IDX_E_COMB_T3]    = combine32(node.getResponseBuffer(6), node.getResponseBuffer(7)) / 100.0f;
  sensorValue[IDX_E_COMB_T4]    = combine32(node.getResponseBuffer(8), node.getResponseBuffer(9)) / 100.0f;
  for (int i = IDX_E_COMB_TOTAL; i <= IDX_E_COMB_T4; i++) sensorValid[i] = true;
}

void pollReactiveEnergyTotals() {
  // Only the totals (not the T1-T4 sub-breakdowns) - reactive energy isn't
  // normally billed by tariff, so this keeps the entity count sane.
  uint8_t res;

  res = mbReadBlock(0x012C, 2); // total forward reactive
  if (res == node.ku8MBSuccess) {
    sensorValue[IDX_E_REACTIVE_FWD] = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
    sensorValid[IDX_E_REACTIVE_FWD] = true;
  }

  res = mbReadBlock(0x0136, 2); // total reverse reactive
  if (res == node.ku8MBSuccess) {
    sensorValue[IDX_E_REACTIVE_REV] = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
    sensorValid[IDX_E_REACTIVE_REV] = true;
  }

  res = mbReadBlock(0x0140, 2); // total reactive
  if (res == node.ku8MBSuccess) {
    sensorValue[IDX_E_REACTIVE_TOTAL] = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
    sensorValid[IDX_E_REACTIVE_TOTAL] = true;
  }
}

void pollResettableEnergy() {
  // 0x0172 - 0x0175, 4 registers: resettable active, resettable reactive
  uint8_t res = mbReadBlock(0x0172, 4);
  if (res != node.ku8MBSuccess) return;

  sensorValue[IDX_E_RESET_ACTIVE]   = combine32(node.getResponseBuffer(0), node.getResponseBuffer(1)) / 100.0f;
  sensorValue[IDX_E_RESET_REACTIVE] = combine32(node.getResponseBuffer(2), node.getResponseBuffer(3)) / 100.0f;
  sensorValid[IDX_E_RESET_ACTIVE] = true;
  sensorValid[IDX_E_RESET_REACTIVE] = true;
}

void pollDemand() {
  // 0x0176 - 0x0187, 18 registers (there's a 2-register reserved gap at
  // 0x017E/0x017F between the active and reactive demand groups - harmless
  // to read through, we just don't use those two words).
  uint8_t res = mbReadBlock(0x0176, 18);
  if (res != node.ku8MBSuccess) return;

  auto val = [&](int wordOffset) {
    return combine32(node.getResponseBuffer(wordOffset), node.getResponseBuffer(wordOffset + 1)) / 10.0f;
  };

  sensorValue[IDX_DEM_FWD_ACT]        = val(0);  // 0x0176
  sensorValue[IDX_DEM_FWD_ACT_MAX]    = val(2);  // 0x0178
  sensorValue[IDX_DEM_REV_ACT]        = val(4);  // 0x017A
  sensorValue[IDX_DEM_REV_ACT_MAX]    = val(6);  // 0x017C
  sensorValue[IDX_DEM_FWD_REACT]      = val(10); // 0x0180 (word offset 10 = 0x0176 + 10)
  sensorValue[IDX_DEM_FWD_REACT_MAX]  = val(12); // 0x0182
  sensorValue[IDX_DEM_REV_REACT]      = val(14); // 0x0184
  sensorValue[IDX_DEM_REV_REACT_MAX]  = val(16); // 0x0186

  for (int i = IDX_DEM_FWD_ACT; i <= IDX_DEM_REV_REACT_MAX; i++) sensorValid[i] = true;
}

void pollMeterIdentity() {
  // One-shot, read at boot: serial number, modbus address, FW/HW version.
  // These are R.W parameter registers -> Holding Registers (FC03), regardless
  // of which function code the measurement zone turned out to need.
  uint8_t res = node.readHoldingRegisters(0x1000, 6);
  if (res == node.ku8MBSuccess) {
    char serial[25] = {0};
    for (int i = 0; i < 6; i++) {
      snprintf(serial + strlen(serial), sizeof(serial) - strlen(serial), "%04X", node.getResponseBuffer(i));
    }
    // NOTE: the manual's bit-packing for this field is ambiguous in the
    // datasheet OCR; this is a best-effort hex dump of the 6 registers.
    // Cross-check against the sticker on the meter if it matters to you.
    textValue[TIDX_METER_SERIAL] = String(serial);
  }

  res = node.readHoldingRegisters(0x1003, 1);
  if (res == node.ku8MBSuccess) {
    textValue[TIDX_METER_ADDR] = String(node.getResponseBuffer(0));
  }

  res = node.readHoldingRegisters(0x1004, 1);
  if (res == node.ku8MBSuccess) {
    textValue[TIDX_METER_FW] = String(node.getResponseBuffer(0));
  }

  res = node.readHoldingRegisters(0x1005, 1);
  if (res == node.ku8MBSuccess) {
    textValue[TIDX_METER_HW] = String(node.getResponseBuffer(0));
  }
}

void pollMeter() {
  uint32_t failBefore = modbusFailCount;

  pollInstantaneous();
  pollForwardActiveEnergy();
  pollReverseActiveEnergy();
  pollCombinedActiveEnergy();
  pollReactiveEnergyTotals();
  pollResettableEnergy();
  pollDemand();

  textValue[TIDX_MODBUS_MODE] = modbusUseInputRegs ? "input_registers (FC04)" : "holding_registers (FC03)";

  uint32_t failsThisCycle = modbusFailCount - failBefore;
  if (failsThisCycle > 0) {
    Serial.printf("[MODBUS] Poll cycle: %u/9 block reads failed. Total fail count: %u\n",
                  failsThisCycle, modbusFailCount);
  } else {
    Serial.printf("[MODBUS] Poll cycle OK. V=%.1f I=%.3f P=%.0fW Total=%.2fkWh\n",
                  sensorValue[IDX_VOLTAGE], sensorValue[IDX_CURRENT],
                  sensorValue[IDX_ACTIVE_POWER], sensorValue[IDX_E_FWD_TOTAL]);
  }
}

// ============================================================================
// MQTT: DISCOVERY
// ============================================================================

String deviceBlockJson() {
  char buf[384];
  snprintf(buf, sizeof(buf),
    "\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\",\"manufacturer\":\"%s\",\"model\":\"%s\",\"sw_version\":\"%s\"}",
    DEVICE_ID, DEVICE_NAME, MANUFACTURER, MODEL_NAME, FIRMWARE_VER);
  return String(buf);
}

void publishFloatDiscovery(const FloatSensorDef& def) {
  String uniqueId = String(DEVICE_ID) + "_" + def.key;
  String stateTopic = baseTopic + "/" + def.key;
  String configTopic = discoveryPrefix + "/sensor/" + DEVICE_ID + "/" + def.key + "/config";

  String payload = "{";
  payload += "\"name\":\"" + String(def.name) + "\",";
  payload += "\"unique_id\":\"" + uniqueId + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"availability_topic\":\"" + availabilityTopic + "\",";
  if (def.deviceClass) payload += String("\"device_class\":\"") + def.deviceClass + "\",";
  if (def.unit)        payload += String("\"unit_of_measurement\":\"") + def.unit + "\",";
  if (def.stateClass)  payload += String("\"state_class\":\"") + def.stateClass + "\",";
  if (def.diagnostic)  payload += "\"entity_category\":\"diagnostic\",";
  payload += deviceBlockJson();
  payload += "}";

  mqttClient.publish(configTopic.c_str(), 1, true, payload.c_str());
}

void publishTextDiscovery(const TextSensorDef& def) {
  String uniqueId = String(DEVICE_ID) + "_" + def.key;
  String stateTopic = baseTopic + "/" + def.key;
  String configTopic = discoveryPrefix + "/sensor/" + DEVICE_ID + "/" + def.key + "/config";

  String payload = "{";
  payload += "\"name\":\"" + String(def.name) + "\",";
  payload += "\"unique_id\":\"" + uniqueId + "\",";
  payload += "\"state_topic\":\"" + stateTopic + "\",";
  payload += "\"availability_topic\":\"" + availabilityTopic + "\",";
  if (def.icon)        payload += String("\"icon\":\"") + def.icon + "\",";
  if (def.diagnostic)  payload += "\"entity_category\":\"diagnostic\",";
  payload += deviceBlockJson();
  payload += "}";

  mqttClient.publish(configTopic.c_str(), 1, true, payload.c_str());
}

void publishRestartButtonDiscovery() {
  String uniqueId = String(DEVICE_ID) + "_restart";
  String configTopic = discoveryPrefix + "/button/" + DEVICE_ID + "/restart/config";
  String payload = "{";
  payload += "\"name\":\"Restart\",";
  payload += "\"unique_id\":\"" + uniqueId + "\",";
  payload += "\"command_topic\":\"" + cmdRestartTopic + "\",";
  payload += "\"availability_topic\":\"" + availabilityTopic + "\",";
  payload += "\"device_class\":\"restart\",";
  payload += "\"entity_category\":\"diagnostic\",";
  payload += deviceBlockJson();
  payload += "}";
  mqttClient.publish(configTopic.c_str(), 1, true, payload.c_str());
}

void publishResetAccumButtonDiscovery() {
  String uniqueId = String(DEVICE_ID) + "_reset_tariff_energy";
  String configTopic = discoveryPrefix + "/button/" + DEVICE_ID + "/reset_tariff_energy/config";
  String payload = "{";
  payload += "\"name\":\"Reset Day/Night Energy\",";
  payload += "\"unique_id\":\"" + uniqueId + "\",";
  payload += "\"command_topic\":\"" + cmdResetAccumTopic + "\",";
  payload += "\"availability_topic\":\"" + availabilityTopic + "\",";
  payload += "\"icon\":\"mdi:restore\",";
  payload += "\"entity_category\":\"diagnostic\",";
  payload += deviceBlockJson();
  payload += "}";
  mqttClient.publish(configTopic.c_str(), 1, true, payload.c_str());
}

void publishDiscovery() {
  for (int i = 0; i < SENSOR_COUNT; i++) publishFloatDiscovery(floatDefs[i]);
  for (int i = 0; i < TEXT_SENSOR_COUNT; i++) publishTextDiscovery(textDefs[i]);
  publishRestartButtonDiscovery();
  publishResetAccumButtonDiscovery();
  mqttDiscoveryPublished = true;
  Serial.println("[MQTT] Discovery published.");
}

// ============================================================================
// MQTT: STATE PUBLISH
// ============================================================================

void publishStates() {
  char buf[32];
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorValid[i]) continue;
    String topic = baseTopic + "/" + floatDefs[i].key;
    dtostrf(sensorValue[i], 0, floatDefs[i].decimals, buf);
    // trim leading spaces dtostrf may add
    char* p = buf; while (*p == ' ') p++;
    mqttClient.publish(topic.c_str(), 0, false, p);
  }
  for (int i = 0; i < TEXT_SENSOR_COUNT; i++) {
    if (textValue[i].length() == 0) continue;
    // Identity fields only need to be published once (or whenever they change)
    if ((i == TIDX_METER_SERIAL || i == TIDX_METER_FW || i == TIDX_METER_HW || i == TIDX_METER_ADDR)
        && textPublishedOnce[i]) continue;
    String topic = baseTopic + "/" + textDefs[i].key;
    mqttClient.publish(topic.c_str(), 0, false, textValue[i].c_str());
    textPublishedOnce[i] = true;
  }
}

void publishDiagnostics() {
  sensorValue[IDX_WIFI_RSSI] = WiFi.RSSI();
  sensorValue[IDX_UPTIME] = millis() / 1000;
  sensorValue[IDX_MODBUS_FAIL_COUNT] = modbusFailCount;
  sensorValue[IDX_MQTT_FAIL_COUNT] = mqttFailCount;
  sensorValid[IDX_WIFI_RSSI] = true;
  sensorValid[IDX_UPTIME] = true;
  sensorValid[IDX_MODBUS_FAIL_COUNT] = true;
  sensorValid[IDX_MQTT_FAIL_COUNT] = true;
}

const char* resetReasonToString(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON: return "power_on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_INT_WDT: return "interrupt_watchdog";
    case ESP_RST_TASK_WDT: return "task_watchdog";
    case ESP_RST_WDT: return "other_watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_DEEPSLEEP: return "deep_sleep_wake";
    default: return "unknown";
  }
}

// ============================================================================
// MQTT: CALLBACKS
// ============================================================================

void onMqttConnect(bool sessionPresent) {
  Serial.println("[MQTT] Connected.");
  mqttReconnectDelayMs = 1000;
  mqttClient.publish(availabilityTopic.c_str(), 1, true, "online");
  mqttClient.subscribe(cmdRestartTopic.c_str(), 1);
  mqttClient.subscribe(cmdResetAccumTopic.c_str(), 1);
  publishDiscovery();

  // Publish diagnostics right away rather than waiting for the next 30s
  // tick, so WiFi RSSI / uptime / reset reason show up immediately even if
  // the Modbus side is still being debugged.
  publishDiagnostics();
  mqttClient.publish((baseTopic + "/" + floatDefs[IDX_WIFI_RSSI].key).c_str(), 0, false, String((int)sensorValue[IDX_WIFI_RSSI]).c_str());
  mqttClient.publish((baseTopic + "/" + floatDefs[IDX_UPTIME].key).c_str(), 0, false, String((uint32_t)sensorValue[IDX_UPTIME]).c_str());
  mqttClient.publish((baseTopic + "/" + floatDefs[IDX_MODBUS_FAIL_COUNT].key).c_str(), 0, false, String((uint32_t)sensorValue[IDX_MODBUS_FAIL_COUNT]).c_str());
  mqttClient.publish((baseTopic + "/" + floatDefs[IDX_MQTT_FAIL_COUNT].key).c_str(), 0, false, String((uint32_t)sensorValue[IDX_MQTT_FAIL_COUNT]).c_str());
  mqttClient.publish((baseTopic + "/" + textDefs[TIDX_RESET_REASON].key).c_str(), 0, false, textValue[TIDX_RESET_REASON].c_str());
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  Serial.println("[MQTT] Disconnected.");
  mqttFailCount++;
  mqttDiscoveryPublished = false;
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                    const uint8_t* payload, size_t len, size_t index, size_t total) {
  String t = String(topic);
  if (t == cmdRestartTopic) {
    Serial.println("[CMD] Restart requested.");
    if (accumDirty) saveAccumulators();
    mqttClient.publish(availabilityTopic.c_str(), 1, true, "offline");
    delay(200);
    ESP.restart();
  } else if (t == cmdResetAccumTopic) {
    Serial.println("[CMD] Reset day/night energy requested.");
    resetAccumulators();
  }
}

// ============================================================================
// WIFI / MQTT CONNECTION MANAGEMENT
// ============================================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, DNS_IP);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] Connecting to %s ...\n", WIFI_SSID);
}

void setupOTA() {
  ArduinoOTA.setHostname(DEVICE_ID);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    otaInProgress = true;
    Serial.println("[OTA] Update starting...");
    if (accumDirty) saveAccumulators(); // don't lose tariff counters mid-flash
    if (mqttClient.connected()) {
      mqttClient.publish(availabilityTopic.c_str(), 1, true, "offline");
    }
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("[OTA] Update complete, rebooting.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    static uint8_t lastPct = 255;
    uint8_t pct = (progress * 100) / total;
    if (pct != lastPct) {
      lastPct = pct;
      Serial.printf("[OTA] Progress: %u%%\n", pct);
    }
  });

  ArduinoOTA.onError([](ota_error_t error) {
    otaInProgress = false;
    Serial.printf("[OTA] Error [%u]: ", error);
    switch (error) {
      case OTA_AUTH_ERROR:    Serial.println("Auth failed"); break;
      case OTA_BEGIN_ERROR:   Serial.println("Begin failed"); break;
      case OTA_CONNECT_ERROR: Serial.println("Connect failed"); break;
      case OTA_RECEIVE_ERROR: Serial.println("Receive failed"); break;
      case OTA_END_ERROR:     Serial.println("End failed"); break;
    }
  });

  ArduinoOTA.begin();
  otaStarted = true;
  Serial.println("[OTA] Ready (ArduinoOTA).");
}

void ensureMqttConnected() {
  if (mqttClient.connected()) return;
  uint32_t now = millis();
  if (now - lastMqttAttemptMs < mqttReconnectDelayMs) return;
  lastMqttAttemptMs = now;
  Serial.println("[MQTT] Attempting connection...");
  mqttClient.connect();
  mqttReconnectDelayMs = min(mqttReconnectDelayMs * 2, (uint32_t)60000);
}

void ensureTimeSynced() {
  if (timeSynced) return;
  time_t now = time(nullptr);
  if (now > 1700000000) { // sanity check: after ~Nov 2023 means NTP has landed
    timeSynced = true;
    struct tm t;
    localtime_r(&now, &t);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &t);
    Serial.printf("[NTP] Time synced: %s\n", buf);
  }
}

// ============================================================================
// SETUP / LOOP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n[BOOT] ESP32-C3 Smart Energy Meter starting...");

  // Give ourselves plenty of headroom before the loop-task watchdog would
  // fire, and don't let it force a reboot (just warn) - Modbus timeouts
  // against an unresponsive/unwired meter can otherwise block loop() long
  // enough to trip the default 5s watchdog and cause a silent crash-reboot
  // loop that looks like "the whole device is broken" from Home Assistant's
  // side even though WiFi/MQTT are fine.
  // (ESP32 Arduino core 3.x changed esp_task_wdt_init() to take a config
  // struct instead of (timeout_s, panic) - the TWDT is also already running
  // by the time setup() executes on core 3.x, so init() would return
  // ESP_ERR_INVALID_STATE; reconfigure() is what actually applies here.)
  esp_task_wdt_config_t twdtConfig = {
    .timeout_ms = 20000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = false,
  };
  esp_err_t twdtErr = esp_task_wdt_init(&twdtConfig);
  if (twdtErr == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&twdtConfig);
  }

  esp_reset_reason_t rr = esp_reset_reason();
  textValue[TIDX_RESET_REASON] = resetReasonToString(rr);
  Serial.printf("[BOOT] Reset reason: %s\n", textValue[TIDX_RESET_REASON].c_str());

  // RS485
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);
  Serial1.begin(MB_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  node.begin(MB_SLAVE_ID, Serial1);
  node.preTransmission(mbPreTransmission);
  node.postTransmission(mbPostTransmission);

  // NVS accumulators
  loadAccumulators();
  sensorValue[IDX_ENERGY_DAY] = (float)energyDayKwh;
  sensorValue[IDX_ENERGY_NIGHT] = (float)energyNightKwh;
  sensorValid[IDX_ENERGY_DAY] = true;
  sensorValid[IDX_ENERGY_NIGHT] = true;

  // WiFi
  connectWiFi();

  // NTP (local timezone applied so localtime_r() gives wall-clock time directly)
  configTzTime(TZ_STRING, NTP_SERVER_1, NTP_SERVER_2);

  // MQTT
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setClientId(DEVICE_ID);
  mqttClient.setWill(availabilityTopic.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);

  // One-shot meter identity read (retried lazily on next poll if it fails now,
  // since the bus/meter might not be ready in the first instant after boot)
  delay(500);
  pollMeterIdentity();
}

void loop() {
  // --- WiFi ---
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      Serial.print("[WiFi] Connected, IP: ");
      Serial.println(WiFi.localIP());
      wifiWasConnected = true;
    }
    if (!otaStarted) setupOTA();
    ArduinoOTA.handle();
    ensureTimeSynced();
    ensureMqttConnected();
  } else {
    wifiWasConnected = false;
  }

  if (otaInProgress) return; // don't touch RS485/MQTT while a flash is in progress

  uint32_t now = millis();

  // --- Poll meter ---
  if (now - lastPollMs >= POLL_INTERVAL_MS) {
    lastPollMs = now;
    pollMeter();
    if (mqttClient.connected()) {
      publishStates();
    }
  }

  // --- WiFi RSSI / diagnostics ---
  if (now - lastRssiMs >= WIFI_RSSI_INTERVAL_MS) {
    lastRssiMs = now;
    publishDiagnostics();
    if (mqttClient.connected()) {
      String topic = baseTopic + "/" + floatDefs[IDX_WIFI_RSSI].key;
      mqttClient.publish(topic.c_str(), 0, false, String((int)sensorValue[IDX_WIFI_RSSI]).c_str());
      topic = baseTopic + "/" + floatDefs[IDX_UPTIME].key;
      mqttClient.publish(topic.c_str(), 0, false, String((uint32_t)sensorValue[IDX_UPTIME]).c_str());
      topic = baseTopic + "/" + floatDefs[IDX_MODBUS_FAIL_COUNT].key;
      mqttClient.publish(topic.c_str(), 0, false, String((uint32_t)sensorValue[IDX_MODBUS_FAIL_COUNT]).c_str());
      topic = baseTopic + "/" + floatDefs[IDX_MQTT_FAIL_COUNT].key;
      mqttClient.publish(topic.c_str(), 0, false, String((uint32_t)sensorValue[IDX_MQTT_FAIL_COUNT]).c_str());
      topic = baseTopic + "/" + textDefs[TIDX_MODBUS_MODE].key;
      mqttClient.publish(topic.c_str(), 0, false, textValue[TIDX_MODBUS_MODE].c_str());
      topic = baseTopic + "/" + textDefs[TIDX_RESET_REASON].key;
      mqttClient.publish(topic.c_str(), 0, false, textValue[TIDX_RESET_REASON].c_str());
    }
  }

  // --- Persist tariff accumulators periodically ---
  if (accumDirty && (now - lastAccumSaveMs >= ACCUM_SAVE_INTERVAL_MS)) {
    lastAccumSaveMs = now;
    saveAccumulators();
  }
}
