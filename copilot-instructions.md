# Freematics ABRP Client - AI Agent Instructions

## Project Overview
This is a vehicle telemetry system running on Freematics ONE+ Model B (ESP32-based OBD dongle). It collects vehicle data (OBD-II, GPS, motion sensors) and transmits it to ABRP (A Better Route Planner) for EV tracking. Key hardware: ESP32, u-blox M10 GNSS, ICM-42627/ICM-20948 MEMS, SIM7070G cellular.

## Architecture Essentials

### Core Components
- **telelogger.cpp** (main): Orchestrates data collection via `setup()` → `initialize()` → `loop()` → `process()` flow. Manages device state machine (7 flags in `telelogger.cpp` lines 32-39)
- **Freematics Hardware Library** (`libraries/FreematicsPlus/*`): Unified API for OBD, GPS, MEMS, cellular/WiFi. ESP32 pin mappings in `FreematicsPlus.h` lines 31-48
- **Buffer System** (`teleclient.h`, `telestore.*`): Circular buffer (CBuffer/CBufferManager) stores data in PSRAM during network outages. Dual mode: PSRAM for 1024 slots (~hours) or IRAM for 32 slots (~minutes)
- **Data Transmission**: UDP/HTTPS POST to ABRP server via WiFi/cellular. Protocol configured in `config.h` (PROTOCOL_UDP=1, PROTOCOL_HTTPS_POST=3)

### Data Flow
1. **Collection**: `process()` polls OBD PIDs (defined in `telelogger.cpp` lines 45-51), reads GPS/MEMS, stores in CBuffer
2. **Buffering**: CBuffer serializes data per ELEMENT_HEAD structure (`teleclient.h` lines 23-26)
3. **Transmission**: Background telemetry task (`subtask.create()` in setup) sends buffered data via configured protocol
4. **Storage**: Optional SD card logging via FileLogger or SPIFFS via CStorageRAM

### Key Design Patterns
- **Lazy initialization**: Sensors initialized only if available (MEMS tries ICM-42627 → ICM-20948, `telelogger.cpp` lines 1392-1418)
- **Task-based multitasking**: Main loop handles data collection; background task handles networking (prevents blocking)
- **Configuration hierarchy**: Environment variables (menuconfig) → config.h macros → runtime NVS (config.xml on SD card)

## Critical Build & Workflow Commands

### Building
```bash
# PlatformIO tasks defined in workspace:
pio run              # Build for ESP32
pio run --target clean
pio run --target upload
```
See `platformio.ini`: Board `esp-wrover-kit`, CPU 160MHz, monitor 115200 baud. PSRAM optional (commented flag).

### Configuration
- **Compile-time**: Edit `config.h` macros (ENABLE_OBD, ENABLE_WIFI, ENABLE_MEMS, etc.)
- **Runtime**: SD card `config.cfg` file (parsed in `loadConfig()`)
- **Persistent**: NVS flash storage for WiFi SSID/password, APN

### Debugging
- USB serial @ 115200: Status messages, error counts (timeoutsOBD, timeoutsNet)
- State flags visible via `Serial.println()` in process loop
- OLED output (if ENABLE_OLED=1) mirrors serial diagnostics

## Project-Specific Patterns

### OBD Data Collection
PIDs in `obdData[]` array use tiered polling: Tier 1 (every read), Tier 2 (50% skips), Tier 3 (75% skips). Add new PID by appending to array with byte `pid` and byte `tier`.

### ABRP Data Requirements
Mandatory fields (High priority): `utc`, `soc`, `power`, `speed`, `lat`, `lon`, `is_charging`, `is_dcfc`, `is_parked`. Optional fields map to PIDs via config. See `README.md` lines 60-95 for full mapping.

### Buffer State Machine
States in `teleclient.h` lines 11-14: EMPTY → FILLING → FILLED → LOCKED. CBuffer managed by CBufferManager pool. Serialize on FILLED, free on transmission complete. Overflow handled by oldest buffer eviction.

### Network Resilience
- UDP: Fire-and-forget, best for continuous data
- HTTPS POST: Reliable but slower; recommended for batch upload after outage recovery
- Fallback chain: Cellular → WiFi (configured in platform config)
- PSRAM buffering allows hours of offline operation; data transmitted when connectivity resumes

## File Conventions
- **Header guards**: `#ifndef CLASS_HEADER_INCLUDED`
- **Config section**: All feature flags in `config.h` (lines 1-50)
- **Library paths**: `libraries/` includes FreematicsPlus and httpd utility libraries
- **External config**: `config.xml` on SD card overrides compiled defaults

## Common Modifications
- **Add new sensor**: Create handler in telelogger.cpp `process()`, add struct to buffer, include in ABRP data if applicable
- **Change polling frequency**: Adjust PID tier values or modify main loop sleep duration
- **Alter transmission protocol**: Update `config.h` protocol define and corresponding handler in teleclient
- **Optimize PSRAM usage**: Adjust BUFFER_SLOTS/BUFFER_LENGTH in `config.h` lines 43-50 based on memory profile

## Gotchas
- **OBD timeout handling**: `MAX_OBD_ERRORS=3` (config.h line 66) triggers standby; avoid aggressive polling
- **GNSS power management**: GPS draws 50mA+; PIN_GPS_POWER (pin 12) must be managed during sleep
- **PSRAM detection**: Requires `BOARD_HAS_PSRAM` flag; verify at compile time via `#if BOARD_HAS_PSRAM`
- **SIM card dependency**: Cellular requires valid SIM and APN config; fallback to WiFi if cellular unavailable
- **Circular buffer wraparound**: Ensure SERIALIZE_BUFFER_SIZE (`config.h` line 45-46) can hold largest serialized frame, or packets truncate
