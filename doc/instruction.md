# Code Review of Freematics-EV

This document describes the code in the **Freematics-EV** project in detail, including the bundled libraries. The focus is on how telemetry data is collected, buffered, stored, and transmitted, as well as which build/configuration files control the behavior.

## Project Overview

The project is a firmware sketch for Freematics ONE+ (an ESP32-based telematics device). It logs and transmits vehicle data (OBD-II), GNSS position, MEMS motion data, signal strength, and device status. The data can be stored locally (SD/SPIFFS), streamed to Freematics Hub via UDP/HTTPS, and exposed through a built-in HTTP server as well as a simple dashboard page.

## Build and Configuration

- **platformio.ini** describes the PlatformIO environment (`env:esp32dev`), Arduino framework, baud rate, flash settings, and extra library directory (`lib_extra_dirs=./libraries`).
- **config.h** contains compile-time configuration (for example whether OBD, MEMS, GNSS, Wi-Fi, BLE, and HTTPD should be active), buffer size, network and storage choices, as well as thresholds/time intervals for data collection and standby.
- **config.xml** defines the same settings as structured “defines” (for example through a configuration tool), with default values for OBD, GNSS mode, storage, Wi-Fi, and server protocol.

### Important Configuration Parameters (examples)

- **OBD and GNSS modes** are controlled by `ENABLE_OBD`, `GNSS`, and `ENABLE_MEMS` in `config.h`.
- **Server settings** (host, port, protocol) are controlled by `SERVER_HOST`, `SERVER_PORT`, and `SERVER_PROTOCOL`.
- **Storage** is controlled by `STORAGE` (SD, SPIFFS, or no storage).
- **Data intervals** are controlled by `DATA_INTERVAL_TABLE` and `STATIONARY_TIME_TABLE` to adjust sampling when the car is stationary.

## Main Application: telelogger.cpp

`telelogger.cpp` is the central Arduino sketch that coordinates all data collection, storage, and transmission.

### Structure and Global State

- **State flags** (`STATE_*`): keep track of whether OBD, GNSS, MEMS, network, and storage are ready, and whether the device is running actively or is in standby.
- **PID list** (`obdData`): defines which OBD PIDs are read and at which “tier” (priority) they are polled.
- **Buffers**: `CBufferManager bufman` manages a ring buffer of data packets (through `CBuffer`).
- **Network client**: `TeleClientUDP` or `TeleClientHTTP` depending on `SERVER_PROTOCOL`.
- **Storage**: `SDLogger` or `SPIFFSLogger` depending on `STORAGE`.

### Initialization (`setup` / `initialize`)

1. **NVS and configuration loading**: NVS is used to read saved configuration.
2. **Serial logging**: USB serial is initialized.
3. **Device ID**: generated to identify the device in logs/telemetry.
4. **MEMS initialization**: first tests `ICM_42627`, then `ICM_20948`, and calibrates accelerometer bias.
5. **GNSS initialization**: starts GNSS, internal or external depending on `GNSS` and hardware support.
6. **OBD initialization**: establishes the OBD connection, reads the VIN, and any DTCs.
7. **Storage**: initializes SD/SPIFFS and opens a new log file.
8. **HTTP server**: if enabled, a local web server is started and the IP is printed.
9. **BLE**: BLE-SPP server is initialized if enabled.
10. **Telemetry thread**: a separate task (`telemetry`) is started for network handling and uploads.

### Data Collection (`process`)

Each cycle in `loop()` calls `process()`, which:

- **OBD polling**: reads PIDs (for example speed, RPM, load, temperature) according to priority, with error handling and retries.
- **GNSS**: collects latitude/longitude, speed, time, and satellite status. Includes reset logic if GNSS becomes “stale.”
- **MEMS**: reads accelerometer/gyroscope/magnetometer data and calculates motion.
- **Device temperature and voltage**: reads battery voltage and chip temperature.
- **Buffering**: writes all values to `CBuffer`, which is then marked as filled.
- **Adaptive sampling**: adjusts `dataInterval` based on motion (stationary => less frequent sampling).

### Network and Telemetry (`telemetry`)

`telemetry()` runs in parallel and manages connections and transmission:

- **Wi-Fi first**: if an SSID is configured, it first attempts to connect via Wi-Fi and send data there.
- **Cellular fallback**: if Wi-Fi is not available, the cellular module (SIMCOM) is activated for connectivity.
- **Ping/keep-alive**: periodic ping during standby, along with RSSI monitoring.
- **Transmission**: buffers are serialized to `CStorageRAM`, packaged, and sent to the server via UDP/HTTP.
- **Switching**: when Wi-Fi becomes available, the cellular module is shut down to save power.

### Standby Logic

- If no motion is detected for an extended period, the device enters **standby**.
- Standby means the network modules are turned off and buffers are cleared; the device then “pings” from time to time to determine whether it should wake up.

### HTTP API and Live Data

If `ENABLE_HTTPD` is active, the following are exposed (via `dataserver.cpp`):

- `/api/info` — CPU temperature, RTC time, storage information.
- `/api/live` — live OBD/GPS/MEMS data in JSON.
- `/api/log/<id>` — raw CSV log file.
- `/api/data/<id>?pid=...` — JSON export for a specific PID.

`handlerLiveData()` in `telelogger.cpp` formats JSON for real-time data.

## Storage: telestore.*

`CStorage` and its subclasses handle serialization and logging:

- **CStorage**: writes PID pairs to serial output (standard) or forwards them to cache/logging.
- **CStorageRAM**: buffers data in RAM and appends a checksum tail for transmission packets.
- **FileLogger**: shared file writing for SD/SPIFFS.
- **SDLogger/SPIFFSLogger**: initialize the media, open files, and flush data.

## Buffering and Telemetry Packets: teleclient.*

- **CBuffer**: stores PID values with type and count in binary format before serialization.
- **CBufferManager**: pool of `CBuffer` slots in RAM/PSRAM with “oldest wins” logic when the buffer is full.
- **TeleClient**: abstract client with tx/rx counters.
- **TeleClientUDP/HTTP**: concrete implementation that sends data packets over Wi-Fi or cellular.

## HTTP Server Library: libraries/httpd

This is a minimal HTTP server implementation (MiniWeb):

- **httpd.h**: types, flags, request/response structures, and HTTP status codes.
- **httpd.c/httppil.c/httpjson.c**: implement socket handling, request parsing, and JSON helpers.
- The server is used by `dataserver.cpp` for the APIs.

## The FreematicsPlus Library

The internal library in `libraries/FreematicsPlus` is the core for low-level hardware functionality.

### FreematicsBase.h

- Defines **custom PIDs** for GPS, MEMS, voltage, temperature, etc.
- Defines `GPS_DATA`, `ORIENTATION`, and abstract links (`CLink`) and device base (`CFreematics`).

### FreematicsPlus.h / FreematicsPlus.cpp

- Contains **hardware-specific pin maps** for ESP32 and the Freematics devices.
- `FreematicsESP32` implements GPS, UART, buzzer, reset, and xBee communication as well as co-processor control.

### FreematicsNetwork.h / FreematicsNetwork.cpp

- Implements **Wi-Fi** and **cellular** clients:
  - `ClientWIFI`, `WifiUDP`, `WifiHTTP`.
  - `CellSIMCOM`, `CellUDP`, `CellHTTP` for SIM7600/7070/5360.
- Handles APN, signal strength, IP resolution, and GNSS data through the modem.

### FreematicsOBD.h / FreematicsOBD.cpp

- `COBD` handles OBD initialization, PID reading, DTC handling, CAN sniffing, and VIN reading.
- Parses responses from the ECU and normalizes the data.

### FreematicsMEMS.h / FreematicsMEMS.cpp

- Support for MEMS sensors (ICM-42627, ICM-20948, and others).
- Provides accelerometer/gyroscope/magnetometer data and orientation.

### FreematicsGPS.h / FreematicsGPS.cpp

- Based on TinyGPS: parses NMEA streams and calculates position, speed, heading, satellites, HDOP, etc.

### Utility Files

`libraries/FreematicsPlus/utility` contains register definitions and C drivers for MEMS sensors, as well as a BLE SPP server.

## Dashboard (web)

The `dashboard/` folder contains a simple HTML/JS dashboard:

- **index.html**: displays device and status values.
- **dashboard.js**: parses serial/communication output and updates UI fields in real time, including icon status for GNSS/OBD/Wi-Fi.

## Summary of the Data Flow

1. **Sensors** (OBD, GNSS, MEMS) are read.
2. **Buffering** occurs in `CBuffer` through `CBufferManager`.
3. **Storage**: data packets are serialized to SD/SPIFFS (optional).
4. **Telemetry**: data packets are packaged in a RAM buffer, a checksum is added, and they are sent via UDP/HTTP.
5. **API/Dashboard**: the HTTP server and simple web page provide real-time status.

This provides a complete telemetry pipeline from the vehicle ECU and sensors all the way to the server, local archive, and real-time dashboard.
