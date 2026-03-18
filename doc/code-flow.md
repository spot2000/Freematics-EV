# Code Flow Overview

This document explains the main runtime flow of the firmware in `telelogger.cpp`, starting from boot, through initialization, normal processing, background telemetry, standby, and wake-up.

## 1. High-level execution model

The firmware has two main execution paths:

1. **The Arduino main thread**
   - runs `setup()` once at boot
   - then runs `loop()` forever
   - is responsible for hardware initialization, sensor polling, logging, trip-state management, and standby transitions

2. **The telemetry background task**
   - started from `setup()` as the `telemetry()` FreeRTOS task
   - runs in parallel with the main loop
   - is responsible for network management and uploading buffered data over Wi-Fi or cellular

The code uses a shared `State` bitmask (`state`) to coordinate whether subsystems are ready, whether the unit is actively working, whether it is in standby, and whether network links are connected.

## 2. Boot sequence: `setup()`

`setup()` is the true entry point when the ESP32 starts.

### What `setup()` does and why

`setup()` performs the one-time platform initialization needed before normal logging can begin:

- waits briefly after boot
- initializes NVS and loads saved configuration such as APN and Wi-Fi settings
- starts OLED output when enabled
- starts the serial console
- optionally loads SD-card INI overrides
- configures LED and external sensor pins
- generates a unique device ID with `genDeviceID()`
- optionally enters configuration mode
- prints hardware/system information with `showSysInfo()`
- initializes the RAM buffer manager with `bufman.init()`
- starts the Freematics hardware link with `sys.begin()` and binds the OBD object to that link
- probes and initializes the MEMS sensor if enabled
- starts the local HTTP server if enabled
- enables BLE if enabled
- calls `initialize()` to prepare the runtime subsystems
- starts the `telemetry()` background task
- gives an audible boot confirmation with `beep(50, 3)`

At the end of `setup()`, the device is ready to enter active logging mode.

## 3. Runtime activation: `initialize()`

`initialize()` is the main "enter active mode" routine. It is used both:

- during the first boot from `setup()`, and
- after waking up from standby, when `loop()` calls it again

### What `initialize()` does

`initialize()` prepares the subsystems needed for a new active logging session:

1. **Clears any stale buffered state**
   - `bufman.purge()` removes any free/queued buffer state from the previous cycle.

2. **Calibrates MEMS when available**
   - `calibrateMEMS()` measures the accelerometer bias so later motion and acceleration values are relative to a stable baseline.

3. **Starts GNSS if needed**
   - `initGPS()` tries external GNSS first and then internal GNSS.
   - If successful, `STATE_GPS_READY` is set.

4. **Starts OBD communication if needed**
   - tries `obd.init(PROTO_ISO15765_11B_500K)`
   - if successful, sets `STATE_OBD_READY`
   - this allows later polling of speed, RPM, throttle, and other configured PIDs

5. **Initializes storage**
   - if storage is enabled, the logger is initialized and `logger.begin()` starts a new logging session/file
   - `STATE_STORAGE_READY` is set when storage is usable

6. **Reads identity/diagnostic metadata**
   - attempts to read VIN with `obd.getVIN()`
   - attempts to read DTCs with `obd.readDTC()`
   - this gives a snapshot of vehicle identity and fault state near session start

7. **Prints valid UTC time when available**
   - `printTime()` reports the current system time if already synchronized

8. **Marks the unit as actively working**
   - `lastMotionTime` is set to the current `millis()` value
   - `STATE_WORKING` is set

This function is important because the rest of the firmware assumes that active logging only happens after `initialize()` has refreshed the runtime state.

## 4. Main control loop: `loop()`

After `setup()` finishes, Arduino repeatedly calls `loop()`.

`loop()` has a very simple but important state machine:

### Case A: the device is not working

If `STATE_WORKING` is **not** set:

1. `standby()` is called
2. after wake-up returns, `initialize()` is called again
3. the next `loop()` iterations continue in active mode

This means `loop()` is the top-level controller that switches between **standby** and **active logging**.

### Case B: the device is working

If `STATE_WORKING` **is** set:

- `process()` is called once per cycle

So the repeated active behavior of the firmware is almost entirely inside `process()`.

## 5. Active data-collection cycle: `process()`

`process()` is the main recurring function during active operation. Each call builds one data sample buffer and decides whether the trip should continue.

### Step 1: reserve a buffer

The function gets a free `CBuffer` from `bufman` and marks it as filling. This buffer becomes the container for the current sample set.

### Step 2: collect OBD data or run the UDS test path

If OBD is ready:

- once per `udsIntervalMs` interval, the code currently runs a UDS/CAN test path
- otherwise it calls `processOBD(buffer)`

`processOBD()`:

- iterates through the configured `obdData[]` PID list
- polls standard OBD PIDs tier-by-tier
- stores successful values into the buffer
- updates the cached live values and timestamps
- increments `timeoutsOBD` on failures
- updates `lastMotionTime` when vehicle speed is at least 2 km/h

If too many OBD errors occur, `process()` attempts to reinitialize OBD. If that fails, it assumes the ECU is off, clears `STATE_OBD_READY` and `STATE_WORKING`, and returns so the top-level loop can move the device into standby.

### Step 3: append link and device health data

Still inside `process()`:

- RSSI changes are stored as `PID_CSQ`
- battery voltage is read and stored as `PID_BATTERY_VOLTAGE`
- optional external sensor inputs are appended
- MEMS samples are appended through `processMEMS(buffer)`
- device temperature is appended at the end of the cycle

`processMEMS()` mainly:

- reads accelerometer/gyro/magnetometer data
- accumulates samples across the interval
- writes averaged acceleration/orientation to the buffer when a buffer is provided

Note that active trip motion tracking is mainly based on **OBD or GNSS speed**, not on MEMS vibration.

### Step 4: process GNSS

`processGPS(buffer)`:

- reads either standalone GNSS data or cellular-provided location
- rejects duplicate timestamps and obviously invalid coordinates
- generates the `isoTime` string
- stores latitude, longitude, altitude, speed, heading, satellite count, and HDOP into the buffer
- updates `lastMotionTime` when GNSS speed is at least 2 km/h

If GNSS reset timeout logic is enabled and GPS stops producing valid updates for too long, `process()` restarts the GNSS subsystem.

### Step 5: finalize and optionally persist the buffer

After sampling:

- the buffer timestamp is set
- its state becomes `BUFFER_STATE_FILLED`
- periodic buffer statistics are printed
- if storage is ready, the buffer is serialized to the logger and flushed when the file grows

At this point, the freshly produced sample is available both for local logging and for later network transmission.

### Step 6: adaptive timing and trip-end decision

`process()` then checks how long it has been since motion was last detected:

- `motionless = (millis() - lastMotionTime) / 1000`

Using `STATIONARY_TIME_TABLE` and `DATA_INTERVAL_TABLE`, it:

- increases the sampling interval as the vehicle remains still longer
- eventually decides the trip is over when the stationary threshold is reached

If the stationary timeout is reached:

- it prints a stationary message
- clears `STATE_WORKING`
- returns immediately

That return is what causes the next `loop()` iteration to go through `standby()`.

### Step 7: idle wait between samples

If the trip is still active, `process()` waits until the target sample interval has elapsed. During that wait it continues to call `processBLE()` so BLE commands remain responsive.

## 6. Background network/upload flow: `telemetry()`

`telemetry()` runs as a separate FreeRTOS task created from `setup()`. It is not responsible for collecting the raw data; instead it moves already-buffered samples to the server.

### What `telemetry()` does during standby

When `STATE_STANDBY` is set, the telemetry task:

- shuts down any active network sessions
- clears network-ready flags
- purges buffers
- waits for the configured ping-back interval
- optionally performs a ping over Wi-Fi or cellular before shutting down again

This allows the device to remain mostly quiet while parked but still occasionally check in.

### What `telemetry()` does during active operation

While `STATE_WORKING` is set, the telemetry task loops continuously and:

1. **prefers Wi-Fi when configured and available**
2. **falls back to cellular when Wi-Fi is not connected**
3. **establishes the transport with `teleClient.connect()`**
4. **tracks RSSI and reconnect health**
5. **takes the newest filled `CBuffer` from `bufman`**
6. **serializes it into a transport payload using `CStorageRAM`**
7. **calls `teleClient.transmit()` to upload the payload**
8. **prints traffic statistics with `showStats()` on success**
9. **tries reconnect strategies and increments timeout counters on failure**
10. **processes inbound server traffic**

This division of labor is central to the design:

- `process()` produces data
- `telemetry()` sends data

Because they run concurrently, data collection can continue even while the network link is slow or temporarily unavailable.

## 7. Standby path: `standby()`

`standby()` is entered when the unit is no longer in active working mode.

### Why `standby()` is entered

The most important reasons are:

- `process()` decided the vehicle has been stationary for too long
- repeated OBD failures caused the firmware to assume the ECU is off
- a BLE command such as `OFF` cleared `STATE_WORKING`

### What `standby()` does

When called, it:

1. sets `STATE_STANDBY`
2. closes the current storage session with `logger.end()` when storage is active
3. powers down standalone GNSS when allowed
4. clears `STATE_WORKING`, `STATE_OBD_READY`, and `STATE_STORAGE_READY`
5. sends the OBD/coproc link into low-power mode with `obd.enterLowPowerMode()`
6. waits for a wake condition

### Wake conditions

The wake behavior depends on enabled hardware:

- **MEMS enabled:** `calibrateMEMS()` then `waitMotion(-1)` waits for accelerometer motion above the configured threshold
- **No MEMS but OBD enabled:** it polls vehicle voltage until it rises above `JUMPSTART_VOLTAGE`
- **Fallback:** it simply delays

After wake is detected, the code:

- prints a wake-up message
- calls `sys.resetLink()`
- usually restarts the ESP with `ESP.restart()` when `RESET_AFTER_WAKEUP` is enabled

That restart causes the entire boot flow to begin again from `setup()`.

## 8. Helper functions that influence flow

A few helper functions do not control the top-level state machine by themselves, but they strongly affect runtime behavior.

### `processBLE()`

This function is called during idle time and waiting periods so BLE commands remain available. It can:

- query runtime values such as VIN, GPS, RSSI, packet counters, and filesystem size
- update APN/Wi-Fi credentials in NVS
- trigger `RESET`
- trigger `OFF`, which clears `STATE_WORKING` and causes the main loop to transition into standby

### `waitMotion()` and `waitMotionGPS()`

These functions are blocking wait helpers used when the firmware needs to pause until movement is detected.

### `initCell()`

This is the main modem bring-up helper. It powers on the cellular module, checks SIM status, applies APN settings, retrieves operator/IP data, and sets network state flags on success.

### `showStats()` and `printTimeoutStats()`

These are observability helpers used to understand link quality, upload throughput, and failure counts while the firmware is running.

## 9. Practical end-to-end summary

In simple terms, the runtime flow is:

1. **Boot** -> `setup()`
2. **Prepare subsystems** -> `initialize()`
3. **Start concurrent uploader** -> `telemetry()` task begins
4. **Repeated active sampling** -> `loop()` calls `process()`
5. **Collect OBD/MEMS/GNSS/device data** -> fill `CBuffer`
6. **Store locally and upload in background** -> logger + `telemetry()`
7. **Detect long stationary period or ECU-off condition** -> clear `STATE_WORKING`
8. **Enter standby** -> `standby()`
9. **Wait for wake motion/voltage**
10. **Restart and repeat**

That is the core code flow of the project: initialize hardware, sample and buffer data in the foreground, transmit in the background, and return to standby when the vehicle appears inactive.
