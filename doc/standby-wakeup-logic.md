# Freematics standby and wake-up logic

This document describes how the Freematics unit decides that a trip has ended, when it pauses into standby, and what makes it start running again.

## 1. What "pause" means in this project

In the code, the unit is considered **active** while `STATE_WORKING` is set. When the software decides that the vehicle is no longer active, it clears `STATE_WORKING`. The main loop then enters `standby()`, which is the low-power paused state.

During standby, the device:

- marks itself as being in `STATE_STANDBY`
- closes the storage session
- turns GNSS off when `GNSS_ALWAYS_ON` is disabled
- clears `STATE_WORKING`, `STATE_OBD_READY`, and `STATE_STORAGE_READY`
- sends the OBD adapter into low-power mode
- waits for a wake condition

## 2. What counts as "motion" while the unit is active

The unit mainly uses **vehicle speed** to decide whether the vehicle is still moving.

### OBD-based motion detection

When OBD data is available, the code reads vehicle speed from OBD. If speed is **2 km/h or higher**, it updates `lastMotionTime`.

### GNSS-based motion detection

When valid GNSS data is available, the code converts GNSS speed to km/h. If the GNSS speed is **2 km/h or higher**, it also updates `lastMotionTime`.

### Important detail

The MEMS accelerometer is **not** what keeps the trip alive during normal active logging. MEMS is used mainly for **wake-up detection during standby**. The motion update inside `processMEMS()` is currently disabled in a `#if 0` block.

## 3. How the device decides it has been still too long

At the end of every active `process()` cycle, the code calculates how many seconds have passed since `lastMotionTime`:

- `motionless = (millis() - lastMotionTime) / 1000`

The firmware uses two configuration tables:

- `STATIONARY_TIME_TABLE {10, 60, 180}` seconds
- `DATA_INTERVAL_TABLE {1000, 2000, 5000}` ms

These tables are used in two ways:

1. **Adaptive logging interval while the vehicle is becoming idle**
   - under 10 seconds since last motion -> 1 second data interval
   - 10 to under 60 seconds -> 2 second data interval
   - 60 to under 180 seconds -> 5 second data interval

2. **Final transition to standby**
   - when the unit has been motionless for **180 seconds or more**, it is considered stationary
   - the code prints `Stationary for ... secs`
   - `STATE_WORKING` is cleared
   - the current loop returns, and the main loop then enters `standby()`

So in normal operation, the Freematics unit pauses after about **3 minutes without detected motion**, where motion means OBD or GNSS speed of at least 2 km/h.

## 4. Other ways the unit can pause or drop into standby-like behavior

Even if the vehicle has not simply been stationary for 180 seconds, the active state can also stop in these cases:

### OBD communication failure / ECU off

If OBD errors reach `MAX_OBD_ERRORS` (configured as `3`), the firmware tries to re-initialize OBD.

- If OBD re-init succeeds, logging continues.
- If OBD re-init fails, the code assumes the ECU is off, prints `[OBD] ECU OFF`, clears `STATE_OBD_READY` and `STATE_WORKING`, and the next main-loop iteration enters standby.

This means the unit can pause because the vehicle effectively turned off, not only because of the stationary timer.

### Manual BLE command

If BLE is enabled and the command `OFF` is received, the code sets `STATE_STANDBY` and clears `STATE_WORKING`, which also causes the system to enter standby logic.

## 5. What happens inside standby

When `standby()` runs, the firmware prepares the hardware for low-power waiting:

1. `STATE_STANDBY` is set.
2. Storage logging is closed.
3. Standalone GNSS is powered down unless `GNSS_ALWAYS_ON` is enabled.
4. Active/ready flags are cleared.
5. The OBD interface enters low-power mode.
6. The device waits until a wake source is detected.

## 6. What wakes the unit up again

The wake-up source depends on which hardware/features are enabled.

### Normal case: MEMS wake-up

If MEMS support is enabled and ready, the firmware:

- calibrates the accelerometer bias
- calls `waitMotion(-1)`
- repeatedly reads accelerometer values
- calculates relative motion against the bias
- wakes when motion is at or above `MOTION_THRESHOLD`

`MOTION_THRESHOLD` is configured as:

- `0.4f` G

In practice, this means a sufficiently strong accelerometer movement wakes the device.

### Fallback case: voltage rise / engine jump-start

If MEMS is not enabled but OBD is enabled, the firmware polls vehicle voltage every 5 seconds and wakes when voltage reaches:

- `JUMPSTART_VOLTAGE = 14 V`

This is intended to detect that the vehicle/charging system has become active again.

### Last fallback

If neither MEMS nor OBD wake logic is available, the code just delays 5 seconds and continues. That is a minimal fallback rather than a motion-based wake-up strategy.

## 7. What happens immediately after wake-up

Once a wake condition is met, the firmware:

1. prints `WAKEUP FROM STANDBY`
2. calls `sys.resetLink()`
3. because `RESET_AFTER_WAKEUP` is set to `1`, it restarts the ESP with `ESP.restart()`

After reboot, the normal startup path runs again, and the unit re-initializes OBD, GNSS, storage, networking, and telemetry.

## 8. Practical summary

### The unit pauses into standby when

- it has been motionless for **180 seconds** based on OBD/GNSS speed tracking
- OBD communication fails repeatedly and the ECU appears to be off
- a manual BLE `OFF` command requests standby

### The unit starts again when

- MEMS detects motion above **0.4 G**, or
- vehicle voltage rises to **14 V** when MEMS wake-up is unavailable

### Important nuance

While the system is active, "still/passive" is not determined by MEMS vibration. It is determined primarily by whether OBD or GNSS keeps updating `lastMotionTime` through speed readings of **2 km/h or more**.
