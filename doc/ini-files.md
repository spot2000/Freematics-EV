# SD Card INI Files

This document describes the INI-style configuration files that should exist on the microSD card, where they must be placed, and which parameters each file must contain.

## SD card folder structure

The firmware looks for configuration files in the `cfg` folder at the root of the SD card.

Required paths:

- `/cfg/wifi.ini`
- `/cfg/abrp.ini`

Example SD card layout:

```text
/
└── cfg/
    ├── wifi.ini
    └── abrp.ini
```

## File format

Each file must be a simple `key=value` text file.

Rules:

- One setting per line.
- No section headers such as `[wifi]` or `[abrp]`.
- The parameter name must match the expected key exactly.
- Values are read as plain text.
- Lines should use the format `parameter_name=value`.

## `/cfg/wifi.ini`

This file contains the Wi-Fi credentials used by the device.

Required parameters:

- `wifi_ssid` — the Wi-Fi network name (SSID).
- `wifi_password` — the Wi-Fi password.

Example:

```ini
wifi_ssid=YOUR_WIFI_SSID
wifi_password=YOUR_WIFI_PASSWORD
```

## `/cfg/abrp.ini`

This file contains the ABRP user key used for telemetry uploads.

Required parameters:

- `abrp_user_key` — the ABRP user key for the vehicle/account.

Example:

```ini
abrp_user_key=YOUR_ABRP_USER_KEY
```

## Summary

To configure the SD card correctly:

1. Create a folder named `cfg` in the root of the SD card.
2. Create `wifi.ini` inside `/cfg` with `wifi_ssid` and `wifi_password`.
3. Create `abrp.ini` inside `/cfg` with `abrp_user_key`.
4. Make sure every line uses the exact `key=value` format.
